"""Edge Impulse Studio API client - training + deployment only.

Endpoint paths here were read out of the installed edge-impulse-cli SDK
(build/sdk/studio/sdk/api/*.js), not guessed. If a call starts 404-ing, that
is where to re-check it.

stdlib + requests only. requests ships with the Anaconda python already on PATH.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import requests

BASE = "https://studio.edgeimpulse.com/v1/api"

# Default for this project. Override with the EI_API_KEY environment variable.
DEFAULT_KEY = "ei_c39ca84a30d6c511470c99f5bddec1481f783ce8ab8c2815"


def log(msg: str) -> None:
    print(msg, flush=True)


class EIError(RuntimeError):
    pass


class EI:
    def __init__(self, api_key: str | None = None, project_id: int | None = None):
        self.key = api_key or os.environ.get("EI_API_KEY") or DEFAULT_KEY
        self.s = requests.Session()
        self.s.headers.update({"x-api-key": self.key})
        self.pid = project_id or self._resolve_project()

    # ---- plumbing ----

    def _req(self, method: str, path: str, **kw):
        r = self.s.request(method, f"{BASE}/{path}", timeout=120, **kw)
        try:
            body = r.json()
        except ValueError:
            raise EIError(f"{method} {path} -> HTTP {r.status_code}, non-JSON body:\n{r.text[:500]}")
        if not body.get("success", False):
            raise EIError(f"{method} {path} -> {body.get('error') or body}")
        return body

    def get(self, path: str, **kw):
        return self._req("GET", path, **kw)

    def post(self, path: str, payload=None, **kw):
        return self._req("POST", path, json=payload if payload is not None else {}, **kw)

    def _resolve_project(self) -> int:
        """A project API key is scoped to exactly one project - just read it back."""
        r = self.s.get(f"{BASE}/projects", timeout=60)
        body = r.json()
        if not body.get("success"):
            raise EIError(f"API key rejected: {body.get('error')}")
        projects = body.get("projects") or []
        if not projects:
            raise EIError("this API key is not attached to any project")
        p = projects[0]
        log(f"[ei] project: {p['name']} (id={p['id']})")
        return p["id"]

    # ---- impulse ----

    def set_impulse(self, impulse: dict) -> None:
        self.post(f"{self.pid}/impulse", impulse)
        log("[ei] impulse configured")

    def set_dsp_config(self, dsp_id: int, config: dict) -> None:
        # The endpoint wants the parameters nested under "config", not at the top
        # level - a flat body comes back as: Missing "config" in body.
        self.post(f"{self.pid}/dsp/{dsp_id}", {"config": config})
        log(f"[ei] dsp block {dsp_id} configured")

    # ---- jobs ----

    def wait(self, job_id: int, what: str) -> None:
        """Poll a job to completion, streaming its stdout as it arrives."""
        log(f"[ei] {what}: job {job_id} started")
        seen = 0
        while True:
            time.sleep(4)
            try:
                out = self.get(f"{self.pid}/jobs/{job_id}/stdout", params={"limit": 500})
                lines = list(reversed(out.get("stdout") or []))
                for line in lines[seen:]:
                    txt = (line.get("data") or "").rstrip()
                    if txt:
                        print(f"    | {txt}", flush=True)
                seen = max(seen, len(lines))
            except EIError:
                pass  # stdout is best-effort; status is what decides

            st = self.get(f"{self.pid}/jobs/{job_id}/status")
            job = st.get("job") or {}
            if job.get("finished"):
                if job.get("finishedSuccessful"):
                    log(f"[ei] {what}: done")
                    return
                raise EIError(f"{what} failed - see the job log above, or {self.url()}")

    def generate_features(self, dsp_id: int) -> None:
        r = self.post(
            f"{self.pid}/jobs/generate-features",
            {"dspId": dsp_id, "calculateFeatureImportance": False, "skipFeatureExplorer": True},
        )
        self.wait(r["id"], "generate features")

    def train(self, learn_id: int, params: dict) -> None:
        r = self.post(f"{self.pid}/jobs/train/keras/{learn_id}", params)
        self.wait(r["id"], "train")

    # ---- results ----

    def metrics(self, learn_id: int) -> None:
        """Report float32 AND int8 accuracy.

        This matters more than it looks: the board runs the int8 model, not the
        float32 one the training log prints. A model can finish at
        val_accuracy 1.0 and still quantize down to coin-flip territory, and
        nothing in the training output warns you.
        """
        try:
            md = self.get(f"{self.pid}/training/keras/{learn_id}/metadata")
        except EIError as e:
            log(f"[ei] (could not read metrics: {e})")
            return
        rows = md.get("modelValidationMetrics") or []
        if not rows:
            return
        acc = {}
        for row in rows:
            kind = row.get("type")
            acc[kind] = row.get("accuracy")
            log(f"[ei] {kind:>7} accuracy: {row.get('accuracy')}  loss: {row.get('loss')}")

        f32, i8 = acc.get("float32"), acc.get("int8")
        if f32 and i8 and f32 - i8 > 0.1:
            log("[ei] WARNING: the int8 model is much worse than float32, and int8")
            log("[ei]          is what runs on the board. Either deploy unoptimized")
            log("[ei]          (float32), or collect more varied training data.")

    def deploy(self, target: str = "arduino", engine: str = "tflite-eon",
               out: Path | None = None) -> Path | None:
        """Build the on-device model and download it as a zip."""
        r = self.post(f"{self.pid}/jobs/build-ondevice-model", {"engine": engine},
                      params={"type": target})
        self.wait(r["id"], f"build {target}")

        resp = self.s.get(f"{BASE}/{self.pid}/deployment/download",
                          params={"type": target}, timeout=300)
        if resp.status_code != 200 or resp.content[:2] != b"PK":
            log(f"[ei] download failed (HTTP {resp.status_code}) - grab it from {self.url()}")
            return None
        out = out or Path.cwd() / f"ei-{target}-{self.pid}.zip"
        out.write_bytes(resp.content)
        log(f"[ei] deployment saved: {out}  ({len(resp.content) // 1024} KB)")
        return out

    def url(self) -> str:
        return f"https://studio.edgeimpulse.com/studio/{self.pid}"


# ---- data upload (delegated to the official CLI - it owns the ingestion format) ----

def upload(files: list[Path], label: str, api_key: str, category: str = "split") -> None:
    """edge-impulse-uploader is a node shim; on Windows it must run through the shell."""
    if not files:
        return
    cmd = [
        "edge-impulse-uploader",
        "--api-key", api_key,
        "--category", category,
        "--label", label,
        "--silent",
    ] + [str(f) for f in files]
    log(f"[upload] {label}: {len(files)} file(s)")
    r = subprocess.run(cmd, shell=(os.name == "nt"), capture_output=True, text=True)
    if r.returncode != 0:
        raise EIError(
            f"upload of label '{label}' failed (exit {r.returncode}):\n"
            f"{(r.stderr or r.stdout)[-2000:]}"
        )


def collect_labels(root: Path, exts: tuple[str, ...]) -> dict[str, list[Path]]:
    """data/<label>/<files> -> {label: [paths]}. Empty label dirs are skipped."""
    if not root.is_dir():
        raise EIError(f"data directory not found: {root}")
    out: dict[str, list[Path]] = {}
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        files = sorted(f for f in d.iterdir() if f.suffix.lower() in exts)
        if files:
            out[d.name] = files
    if not out:
        raise EIError(
            f"no {'/'.join(exts)} files under {root}\n"
            f"  put samples in {root}\\<label>\\ first - one folder per class"
        )
    return out


def die(msg: str) -> None:
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(1)
