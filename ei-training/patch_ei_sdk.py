#!/usr/bin/env python3
"""Re-apply the ESP32-S3 fixes that a fresh Edge Impulse library overwrites.

Every retrain produces a new Arduino library zip, and installing it restores
the SDK's own copy of the porting header - which silently undoes this fix and
brings back a crash that looks nothing like a configuration problem.

The fix: edge-impulse-sdk/porting/ei_classifier_porting.h hard-defines

    #if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define EI_MAX_OVERFLOW_BUFFER_COUNT   30
    #endif

with no #ifndef guard, so -D flags and the model's own value are all ignored.
Thirty overflow buffers is not enough for a MobileNet image model on this
board: run_classifier fails with

    ERR: Failed to allocate persistent buffer of size N, does not fit in
         tensor arena and reached EI_MAX_OVERFLOW_BUFFER_COUNT
    Guru Meditation Error: Core 1 panic'ed (StoreProhibited)

and the board reboot-loops. Raising it to 2048 fixes it. Documented in
day3/.agents/skills/xiao-edgeimpulse-train/SKILL.md step 5.

    python patch_ei_sdk.py                  patch the default library
    python patch_ei_sdk.py <library_name>   patch a differently named one

Idempotent - safe to run when already patched. ALWAYS compile with --clean
afterwards: arduino-cli caches objects per library name/version, so an edited
header can otherwise produce a byte-identical binary.
"""
import subprocess
import sys
from pathlib import Path

DEFAULT_LIB = "team_project_inferencing"
WANT = 2048


def sketchbook() -> Path:
    r = subprocess.run(["arduino-cli", "config", "get", "directories.user"],
                       capture_output=True, text=True, shell=True)
    if r.returncode != 0 or not r.stdout.strip():
        raise SystemExit("could not read arduino-cli sketchbook path")
    return Path(r.stdout.strip())


def patch(lib_name: str = DEFAULT_LIB) -> bool:
    header = (sketchbook() / "libraries" / lib_name / "src" / "edge-impulse-sdk"
              / "porting" / "ei_classifier_porting.h")
    if not header.is_file():
        raise SystemExit(f"header not found - is '{lib_name}' installed?\n  {header}")

    text = header.read_text(encoding="utf-8", errors="replace")

    # Only the ESP32-S3 branch. The Armv8.1-M branch above it defines the same
    # macro with a different value and must be left alone.
    marker = "#if defined(CONFIG_IDF_TARGET_ESP32S3)"
    i = text.find(marker)
    if i < 0:
        raise SystemExit("ESP32-S3 branch not found - the SDK layout changed; "
                         "re-read SKILL.md step 5 before guessing")
    end = text.find("#endif", i)
    block = text[i:end]

    if f"EI_MAX_OVERFLOW_BUFFER_COUNT\t{WANT}" in block or \
       f"EI_MAX_OVERFLOW_BUFFER_COUNT {WANT}" in block:
        print(f"[patch] already patched ({WANT}) - nothing to do")
        return False

    import re
    new_block, n = re.subn(r"(EI_MAX_OVERFLOW_BUFFER_COUNT\s+)\d+",
                           lambda m: m.group(1) + str(WANT), block)
    if n != 1:
        raise SystemExit(f"expected exactly 1 define in the ESP32-S3 branch, found {n}")

    header.write_text(text[:i] + new_block + text[end:], encoding="utf-8")
    print(f"[patch] EI_MAX_OVERFLOW_BUFFER_COUNT -> {WANT} in {lib_name}")
    print("[patch] now compile with --clean, or the cached objects win")
    return True


if __name__ == "__main__":
    patch(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LIB)
