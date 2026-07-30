#!/usr/bin/env python3
"""Image data collector for XIAO ESP32S3 Sense vision node (/jpg HTTP endpoint)."""

import sys
import os
import time
import glob
import requests

BOARD_IP = os.environ.get("IMG_IP", "192.168.0.30")
INGEST_URL = "https://ingestion.edgeimpulse.com/api/training/files"
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MENU = {"1": "mouse", "2": "cup", "3": "phone", "4": "unknown"}


def load_key() -> str:
    env_path = os.path.join(HERE, ".env")
    try:
        with open(env_path, "r") as f:
            for line in f:
                if line.startswith("EI_API_KEY="):
                    return line.split("=", 1)[1].strip()
    except FileNotFoundError:
        pass
    raise SystemExit("EI_API_KEY not found in .env")


def capture_burst(label: str, count: int, interval_ms: int, upload: bool) -> None:
    if upload:
        key = load_key()

    print("  3")
    time.sleep(0.6)
    print("  2")
    time.sleep(0.6)
    print("  1")
    time.sleep(0.6)
    print("  GO")
    time.sleep(0.6)

    for i in range(count):
        try:
            r = requests.get("http://" + BOARD_IP + "/jpg", timeout=5)
            if r.status_code == 200 and len(r.content) > 0:
                label_dir = os.path.join(HERE, "dataset", label)
                os.makedirs(label_dir, exist_ok=True)
                existing = glob.glob(os.path.join(label_dir, "*.jpg"))
                n = len(existing) + 1
                filename = "%s.%d.jpg" % (label, n)
                filepath = os.path.join(label_dir, filename)
                with open(filepath, "wb") as f:
                    f.write(r.content)
                rel_path = os.path.join("dataset", label, filename)
                print("saved  %s (%d bytes)" % (rel_path, len(r.content)))

                if upload:
                    with open(filepath, "rb") as fh:
                        r2 = requests.post(
                            INGEST_URL,
                            headers={"x-api-key": key, "x-label": label},
                            files={"data": (filename, fh, "image/jpeg")},
                            timeout=30,
                        )
                    if r2.status_code == 200:
                        print(">> uploaded (label=%s)" % label)
                    else:
                        body_preview = r2.text[:150]
                        print("!! UPLOAD FAILED %d: %s (local file kept)" % (r2.status_code, body_preview))
            else:
                reason = "HTTP %d" % r.status_code if r.status_code != 200 else "empty body"
                print("!! frame failed: %s" % reason)
        except requests.exceptions.RequestException as e:
            print("!! frame failed: %s" % e)

        if i < count - 1:
            time.sleep(interval_ms / 1000.0)


def interactive(upload: bool) -> None:
    print("Edge Impulse image collector (board: %s)" % BOARD_IP)
    print("   1 = mouse")
    print("   2 = cup")
    print("   3 = phone")
    print("   4 = unknown")
    print("   or type any custom label")
    print("   q = quit")

    default_shots = 10
    default_interval = 500

    while True:
        try:
            raw = input("\nlabel> ")
        except (EOFError, KeyboardInterrupt):
            print("bye")
            return
        inp = raw.strip()
        if inp.lower() in ("q", "quit", "exit"):
            print("bye")
            return
        if not inp:
            continue
        if inp in MENU:
            label = MENU[inp]
        else:
            label = inp

        try:
            raw_shots = input("shots [%d]> " % default_shots).strip()
        except (EOFError, KeyboardInterrupt):
            print("bye")
            return
        if raw_shots:
            try:
                default_shots = int(raw_shots)
            except ValueError:
                pass

        try:
            raw_int = input("interval ms [%d]> " % default_interval).strip()
        except (EOFError, KeyboardInterrupt):
            print("bye")
            return
        if raw_int:
            try:
                default_interval = int(raw_int)
            except ValueError:
                pass

        try:
            capture_burst(label, default_shots, default_interval, upload)
        except Exception as e:
            print("!! error: %s" % e)


def main() -> None:
    upload = True
    args = list(sys.argv[1:])
    if "--no-upload" in args:
        upload = False
        args.remove("--no-upload")

    if not args:
        interactive(upload)
        return

    label = args[0]
    count = int(args[1]) if len(args) > 1 else 10
    interval_ms = int(args[2]) if len(args) > 2 else 500
    capture_burst(label, count, interval_ms, upload)


if __name__ == "__main__":
    main()
