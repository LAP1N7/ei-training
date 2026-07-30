#!/usr/bin/env python3
"""Gesture collector - XIAO ESP32S3 + BNO055.

Ported from day3/tools/imu_collect.py for this project. Reads the 100 Hz
"ax,ay,az" stream the board emits (gesture_collect sketch), records one labeled
window, saves it as an Edge Impulse data-acquisition JSON under
data/gesture/<label>/, and uploads it straight to the ingestion API.

    python collect_gesture.py                       interactive (recommended)
    python collect_gesture.py shaking 10            record 10s of shaking
    python collect_gesture.py --no-upload spin 8    save locally only

The alarm mission only needs three classes:
    idle     hold still - without it, any movement leaks through as a success
    shaking  swing the arm back and forth, like walking
    spin     move the sensor in a circle

At least 10 recordings of 10s per class. Keep the wrist orientation and the
hand you hold it in the same every time.
"""
import glob
import json
import os
import sys
import time

import requests
import serial  # pyserial

from ei import DEFAULT_KEY
from port import detect

# Detected per machine - do not hard-code a COM number, it differs per PC.
PORT = detect()
BAUD = 115200
HZ = 100
INTERVAL_MS = 1000 // HZ
SENSORS = [{"name": "accX", "units": "m/s2"},
           {"name": "accY", "units": "m/s2"},
           {"name": "accZ", "units": "m/s2"}]
INGEST_URL = "https://ingestion.edgeimpulse.com/api/training/files"

HERE = os.path.dirname(os.path.abspath(__file__))
DATASET = os.path.join(HERE, "data", "gesture")

MENU = {"1": "idle", "2": "shaking", "3": "spin"}


def api_key():
    return os.environ.get("EI_API_KEY") or DEFAULT_KEY


def open_port():
    """Only one program may hold the port, so translate the OS error into advice."""
    if not PORT:
        raise SystemExit(
            "no board detected. Plug the XIAO in over USB, or name the port\n"
            "  explicitly:  set IMU_PORT=COM7")
    try:
        return serial.Serial(PORT, BAUD, timeout=2)
    except serial.SerialException as e:
        if "PermissionError" in str(e) or "Access is denied" in str(e):
            raise SystemExit(
                "cannot open %s - another program is holding it.\n"
                "  Close the serial monitor (arduino-cli monitor / Arduino IDE)\n"
                "  and any other collector window, then try again." % PORT)
        raise SystemExit("cannot open %s: %s" % (PORT, e))


def record(seconds):
    """Open the port, warm up, then collect `seconds` worth of ax,ay,az rows."""
    ser = open_port()
    time.sleep(2.0)  # in case the USB-CDC reset reboots the board
    ser.reset_input_buffer()

    print("connecting to board...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 6:
        ln = ser.readline().decode("utf-8", "replace").strip()
        if ln and not ln.startswith("#") and ln.count(",") == 2:
            break
    else:
        ser.close()
        raise SystemExit(
            "no data from board - is gesture_collect flashed on %s?\n"
            "  Run flash_gesture.bat first." % PORT)

    for c in ("3", "2", "1", "GO"):  # time to get into position
        print("  " + c, flush=True)
        time.sleep(0.6)

    values, need = [], seconds * HZ
    ser.reset_input_buffer()
    while len(values) < need:
        ln = ser.readline().decode("utf-8", "replace").strip()
        if not ln or ln.startswith("#"):
            continue
        try:
            x, y, z = (float(v) for v in ln.split(","))
        except ValueError:
            continue
        values.append([x, y, z])
        if len(values) % HZ == 0:
            print("  %2d/%2d s" % (len(values) // HZ, seconds), flush=True)
    ser.close()
    return values


def build_json(values):
    # alg "none" with a zero signature. This passes as long as the project does
    # not enforce HMAC - the day3 dataset was uploaded in exactly this shape.
    return {
        "protected": {"ver": "v1", "alg": "none", "iat": int(time.time())},
        "signature": "0" * 64,
        "payload": {
            "device_name": "xiao-bno055",
            "device_type": "XIAO_ESP32S3",
            "interval_ms": INTERVAL_MS,
            "sensors": SENSORS,
            "values": values,
        },
    }


def capture_one(label, seconds, upload=True):
    print("\n== recording '%s' for %ds on %s ==" % (label, seconds, PORT))
    values = record(seconds)
    print("captured %d samples (%.1f s)" % (len(values), len(values) / HZ))

    outdir = os.path.join(DATASET, label)
    os.makedirs(outdir, exist_ok=True)
    n = len(glob.glob(os.path.join(outdir, "*.json"))) + 1
    fname = "%s.%d.json" % (label, n)
    fpath = os.path.join(outdir, fname)
    with open(fpath, "w") as f:
        json.dump(build_json(values), f)
    print("saved  data/gesture/%s/%s" % (label, fname))

    if upload:
        with open(fpath, "rb") as fh:
            r = requests.post(
                INGEST_URL,
                headers={"x-api-key": api_key(), "x-label": label},
                files={"data": (fname, fh, "application/json")},
                timeout=30)
        ok = False
        try:
            ok = r.json().get("success", False)
        except Exception:
            pass
        if r.status_code == 200 and ok:
            print(">> uploaded to Edge Impulse (label=%s)  [total %s: %d]" % (label, label, n))
        else:
            print("!! UPLOAD FAILED %d: %s (local file kept)" % (r.status_code, r.text[:150]))


def interactive():
    print("=" * 48)
    print(" gesture collector  (port %s, %d Hz)" % (PORT, HZ))
    print("=" * 48)
    print(" pick a motion, then do only that motion for the whole recording:")
    for k, v in MENU.items():
        print("   %s = %s" % (k, v))
    print("   or type any custom label")
    print("   q = quit")
    seconds = 10
    while True:
        try:
            sel = input("\nmotion> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nbye")
            return
        if sel.lower() in ("q", "quit", "exit"):
            print("bye")
            return
        if not sel:
            continue
        label = MENU.get(sel, sel)
        s = input("seconds [%d]> " % seconds).strip()
        if s:
            try:
                seconds = int(s)
            except ValueError:
                pass
        try:
            capture_one(label, seconds, upload=True)
        except SystemExit as e:
            print("!! %s" % e)
        except Exception as e:
            print("!! error: %s" % e)


def main():
    args = list(sys.argv[1:])
    upload = True
    if "--no-upload" in args:
        upload = False
        args.remove("--no-upload")
    if not args:
        interactive()
        return
    label = args[0]
    seconds = int(args[1]) if len(args) > 1 else 10
    capture_one(label, seconds, upload=upload)


if __name__ == "__main__":
    main()
