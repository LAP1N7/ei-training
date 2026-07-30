#!/usr/bin/env python3
"""IMU gesture data collector for XIAO ESP32S3 + BNO055.

Reads the 100 Hz "ax,ay,az" CSV stream from the board (sketch 10_imu_collect),
records a labeled window, saves it as an Edge Impulse data-acquisition JSON in
dataset/<label>/, and uploads it to the Edge Impulse ingestion API.

Usage (run it yourself in a terminal, sensor in hand):

    python tools/imu_collect.py <label> [seconds]
    python tools/imu_collect.py walking 12
    python tools/imu_collect.py idle 10
    python tools/imu_collect.py --no-upload wave 8   # save locally only

Repeat several times per class. Include an 'idle' class (hold still).
Tip: keep the sensor orientation/hand consistent; do many repetitions.
"""
import sys, os, time, json, glob
import serial            # pip install pyserial
import requests          # pip install requests

PORT       = os.environ.get("IMU_PORT", "COM6")
BAUD       = 115200
HZ         = 100
INTERVAL_MS = 1000 // HZ
SENSORS    = [{"name": "accX", "units": "m/s2"},
              {"name": "accY", "units": "m/s2"},
              {"name": "accZ", "units": "m/s2"}]
INGEST_URL = "https://ingestion.edgeimpulse.com/api/training/files"

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # day3/


def load_key():
    env = os.path.join(HERE, ".env")
    with open(env) as f:
        for line in f:
            if line.startswith("EI_API_KEY="):
                return line.strip().split("=", 1)[1]
    raise SystemExit("EI_API_KEY not found in .env")


def record(seconds):
    """Open serial, warm up, then collect `seconds` of ax,ay,az rows."""
    ser = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(2.0)                 # allow a possible USB-CDC reset to boot
    ser.reset_input_buffer()

    # wait for a clean data line (skip '#' banners)
    print("connecting to board...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 6:
        ln = ser.readline().decode("utf-8", "replace").strip()
        if ln and not ln.startswith("#") and ln.count(",") == 2:
            break
    else:
        ser.close(); raise SystemExit("no data from board — is 10_imu_collect flashed on %s?" % PORT)

    for c in ("3", "2", "1", "GO"):     # countdown so you can get ready
        print("  " + c, flush=True); time.sleep(0.6)

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
            print("  %2d/%2d s" % (len(values)//HZ, seconds), flush=True)
    ser.close()
    return values


def build_json(values):
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
    """Record one window for `label`, save locally, and (optionally) upload."""
    print("\n== recording '%s' for %ds on %s ==" % (label, seconds, PORT))
    values = record(seconds)
    print("captured %d samples (%.1f s)" % (len(values), len(values)/HZ))

    outdir = os.path.join(HERE, "dataset", label)
    os.makedirs(outdir, exist_ok=True)
    n = len(glob.glob(os.path.join(outdir, "*.json"))) + 1
    fname = "%s.%d.json" % (label, n)
    fpath = os.path.join(outdir, fname)
    with open(fpath, "w") as f:
        json.dump(build_json(values), f)
    print("saved  dataset/%s/%s" % (label, fname))

    if upload:
        key = load_key()
        with open(fpath, "rb") as fh:
            r = requests.post(
                INGEST_URL,
                headers={"x-api-key": key, "x-label": label},
                files={"data": (fname, fh, "application/json")},
                timeout=30)
        ok = False
        try: ok = r.json().get("success", False)
        except Exception: pass
        if r.status_code == 200 and ok:
            print(">> uploaded to Edge Impulse (label=%s)  [total %s: %d]" % (label, label, n))
        else:
            print("!! UPLOAD FAILED %d: %s (local file kept)" % (r.status_code, r.text[:150]))


MENU = {"1": "idle", "2": "walking", "3": "running", "4": "wave", "5": "spin"}


def interactive():
    print("=" * 48)
    print(" BNO055 gesture collector  (port %s, %d Hz)" % (PORT, HZ))
    print("=" * 48)
    print(" pick a motion, do it for the whole recording:")
    for k, v in MENU.items():
        print("   %s = %s" % (k, v))
    print("   or type any custom label")
    print("   q = quit")
    seconds = 10
    while True:
        try:
            sel = input("\nmotion> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nbye"); return
        if sel.lower() in ("q", "quit", "exit"):
            print("bye"); return
        if not sel:
            continue
        label = MENU.get(sel, sel)
        s = input("seconds [%d]> " % seconds).strip()
        if s:
            try: seconds = int(s)
            except ValueError: pass
        try:
            capture_one(label, seconds, upload=True)
        except Exception as e:
            print("!! error: %s" % e)


def main():
    args = [a for a in sys.argv[1:]]
    upload = True
    if "--no-upload" in args:
        upload = False; args.remove("--no-upload")
    if not args:
        interactive(); return
    label = args[0]
    seconds = int(args[1]) if len(args) > 1 else 10
    capture_one(label, seconds, upload=upload)


if __name__ == "__main__":
    main()
