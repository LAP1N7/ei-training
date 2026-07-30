"""Gesture training - idle / shaking / spin.

Uploads data/gesture/<label>/*.json, configures a time-series impulse
(Spectral Analysis + Classification), and runs the training job. Training and
inference only - none of the alarm app logic lives here.

    python train_gesture.py                 upload + train
    python train_gesture.py --skip-upload   retrain on data already on the server
    python train_gesture.py --deploy        also download the Arduino library
"""
from __future__ import annotations

import argparse
from pathlib import Path

from ei import EI, EIError, collect_labels, die, log, upload

HERE = Path(__file__).resolve().parent

# IMU axis names. These must match what collect_gesture.py writes - if they
# differ the DSP block cannot find the axes and feature generation dies.
AXES = ["accX", "accY", "accZ"]

DSP_ID = 2
LEARN_ID = 3


def build_impulse(window_ms: int, stride_ms: int, freq_hz: int) -> dict:
    return {
        "inputBlocks": [{
            "id": 1, "type": "time-series", "name": "Time series",
            "title": "Time series data",
            "windowSizeMs": window_ms,
            "windowIncreaseMs": stride_ms,
            "frequencyHz": freq_hz,
            "padZeros": True,
        }],
        "dspBlocks": [{
            "id": DSP_ID, "type": "spectral-analysis", "name": "Spectral features",
            "title": "Spectral Analysis", "axes": AXES, "input": 1,
            "implementationVersion": 4,
        }],
        "learnBlocks": [{
            "id": LEARN_ID, "type": "keras", "name": "Classifier",
            "title": "Classification", "dsp": [DSP_ID],
        }],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Edge Impulse gesture training")
    ap.add_argument("--data", type=Path, default=HERE / "data" / "gesture")
    ap.add_argument("--api-key", default=None, help="defaults to $EI_API_KEY, then ei.py DEFAULT_KEY")
    # 100 Hz over 2000 ms = 200 samples, the same window as RAW(=200) in the
    # day3 inference sketch. If any of these three drifts away from the
    # collector (collect_gesture.py HZ=100), accuracy falls apart.
    ap.add_argument("--window", type=int, default=2000, help="window length (ms)")
    ap.add_argument("--stride", type=int, default=250, help="window increase (ms)")
    ap.add_argument("--freq", type=int, default=100, help="sampling frequency (Hz)")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--learning-rate", type=float, default=0.0005)
    ap.add_argument("--skip-upload", action="store_true")
    ap.add_argument("--deploy", action="store_true", help="download the Arduino library zip after training")
    args = ap.parse_args()

    try:
        ei = EI(api_key=args.api_key)

        if not args.skip_upload:
            labels = collect_labels(args.data, (".json", ".cbor"))
            log(f"[gesture] labels: {', '.join(labels)}")
            for label, files in labels.items():
                upload(files, label, ei.key)

        ei.set_impulse(build_impulse(args.window, args.stride, args.freq))
        ei.generate_features(DSP_ID)
        ei.train(LEARN_ID, {
            "mode": "visual",
            "trainingCycles": args.epochs,
            "learningRate": args.learning_rate,
            "trainTestSplit": 0.2,
            "visualLayers": [
                {"type": "dense", "neurons": 20, "dropoutRate": 0.25},
                {"type": "dense", "neurons": 10, "dropoutRate": 0.25},
            ],
        })
        ei.metrics(LEARN_ID)

        if args.deploy:
            ei.deploy("arduino", out=HERE / "gesture-arduino-lib.zip")

        log(f"\n[gesture] done - {ei.url()}")
    except EIError as e:
        die(str(e))


if __name__ == "__main__":
    main()
