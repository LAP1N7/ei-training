"""Image classification training - mouse / papercup / phonecase.

Uploads data/vision/<label>/*.jpg, configures an image impulse (Image DSP +
Transfer Learning), and runs the training job. Training and inference only.

    python train_vision.py                 upload + train
    python train_vision.py --skip-upload   retrain on data already on the server
    python train_vision.py --deploy        also download the Arduino library

NOTE: gesture and image models cannot share one Edge Impulse project - a
project holds a single impulse and the two data types do not mix. Create a
second project in the studio and pass its key via --api-key or EI_API_KEY.
See README.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from ei import EI, EIError, collect_labels, die, log, upload

HERE = Path(__file__).resolve().parent

DSP_ID = 2
LEARN_ID = 3

# Sized for XIAO ESP32S3 Sense with PSRAM: 96x96 runs comfortably on
# MobileNetV2 0.35. Going to 160x160 buys accuracy but grows the tensor arena
# and slows inference down.
IMAGE_SIZE = 96


def build_impulse(size: int, color: str) -> dict:
    return {
        "inputBlocks": [{
            "id": 1, "type": "image", "name": "Image", "title": "Image data",
            "imageWidth": size, "imageHeight": size,
            "resizeMode": "squash", "cropAnchor": "middle-center",
        }],
        "dspBlocks": [{
            "id": DSP_ID, "type": "image", "name": "Image", "title": "Image",
            "axes": ["image"], "input": 1, "implementationVersion": 1,
        }],
        "learnBlocks": [{
            "id": LEARN_ID, "type": "keras-transfer-image", "name": "Transfer learning",
            "title": "Transfer learning (Images)", "dsp": [DSP_ID],
        }],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Edge Impulse image classification training")
    ap.add_argument("--data", type=Path, default=HERE / "data" / "vision")
    ap.add_argument("--api-key", default=None, help="vision project key - must differ from the gesture one")
    ap.add_argument("--size", type=int, default=IMAGE_SIZE, help="input resolution (square)")
    ap.add_argument("--color", default="RGB", choices=["RGB", "Grayscale"])
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--learning-rate", type=float, default=0.0005)
    ap.add_argument("--skip-upload", action="store_true")
    ap.add_argument("--deploy", action="store_true")
    args = ap.parse_args()

    try:
        ei = EI(api_key=args.api_key)

        if not args.skip_upload:
            labels = collect_labels(args.data, (".jpg", ".jpeg", ".png", ".bmp"))
            log(f"[vision] labels: {', '.join(labels)}")
            for label, files in labels.items():
                upload(files, label, ei.key)

        ei.set_impulse(build_impulse(args.size, args.color))
        ei.set_dsp_config(DSP_ID, {"channels": args.color})
        ei.generate_features(DSP_ID)
        ei.train(LEARN_ID, {
            "mode": "visual",
            "trainingCycles": args.epochs,
            "learningRate": args.learning_rate,
            "trainTestSplit": 0.2,
            "augmentationPolicyImage": "all",
        })
        ei.metrics(LEARN_ID)

        if args.deploy:
            ei.deploy("arduino", out=HERE / "vision-arduino-lib.zip")

        log(f"\n[vision] done - {ei.url()}")
    except EIError as e:
        die(str(e))


if __name__ == "__main__":
    main()
