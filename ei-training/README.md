# Edge Impulse training kit

Training and inference only. Gesture and image classification are trained
separately, each launched from a `.bat` you can double-click.

## On a new PC

Install these first, both with "Add to PATH" enabled:

- [Python 3](https://www.python.org/downloads/)
- [arduino-cli](https://arduino.github.io/arduino-cli/latest/installation/) (only needed to flash the board)

Then run once:

    setup.bat

It installs `pyserial` + `requests`, the ESP32 board package, and the Adafruit
BNO055 libraries, then checks whether a board is plugged in.

## Gesture model - idle / shaking / spin

| step | command | what it does |
|---|---|---|
| 1 | `flash_gesture.bat` | burns the collection sketch onto the XIAO |
| 2 | `collect_gesture.bat` | records 100 Hz IMU windows, uploads them |
| 3 | `train_gesture.bat` | builds the impulse and trains |

In the collector, pick `1` idle / `2` shaking / `3` spin, then a duration.
Do **only** that motion for the whole recording. Aim for 10+ recordings of 10s
per class, keeping the wrist orientation and the hand the same each time.

`idle` matters as much as the other two - without it, any movement at all
classifies as a mission success.

## Image model - background / cup / mouse / phonecase

    train_vision.bat        train
    install_model.bat       install the new library, patch the SDK, rebuild
    flash_vision_infer.bat  flash the live inference viewer

`install_model.bat` must run after every retrain. A fresh Edge Impulse library
ships the SDK's own porting header, which caps `EI_MAX_OVERFLOW_BUFFER_COUNT`
at 30 on the ESP32-S3 and makes `run_classifier` crash the board with
`Guru Meditation Error`. `patch_ei_sdk.py` raises it to 2048, and the rebuild
uses `--clean` because arduino-cli caches objects per library name/version.
See `day3/.agents/skills/xiao-edgeimpulse-train/SKILL.md` step 5.

**The `background` class carries its weight.** Trained on the three object
classes alone, int8 accuracy sat at 0.67 while float32 reported 1.0 - the model
had memorised the shared backdrop and quantisation destroyed the margin it was
relying on. Adding 80 background images took int8 to 0.95. Keep the class
counts comparable; if `background` outnumbers the objects it becomes the prior
and swallows every ambiguous frame.

Photos are collected in the Edge Impulse studio (phone or webcam), the same way
as before. To train from a local folder instead, drop files into
`data/vision/<label>/` and run `train_vision.bat --reupload`.

**This needs its own Edge Impulse project.** One project holds one impulse, and
time-series and image data do not mix. Create a second project in the studio,
then before running:

    set EI_API_KEY=ei_your_vision_project_key

## Ports

The board is found by USB vendor ID (Espressif, `0x303A`), so the COM number
does not need to match anyone else's machine. To force one:

    set IMU_PORT=COM7

**Only one program can hold the serial port.** Close the serial monitor before
recording, or the port open fails with `PermissionError 13`.

## Layout

    gesture_collect/    Arduino sketch: streams ax,ay,az at 100 Hz
    collect_gesture.py  serial -> data/gesture/<label>/*.json -> Edge Impulse
    train_gesture.py    Spectral Analysis + Classification, 100 Hz / 2000 ms
    train_vision.py     Image DSP + Transfer Learning, 96x96 RGB
    port.py             USB vendor-ID board detection
    ei.py               Edge Impulse Studio API client

Window settings (100 Hz, 2000 ms = 200 samples) match `RAW` in the day3
inference sketch. Change one and you have to change the other.
