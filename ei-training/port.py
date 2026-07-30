#!/usr/bin/env python3
"""Find the XIAO ESP32S3 serial port, so this works on anyone's machine.

The board is not always on COM6 - it depends on which USB slot it lands in and
what else is plugged into that PC. Espressif's USB vendor ID is 0x303A, which
is what actually identifies the board; the port name is not.

    python port.py          print the detected port, or exit 1 if none
    IMU_PORT=COM7           an explicit override always wins

Used both by collect_gesture.py and by the .bat launchers.
"""
import os
import sys

from serial.tools import list_ports

ESPRESSIF_VID = 0x303A


def detect(explicit: str | None = None) -> str | None:
    """Explicit override, then Espressif VID, then a lone USB serial port."""
    override = explicit or os.environ.get("IMU_PORT")
    if override:
        return override

    ports = list(list_ports.comports())
    espressif = [p.device for p in ports if p.vid == ESPRESSIF_VID]
    if len(espressif) == 1:
        return espressif[0]
    if len(espressif) > 1:
        # More than one board plugged in - we cannot guess which is the wearable.
        print("multiple ESP32 boards found: %s\n"
              "  pick one with:  set IMU_PORT=%s"
              % (", ".join(espressif), espressif[0]), file=sys.stderr)
        return None

    # No Espressif VID. Some clone bridges (CH340/CP210x) report their own, so
    # fall back only when the choice is unambiguous.
    usb = [p.device for p in ports if p.vid is not None]
    if len(usb) == 1:
        return usb[0]

    if usb:
        print("no ESP32 board found. USB serial ports present: %s\n"
              "  set IMU_PORT to the right one." % ", ".join(usb), file=sys.stderr)
    else:
        print("no USB serial ports found - is the board plugged in?", file=sys.stderr)
    return None


def main() -> None:
    p = detect()
    if not p:
        sys.exit(1)
    print(p)


if __name__ == "__main__":
    main()
