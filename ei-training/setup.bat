@echo off
cd /d "%~dp0"
title setup - install everything this kit needs

rem Run this ONCE on a new PC. It installs the Python packages, the ESP32 board
rem support, and the Arduino libraries the collection sketch needs.
rem
rem Not installed here (install manually if missing):
rem   - Python 3      https://www.python.org/downloads/   (tick "Add to PATH")
rem   - arduino-cli   https://arduino.github.io/arduino-cli/latest/installation/

echo ============================================================
echo  1/4  checking python
echo ============================================================
python --version
if errorlevel 1 (
  echo.
  echo [X] python not found on PATH.
  echo     Install Python 3 and tick "Add python.exe to PATH", then rerun.
  goto fail
)

echo.
echo ============================================================
echo  2/4  python packages
echo ============================================================
python -m pip install --quiet --upgrade pyserial requests
if errorlevel 1 goto fail
echo     pyserial, requests OK

echo.
echo ============================================================
echo  3/4  checking arduino-cli
echo ============================================================
arduino-cli version
if errorlevel 1 (
  echo.
  echo [X] arduino-cli not found on PATH.
  echo     https://arduino.github.io/arduino-cli/latest/installation/
  echo     You can still collect and train without it - you just cannot
  echo     flash the board from here.
  goto skipboard
)

echo.
echo --- ESP32 board package (this one is a big download, be patient) ---
arduino-cli config init --overwrite >nul 2>&1
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
if errorlevel 1 goto fail

echo.
echo --- Arduino libraries for the BNO055 IMU ---
arduino-cli lib install "Adafruit BNO055"
arduino-cli lib install "Adafruit Unified Sensor"
if errorlevel 1 goto fail

:skipboard
echo.
echo ============================================================
echo  4/4  looking for a board
echo ============================================================
python port.py
if errorlevel 1 (
  echo     No board detected right now - that is fine, plug it in later.
) else (
  echo     Board found.
)

echo.
echo ============================================================
echo  SETUP COMPLETE
echo ============================================================
echo.
echo  Next:
echo    1. flash_gesture.bat     burn the collection sketch onto the board
echo    2. collect_gesture.bat   record idle / shaking / spin
echo    3. train_gesture.bat     train the model on Edge Impulse
echo.
echo  Using your own Edge Impulse project instead of the shared one?
echo    set EI_API_KEY=ei_your_key_here
echo.
goto done

:fail
echo.
echo SETUP FAILED - see the error above.

:done
echo.
pause
