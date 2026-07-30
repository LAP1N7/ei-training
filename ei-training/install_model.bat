@echo off
cd /d "%~dp0"
title install trained model -> patch SDK -> build vision_infer

rem Run this after train_vision.bat produces a new vision-arduino-lib.zip.
rem
rem The patch step is not optional. A fresh library ships the SDK's own
rem porting header, which caps EI_MAX_OVERFLOW_BUFFER_COUNT at 30 on the
rem ESP32-S3 and makes run_classifier crash the board. patch_ei_sdk.py raises
rem it to 2048. Skipping this is exactly the bug that cost a whole afternoon.
rem
rem --clean is also not optional: arduino-cli caches objects per library
rem name/version, so an edited header would otherwise be ignored entirely.

set FQBN=esp32:esp32:XIAO_ESP32S3:PSRAM=opi

if not exist vision-arduino-lib.zip (
  echo [X] vision-arduino-lib.zip not found. Run train_vision.bat first.
  goto done
)

echo === 1/4 allow zip installs ===
arduino-cli config set library.enable_unsafe_install true

echo.
echo === 2/4 reinstall the model library ===
arduino-cli lib uninstall team_project_inferencing
arduino-cli lib install --zip-path vision-arduino-lib.zip
if errorlevel 1 goto fail

echo.
echo === 3/4 re-apply the ESP32-S3 overflow-buffer patch ===
python patch_ei_sdk.py
if errorlevel 1 goto fail

echo.
echo === 4/4 clean rebuild ===
arduino-cli compile --clean --fqbn %FQBN% vision_infer
if errorlevel 1 goto fail

echo.
echo OK - now flash it with flash_vision_infer.bat
goto done

:fail
echo.
echo FAILED - see the error above.

:done
echo.
pause
