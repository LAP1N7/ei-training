@echo off
cd /d "%~dp0"
title flash vision_infer (live inference viewer) -> XIAO ESP32S3 Sense

rem Burns the live inference viewer onto the board (day3 03_web_infer_sta,
rem repointed at the projectbee network and at the team_project model).
rem The board serves a camera page with the prediction drawn over the image:
rem framed box, top label, and a confidence bar per class.
rem
rem This OVERWRITES the collection dashboard. Reflash flash_vision_collect.bat
rem to go back to capturing images.
rem
rem The model library must be installed first:
rem   arduino-cli config set library.enable_unsafe_install true
rem   arduino-cli lib install --zip-path vision-arduino-lib.zip
rem train_vision.bat regenerates that zip whenever you retrain.

set FQBN=esp32:esp32:XIAO_ESP32S3:PSRAM=opi

if "%IMU_PORT%"=="" (
  for /f "delims=" %%p in ('python port.py 2^>nul') do set IMU_PORT=%%p
)
if "%IMU_PORT%"=="" (
  echo.
  echo [X] No board detected. Plug the XIAO in over USB, or name the port:
  echo     set IMU_PORT=COM9 ^&^& flash_vision_infer.bat
  goto done
)

echo === compile (%FQBN%) ===
arduino-cli compile --fqbn %FQBN% vision_infer
if errorlevel 1 goto fail

echo.
echo === upload to %IMU_PORT% ===
arduino-cli upload -p %IMU_PORT% --fqbn %FQBN% vision_infer
if errorlevel 1 goto fail

echo.
echo OK - the board prints its address on serial at boot. Give it about
echo 15 seconds after reset before the page answers, then open that address
echo or http://xiao.local
goto done

:fail
echo.
echo FAILED. Check the board is on %IMU_PORT% and no serial monitor holds it.
echo If team_project_inferencing.h is missing, install the library (see above).

:done
echo.
pause
