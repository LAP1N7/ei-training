@echo off
cd /d "%~dp0"
title flash vision_collect (camera dataset dashboard) -> XIAO ESP32S3 Sense

rem Burns the camera collection dashboard onto the board (day3 02_web_collect_sta,
rem repointed at the projectbee network). The board then serves a Teachable
rem Machine style page: type a label, capture, and each shot downloads to the
rem browser as "<label>.<n>.jpg" - ready to upload to Edge Impulse.
rem
rem This OVERWRITES whatever is on the board. Reflash 21_vision_node_jpg
rem afterwards to go back to live inference.
rem
rem Labels to collect: mouse, papercup, phonecase

set FQBN=esp32:esp32:XIAO_ESP32S3:PSRAM=opi

if "%IMU_PORT%"=="" (
  for /f "delims=" %%p in ('python port.py 2^>nul') do set IMU_PORT=%%p
)
if "%IMU_PORT%"=="" (
  echo.
  echo [X] No board detected. Plug the XIAO in over USB, or name the port:
  echo     set IMU_PORT=COM9 ^&^& flash_vision_collect.bat
  goto done
)

echo === compile (%FQBN%) ===
arduino-cli compile --fqbn %FQBN% vision_collect
if errorlevel 1 goto fail

echo.
echo === upload to %IMU_PORT% ===
arduino-cli upload -p %IMU_PORT% --fqbn %FQBN% vision_collect
if errorlevel 1 goto fail

echo.
echo OK - the board prints its address on serial. Open that in a browser,
echo or try http://xiao.local
echo Run open_vision_dashboard.bat to find it automatically.
goto done

:fail
echo.
echo FAILED. Check the board is on %IMU_PORT% and no serial monitor holds it.

:done
echo.
pause
