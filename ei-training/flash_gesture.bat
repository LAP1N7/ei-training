@echo off
cd /d "%~dp0"
title flash gesture_collect -> XIAO ESP32S3

rem Build and flash the collection sketch. Run this before collect_gesture.bat.
rem The port is detected by USB vendor ID, so it works on any PC. To force one:
rem   set IMU_PORT=COM7 && flash_gesture.bat

set FQBN=esp32:esp32:XIAO_ESP32S3

if "%IMU_PORT%"=="" (
  for /f "delims=" %%p in ('python port.py 2^>nul') do set IMU_PORT=%%p
)
if "%IMU_PORT%"=="" (
  echo.
  echo [X] No board detected. Plug the XIAO in over USB, or name the port:
  echo     set IMU_PORT=COM7 ^&^& flash_gesture.bat
  goto done
)

echo === compile (%FQBN%) ===
arduino-cli compile --fqbn %FQBN% gesture_collect
if errorlevel 1 goto fail

echo.
echo === upload to %IMU_PORT% ===
arduino-cli upload -p %IMU_PORT% --fqbn %FQBN% gesture_collect
if errorlevel 1 goto fail

echo.
echo OK - now run collect_gesture.bat to record data.
goto done

:fail
echo.
echo FAILED. Check that the board is on %IMU_PORT% and that no other program
echo holds the port (close any serial monitor or collector window).
echo If the board package or libraries are missing, run setup.bat first.

:done
echo.
pause
