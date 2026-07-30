@echo off
cd /d "%~dp0"
title gesture collector (idle / shaking / spin)

rem Record 100Hz IMU windows from the board and upload them to Edge Impulse.
rem Default port is COM6. To override:  set IMU_PORT=COM7 && collect_gesture.bat
rem
rem NOTE: only one program can hold the serial port. Close any serial monitor
rem before recording, or the port open fails with PermissionError 13.

python collect_gesture.py %*

echo.
pause
