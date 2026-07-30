@echo off
cd /d "%~dp0"
title Edge Impulse - gesture training

rem Configure the impulse and train, using data collect_gesture already uploaded.
rem That data is on the server, so --skip-upload is the default here.
rem To re-upload everything under data/gesture:  train_gesture.bat --reupload

set ARGS=--skip-upload
if /i "%~1"=="--reupload" set ARGS=

python train_gesture.py %ARGS%

echo.
pause
