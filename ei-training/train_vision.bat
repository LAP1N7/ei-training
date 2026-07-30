@echo off
cd /d "%~dp0"
title Edge Impulse - vision training

rem Image classification training (mouse / papercup / phonecase).
rem
rem IMPORTANT: this cannot share a project with the gesture model - one Edge
rem Impulse project holds one impulse, and time-series and image data do not
rem mix. Create a second project in the studio and set its key here:
rem   set EI_API_KEY=ei_...vision_project_key...
rem
rem Photos collected on the web (studio/phone) are already uploaded, so:
rem   train_vision.bat              train only
rem   train_vision.bat --reupload   also upload data/vision/<label>/*.jpg

set ARGS=--skip-upload
if /i "%~1"=="--reupload" set ARGS=

if "%EI_API_KEY%"=="" (
  echo.
  echo [!] EI_API_KEY is empty - falling back to the gesture project key.
  echo     Images need their own project. See README.
  echo.
)

python train_vision.py %ARGS%

echo.
pause
