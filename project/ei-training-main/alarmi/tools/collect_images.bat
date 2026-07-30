@echo off
chcp 65001 >nul
cd /d "%~dp0.."
title vision image collector
python tools\img_collect.py
echo.
echo (press any key to close)
pause >nul
