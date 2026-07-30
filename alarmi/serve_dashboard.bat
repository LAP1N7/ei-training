@echo off
cd /d "%~dp0dashboard"
title alarmi dashboard

rem Serve the dashboard over http.
rem
rem A page opened with file:// cannot open a WebSocket, so the dashboard cannot
rem reach the broker that way - it must be served over http. Same on a phone.
rem
rem The broker itself is separate: run mosquitto with dashboard\mosquitto-websockets.conf
rem (needs 1883 for the boards and 9001 for this page).

echo.
echo   Your addresses:
for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /c:"IPv4"') do echo     http://%%a:8000
echo.
echo   In the page, point the broker at the same IP on port 9001, then click connect.
echo   Ctrl+C to stop.
echo.

python -m http.server 8000 --bind 0.0.0.0
