@echo off
REM 대시보드를 LAN 에 공개한다. 같은 Wi-Fi 의 폰/태블릿에서 아래 주소로 접속:
REM     http://192.168.0.27:8000
REM
REM 브라우저는 file:// 로 연 페이지에서 WebSocket 을 막기 때문에,
REM 폰에서 보려면 반드시 이렇게 http 로 서빙해야 한다.
cd /d "%~dp0dashboard"
echo.
echo   대시보드:  http://192.168.0.27:8000
echo   (브로커는 192.168.0.27:9001 - 페이지에서 그대로 두고 [연결] 클릭)
echo.
echo   Ctrl+C 로 종료
echo.
python -m http.server 8000 --bind 0.0.0.0
