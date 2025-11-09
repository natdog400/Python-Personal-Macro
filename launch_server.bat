@echo off
REM Launch the remote control web server
setlocal
cd /d "%~dp0"

REM Optional: override bind/port/token via env vars
REM set WEB_BIND=0.0.0.0
REM set WEB_PORT=8765
REM set WEB_TOKEN=CHANGE_ME_TOKEN

echo Starting web server...
python web_server.py
endlocal