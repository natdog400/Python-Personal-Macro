@echo off
setlocal
set SCRIPT_DIR=%~dp0

REM Prefer a venv Python if present, fallback to system Python
IF EXIST "%SCRIPT_DIR%venv\Scripts\python.exe" (
  "%SCRIPT_DIR%venv\Scripts\python.exe" "%SCRIPT_DIR%launcher.py"
) ELSE (
  python "%SCRIPT_DIR%launcher.py"
)

endlocal