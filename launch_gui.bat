@echo off

:: Check if Python is installed
where python >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Python is not installed or not in PATH.
    echo Please install Python 3.8 or later from https://www.python.org/downloads/
    echo Make sure to check "Add Python to PATH" during installation.
    rem Optionally, uncomment the next lines to auto-download and install Python silently (requires admin):
    rem powershell -NoProfile -Command "Write-Host 'Downloading Python installer...'; $url='https://www.python.org/ftp/python/3.11.6/python-3.11.6-amd64.exe'; $out='python-installer.exe'; Invoke-WebRequest -Uri $url -OutFile $out; Write-Host 'Running installer...'; Start-Process -FilePath $out -ArgumentList '/quiet InstallAllUsers=1 PrependPath=1' -Wait; Remove-Item $out -Force"
    pause
    exit /b 1
)

:: Check Python version (requires Python 3.8+)
python -c "import sys; exit(0) if sys.version_info >= (3, 8) else exit(1)" >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Python 3.8 or later is required. Please update your Python installation.
    pause
    exit /b 1
)

echo Checking for required packages...

:: Check if pip is available
python -m pip --version >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo pip is not available. Attempting to install pip...
    python -m ensurepip --default-pip
    if %ERRORLEVEL% NEQ 0 (
        echo Failed to install pip. Please install pip manually.
        pause
        exit /b 1
    )
)

:: Install/upgrade pip and setuptools
echo Updating pip and setuptools...
python -m pip install --upgrade pip setuptools wheel
if %ERRORLEVEL% NEQ 0 (
    echo Failed to update pip and setuptools.
    pause
    exit /b 1
)

:: Install required packages
echo Installing required packages...
python -m pip install -r requirements.txt
if %ERRORLEVEL% NEQ 0 (
    echo Failed to install required packages.
    pause
    exit /b 1
)

:: Ensure opencv-contrib-python is available for SIFT support (feature strategy)
python -c "import cv2, sys; sys.exit(0 if hasattr(cv2, 'SIFT_create') else 1)" >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Installing opencv-contrib-python for SIFT support...
    python -m pip install --upgrade opencv-contrib-python
    python -c "import cv2, sys; sys.exit(0 if hasattr(cv2, 'SIFT_create') else 1)" >nul 2>nul
    if %ERRORLEVEL% NEQ 0 (
        echo SIFT not available. Some feature detection fallbacks may be disabled.
        echo You can manually install: python -m pip install opencv-contrib-python
    )
)

:: Verify critical Python imports before launching
echo Verifying Python imports...
python -c "import cv2, pyautogui, PIL, PyQt6, numpy; print('Imports OK')" >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo One or more Python packages failed to import. Please check your Python installation.
    echo Try: python -m pip install --upgrade -r requirements.txt
    pause
    exit /b 1
)

:: Additional environment checks
:: Ensure images and subfolders are writable
if not exist images\temp (
    mkdir images\temp >nul 2>nul
)
> images\temp\write_test.tmp echo test >nul 2>nul
if not exist images\temp\write_test.tmp (
    echo Cannot write to images\temp. Please check folder permissions.
    pause
    exit /b 1
)
del /q images\temp\write_test.tmp >nul 2>nul

:: Basic repo checks
if not exist bot_gui.py (
    echo Missing bot_gui.py. Please ensure you are running this launcher in the project folder.
    pause
    exit /b 1
)

:: Ensure expected folders exist
if not exist images (
    echo Creating images directory...
    mkdir images
)
if not exist failsafe_images (
    echo Creating failsafe_images directory...
    mkdir failsafe_images
)

:: Ensure a minimal config.json exists and is valid
if not exist config.json (
    echo Creating default config.json...
    > config.json echo {
    >> config.json echo   "templates": {},
    >> config.json echo   "sequences": [],
    >> config.json echo   "failsafe": {"enabled": false}
    >> config.json echo }
)
python -c "import json; json.load(open('config.json')); print('Config OK')" >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo config.json is invalid JSON. Please fix or delete it and rerun to regenerate.
    pause
    exit /b 1
)

:: Touch log file if missing
if not exist bot_debug.log (
    type nul >> bot_debug.log
)

echo All requirements are satisfied. Starting application...
python bot_gui.py %*

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo An error occurred while running the application.
    pause
    exit /b 1
)
