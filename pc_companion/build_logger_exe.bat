@echo off
REM Build RobotellCanLogger.exe (run from CMD, not PowerShell)
setlocal
cd /d "%~dp0"

echo === Creating venv (if needed) ===
if not exist ".venv\Scripts\python.exe" (
  python -m venv .venv
  if errorlevel 1 (
    echo ERROR: python not found. Install Python 3 from python.org and tick "Add to PATH".
    pause
    exit /b 1
  )
)

echo === Installing dependencies ===
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 (
  echo ERROR: pip install failed
  pause
  exit /b 1
)

echo === PyInstaller ===
".venv\Scripts\pyinstaller.exe" --noconfirm --onefile --windowed --name RobotellCanLogger --hidden-import robotell_can --hidden-import can_log --hidden-import serial.tools.list_ports can_logger_app.py
if errorlevel 1 (
  echo ERROR: PyInstaller failed
  pause
  exit /b 1
)

echo.
echo OK — EXE ready:
echo   %cd%\dist\RobotellCanLogger.exe
echo.
explorer "%cd%\dist"
pause
