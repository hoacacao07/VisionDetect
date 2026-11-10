@echo off
setlocal
rem Always use this project's Python, even when launched from another folder.
cd /d "%~dp0"
if not exist "%~dp0.venv\Scripts\python.exe" (
  echo Project Python is missing. Open docs\START_HERE.md for setup help.
  if "%~1"=="" pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0tools\start_webots_simulation.py" --moving --stress %*
set "launch_exit_code=%errorlevel%"
rem Keep a double-clicked error visible; commands with arguments return directly.
if not "%launch_exit_code%"=="0" if "%~1"=="" pause
exit /b %launch_exit_code%
