@echo off
setlocal
if not exist "%~dp0.env" (
  copy /Y "%~dp0.env.example" "%~dp0.env" >nul
  echo Edit the connection settings, save, and close Notepad.
  start "" /wait notepad.exe "%~dp0.env"
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup_ssh_key.ps1"
if errorlevel 1 pause
