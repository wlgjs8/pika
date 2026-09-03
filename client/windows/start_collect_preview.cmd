@echo off
setlocal
chcp 65001 >nul
if not exist "%~dp0.env" (
  echo Missing %~dp0.env
  echo Run setup_ssh_key.cmd first.
  pause
  exit /b 1
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_collect_preview.ps1" -EnvFile "%~dp0.env"
if errorlevel 1 pause
