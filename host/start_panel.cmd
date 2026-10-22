@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -STA -File "%~dp0vision_panel_cn.ps1" %*
set "panel_exit=%errorlevel%"
if not "%panel_exit%"=="0" if "%~1"=="" pause
exit /b %panel_exit%
