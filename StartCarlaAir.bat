@echo off
setlocal
powershell -ExecutionPolicy Bypass -File "%~dp0CarlaAir.ps1" %*
exit /b %errorlevel%
