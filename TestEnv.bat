@echo off
setlocal
powershell -ExecutionPolicy Bypass -File "%~dp0env_setup\TestEnv.ps1" %*
exit /b %errorlevel%
