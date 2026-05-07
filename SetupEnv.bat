@echo off
setlocal
powershell -ExecutionPolicy Bypass -File "%~dp0env_setup\SetupEnv.ps1" %*
exit /b %errorlevel%
