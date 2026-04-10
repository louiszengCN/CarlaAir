@echo off
setlocal

set "MAVLINKCOM_ROOT=..\MavLinkCom"
set "INCLUDE_SRC=%MAVLINKCOM_ROOT%\SDK\Includes"
set "LIB_SRC=%MAVLINKCOM_ROOT%\SDK\Libraries"

if exist "%MAVLINKCOM_ROOT%\include" set "INCLUDE_SRC=%MAVLINKCOM_ROOT%\include"
if exist "%MAVLINKCOM_ROOT%\lib" set "LIB_SRC=%MAVLINKCOM_ROOT%\lib"

if not exist "%INCLUDE_SRC%" (
    echo [update_mavlibkcom] Include source not found: %INCLUDE_SRC%
    exit /b 1
)

if not exist "%LIB_SRC%" (
    echo [update_mavlibkcom] Library source not found: %LIB_SRC%
    exit /b 1
)

robocopy /MIR "%INCLUDE_SRC%" deps\MavLinkCom\include /XD temp *. /njh /njs /ndl /np
if %errorlevel% geq 8 exit /b %errorlevel%

robocopy /MIR "%LIB_SRC%" deps\MavLinkCom\lib /XD temp *. /njh /njs /ndl /np
if %errorlevel% geq 8 exit /b %errorlevel%

exit /b 0
