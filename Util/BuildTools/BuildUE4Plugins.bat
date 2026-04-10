@REM @echo off
setlocal enabledelayedexpansion

rem Run it through a cmd with the x64 Visual C++ Toolset enabled.

set LOCAL_PATH=%~dp0
set FILE_N=-[%~n0]:

call "%LOCAL_PATH%Bootstrap.bat"

rem Print batch params (debug purpose)
echo %FILE_N% [Batch params]: %*

rem ============================================================================
rem -- Parse arguments ---------------------------------------------------------
rem ============================================================================

set DOC_STRING=Build LibCarla.
set USAGE_STRING=Usage: %FILE_N% [-h^|--help] [--rebuild] [--build] [--clean] [--no-pull]

set BUILD_STREETMAP=false
set GIT_PULL=true
set CURRENT_STREETMAP_COMMIT=260273d6b7c3f28988cda31fd33441de7e272958
set STREETMAP_BRANCH=master
set STREETMAP_REPO=https://github.com/carla-simulator/StreetMap.git
set STREETMAP_MARKER=StreetMap.uplugin

:arg-parse
if not "%1"=="" (
    if "%1"=="--rebuild" (
        set REMOVE_INTERMEDIATE=true
        set BUILD_STREETMAP=true
    )
    if "%1"=="--build" (
        set BUILD_STREETMAP=true
    )
    if "%1"=="--no-pull" (
        set GIT_PULL=false
    )
    if "%1"=="--clean" (
        set REMOVE_INTERMEDIATE=true
    )
    if "%1"=="-h" (
        echo %DOC_STRING%
        echo %USAGE_STRING%
        GOTO :eof
    )
    if "%1"=="--help" (
        echo %DOC_STRING%
        echo %USAGE_STRING%
        GOTO :eof
    )
    shift
    goto :arg-parse
)

rem ============================================================================
rem -- Local Variables ---------------------------------------------------------
rem ============================================================================

rem Set the visual studio solution directory
rem
set CARLA_PLUGINS_PATH=%ROOT_PATH:/=\%Unreal\CarlaUE4\Plugins\
set CARLA_STREETMAP_PLUGINS_PATH=%ROOT_PATH:/=\%Unreal\CarlaUE4\Plugins\StreetMap\

rem Build STREETMAP

if "%BUILD_STREETMAP%" == "true" (
    if not exist "%CARLA_STREETMAP_PLUGINS_PATH%\%STREETMAP_MARKER%" (
        if exist "%CARLA_STREETMAP_PLUGINS_PATH%" (
            rmdir /s /q "%CARLA_STREETMAP_PLUGINS_PATH%"
        )
        git clone -b %STREETMAP_BRANCH% %STREETMAP_REPO% %CARLA_STREETMAP_PLUGINS_PATH%
        if errorlevel 1 goto error_clone
    )

    if %GIT_PULL% == true if exist "%CARLA_STREETMAP_PLUGINS_PATH%\.git" (
        cd "%CARLA_STREETMAP_PLUGINS_PATH%"
        git fetch
        if errorlevel 1 echo %FILE_N% [WARNING] git fetch failed, using existing StreetMap sources.
        git checkout %CURRENT_STREETMAP_COMMIT%
        if errorlevel 1 echo %FILE_N% [WARNING] git checkout %CURRENT_STREETMAP_COMMIT% failed, using existing StreetMap sources.
    )
)


goto success

rem ============================================================================
rem -- Messages and Errors -----------------------------------------------------
rem ============================================================================

:success
    if %BUILD_STREETMAP% == true echo %FILE_N% STREETMAP has been successfully installed in "%CARLA_PLUGINS_PATH%"!
    goto good_exit

:error_clone
    echo.
    echo %FILE_N% [ERROR] Failed to clone StreetMap plugin sources from %STREETMAP_REPO%.
    goto bad_exit

:good_exit
    endlocal
    exit /b 0

:bad_exit
    endlocal
    exit /b 1
