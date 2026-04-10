@echo off
setlocal enabledelayedexpansion

rem BAT script that creates the client python api of LibCarla (carla.org).
rem Run it through a cmd with the x64 Visual C++ Toolset enabled.

set LOCAL_PATH=%~dp0
set FILE_N=-[%~n0]:

call "%LOCAL_PATH%Bootstrap.bat"

rem Print batch params (debug purpose)
echo %FILE_N% [Batch params]: %*

rem ============================================================================
rem -- Parse arguments ---------------------------------------------------------
rem ============================================================================

set DOC_STRING=Build and package CARLA Python API.
set "USAGE_STRING=Usage: %FILE_N% [-h^|--help] [--build-wheel] [--rebuild]  [--clean]"

set REMOVE_INTERMEDIATE=false
set BUILD_PYTHONAPI=true
set INSTALL_PYTHONAPI=true

:arg-parse
if not "%1"=="" (
    if "%1"=="--rebuild" (
        set REMOVE_INTERMEDIATE=true
        set BUILD_PYTHONAPI=true
        set INSTALL_PYTHONAPI=true
    )

    if "%1"=="--build-wheel" (
        set BUILD_PYTHONAPI=true
        set INSTALL_PYTHONAPI=false
    )

    if "%1"=="--clean" (
        set REMOVE_INTERMEDIATE=true
        set BUILD_PYTHONAPI=false
        set INSTALL_PYTHONAPI=false
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

set PYTHON_LIB_PATH=%ROOT_PATH:/=\%PythonAPI\carla\

if %REMOVE_INTERMEDIATE% == false (
    if %BUILD_PYTHONAPI% == false (
        echo Nothing selected to be done.
        goto :eof
    )
)

if %REMOVE_INTERMEDIATE% == true (
    rem Remove directories
    for %%G in (
        "%PYTHON_LIB_PATH%build",
        "%PYTHON_LIB_PATH%dist",
        "%PYTHON_LIB_PATH%source\carla.egg-info"
    ) do (
        if exist %%G (
            echo %FILE_N% Cleaning %%G
            rmdir /s/q %%G
        )
    )
    if %BUILD_PYTHONAPI% == false (
        goto good_exit
    )
)

cd "%PYTHON_LIB_PATH%"
rem if exist "%PYTHON_LIB_PATH%dist" goto already_installed

rem ============================================================================
rem -- Check for py ------------------------------------------------------------
rem ============================================================================

if not defined CARLAAIR_PYTHON_EXE goto error_py
"%CARLAAIR_PYTHON_EXE%" -c "import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)" 1>nul 2>nul
if %errorlevel% neq 0 goto error_py310

echo Ensuring Python packaging dependencies are installed.
"%CARLAAIR_PYTHON_EXE%" -m pip install --disable-pip-version-check --upgrade pip setuptools wheel build
if %errorlevel% neq 0 goto error_pip_prereqs

rem Build for Python 3
rem
if %BUILD_PYTHONAPI%==true (
    echo Building Python API wheel for Python 3.
    "%CARLAAIR_PYTHON_EXE%" -m build --wheel --outdir dist\.tmp .
    if %errorlevel% neq 0 goto error_build_wheel

    set WHEEL_FILE=
    for %%f in (dist\.tmp\*.whl) do set WHEEL_FILE=%%f
    if not defined WHEEL_FILE goto error_missing_wheel

    if %INSTALL_PYTHONAPI%==true (
        "%CARLAAIR_PYTHON_EXE%" -m pip install --force-reinstall "!WHEEL_FILE!"
        if %errorlevel% neq 0 goto error_install_wheel
    )

    copy "!WHEEL_FILE!" dist
    if %errorlevel% neq 0 goto error_copy_wheel
    if exist dist\.tmp rmdir /s /q dist\.tmp

)

goto success

rem ============================================================================
rem -- Messages and Errors -----------------------------------------------------
rem ============================================================================

:success
    echo.
    if %BUILD_PYTHONAPI%==true echo %FILE_N% Carla lib for python has been successfully installed in "%PYTHON_LIB_PATH%dist"!
    goto good_exit

:already_installed
    echo.
    echo %FILE_N% [ERROR] Already installed in "%PYTHON_LIB_PATH%dist"
    goto good_exit

:error_py
    echo.
    echo %FILE_N% [ERROR] A usable Python executable was not found.
    echo %FILE_N% [ERROR] Possible causes:
    echo %FILE_N% [ERROR]  - Set CARLAAIR_PYTHON_EXE to a Python interpreter.
    echo %FILE_N% [ERROR]  - Install Python 3.10 and ensure either python.exe or py.exe is available.
    echo %FILE_N% [ERROR]  - Prefer running BuildWindows.ps1, which resolves the interpreter automatically.
    goto bad_exit

:error_py310
    echo.
    echo %FILE_N% [ERROR] Python 3.10 is required to build the CarlaAir wheel.
    echo %FILE_N% [ERROR] Set CARLAAIR_PYTHON_EXE to a Python 3.10 interpreter or run BuildWindows.ps1.
    goto bad_exit

:error_pip_prereqs
    echo.
    echo %FILE_N% [ERROR] Failed to install Python packaging prerequisites (pip/setuptools/wheel/build).
    goto bad_exit

:error_build_wheel
    echo.
    echo %FILE_N% [ERROR] An error occurred while building the wheel file.
    goto bad_exit

:error_missing_wheel
    echo.
    echo %FILE_N% [ERROR] Wheel build finished without producing a .whl file in dist\.tmp.
    goto bad_exit

:error_install_wheel
    echo.
    echo %FILE_N% [ERROR] Failed to install the generated wheel into the selected Python environment.
    goto bad_exit

:error_copy_wheel
    echo.
    echo %FILE_N% [ERROR] Failed to copy the generated wheel into the dist folder.
    goto bad_exit

:good_exit
    endlocal
    exit /b 0

:bad_exit
    endlocal
    exit /b 1

