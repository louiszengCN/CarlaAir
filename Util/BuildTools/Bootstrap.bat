@echo off
setlocal

if not defined ROOT_PATH (
    for %%I in ("%~dp0..\..") do set "ROOT_PATH=%%~fI"
)
if not "%ROOT_PATH:~-1%"=="\" set "ROOT_PATH=%ROOT_PATH%\"

if not defined INSTALLATION_DIR (
    set "INSTALLATION_DIR=%ROOT_PATH%Build\"
)
if not "%INSTALLATION_DIR:~-1%"=="\" set "INSTALLATION_DIR=%INSTALLATION_DIR%\"

if not defined CARLAAIR_PYTHON_EXE (
    for /f "usebackq delims=" %%I in (`py -3.10 -c "import sys; print(sys.executable)" 2^>nul`) do (
        if not defined CARLAAIR_PYTHON_EXE set "CARLAAIR_PYTHON_EXE=%%I"
    )
)

if not defined CARLAAIR_PYTHON_EXE (
    for /f "usebackq delims=" %%I in (`py -3 -c "import sys; print(sys.executable)" 2^>nul`) do (
        if not defined CARLAAIR_PYTHON_EXE set "CARLAAIR_PYTHON_EXE=%%I"
    )
)

if not defined CARLAAIR_PYTHON_EXE (
    for %%I in (python.exe) do (
        if not defined CARLAAIR_PYTHON_EXE if not "%%~$PATH:I"=="" set "CARLAAIR_PYTHON_EXE=%%~$PATH:I"
    )
)

if not defined CARLAAIR_PYTHON_EXE (
    for /f "usebackq delims=" %%I in (`py -c "import sys; print(sys.executable)" 2^>nul`) do (
        if not defined CARLAAIR_PYTHON_EXE set "CARLAAIR_PYTHON_EXE=%%I"
    )
)

endlocal & (
    set "ROOT_PATH=%ROOT_PATH%"
    set "INSTALLATION_DIR=%INSTALLATION_DIR%"
    if defined CARLAAIR_PYTHON_EXE set "CARLAAIR_PYTHON_EXE=%CARLAAIR_PYTHON_EXE%"
)
exit /b 0
