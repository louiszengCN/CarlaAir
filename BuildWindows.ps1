param(
    [switch]$Full,
    [switch]$Setup,
    [switch]$BuildLibCarla,
    [switch]$BuildOSM2ODR,
    [switch]$BuildPythonApi,
    [switch]$BuildUE4,
    [switch]$Package,
    [string]$Configuration = "Shipping",
    [string]$Generator = "Visual Studio 17 2022",
    [string]$UE4Root,
    [string]$PythonExe,
    [string]$InstallationDir
)

$ErrorActionPreference = "Stop"

function Write-Phase {
    param([string]$Message)
    Write-Host ""
    Write-Host "== $Message =="
}

function Resolve-VsDevCmd {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        throw "vswhere.exe not found."
    }

    $installPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $installPath) {
        throw "Visual Studio with Desktop C++ tools was not found."
    }

    $devCmd = Join-Path $installPath "Common7\Tools\VsDevCmd.bat"
    if (-not (Test-Path $devCmd)) {
        throw "VsDevCmd.bat not found under $installPath."
    }

    return $devCmd
}

function Resolve-WindowsSdkBin {
    $sdkRoots = New-Object System.Collections.Generic.List[string]

    try {
        $installedRoots = Get-ItemProperty "HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots" -ErrorAction Stop
        if ($installedRoots.KitsRoot10) {
            $sdkRoots.Add($installedRoots.KitsRoot10)
        }
    } catch {
    }

    $sdkRoots.Add((Join-Path ${env:ProgramFiles(x86)} "Windows Kits\10"))

    foreach ($sdkRoot in $sdkRoots | Where-Object { $_ } | Select-Object -Unique) {
        $sdkBinRoot = Join-Path $sdkRoot "bin"
        if (-not (Test-Path $sdkBinRoot)) {
            continue
        }

        $versionDirectories = Get-ChildItem -Path $sdkBinRoot -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '^\d+\.\d+\.\d+\.\d+$' } |
            Sort-Object { [version]$_.Name } -Descending

        foreach ($versionDirectory in $versionDirectories) {
            $x64Bin = Join-Path $versionDirectory.FullName "x64"
            if (Test-Path (Join-Path $x64Bin "mt.exe")) {
                return $x64Bin
            }
        }
    }

    throw "Windows SDK x64 mt.exe was not found. Install the Windows 10/11 SDK component."
}

function Resolve-UE4Root {
    param([string]$ExplicitRoot)

    $candidates = @()
    if ($ExplicitRoot) { $candidates += $ExplicitRoot }
    if ($env:UE4_ROOT) { $candidates += $env:UE4_ROOT }

    try {
        $reg = Get-ItemProperty "HKLM:\SOFTWARE\EpicGames\Unreal Engine\4.26" -ErrorAction Stop
        if ($reg.InstalledDirectory) { $candidates += $reg.InstalledDirectory }
    } catch {
    }

    $candidates += @(
        "C:\Program Files\Epic Games\UE_4.26",
        "D:\Software\Epic Games\UE_4.26",
        "D:\Software\Epic\Epic Games\UE_4.26"
    )

    foreach ($candidate in $candidates | Where-Object { $_ } | Select-Object -Unique) {
        if (Test-Path (Join-Path $candidate "Engine\Build\BatchFiles\Build.bat")) {
            return (Resolve-Path $candidate).Path
        }
    }

    throw "UE4.26 was not found. Set -UE4Root or UE4_ROOT."
}

function Resolve-CondaExe {
    $candidates = @(
        $env:CONDA_EXE,
        "D:\Code\Anaconda3\Scripts\conda.exe",
        (Join-Path $env:USERPROFILE "miniconda3\Scripts\conda.exe"),
        (Join-Path $env:USERPROFILE "anaconda3\Scripts\conda.exe")
    ) | Where-Object { $_ }

    foreach ($candidate in $candidates | Select-Object -Unique) {
        if (Test-Path $candidate) {
            return (Resolve-Path $candidate).Path
        }
    }

    return $null
}

function Resolve-CondaPython {
    param([string]$VersionMajorMinor)

    $condaExe = Resolve-CondaExe
    if (-not $condaExe) {
        return $null
    }

    $envList = $null
    try {
        $envList = & $condaExe env list --json | ConvertFrom-Json
    } catch {
        return $null
    }

    $orderedEnvPaths = @()
    if ($envList.envs) {
        $orderedEnvPaths += $envList.envs | Where-Object { (Split-Path $_ -Leaf) -eq "carlaAir" }
        $orderedEnvPaths += $envList.envs | Where-Object { (Split-Path $_ -Leaf) -ne "carlaAir" }
    }

    foreach ($envPath in $orderedEnvPaths | Select-Object -Unique) {
        $pythonExe = Join-Path $envPath "python.exe"
        if (-not (Test-Path $pythonExe)) {
            continue
        }

        if (-not $VersionMajorMinor) {
            return (Resolve-Path $pythonExe).Path
        }

        try {
            $resolvedVersion = (& $pythonExe -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')" 2>$null | Select-Object -Last 1).Trim()
            if (($LASTEXITCODE -eq 0) -and ($VersionMajorMinor -eq $resolvedVersion)) {
                return (Resolve-Path $pythonExe).Path
            }
        } catch {
        }
    }

    return $null
}

function Resolve-Python310 {
    param([string]$ExplicitPython)

    $candidates = @()
    if ($ExplicitPython) { $candidates += $ExplicitPython }
    if ($env:CARLAAIR_PYTHON_EXE) { $candidates += $env:CARLAAIR_PYTHON_EXE }

    foreach ($candidate in $candidates | Where-Object { $_ } | Select-Object -Unique) {
        if (Test-Path $candidate) {
            $resolvedPath = (Resolve-Path $candidate).Path
            try {
                & $resolvedPath -c "import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)" 2>$null
                if ($LASTEXITCODE -eq 0) {
                    return $resolvedPath
                }
            } catch {
            }
        }
    }

    $condaPython = Resolve-CondaPython -VersionMajorMinor "3.10"
    if ($condaPython) {
        return $condaPython
    }

    $pathPython = Get-Command python -ErrorAction SilentlyContinue
    if ($pathPython) {
        try {
            & $pathPython.Source -c "import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)" 2>$null
            if ($LASTEXITCODE -eq 0) {
                return $pathPython.Source
            }
        } catch {
        }
    }

    $resolved = $null
    try {
        $resolved = & py -3.10 -c "import sys; print(sys.executable)" 2>$null
    } catch {
    }
    if ($resolved) {
        return $resolved.Trim()
    }

    throw "Python 3.10 was not found. Install it or pass -PythonExe."
}

function Resolve-AnyPython {
    param([string]$ExplicitPython)

    $candidates = @()
    if ($ExplicitPython) { $candidates += $ExplicitPython }
    if ($env:CARLAAIR_PYTHON_EXE) { $candidates += $env:CARLAAIR_PYTHON_EXE }

    foreach ($candidate in $candidates | Where-Object { $_ } | Select-Object -Unique) {
        if (Test-Path $candidate) {
            return (Resolve-Path $candidate).Path
        }
    }

    $condaPython = Resolve-CondaPython
    if ($condaPython) {
        return $condaPython
    }

    $pathPython = Get-Command python -ErrorAction SilentlyContinue
    if ($pathPython) {
        return $pathPython.Source
    }

    $resolved = $null
    try {
        $resolved = & py -3 -c "import sys; print(sys.executable)" 2>$null
    } catch {
    }
    if ($resolved) {
        return $resolved.Trim()
    }

    throw "No usable Python interpreter was found. Install Python or pass -PythonExe."
}

function Convert-ToCmdArgument {
    param([string]$Value)
    if ($null -eq $Value) {
        return '""'
    }

    if ($Value -match '^[A-Za-z0-9_:\\/.\-]+$') {
        return $Value
    }

    return '"' + ($Value -replace '"', '""') + '"'
}

function Invoke-BatchScript {
    param(
        [string]$BatchPath,
        [string[]]$Arguments,
        [string]$VsDevCmd,
        [hashtable]$Environment
    )

    $tempFile = [System.IO.Path]::ChangeExtension([System.IO.Path]::GetTempFileName(), ".cmd")
    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add("@echo off")
    $lines.Add("setlocal")
    $lines.Add(('call "{0}" -arch=x64 -host_arch=x64 >nul' -f $VsDevCmd))

    foreach ($pair in $Environment.GetEnumerator()) {
        if ($pair.Value) {
            $lines.Add(('set "{0}={1}"' -f $pair.Key, $pair.Value))
        }
    }
    $lines.Add('if defined WINDOWS_SDK_BIN set "PATH=%WINDOWS_SDK_BIN%;%PATH%"')
    $lines.Add('if defined WINDOWS_SDK_MT if not exist "%WINDOWS_SDK_MT%" exit /b 1')

    $argLine = ""
    if ($Arguments) {
        $argLine = ($Arguments | ForEach-Object { Convert-ToCmdArgument $_ }) -join " "
    }
    $lines.Add((('call "{0}" {1}' -f $BatchPath, $argLine)).Trim())
    $lines.Add("exit /b %errorlevel%")
    [System.IO.File]::WriteAllLines($tempFile, $lines, [System.Text.Encoding]::ASCII)

    try {
        & cmd.exe /d /c $tempFile
        if ($LASTEXITCODE -ne 0) {
            throw "Batch failed: $BatchPath"
        }
    } finally {
        Remove-Item $tempFile -ErrorAction SilentlyContinue
    }
}

function Get-PluginEditorModuleMap {
    return [ordered]@{
        "Carla" = @("Carla")
        "AirSim" = @("AirSim")
        "CarlaTools" = @("CarlaTools")
        "StreetMap" = @("StreetMapRuntime", "StreetMapImporting")
    }
}

function Read-EditorModuleManifest {
    param([string]$Path)

    if (-not (Test-Path $Path)) {
        throw "Editor module manifest was not found: $Path"
    }

    try {
        $manifest = Get-Content -Path $Path -Raw | ConvertFrom-Json
    } catch {
        throw "Editor module manifest is not valid JSON: $Path"
    }

    if (-not $manifest.BuildId) {
        throw "Editor module manifest has no BuildId: $Path"
    }
    if (-not $manifest.Modules) {
        throw "Editor module manifest has no module entries: $Path"
    }

    return $manifest
}

function Assert-EditorModuleBinary {
    param(
        [object]$Manifest,
        [string]$ManifestPath,
        [string]$BinariesDirectory,
        [string]$ModuleName
    )

    $moduleBinaryName = $Manifest.Modules.$ModuleName
    if (-not $moduleBinaryName) {
        throw "Module '$ModuleName' was not found in $ManifestPath"
    }

    $moduleBinaryPath = Join-Path $BinariesDirectory $moduleBinaryName
    if (-not (Test-Path $moduleBinaryPath)) {
        throw "Module binary '$moduleBinaryName' was not found: $moduleBinaryPath"
    }
}

function Test-EditorModuleManifests {
    param([string]$SourceRoot)

    $projectRoot = Join-Path $SourceRoot "Unreal\CarlaUE4"
    $projectBinaries = Join-Path $projectRoot "Binaries\Win64"
    $projectManifestPath = Join-Path $projectBinaries "UE4Editor.modules"
    $projectManifest = Read-EditorModuleManifest -Path $projectManifestPath
    Assert-EditorModuleBinary -Manifest $projectManifest -ManifestPath $projectManifestPath -BinariesDirectory $projectBinaries -ModuleName "CarlaUE4"

    $projectBuildId = [string]$projectManifest.BuildId
    foreach ($pluginEntry in (Get-PluginEditorModuleMap).GetEnumerator()) {
        $pluginName = $pluginEntry.Key
        $pluginBinaries = Join-Path $projectRoot ("Plugins\" + $pluginName + "\Binaries\Win64")
        $pluginManifestPath = Join-Path $pluginBinaries "UE4Editor.modules"
        $pluginManifest = Read-EditorModuleManifest -Path $pluginManifestPath

        if ([string]$pluginManifest.BuildId -ne $projectBuildId) {
            throw "Plugin '$pluginName' BuildId '$($pluginManifest.BuildId)' does not match project BuildId '$projectBuildId'. Rebuild the plugin with this CarlaUE4Editor target."
        }

        foreach ($moduleName in $pluginEntry.Value) {
            Assert-EditorModuleBinary -Manifest $pluginManifest -ManifestPath $pluginManifestPath -BinariesDirectory $pluginBinaries -ModuleName $moduleName
        }
    }

    Write-Host "Validated CarlaUE4 and plugin UE4Editor.modules manifests (BuildId: $projectBuildId)."
}

function Sync-WindowsPackageLauncherFiles {
    param([string]$SourceRoot, [string]$BuildRoot)

    $packageContainer = Join-Path $BuildRoot "UE4Carla"
    if (-not (Test-Path $packageContainer)) {
        return
    }

    $runtimeSources = @(
        @{
            Source = Join-Path $SourceRoot "CarlaAir.ps1"
            Destination = "CarlaAir.ps1"
            Type = "file"
        },
        @{
            Source = Join-Path $SourceRoot "auto_traffic.py"
            Destination = "auto_traffic.py"
            Type = "file"
        },
        @{
            Source = Join-Path $SourceRoot "StartCarlaAir.bat"
            Destination = "StartCarlaAir.bat"
            Type = "file"
        },
        @{
            Source = Join-Path $SourceRoot "StopCarlaAir.bat"
            Destination = "StopCarlaAir.bat"
            Type = "file"
        },
        @{
            Source = Join-Path $SourceRoot "SetupEnv.bat"
            Destination = "SetupEnv.bat"
            Type = "file"
        },
        @{
            Source = Join-Path $SourceRoot "TestEnv.bat"
            Destination = "TestEnv.bat"
            Type = "file"
        },
        @{
            Source = Join-Path $SourceRoot "AirSimConfig"
            Destination = "AirSimConfig"
            Type = "directory"
        },
        @{
            Source = Join-Path $SourceRoot "env_setup"
            Destination = "env_setup"
            Type = "directory"
        }
    )

    $packageRoots = Get-ChildItem -Path $packageContainer -Directory -ErrorAction SilentlyContinue | ForEach-Object {
        Join-Path $_.FullName "WindowsNoEditor"
    }

    foreach ($packageRoot in $packageRoots | Select-Object -Unique) {
        if (-not (Test-Path $packageRoot)) {
            continue
        }

        foreach ($runtimeSource in $runtimeSources) {
            if (-not (Test-Path $runtimeSource.Source)) {
                continue
            }

            $destinationPath = Join-Path $packageRoot $runtimeSource.Destination
            if ($runtimeSource.Type -eq "directory") {
                New-Item -ItemType Directory -Force -Path $destinationPath | Out-Null
                Copy-Item -Path (Join-Path $runtimeSource.Source "*") -Destination $destinationPath -Recurse -Force
            } else {
                Copy-Item -Path $runtimeSource.Source -Destination $destinationPath -Force
            }
        }
    }
}

$sourceRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$rootPath = ((Resolve-Path $sourceRoot).Path.TrimEnd("\")) + "\"
$buildPath = if ($InstallationDir) { $InstallationDir } else { Join-Path $sourceRoot "Build" }
$buildPath = ((Resolve-Path (New-Item -ItemType Directory -Force -Path $buildPath)).Path.TrimEnd("\")) + "\"

if (-not ($Full -or $Setup -or $BuildLibCarla -or $BuildOSM2ODR -or $BuildPythonApi -or $BuildUE4 -or $Package)) {
    $Full = $true
}

$needUE4 = $Full -or $BuildUE4 -or $Package
$needPython310 = $Full -or $Setup -or $BuildPythonApi
$needAnyPython = $needPython310 -or $BuildUE4 -or $Package

$vsDevCmd = Resolve-VsDevCmd
$windowsSdkBin = if ($needUE4) { Resolve-WindowsSdkBin } else { $null }
$resolvedPython = $null
if ($needPython310) {
    $resolvedPython = Resolve-Python310 -ExplicitPython $PythonExe
} elseif ($needAnyPython -or $PythonExe) {
    $resolvedPython = Resolve-AnyPython -ExplicitPython $PythonExe
}
$resolvedPythonVersion = $null
$resolvedPythonRoot = $null
if ($resolvedPython) {
    try {
        $resolvedPythonVersion = (& $resolvedPython -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')" 2>$null | Select-Object -Last 1).Trim()
    } catch {
    }
    try {
        $resolvedPythonRoot = (& $resolvedPython -c "import sys; print(sys.prefix)" 2>$null | Select-Object -Last 1).Trim()
    } catch {
    }
}
$resolvedUE4 = $null
if ($needUE4) {
    $resolvedUE4 = Resolve-UE4Root -ExplicitRoot $UE4Root
}

$commonEnv = @{
    ROOT_PATH = $rootPath
    INSTALLATION_DIR = $buildPath
    UE4_ROOT = $resolvedUE4
    CARLAAIR_PYTHON_EXE = $resolvedPython
    CARLAAIR_PYTHON_VERSION = $resolvedPythonVersion
    CARLAAIR_PYTHON_ROOT = $resolvedPythonRoot
    WINDOWS_SDK_BIN = $windowsSdkBin
    WINDOWS_SDK_MT = if ($windowsSdkBin) { Join-Path $windowsSdkBin "mt.exe" } else { $null }
}

Write-Host "SourceRoot: $sourceRoot"
Write-Host "BuildRoot:  $buildPath"
Write-Host "VS DevCmd:  $vsDevCmd"
Write-Host "SDK Bin:    $windowsSdkBin"
Write-Host "UE4_ROOT:   $resolvedUE4"
Write-Host "PythonExe:  $resolvedPython"

if ($Full -or $Setup) {
    Write-Phase "Plugins"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\BuildUE4Plugins.bat") -Arguments @("--build") -VsDevCmd $vsDevCmd -Environment $commonEnv

    Write-Phase "Setup"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\Setup.bat") -Arguments @("--boost-toolset", "msvc-14.3", "--generator", $Generator) -VsDevCmd $vsDevCmd -Environment $commonEnv
}

if ($Full -or $BuildLibCarla) {
    Write-Phase "LibCarla"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\BuildLibCarla.bat") -Arguments @("--server", "--client", "--generator", $Generator) -VsDevCmd $vsDevCmd -Environment $commonEnv
}

if ($Full -or $BuildOSM2ODR) {
    Write-Phase "OSM2ODR"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\BuildOSM2ODR.bat") -Arguments @("--build", "--generator", $Generator) -VsDevCmd $vsDevCmd -Environment $commonEnv
}

if ($Full -or $BuildPythonApi) {
    Write-Phase "Python API"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\BuildPythonAPI.bat") -Arguments @("--build-wheel") -VsDevCmd $vsDevCmd -Environment $commonEnv
}

if ($Full -or $BuildUE4) {
    Write-Phase "UE4 Build"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\BuildCarlaUE4.bat") -Arguments @("--build") -VsDevCmd $vsDevCmd -Environment $commonEnv

    Write-Phase "Validate Editor Module Manifests"
    Test-EditorModuleManifests -SourceRoot $sourceRoot
}

if ($Full -or $Package) {
    Write-Phase "Validate Editor Module Manifests"
    Test-EditorModuleManifests -SourceRoot $sourceRoot

    Write-Phase "Package"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\Package.bat") -Arguments @("--config", $Configuration, "--no-zip") -VsDevCmd $vsDevCmd -Environment $commonEnv

    Write-Phase "Sync Windows Launchers"
    Sync-WindowsPackageLauncherFiles -SourceRoot $sourceRoot -BuildRoot $buildPath
}

Write-Host ""
Write-Host "Windows build pipeline completed."
