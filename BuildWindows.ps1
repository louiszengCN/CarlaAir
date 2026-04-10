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

function Get-EngineEditorBuildId {
    param([string]$UE4Root)

    $versionPath = Join-Path $UE4Root "Engine\Binaries\Win64\UE4Editor.version"
    if (Test-Path $versionPath) {
        $versionJson = Get-Content -Path $versionPath -Raw | ConvertFrom-Json
        if ($versionJson.BuildId) {
            return [string]$versionJson.BuildId
        }
    }

    $manifestPath = Join-Path $UE4Root "Engine\Binaries\Win64\UE4Editor.modules"
    if (Test-Path $manifestPath) {
        $manifestJson = Get-Content -Path $manifestPath -Raw | ConvertFrom-Json
        if ($manifestJson.BuildId) {
            return [string]$manifestJson.BuildId
        }
    }

    throw "Unable to resolve the engine editor BuildId from $UE4Root"
}

function Set-JsonBuildId {
    param(
        [string]$Path,
        [string]$BuildId
    )

    $json = Get-Content -Path $Path -Raw | ConvertFrom-Json
    $updated = $false

    if ($json.PSObject.Properties.Name -contains "BuildId") {
        $json.BuildId = $BuildId
        $updated = $true
    }
    if (($json.PSObject.Properties.Name -contains "Version") -and $json.Version -and ($json.Version.PSObject.Properties.Name -contains "BuildId")) {
        $json.Version.BuildId = $BuildId
        $updated = $true
    }

    if ($updated) {
        $jsonText = $json | ConvertTo-Json -Depth 100
        [System.IO.File]::WriteAllText($Path, $jsonText, [System.Text.Encoding]::ASCII)
    }
}

function Sync-EditorBuildIds {
    param(
        [string]$SourceRoot,
        [string]$UE4Root
    )

    $engineBuildId = Get-EngineEditorBuildId -UE4Root $UE4Root
    $projectRoot = Join-Path $SourceRoot "Unreal\CarlaUE4"
    $candidateDirectories = New-Object System.Collections.Generic.List[string]
    $candidateDirectories.Add((Join-Path $projectRoot "Binaries\Win64"))

    $pluginRoot = Join-Path $projectRoot "Plugins"
    if (Test-Path $pluginRoot) {
        foreach ($pluginDir in Get-ChildItem -Path $pluginRoot -Directory) {
            $pluginBinaries = Join-Path $pluginDir.FullName "Binaries\Win64"
            if (Test-Path $pluginBinaries) {
                $candidateDirectories.Add($pluginBinaries)
            }
        }
    }

    foreach ($directory in $candidateDirectories | Select-Object -Unique) {
        Get-ChildItem -Path $directory -File -ErrorAction SilentlyContinue |
            Where-Object { $_.Extension -in @(".modules", ".version", ".target") } |
            ForEach-Object { Set-JsonBuildId -Path $_.FullName -BuildId $engineBuildId }
    }

    return $engineBuildId
}

function Sync-PluginEditorBinaries {
    param(
        [string]$SourceRoot,
        [string]$UE4Root
    )

    $projectRoot = Join-Path $SourceRoot "Unreal\CarlaUE4"
    $projectBinaries = Join-Path $projectRoot "Binaries\Win64"
    $editorManifestPath = Join-Path $projectBinaries "CarlaUE4Editor.modules"
    if (-not (Test-Path $editorManifestPath)) {
        throw "Editor module manifest was not found: $editorManifestPath"
    }

    $engineBuildId = Sync-EditorBuildIds -SourceRoot $SourceRoot -UE4Root $UE4Root

    $editorManifest = Get-Content -Path $editorManifestPath -Raw | ConvertFrom-Json
    if (-not $editorManifest.Modules) {
        throw "Editor module manifest is missing module entries: $editorManifestPath"
    }

    foreach ($pluginEntry in (Get-PluginEditorModuleMap).GetEnumerator()) {
        $pluginName = $pluginEntry.Key
        $pluginRoot = Join-Path $projectRoot ("Plugins\" + $pluginName)
        if (-not (Test-Path $pluginRoot)) {
            throw "Plugin root was not found: $pluginRoot"
        }

        $pluginBinaries = Join-Path $pluginRoot "Binaries\Win64"
        New-Item -ItemType Directory -Force -Path $pluginBinaries | Out-Null

        $pluginModules = [ordered]@{}
        foreach ($moduleName in $pluginEntry.Value) {
            $moduleBinaryName = $editorManifest.Modules.$moduleName
            if (-not $moduleBinaryName) {
                throw "Module '$moduleName' was not found in $editorManifestPath"
            }

            $sourceBinaryPath = Join-Path $projectBinaries $moduleBinaryName
            if (-not (Test-Path $sourceBinaryPath)) {
                throw "Module binary '$moduleBinaryName' was not found in $projectBinaries"
            }

            Get-ChildItem -Path $pluginBinaries -Filter "*$moduleName*" -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue
            Copy-Item -Path $sourceBinaryPath -Destination $pluginBinaries -Force

            $sourcePdbPath = [System.IO.Path]::ChangeExtension($sourceBinaryPath, ".pdb")
            if (Test-Path $sourcePdbPath) {
                Copy-Item -Path $sourcePdbPath -Destination $pluginBinaries -Force
            }

            $pluginModules[$moduleName] = $moduleBinaryName
        }

        $pluginManifestPath = Join-Path $pluginBinaries "UE4Editor.modules"
        $pluginManifestJson = [ordered]@{
            BuildId = [string]$engineBuildId
            Modules = $pluginModules
        } | ConvertTo-Json -Depth 4
        [System.IO.File]::WriteAllText($pluginManifestPath, $pluginManifestJson, [System.Text.Encoding]::ASCII)
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
}

Write-Host "SourceRoot: $sourceRoot"
Write-Host "BuildRoot:  $buildPath"
Write-Host "VS DevCmd:  $vsDevCmd"
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

    Write-Phase "Sync Plugin Editor Binaries"
    Sync-PluginEditorBinaries -SourceRoot $sourceRoot -UE4Root $resolvedUE4
}

if ($Full -or $Package) {
    Write-Phase "Sync Plugin Editor Binaries"
    Sync-PluginEditorBinaries -SourceRoot $sourceRoot -UE4Root $resolvedUE4

    Write-Phase "Package"
    Invoke-BatchScript -BatchPath (Join-Path $sourceRoot "Util\BuildTools\Package.bat") -Arguments @("--config", $Configuration, "--no-zip") -VsDevCmd $vsDevCmd -Environment $commonEnv
}

Write-Host ""
Write-Host "Windows build pipeline completed."
