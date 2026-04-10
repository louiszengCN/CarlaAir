param(
    [string]$EnvName = "carlaAir",
    [string]$PythonVersion = "3.10",
    [string]$WheelPath,
    [string]$CondaExe
)

$ErrorActionPreference = "Stop"

function Invoke-Checked {
    param(
        [string]$FilePath,
        [string[]]$Arguments
    )

    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed: $FilePath $($Arguments -join ' ')"
    }
}

function Resolve-CondaExe {
    param([string]$ExplicitPath)

    $commandConda = Get-Command conda -ErrorAction SilentlyContinue
    $candidates = @(
        $ExplicitPath,
        $env:CONDA_EXE,
        $commandConda.Source,
        "D:\Code\Anaconda3\Scripts\conda.exe",
        (Join-Path $env:LOCALAPPDATA "miniconda3\Scripts\conda.exe"),
        (Join-Path $env:LOCALAPPDATA "anaconda3\Scripts\conda.exe"),
        (Join-Path $env:USERPROFILE "miniconda3\Scripts\conda.exe"),
        (Join-Path $env:USERPROFILE "anaconda3\Scripts\conda.exe"),
        "C:\ProgramData\miniconda3\Scripts\conda.exe",
        "C:\ProgramData\anaconda3\Scripts\conda.exe"
    ) | Where-Object { $_ }

    foreach ($candidate in $candidates | Select-Object -Unique) {
        if (Test-Path $candidate) {
            return (Resolve-Path $candidate).Path
        }
    }

    throw "conda.exe was not found. Install Miniconda/Anaconda or pass -CondaExe."
}

function Resolve-WheelPath {
    param([string]$ExplicitPath, [string]$RepoRoot)

    if ($ExplicitPath) {
        if (-not (Test-Path $ExplicitPath)) {
            throw "Wheel not found: $ExplicitPath"
        }
        return (Resolve-Path $ExplicitPath).Path
    }

    $wheel = Get-ChildItem (Join-Path $RepoRoot "PythonAPI\carla\dist") -Filter "*.whl" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($wheel) {
        return $wheel.FullName
    }

    throw "No Windows carla wheel found under PythonAPI\carla\dist. Build it first with .\BuildWindows.ps1 -BuildPythonApi or .\BuildWindows.ps1."
}

$repoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$resolvedConda = Resolve-CondaExe -ExplicitPath $CondaExe
$resolvedWheel = Resolve-WheelPath -ExplicitPath $WheelPath -RepoRoot $repoRoot

$linuxTarball = Join-Path $repoRoot "env_setup\carla_python_module.tar.gz"
if (Test-Path $linuxTarball) {
    Write-Host "Ignoring Linux-only tarball: $linuxTarball"
}

Write-Host "Conda: $resolvedConda"
Write-Host "Wheel: $resolvedWheel"

$envs = & $resolvedConda env list --json | ConvertFrom-Json
$hasEnv = $false
foreach ($envPath in $envs.envs) {
    if ((Split-Path $envPath -Leaf) -eq $EnvName) {
        $hasEnv = $true
        break
    }
}

if (-not $hasEnv) {
    Write-Host "Creating conda environment '$EnvName' (Python $PythonVersion)..."
    Invoke-Checked -FilePath $resolvedConda -Arguments @("create", "-n", $EnvName, "python=$PythonVersion", "-y")
}

Write-Host "Installing Python dependencies..."
Invoke-Checked -FilePath $resolvedConda -Arguments @("run", "-n", $EnvName, "python", "-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel", "build")
Invoke-Checked -FilePath $resolvedConda -Arguments @("run", "-n", $EnvName, "python", "-m", "pip", "install", "numpy")
Invoke-Checked -FilePath $resolvedConda -Arguments @("run", "-n", $EnvName, "python", "-m", "pip", "install", "msgpack-rpc-python")
Invoke-Checked -FilePath $resolvedConda -Arguments @("run", "-n", $EnvName, "python", "-m", "pip", "install", "--no-build-isolation", "airsim")
Invoke-Checked -FilePath $resolvedConda -Arguments @("run", "-n", $EnvName, "python", "-m", "pip", "install", "pygame", "opencv-python", "pillow")

Write-Host "Removing any incompatible pip-installed carla package..."
try {
    & $resolvedConda run -n $EnvName python -m pip uninstall -y carla | Out-Null
} catch {
}

Write-Host "Installing CarlaAir wheel..."
Invoke-Checked -FilePath $resolvedConda -Arguments @("run", "-n", $EnvName, "python", "-m", "pip", "install", "--force-reinstall", $resolvedWheel)

Write-Host "Verifying imports..."
Invoke-Checked -FilePath $resolvedConda -Arguments @("run", "-n", $EnvName, "python", "-c", "import carla, airsim, pygame, numpy, cv2; print('carla:', carla.__file__)")

Write-Host ""
Write-Host "Environment ready."
Write-Host "Use: conda activate $EnvName"
