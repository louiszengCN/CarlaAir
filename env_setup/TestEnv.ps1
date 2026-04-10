param(
    [string]$EnvName = "carlaAir",
    [string]$CondaExe
)

$ErrorActionPreference = "Stop"

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

function Test-Port {
    param([int]$Port)
    try {
        $client = New-Object System.Net.Sockets.TcpClient
        $iar = $client.BeginConnect("127.0.0.1", $Port, $null, $null)
        $ok = $iar.AsyncWaitHandle.WaitOne(500)
        if (-not $ok) {
            $client.Close()
            return $false
        }
        $client.EndConnect($iar)
        $client.Close()
        return $true
    } catch {
        return $false
    }
}

$resolvedConda = Resolve-CondaExe -ExplicitPath $CondaExe
$pass = 0
$fail = 0
$warn = 0

function Add-Pass($Message) { $script:pass++; Write-Host "  [PASS] $Message" }
function Add-Fail($Message) { $script:fail++; Write-Host "  [FAIL] $Message" }
function Add-Warn($Message) { $script:warn++; Write-Host "  [WARN] $Message" }

Write-Host "============================================"
Write-Host "  CarlaAir Windows Environment Test"
Write-Host "============================================"
Write-Host ""

try {
    $pythonVersion = & $resolvedConda run -n $EnvName python --version 2>&1
    if ($LASTEXITCODE -eq 0) {
        Add-Pass "python found: $pythonVersion"
    } else {
        Add-Fail "python not available in conda env '$EnvName'"
    }
} catch {
    Add-Fail "python not available in conda env '$EnvName'"
}

foreach ($module in @("carla", "airsim", "pygame", "numpy")) {
    try {
        & $resolvedConda run -n $EnvName python -c "import $module" 2>$null
        if ($LASTEXITCODE -eq 0) {
            Add-Pass $module
        } else {
            Add-Fail "$module not installed"
        }
    } catch {
        Add-Fail "$module not installed"
    }
}

foreach ($module in @("cv2", "PIL")) {
    try {
        & $resolvedConda run -n $EnvName python -c "import $module" 2>$null
        if ($LASTEXITCODE -eq 0) {
            Add-Pass "$module (optional)"
        } else {
            Add-Warn "$module not installed"
        }
    } catch {
        Add-Warn "$module not installed"
    }
}

if (Test-Port -Port 2000) {
    Add-Pass "CARLA port 2000 is listening"
    try {
        & $resolvedConda run -n $EnvName python -c "import carla; c=carla.Client('localhost', 2000); c.set_timeout(5.0); print(c.get_world().get_map().name)" 2>$null
        if ($LASTEXITCODE -eq 0) {
            Add-Pass "CARLA world accessible"
        } else {
            Add-Fail "CARLA world not accessible"
        }
    } catch {
        Add-Fail "CARLA world not accessible"
    }
} else {
    Add-Warn "CARLA port 2000 not listening"
}

if (Test-Port -Port 41451) {
    Add-Pass "AirSim port 41451 is listening"
    try {
        & $resolvedConda run -n $EnvName python -c "import airsim; c=airsim.MultirotorClient(port=41451); c.confirmConnection()" 2>$null
        if ($LASTEXITCODE -eq 0) {
            Add-Pass "AirSim RPC responsive"
        } else {
            Add-Fail "AirSim RPC not responsive"
        }
    } catch {
        Add-Fail "AirSim RPC not responsive"
    }
} else {
    Add-Warn "AirSim port 41451 not listening"
}

Write-Host ""
Write-Host "============================================"
Write-Host "  Results: $pass PASS, $fail FAIL, $warn WARN"
if ($fail -eq 0) {
    Write-Host "  Status: ALL CHECKS PASSED"
} else {
    Write-Host "  Status: SOME CHECKS FAILED"
}
Write-Host "============================================"

exit $fail
