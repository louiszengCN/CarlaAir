$ErrorActionPreference = "Stop"

function Show-Usage {
    @"
Usage: .\CarlaAir.ps1 [MAP] [OPTIONS]

Options:
  --res WxH                Window resolution (default: 1280x720)
  --port PORT              CARLA RPC port (default: 2000)
  --quality LEVEL          Quality level: Low, Medium, High, Epic
  --fg                     Keep this PowerShell session attached to the process
  --kill                   Stop the last CarlaAir instance started by this script
  --log                    Tail the CarlaAir log file
  --no-traffic             Do not auto-start auto_traffic.py
  --traffic-vehicles N     Vehicle count for auto traffic (default: 30)
  --traffic-walkers N      Walker count for auto traffic (default: 50)
  --package-root PATH      Explicit WindowsNoEditor root
  --python PATH            Explicit python.exe for auto traffic
  --help                   Show this help
"@ | Write-Host
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

function Resolve-CondaExe {
    $commandConda = Get-Command conda -ErrorAction SilentlyContinue
    $candidates = @(
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

    return $null
}

function Resolve-TrafficPython {
    param([string]$ExplicitPath)

    if ($ExplicitPath) {
        if (-not (Test-Path $ExplicitPath)) {
            throw "Python path not found: $ExplicitPath"
        }
        return (Resolve-Path $ExplicitPath).Path
    }

    if ($env:CARLAAIR_PYTHON_EXE -and (Test-Path $env:CARLAAIR_PYTHON_EXE)) {
        return (Resolve-Path $env:CARLAAIR_PYTHON_EXE).Path
    }

    $pathPython = Get-Command python -ErrorAction SilentlyContinue
    if ($pathPython) {
        try {
            & $pathPython.Source -c "import carla" 2>$null
            if ($LASTEXITCODE -eq 0) {
                return $pathPython.Source
            }
        } catch {
        }
    }

    $condaExe = Resolve-CondaExe
    if ($condaExe) {
        try {
            $resolved = & $condaExe run -n carlaAir python -c "import carla, sys; print(sys.executable)" 2>$null
            if ($LASTEXITCODE -eq 0 -and $resolved) {
                $resolvedLines = @($resolved) | ForEach-Object { "$_".Trim() } | Where-Object { $_ }
                $candidate = $resolvedLines | Select-Object -Last 1
                if ($candidate -and (Test-Path $candidate)) {
                    return (Resolve-Path $candidate).Path
                }
            }
        } catch {
        }
    }

    return $null
}

function Resolve-CarlaBinary {
    param([string]$RepoRoot, [string]$ExplicitPackageRoot)

    $packageRoots = @()
    if ($ExplicitPackageRoot) {
        $packageRoots += $ExplicitPackageRoot
    } else {
        $packageRoots += Join-Path $RepoRoot "WindowsNoEditor"
        $packageRoots += Get-ChildItem (Join-Path $RepoRoot "Build\UE4Carla") -Directory -ErrorAction SilentlyContinue | ForEach-Object {
            Join-Path $_.FullName "WindowsNoEditor"
        }
    }

    foreach ($packageRoot in $packageRoots | Select-Object -Unique) {
        if (-not $packageRoot) { continue }

        $rootExe = Join-Path $packageRoot "CarlaUE4.exe"
        if (Test-Path $rootExe) {
            return @{
                Binary = (Resolve-Path $rootExe).Path
                WorkingDirectory = (Resolve-Path $packageRoot).Path
                NeedsProjectArg = $false
            }
        }

        $shippingExe = Join-Path $packageRoot "CarlaUE4\Binaries\Win64\CarlaUE4-Win64-Shipping.exe"
        if (Test-Path $shippingExe) {
            return @{
                Binary = (Resolve-Path $shippingExe).Path
                WorkingDirectory = (Resolve-Path $packageRoot).Path
                NeedsProjectArg = $true
            }
        }
    }

    $devExe = Join-Path $RepoRoot "Unreal\CarlaUE4\Binaries\Win64\CarlaUE4.exe"
    if (Test-Path $devExe) {
        return @{
            Binary = (Resolve-Path $devExe).Path
            WorkingDirectory = Split-Path -Parent $devExe
            NeedsProjectArg = $false
        }
    }

    throw "No Windows Carla binary found. Build the project first with .\BuildWindows.ps1."
}

function Assert-NonNegative {
    param(
        [string]$Name,
        [int]$Value
    )

    if ($Value -lt 0) {
        throw "$Name must be a non-negative integer."
    }
}

function Stop-CarlaAirProcess {
    param([string]$RepoRoot)

    foreach ($pidFile in @((Join-Path $RepoRoot ".carlaair.pid"), (Join-Path $RepoRoot ".traffic.pid"))) {
        if (-not (Test-Path $pidFile)) { continue }
        $pidValue = Get-Content $pidFile -ErrorAction SilentlyContinue | Select-Object -First 1
        $pidInt = 0
        if ([int]::TryParse($pidValue, [ref]$pidInt)) {
            $process = Get-Process -Id $pidInt -ErrorAction SilentlyContinue
            if ($process) {
                Stop-Process -Id $pidInt -Force
            }
        }
        Remove-Item $pidFile -ErrorAction SilentlyContinue
    }

    Get-Process CarlaUE4, CarlaUE4-Win64-Shipping -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
}

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$mapName = "Town10HD"
$resX = 1280
$resY = 720
$carlaPort = 2000
$airsimPort = 41451
$quality = "Epic"
$foreground = $false
$showLog = $false
$killOnly = $false
$autoTraffic = $true
$trafficVehicles = 30
$trafficWalkers = 50
$explicitPackageRoot = $null
$explicitPython = $null

for ($i = 0; $i -lt $args.Count; $i++) {
    $arg = [string]$args[$i]
    switch -Regex ($arg) {
        '^--help$|^-h$' { Show-Usage; exit 0 }
        '^--fg$' { $foreground = $true; continue }
        '^--kill$' { $killOnly = $true; continue }
        '^--log$' { $showLog = $true; continue }
        '^--no-traffic$' { $autoTraffic = $false; continue }
        '^--traffic-vehicles$' { $i++; $trafficVehicles = [int]$args[$i]; continue }
        '^--traffic-walkers$' { $i++; $trafficWalkers = [int]$args[$i]; continue }
        '^--res$' {
            $i++
            $parts = ([string]$args[$i]).Split("x")
            if ($parts.Count -ne 2) { throw "Invalid resolution: $($args[$i])" }
            $resX = [int]$parts[0]
            $resY = [int]$parts[1]
            continue
        }
        '^--port$' { $i++; $carlaPort = [int]$args[$i]; continue }
        '^--quality$' { $i++; $quality = [string]$args[$i]; continue }
        '^--package-root$' { $i++; $explicitPackageRoot = [string]$args[$i]; continue }
        '^--python$' { $i++; $explicitPython = [string]$args[$i]; continue }
        '^Town.*|^town.*' { $mapName = $arg; continue }
        default { throw "Unknown option: $arg" }
    }
}

Assert-NonNegative -Name "--traffic-vehicles" -Value $trafficVehicles
Assert-NonNegative -Name "--traffic-walkers" -Value $trafficWalkers
Assert-NonNegative -Name "--port" -Value $carlaPort

$pidFile = Join-Path $repoRoot ".carlaair.pid"
$trafficPidFile = Join-Path $repoRoot ".traffic.pid"
$logFile = Join-Path $repoRoot "CarlaAir.log"
$trafficLogFile = Join-Path $repoRoot "traffic.log"
$trafficErrFile = Join-Path $repoRoot "traffic.err.log"

if ($killOnly) {
    Stop-CarlaAirProcess -RepoRoot $repoRoot
    Write-Host "CarlaAir stopped."
    exit 0
}

if ($showLog) {
    if (-not (Test-Path $logFile)) {
        throw "No log file found at $logFile"
    }
    Get-Content $logFile -Wait -Tail 100
    exit 0
}

$binaryInfo = Resolve-CarlaBinary -RepoRoot $repoRoot -ExplicitPackageRoot $explicitPackageRoot
$trafficPython = if ($autoTraffic) { Resolve-TrafficPython -ExplicitPath $explicitPython } else { $null }

$airsimSettingsSource = Join-Path $repoRoot "AirSimConfig\settings.json"
if (-not (Test-Path $airsimSettingsSource)) {
    throw "AirSim settings template not found: $airsimSettingsSource"
}

$airsimSettingsDir = Join-Path $env:USERPROFILE "Documents\AirSim"
$airsimSettingsTarget = Join-Path $airsimSettingsDir "settings.json"
New-Item -ItemType Directory -Force -Path $airsimSettingsDir | Out-Null
Copy-Item $airsimSettingsSource $airsimSettingsTarget -Force

Stop-CarlaAirProcess -RepoRoot $repoRoot

$launchArgs = New-Object System.Collections.Generic.List[string]
if ($binaryInfo.NeedsProjectArg) {
    $launchArgs.Add("CarlaUE4")
}
$launchArgs.Add($mapName)
$launchArgs.Add("-windowed")
$launchArgs.Add("-ResX=$resX")
$launchArgs.Add("-ResY=$resY")
$launchArgs.Add("-carla-rpc-port=$carlaPort")
$launchArgs.Add("-quality-level=$quality")
$launchArgs.Add("-TexturePoolSize=2048")
$launchArgs.Add("-unattended")
$launchArgs.Add("-nosound")
$launchArgs.Add("-UseVSync")
$launchArgs.Add("-abslog=`"$logFile`"")

Write-Host "============================================"
Write-Host "  CarlaAir - Windows Launcher"
Write-Host "============================================"
Write-Host "  Map:        $mapName"
Write-Host "  Resolution: ${resX}x${resY}"
Write-Host "  CARLA Port: $carlaPort"
Write-Host "  AirSim Port: $airsimPort"
Write-Host "  Quality:    $quality"
Write-Host "  Binary:     $($binaryInfo.Binary)"
Write-Host "============================================"

if ($foreground) {
    if ($autoTraffic) {
        Write-Warning "Foreground mode does not auto-start auto_traffic.py. Start it separately after CARLA is ready if needed."
    }
    Push-Location $binaryInfo.WorkingDirectory
    try {
        & $binaryInfo.Binary @launchArgs
        exit $LASTEXITCODE
    } finally {
        Pop-Location
    }
}

$process = Start-Process -FilePath $binaryInfo.Binary -ArgumentList $launchArgs -WorkingDirectory $binaryInfo.WorkingDirectory -PassThru
Set-Content -Path $pidFile -Value $process.Id

Write-Host "Waiting for CARLA port..."
$carlaReady = $false
for ($attempt = 0; $attempt -lt 120; $attempt++) {
    Start-Sleep -Seconds 2
    if ($process.HasExited) {
        throw "CarlaAir exited early. Check $logFile"
    }
    if (Test-Port -Port $carlaPort) {
        Write-Host "  CARLA (port $carlaPort): Ready"
        $carlaReady = $true
        break
    }
}
if (-not $carlaReady) {
    throw "CARLA port $carlaPort did not become ready within 240 seconds. Check $logFile"
}

Write-Host "Waiting for AirSim port..."
$airsimReady = $false
for ($attempt = 0; $attempt -lt 60; $attempt++) {
    Start-Sleep -Seconds 2
    if (Test-Port -Port $airsimPort) {
        Write-Host "  AirSim (port $airsimPort): Ready"
        $airsimReady = $true
        break
    }
}
if (-not $airsimReady) {
    throw "AirSim port $airsimPort did not become ready within 120 seconds. Check $logFile"
}

if ($autoTraffic) {
    if (($trafficVehicles -eq 0) -and ($trafficWalkers -eq 0)) {
        Write-Host "Traffic disabled by count (0 vehicles + 0 walkers)."
    } elseif ($trafficPython) {
        $trafficArgs = @(
            (Join-Path $repoRoot "auto_traffic.py"),
            "--vehicles", [string]$trafficVehicles,
            "--walkers", [string]$trafficWalkers,
            "--port", [string]$carlaPort
        )
        $trafficProcess = Start-Process -FilePath $trafficPython -ArgumentList $trafficArgs -WorkingDirectory $repoRoot -RedirectStandardOutput $trafficLogFile -RedirectStandardError $trafficErrFile -PassThru
        Set-Content -Path $trafficPidFile -Value $trafficProcess.Id
        Write-Host "Traffic process started: $($trafficProcess.Id)"
    } else {
        Write-Warning "auto_traffic.py was not started because no Python with the 'carla' module was found."
    }
}

Write-Host ""
Write-Host "CarlaAir is ready."
Write-Host "Log: $logFile"
