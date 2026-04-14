# Startup Guide

## Where to Run It

Run the command from the **project root directory**, which is the directory containing:

- `CarlaAir.ps1`
- `AirSimConfig\settings.json`
- `BuildWindows.ps1`

## Basic Startup Command

Open PowerShell in the project root and run:

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD
```

If you are already inside PowerShell and your current directory is the project root, you can also run:

```powershell
.\CarlaAir.ps1 Town10HD
```

If you hit an execution policy error, prefer the first form.

## Recommended Command for Initial Troubleshooting

For the first startup check, do not auto-start traffic:

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

This makes it easier to verify the main process and the `AirSim` port first.

## Common Commands

### Start the Default Map

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD
```

### Start Without Auto Traffic

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

### Set Resolution

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --res 1920x1080
```

### Set the CARLA Main Port

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --port 2000
```

### Set Quality Level

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --quality High
```

### Foreground Mode

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --fg --no-traffic
```

Notes:

- `--fg` attaches the current PowerShell session to the game process.
- In foreground mode, `auto_traffic.py` is not started automatically.
- This mode is useful for immediate error inspection, not for normal use.

### View the Log

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 --log
```

### Stop CarlaAir

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 --kill
```

## How to Interpret Ports 2000 and 41451

- `2000` is the `CARLA` main port
- `41451` is the `AirSim` port

Use this rule:

1. If `2000` is not working
   troubleshoot the main process startup first
2. If `2000` is working but `41451` is not
   troubleshoot `AirSim` initialization first

## Most Common Issue: 41451 Not Working

If `2000` is fine but `41451` is not, check the following items first.

### 1. Make Sure CarlaAir Was Started Through `CarlaAir.ps1`

The expected command is:

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

Do not launch the exe directly.

### 2. Check the Real Documents Path

Run this in PowerShell:

```powershell
[Environment]::GetFolderPath('MyDocuments')
```

Save the returned path.

### 3. Check Whether the AirSim Config File Exists There

Then verify that this file exists:

```text
<real Documents path>\AirSim\settings.json
```

### 4. If It Does Not Exist, Copy It Manually

Copy this file from the project root:

```text
AirSimConfig\settings.json
```

to:

```text
<real Documents path>\AirSim\settings.json
```

Then start CarlaAir again through `CarlaAir.ps1`.
