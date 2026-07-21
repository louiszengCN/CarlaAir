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

## Capturing AirSim Images in Synchronous Mode

AirSim's `simGetImages()` waits for an Unreal render frame. In CARLA synchronous mode, the next frame is only produced when the script calls `world.tick()`. Calling `simGetImages()` directly from the only tick-owning thread can therefore deadlock and eventually time out.

Start the Windows release without automatic traffic:

```powershell
StartCarlaAir.bat Town10HD --no-traffic
```

Then run the compatibility example from another terminal with the `carlaAir` environment active:

```powershell
python examples\sync_airsim_images.py --frames 3 --output recordings\sync_airsim
```

The example uses `examples\airsim_image_fetcher.py` to perform the image RPC on a worker thread while the calling thread continues to run `world.tick()`.

- Only one thread or process may own `world.tick()`.
- Create a new `AirSimImageFetcher` after a capture timeout.
- Always restore the original World settings in `finally`.
- This avoids the RPC deadlock, but AirSim images do not carry a CARLA frame ID and are not guaranteed to be atomically aligned with a CARLA sensor callback.
