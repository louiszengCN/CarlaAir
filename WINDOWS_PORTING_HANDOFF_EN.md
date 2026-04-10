# CarlaAir Windows Porting Handoff

This document is the English handoff version of the current Windows porting work for `CarlaAir`.

It is intended for:

1. Developers continuing Windows-side maintenance or feature work.
2. New agents that need enough context to quickly resume work after a `CarlaAir` version update.

## 1. Current Baseline

- Workspace root: `D:\Code\CarlaAir`
- Current upstream source tree: `D:\Code\CarlaAir\CarlaAir-v0.1.7`
- Current UE4 root: `D:\Code\CarlaAir\UE4`
- Current standalone Windows release directory: `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`
- Old Linux binary package: `D:\Code\CarlaAir\CarlaAir-v0.1.6`
- Current source baseline commit: `7a7fdb33c15a31fbcc55040d9aaf42344ce9bfde`

Version alignment:

- CarlaAir: `v0.1.7`
- CARLA: `0.9.16`
- AirSim: `1.8.1`
- UE4: `CARLA fork UE4.26`

Important:

- The Windows port depends on `CARLA fork UE4.26`, not the Epic Launcher stock UE4.26 build.
- The local `D:\Code\CarlaAir\UE4` tree is the validated working `UE4_ROOT`.
- The source tree and the final Windows release directory are now separated.

## 2. Directory Roles

### 2.1 Source Tree

- `D:\Code\CarlaAir\CarlaAir-v0.1.7`
  This is the active development tree for rebuilding, upgrading, and repackaging.

### 2.2 UE4 Toolchain Tree

- `D:\Code\CarlaAir\UE4`
  This is the validated `UE4_ROOT`.

### 2.3 Windows Release Directory

- `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`
  This is the final runtime distribution, not the development tree.

The release directory keeps:

- `WindowsNoEditor/`
- `CarlaAir.ps1`
- `StartCarlaAir.bat`
- `StopCarlaAir.bat`
- `SetupEnv.bat`
- `TestEnv.bat`
- `env_setup/`
- `PythonAPI/carla/dist/*.whl`
- `examples/`
- `examples_record_demo/`
- `AirSimConfig/settings.json`

### 2.4 Old Linux Package

- `D:\Code\CarlaAir\CarlaAir-v0.1.6`
  This is only a historical runtime package and compatibility reference. It is not the Windows build base.

## 3. What Has Already Been Validated

The following capabilities were validated end-to-end on native Windows 11 x86_64:

- `BuildUE4`
- `Package`
- Native Windows startup
- CARLA port `2000`
- AirSim port `41451`
- Python `carla.Client(...)` connectivity
- Python `airsim.MultirotorClient(...)` connectivity
- `auto_traffic.py`
- `record_vehicle.py`
- `record_walker.py`
- `record_drone.py`
- `demo_director.py`
- MP4 export
- backward compatibility with `v0.1.6` trajectory JSON

Conclusion:

- The current Windows work is not just “compiles on Windows”.
- It is a fully validated native Windows workflow including runtime and demo scripts.

## 4. Main Windows-Specific Differences

The Windows port required changes across four layers:

1. Build pipeline
2. Python environment pipeline
3. UE4 / CARLA / AirSim source compatibility
4. Runtime and example entrypoints

### 4.1 New Windows Entry Files

The main Windows entry files are:

- `BuildWindows.ps1`
- `CarlaAir.ps1`
- `env_setup/SetupEnv.ps1`
- `env_setup/TestEnv.ps1`
- the `.bat` wrappers inside the release directory

Responsibilities:

- `BuildWindows.ps1`
  Drives Windows build phases through `-Setup`, `-BuildLibCarla`, `-BuildOSM2ODR`, `-BuildPythonApi`, `-BuildUE4`, and `-Package`
- `CarlaAir.ps1`
  Launches the packaged Windows build, writes AirSim settings, waits for ports, and optionally starts `auto_traffic.py`
- `SetupEnv.ps1`
  Creates the `carlaAir` Conda environment and installs the Win64 `carla` wheel
- `TestEnv.ps1`
  Checks Python modules, ports, and basic runtime readiness

### 4.2 Build-System Changes

Most build-system changes live under:

- `Util/BuildTools/*.bat`
- `Util/InstallersWin/*.bat`
- `Unreal/CarlaUE4/Source/*.Target.cs`

The goals were:

- make `VS2022 + Windows + CARLA fork UE4.26` actually buildable
- fix batch argument propagation and exit-code handling
- make `Package.bat` reliably complete `cook / stage / package`
- stop the build system from assuming Linux artifacts or stock Epic UE4.26

### 4.3 LibCarla / Compiler Compatibility

Main files:

- `LibCarla/source/compiler/disable-ue4-macros.h`
- `LibCarla/source/compiler/enable-ue4-macros.h`
- `LibCarla/source/carla/rpc/Server.h`

Purpose:

- resolve Windows/MSVC macro pollution
- avoid UE4 macro conflicts with Boost and standard headers

### 4.4 AirSim Compatibility Work

Main files:

- `Unreal/CarlaUE4/Plugins/AirSim/AirSim.uplugin`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirSim.Build.cs`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirBlueprintLib.cpp`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirSim.cpp`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/SimWorldGameMode.cpp`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/WorldSimApi.cpp`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/WorldSimApi.h`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/include/common/common_utils/Utils.hpp`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/SimJoyStick/DirectInputJoyStick.cpp`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/update_mavlibkcom.bat`
- added `Unreal/CarlaUE4/Plugins/AirSim/Source/MavLinkCom/`
- added `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/lib/`
- added `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/AirSim.props`

Purpose:

- restore AirSim 1.8.1 Windows-side build dependencies
- fix Windows behavior around joystick, world sim, and plugin compilation
- make AirSim coexist with the Windows packaging workflow

### 4.5 Carla Plugin Compatibility Work

Main files:

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Carla.Build.cs`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/SceneCaptureSensor.h`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/SceneCaptureSensor.cpp`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/DepthCamera_WideAngleLens.cpp`
- `Unreal/CarlaUE4/Config/DefaultEngine.ini`

This layer is especially fragile because:

- CarlaAir assumes `CARLA fork UE4.26`
- incorrect Carla plugin assets can break `cook`

Critical lesson:

- Do not copy `Plugins/Carla/Content/PostProcessingMaterials` from the old `v0.1.6` Linux runtime package into a Windows build tree.
- Those assets are cooked / unversioned runtime artifacts and they can break Windows `Package` during shader / cook stages.
- Use the Carla plugin source assets that match `CARLA 0.9.16`.

## 5. Main Local Change Surface

At handoff time, the main local changes in `git status` are grouped under:

- `LibCarla/source/carla/rpc/Server.h`
- `LibCarla/source/compiler/*`
- `Unreal/CarlaUE4/Config/DefaultEngine.ini`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/*`
- `Unreal/CarlaUE4/Plugins/Carla/Source/*`
- `Unreal/CarlaUE4/Source/*.Target.cs`
- `Util/BuildTools/*.bat`
- `Util/InstallersWin/*.bat`
- `examples_record_demo/*.py`

New files or directories include:

- `AirSimConfig/`
- `BuildWindows.ps1`
- `CarlaAir.ps1`
- `env_setup/SetupEnv.ps1`
- `env_setup/TestEnv.ps1`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/AirSim.props`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/lib/`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/MavLinkCom/`
- `Util/BuildTools/Bootstrap.bat`

## 6. Local Environment Assumptions

Required:

- Windows 11 x86_64
- NVIDIA discrete GPU
- Visual Studio 2022 Desktop C++
- CMake
- Git
- Miniconda or Anaconda
- Python 3.10
- `CARLA fork UE4.26`

Known local path assumptions:

- `UE4_ROOT = D:\Code\CarlaAir\UE4`
- source tree: `D:\Code\CarlaAir\CarlaAir-v0.1.7`
- runtime release: `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`

Conda convention:

- environment name: `carlaAir`

The Windows scripts attempt to discover `conda.exe` through:

- `CONDA_EXE`
- `PATH`
- `D:\Code\Anaconda3\Scripts\conda.exe`
- `%LOCALAPPDATA%\\miniconda3\\Scripts\\conda.exe`
- `%LOCALAPPDATA%\\anaconda3\\Scripts\\conda.exe`
- `%USERPROFILE%\\miniconda3\\Scripts\\conda.exe`
- `%USERPROFILE%\\anaconda3\\Scripts\\conda.exe`
- `C:\ProgramData\miniconda3\Scripts\conda.exe`
- `C:\ProgramData\anaconda3\Scripts\conda.exe`

## 7. Correct Rebuild Sequence

If you need to fully rebuild the Windows port on the current `v0.1.7` tree, use this order:

1. Prepare `UE4_ROOT`
2. Restore official `Content/Carla` assets
3. Restore AirSim 1.8.1 dependencies
4. Run the Windows build pipeline
5. Configure Python
6. Run validation
7. Assemble the runtime release directory

Build sequence:

```powershell
cd D:\Code\CarlaAir\CarlaAir-v0.1.7
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -Setup
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildLibCarla
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildOSM2ODR
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildPythonApi
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildUE4
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -Package
```

Environment and runtime:

```powershell
powershell -ExecutionPolicy Bypass -File .\env_setup\SetupEnv.ps1 -EnvName carlaAir
powershell -ExecutionPolicy Bypass -File .\env_setup\TestEnv.ps1 -EnvName carlaAir
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

## 8. Fastest Strategy for Future CarlaAir Upgrades

When CarlaAir updates to a new upstream version:

1. unpack the new version into a new directory
2. keep `UE4_ROOT` pointing at the validated `D:\Code\CarlaAir\UE4`
3. use the current `v0.1.7` Windows worktree as the patch source
4. migrate in layers instead of copying the whole tree
5. restore assets and AirSim dependencies before rebuilding
6. only then do Windows compilation and validation

Recommended migration order:

1. Windows entry scripts
2. build-system patches
3. LibCarla / AirSim / Carla source compatibility patches
4. runtime helper assets and demo script adjustments
5. official content assets and AirSim dependencies
6. build and validate

Do not:

- overwrite the new upstream tree with the entire old tree
- use the old Linux package as the Windows build base
- copy cooked plugin assets from `v0.1.6` into the new tree

## 9. Common Failure Points

### 9.1 Wrong UE4

Wrong:

- Epic stock UE4.26

Correct:

- `CARLA fork UE4.26`

### 9.2 Wrong Plugin Materials

Wrong:

- copying `PostProcessingMaterials` from the old Linux runtime package

Consequence:

- shader / cook failure during `Package`

Correct:

- use Carla plugin source assets matching `CARLA 0.9.16`

### 9.3 Treating the Release Directory as a Build Tree

Wrong:

- compiling inside `CarlaAir-v0.1.7-Windows11-x86_64`

Correct:

- build from the source tree
- run from the release directory

### 9.4 Missing AirSim Dependency Restoration

If you only copy CarlaAir source but do not restore:

- `MavLinkCom`
- `AirLib/lib`
- `AirSim.props`
- related AirSim dependency content

then the AirSim plugin is likely to fail on Windows.

## 10. Current Runtime Entry

Final runtime release directory:

- `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`

Common entrypoints:

```bat
StartCarlaAir.bat
SetupEnv.bat
TestEnv.bat
StopCarlaAir.bat
```

Direct PowerShell entry:

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD
```

## 11. Current Cleanup State

To save disk space, the following were already cleaned locally:

- old `Build\UE4Carla` package directories inside the source tree
- most `Intermediate` directories
- `DerivedDataCache`
- logs and temporary validation artifacts
- the temporary `_vendor/AirSim-1.8.1` directory

If a future rebuild needs full packaging again, some intermediates will be regenerated. That is expected.

## 12. Recommended First Steps for a New Agent

Before changing anything:

1. read this file
2. verify these directories exist:
   - `D:\Code\CarlaAir\CarlaAir-v0.1.7`
   - `D:\Code\CarlaAir\UE4`
   - `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`
3. inspect `git status`
4. inspect `BuildWindows.ps1`, `CarlaAir.ps1`, `env_setup/SetupEnv.ps1`, and `env_setup/TestEnv.ps1`
5. only start builds after confirming `UE4_ROOT`, assets, and AirSim dependencies are correct

## 13. Prompt for a New Agent

```text
You are taking over a CarlaAir Windows porting workspace.

Workspace:
- D:\Code\CarlaAir\CarlaAir-v0.1.7 : current Windows-adapted source tree
- D:\Code\CarlaAir\UE4 : validated CARLA fork UE4.26, used as UE4_ROOT
- D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64 : final standalone Windows release directory

Baseline:
- CarlaAir v0.1.7
- CARLA 0.9.16
- AirSim 1.8.1
- Windows 11 x86_64
- current source commit: 7a7fdb33c15a31fbcc55040d9aaf42344ce9bfde

Important facts:
- The Windows port has already been validated end-to-end: BuildUE4, Package, native runtime startup, CARLA/AirSim connectivity, auto_traffic, record_vehicle, record_walker, record_drone, demo_director, MP4 export, and v0.1.6 JSON compatibility.
- Do not use Epic stock UE4.26. Use CARLA fork UE4.26.
- Do not copy cooked PostProcessingMaterials from the v0.1.6 Linux package into a Windows build tree.
- The release directory is for runtime, not for rebuilding.

Please start by:
1. reading D:\Code\CarlaAir\WINDOWS_PORTING_HANDOFF_CN.md
2. checking git status in D:\Code\CarlaAir\CarlaAir-v0.1.7
3. reviewing BuildWindows.ps1, CarlaAir.ps1, env_setup/SetupEnv.ps1, and env_setup/TestEnv.ps1
4. if the task is upgrading to a newer CarlaAir version, use the current v0.1.7 Windows worktree as the patch baseline and migrate in this order: entry scripts -> build-system patches -> source compatibility -> assets/dependencies -> build/validation

Your goal is not to reinvent the Windows pipeline. Your goal is to reuse the validated Windows adaptations with the minimum necessary delta to get the newer CarlaAir version compiling, packaging, and running again.
```

## 14. Final Summary

Two facts matter most:

- the Windows port is already real and validated
- future upgrades should start from this `v0.1.7` Windows worktree as the known-good patch baseline, not from scratch
