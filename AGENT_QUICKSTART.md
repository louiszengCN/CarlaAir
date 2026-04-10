# CarlaAir Windows Agent Quickstart

这是给新 agent 的短版启动文档。目标是用最少上下文，快速接手当前 `CarlaAir` 的 Windows 工作区。

## 1. 先记住这 5 件事

1. 当前可用源码树是 `D:\Code\CarlaAir\CarlaAir-v0.1.7`
2. 当前可用 `UE4_ROOT` 是 `D:\Code\CarlaAir\UE4`
3. 当前最终运行版是 `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`
4. 不能用 Epic 官方 UE4.26，必须用 `CARLA fork UE4.26`
5. 不能把 `v0.1.6` Linux 包里的 cooked `PostProcessingMaterials` 直接拿来做 Windows 构建

## 2. 当前基线

- CarlaAir: `v0.1.7`
- CARLA: `0.9.16`
- AirSim: `1.8.1`
- UE4: `CARLA fork UE4.26`
- 当前源码 commit: `7a7fdb33c15a31fbcc55040d9aaf42344ce9bfde`

## 3. 当前已验证通过

- `BuildUE4`
- `Package`
- 原生 Windows 启动
- `2000` / `41451` 端口连通
- `carla` / `airsim` Python 客户端连通
- `auto_traffic.py`
- `record_vehicle.py`
- `record_walker.py`
- `record_drone.py`
- `demo_director.py`
- MP4 输出
- `v0.1.6` 旧 JSON 回放兼容

结论：

- 当前 Windows 版不是“能编译”，而是“已经完整跑通过”。

## 4. 先看哪些文件

优先阅读：

- `D:\Code\CarlaAir\WINDOWS_PORTING_HANDOFF_CN.md`
- `D:\Code\CarlaAir\CarlaAir-v0.1.7\BuildWindows.ps1`
- `D:\Code\CarlaAir\CarlaAir-v0.1.7\CarlaAir.ps1`
- `D:\Code\CarlaAir\CarlaAir-v0.1.7\env_setup\SetupEnv.ps1`
- `D:\Code\CarlaAir\CarlaAir-v0.1.7\env_setup\TestEnv.ps1`

## 5. 当前改动的重点区域

- `Util/BuildTools/*.bat`
- `Util/InstallersWin/*.bat`
- `LibCarla/source/compiler/*`
- `LibCarla/source/carla/rpc/Server.h`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/*`
- `Unreal/CarlaUE4/Plugins/Carla/Source/*`
- `Unreal/CarlaUE4/Source/*.Target.cs`
- `examples_record_demo/*.py`

## 6. 重新构建的标准顺序

```powershell
cd D:\Code\CarlaAir\CarlaAir-v0.1.7
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -Setup
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildLibCarla
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildOSM2ODR
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildPythonApi
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildUE4
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -Package
```

环境和运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\env_setup\SetupEnv.ps1 -EnvName carlaAir
powershell -ExecutionPolicy Bypass -File .\env_setup\TestEnv.ps1 -EnvName carlaAir
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

## 7. 升级到新版本时怎么做

如果 CarlaAir 升级到 `vNext`：

1. 把新版本解压到新目录，不覆盖 `v0.1.7`
2. 保持 `UE4_ROOT` 继续指向 `D:\Code\CarlaAir\UE4`
3. 从当前 `v0.1.7` Windows 工作树迁移补丁，而不是重头排障
4. 迁移顺序固定为：

- Windows 入口脚本
- 构建链补丁
- LibCarla / AirSim / Carla 源码兼容补丁
- 资源和依赖恢复
- 编译和打包验证

## 8. 不要做的事

- 不要把发行目录当源码树
- 不要把 Linux 包当 Windows 构建底座
- 不要把 `v0.1.6` 的 cooked 插件材质直接拷入新版本
- 不要先重写构建链，再去看现有脚本

## 9. 一段最短起始 Prompt

```text
你现在接手 CarlaAir 的 Windows 工作区。

请先阅读：
- D:\Code\CarlaAir\WINDOWS_PORTING_HANDOFF_CN.md

当前关键目录：
- 源码树：D:\Code\CarlaAir\CarlaAir-v0.1.7
- UE4_ROOT：D:\Code\CarlaAir\UE4
- Windows 发行版：D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64

当前基线：
- CarlaAir v0.1.7
- CARLA 0.9.16
- AirSim 1.8.1
- CARLA fork UE4.26
- commit: 7a7fdb33c15a31fbcc55040d9aaf42344ce9bfde

注意：
- 不能用 Epic 官方 UE4.26
- 不能直接使用 v0.1.6 Linux 包里的 cooked PostProcessingMaterials
- 当前 Windows 版已经完整验证通过，目标应是复用现有补丁和流程，而不是重写

请先检查：
1. git status
2. BuildWindows.ps1
3. CarlaAir.ps1
4. env_setup/SetupEnv.ps1
5. env_setup/TestEnv.ps1
```
