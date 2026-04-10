# CarlaAir Windows 适配交接文档

本文档面向两类用途：

1. 作为当前 `CarlaAir` Windows 版本的二次开发参考。
2. 作为新 agent 的交接上下文，用于在 `CarlaAir` 升级后快速恢复和更新 Windows 版源码。

## 1. 当前基线

- 工作区根目录：`D:\Code\CarlaAir`
- 当前上游源码目录：`D:\Code\CarlaAir\CarlaAir-v0.1.7`
- 当前 UE4 根目录：`D:\Code\CarlaAir\UE4`
- 当前整理好的 Windows 发行目录：`D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`
- 旧 Linux 二进制包：`D:\Code\CarlaAir\CarlaAir-v0.1.6`
- 当前源码基线 commit：`7a7fdb33c15a31fbcc55040d9aaf42344ce9bfde`

版本关系：

- CarlaAir 基线：`v0.1.7`
- CARLA 基线：`0.9.16`
- AirSim 基线：`1.8.1`
- UE4 基线：`CARLA fork UE4.26`

必须强调：

- Windows 移植成功依赖的是 `CARLA fork UE4.26`，不是 Epic Launcher 官方 UE4.26。
- 当前已经验证可用的是本地 `D:\Code\CarlaAir\UE4` 这棵源码引擎树。
- 当前源码树和发行目录已经分离。源码树里原始 `Build\UE4Carla\...` 打包目录已在清理时删除，最终可运行包以独立发行目录为准。

## 2. 当前目录角色

### 2.1 源码树

- `D:\Code\CarlaAir\CarlaAir-v0.1.7`
  用于继续开发、重新编译、升级到新版本、重新打包。

### 2.2 UE4 工具链树

- `D:\Code\CarlaAir\UE4`
  已补齐为可用的 `UE4_ROOT`。

### 2.3 Windows 发行目录

- `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`
  这是最终运行版，不是开发树。

发行目录内保留了：

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

### 2.4 旧版 Linux 包

- `D:\Code\CarlaAir\CarlaAir-v0.1.6`
  仅作为历史运行包或兼容参考，不再作为 Windows 开发底座。

## 3. 当前已验证的能力

以下能力已经在 Windows 11 x86_64 本机上完成闭环验证：

- `BuildUE4` 成功
- `Package` 成功
- 原生 Windows 启动成功
- `CARLA` 端口 `2000` 可连通
- `AirSim` 端口 `41451` 可连通
- `carla.Client(...)` 可连接并读取地图
- `airsim.MultirotorClient(...)` 可连接
- `auto_traffic.py` 可启动
- `record_vehicle.py` 可生成轨迹 JSON
- `record_walker.py` 可生成轨迹 JSON
- `record_drone.py` 可生成轨迹 JSON
- `demo_director.py` 可加载三类 JSON 并输出 MP4
- `v0.1.6` 旧格式轨迹 JSON 可被 `v0.1.7` 导演脚本加载

结论：

- 当前 Windows 移植目标已经完成，不只是“能编译”，而是“能运行、能录制、能导出视频”。

## 4. Windows 适配的关键差异

Windows 适配不是简单换脚本，而是同时改了 4 层：

1. 构建链
2. Python 环境链
3. UE4/CARLA/AirSim 源码兼容
4. 运行和示例验证入口

### 4.1 新增的 Windows 入口文件

以下文件是 Windows 适配的显式入口：

- `BuildWindows.ps1`
- `CarlaAir.ps1`
- `env_setup/SetupEnv.ps1`
- `env_setup/TestEnv.ps1`
- 发行目录中的 `.bat` 包装器

这些文件的职责：

- `BuildWindows.ps1`
  统一调用 Windows 构建阶段，支持 `-Setup`、`-BuildLibCarla`、`-BuildOSM2ODR`、`-BuildPythonApi`、`-BuildUE4`、`-Package`
- `CarlaAir.ps1`
  统一启动 `WindowsNoEditor` 包、写入 AirSim 配置、等待端口、可选拉起 `auto_traffic.py`
- `SetupEnv.ps1`
  创建和配置 `carlaAir` Conda 环境，并安装 Win64 `carla` wheel
- `TestEnv.ps1`
  检查 Python 模块、端口和基本运行条件

### 4.2 构建链修改的重点

改动主要集中在这些目录：

- `Util/BuildTools/*.bat`
- `Util/InstallersWin/*.bat`
- `Unreal/CarlaUE4/Source/*.Target.cs`

构建链修改目标：

- 让 `VS2022 + Windows + CARLA fork UE4.26` 真正可用
- 修正批处理参数传递、返回码传播、路径解析
- 保证 `Package.bat` 能跑完 `cook/stage/package`
- 避免脚本错误地依赖 Linux 产物或官方 Epic UE4.26

### 4.3 LibCarla / 编译器兼容修改

改动集中在：

- `LibCarla/source/compiler/disable-ue4-macros.h`
- `LibCarla/source/compiler/enable-ue4-macros.h`
- `LibCarla/source/carla/rpc/Server.h`

主要目的：

- 处理 Windows/MSVC 下的宏污染和头文件冲突
- 避免 UE4 宏与 Boost/标准库符号冲突

### 4.4 AirSim 兼容修改

改动集中在：

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
- 新增 `Unreal/CarlaUE4/Plugins/AirSim/Source/MavLinkCom/`
- 新增 `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/lib/`
- 新增 `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/AirSim.props`

主要目的：

- 补齐 AirSim 1.8.1 的 Windows 编译依赖
- 修正 Windows 下 joystick、world sim 和构建行为
- 让 AirSim 插件能在当前 CarlaAir 工程中与 Windows 打包链共同工作

### 4.5 Carla 插件兼容修改

改动集中在：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Carla.Build.cs`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/SceneCaptureSensor.h`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/SceneCaptureSensor.cpp`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/DepthCamera_WideAngleLens.cpp`
- `Unreal/CarlaUE4/Config/DefaultEngine.ini`

这里是最脆弱的一层，原因有两个：

- CarlaAir 依赖的是 `CARLA fork UE4.26`
- 不正确的插件材质或旧 cooked 资产会导致 `cook` 崩溃

历史经验必须记住：

- 不要从 `v0.1.6` Linux 运行包里直接拿 `Plugins/Carla/Content/PostProcessingMaterials` 作为 Windows 构建资产。
- 旧包里的这批资产是 cooked/unversioned 产物，会导致 `Package` 在 shader/cook 阶段崩溃。
- 这里最终应以 `CARLA 0.9.16` 官方源码对应的 Carla 插件资产为准。

## 5. 当前工作副本中的主要改动面

截至本次交接，`git status` 显示的本地改动主要包括：

### 5.1 修改文件

- `LibCarla/source/carla/rpc/Server.h`
- `LibCarla/source/compiler/disable-ue4-macros.h`
- `LibCarla/source/compiler/enable-ue4-macros.h`
- `Unreal/CarlaUE4/Config/DefaultEngine.ini`
- `Unreal/CarlaUE4/Plugins/AirSim/AirSim.uplugin`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/...`
- `Unreal/CarlaUE4/Plugins/Carla/Source/...`
- `Unreal/CarlaUE4/Source/CarlaUE4.Target.cs`
- `Unreal/CarlaUE4/Source/CarlaUE4Editor.Target.cs`
- `Util/BuildTools/*.bat`
- `Util/InstallersWin/*.bat`
- `examples_record_demo/*.py`

### 5.2 新增文件或目录

- `AirSimConfig/`
- `BuildWindows.ps1`
- `CarlaAir.ps1`
- `env_setup/SetupEnv.ps1`
- `env_setup/TestEnv.ps1`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/AirSim.props`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/lib/`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/MavLinkCom/`
- `Util/BuildTools/Bootstrap.bat`

## 6. 当前本地依赖前提

### 6.1 必须满足

- Windows 11 x86_64
- NVIDIA 独立 GPU
- Visual Studio 2022 Desktop C++
- CMake
- Git
- Miniconda 或 Anaconda
- Python 3.10
- `CARLA fork UE4.26`

### 6.2 当前已知本机路径

- `UE4_ROOT = D:\Code\CarlaAir\UE4`
- 常用源码树：`D:\Code\CarlaAir\CarlaAir-v0.1.7`
- 发行目录：`D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`

### 6.3 Conda 环境约定

- 环境名：`carlaAir`
- `SetupEnv.ps1` 和 `CarlaAir.ps1` 都会优先尝试自动发现 `conda.exe`
- 兼容这些常见位置：
  - `CONDA_EXE`
  - `PATH` 中的 `conda`
  - `D:\Code\Anaconda3\Scripts\conda.exe`
  - `%LOCALAPPDATA%\\miniconda3\\Scripts\\conda.exe`
  - `%LOCALAPPDATA%\\anaconda3\\Scripts\\conda.exe`
  - `%USERPROFILE%\\miniconda3\\Scripts\\conda.exe`
  - `%USERPROFILE%\\anaconda3\\Scripts\\conda.exe`
  - `C:\ProgramData\miniconda3\Scripts\conda.exe`
  - `C:\ProgramData\anaconda3\Scripts\conda.exe`

## 7. 从头重建 Windows 版的正确顺序

如果需要在当前 `v0.1.7` 上完整重建 Windows 版，顺序如下：

1. 准备 `UE4_ROOT`
2. 准备 `Content/Carla` 官方资产
3. 准备 AirSim 1.8.1 依赖
4. 跑 Windows 构建链
5. 配置 Python 环境
6. 启动并做功能验证
7. 整理发行目录

标准命令序列：

```powershell
cd D:\Code\CarlaAir\CarlaAir-v0.1.7
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -Setup
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildLibCarla
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildOSM2ODR
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildPythonApi
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -BuildUE4
powershell -ExecutionPolicy Bypass -File .\BuildWindows.ps1 -Package
```

环境与启动：

```powershell
powershell -ExecutionPolicy Bypass -File .\env_setup\SetupEnv.ps1 -EnvName carlaAir
powershell -ExecutionPolicy Bypass -File .\env_setup\TestEnv.ps1 -EnvName carlaAir
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

## 8. CarlaAir 升级到新版本时的最短更新策略

如果未来 CarlaAir 升级到 `vNext`，不要直接覆盖当前 `v0.1.7` 工作树。正确做法是：

1. 下载新版本源码到新目录
2. 保持 `UE4_ROOT` 仍然指向可用的 `CARLA fork UE4.26`
3. 以当前 `v0.1.7` Windows 工作树为“补丁来源”
4. 按类别迁移，而不是整目录覆盖
5. 先恢复内容资产和 AirSim 依赖，再编译
6. 只在新版本源码树上做新的 Windows 适配

### 8.1 推荐迁移顺序

第一步，迁移脚本和构建入口：

- `BuildWindows.ps1`
- `CarlaAir.ps1`
- `env_setup/SetupEnv.ps1`
- `env_setup/TestEnv.ps1`
- 必要的 `.bat` 构建和安装器修改

第二步，迁移构建系统补丁：

- `Util/BuildTools/*.bat`
- `Util/InstallersWin/*.bat`
- `*.Target.cs`

第三步，迁移源码兼容补丁：

- `LibCarla/source/compiler/*`
- `LibCarla/source/carla/rpc/Server.h`
- `Unreal/CarlaUE4/Plugins/AirSim/Source/*`
- `Unreal/CarlaUE4/Plugins/Carla/Source/*`

第四步，迁移运行期辅助内容：

- `AirSimConfig/`
- `examples_record_demo/*.py` 中的自动化验证增强

第五步，再补内容资源：

- `Content/Carla` 官方资产
- AirSim 1.8.1 对应依赖

### 8.2 不建议的升级方式

- 不要把旧版整个源码树直接覆盖到新版
- 不要把旧版 Linux 二进制包当作新版 Windows 构建底座
- 不要把 `v0.1.6` 里的 cooked 插件材质直接复制到新版

## 9. 新版升级时最容易踩坑的点

### 9.1 UE4 用错

错误做法：

- 使用 Epic Launcher 官方 UE4.26

正确做法：

- 使用 `CARLA fork UE4.26`

### 9.2 插件材质用错

错误做法：

- 从旧 Linux 包复制 `PostProcessingMaterials`

后果：

- `Package` 在 shader/cook 阶段崩溃

正确做法：

- 使用与 `CARLA 0.9.16` 对应的 Carla 插件源码资产

### 9.3 把发行目录当源码树

错误做法：

- 在 `CarlaAir-v0.1.7-Windows11-x86_64` 里继续编译

正确做法：

- 发行目录只负责运行
- 编译必须回到源码树

### 9.4 忽略 AirSim 依赖

如果只复制 CarlaAir 源码而不恢复：

- `MavLinkCom`
- `AirLib/lib`
- `AirSim.props`
- 相关 `deps`

则 AirSim 插件很容易在 Windows 下编不过。

## 10. 当前发行版使用方式

当前最终发行目录：

- `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`

最常用入口：

```bat
StartCarlaAir.bat
SetupEnv.bat
TestEnv.bat
StopCarlaAir.bat
```

如果需要直接调用 PowerShell：

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD
```

## 11. 当前清理状态

为了节省空间，以下内容已经在本地清理：

- 源码树里的旧 `Build\UE4Carla` 打包目录
- 各类 `Intermediate`
- `DerivedDataCache`
- 大部分日志和临时验证产物
- 临时 `_vendor/AirSim-1.8.1` 目录

因此如果未来需要再次完整 `Package`，会重新生成部分中间文件，这是正常现象。

## 12. 给新 agent 的建议工作方式

新 agent 接手时，优先做这几件事：

1. 先读本文件
2. 确认这 3 个目录是否存在：
   - `D:\Code\CarlaAir\CarlaAir-v0.1.7`
   - `D:\Code\CarlaAir\UE4`
   - `D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64`
3. 先看 `git status`
4. 先检查 `BuildWindows.ps1`、`CarlaAir.ps1`、`env_setup/*.ps1`
5. 只有确认 `UE4_ROOT`、资产、AirSim 依赖都对了，再跑构建

## 13. 可直接交给新 agent 的 Prompt

下面这段可以直接作为新 agent 的起始 prompt：

```text
你现在接手的是 CarlaAir 的 Windows 适配工作区。

工作区根目录：
- D:\Code\CarlaAir\CarlaAir-v0.1.7：当前 Windows 适配源码树
- D:\Code\CarlaAir\UE4：当前可用的 CARLA fork UE4.26，作为 UE4_ROOT
- D:\Code\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64：当前整理好的最终 Windows 发行目录

当前基线：
- CarlaAir v0.1.7
- CARLA 0.9.16
- AirSim 1.8.1
- Windows 11 x86_64
- 当前源码 commit：7a7fdb33c15a31fbcc55040d9aaf42344ce9bfde

重要事实：
- Windows 版已经完成过端到端验证：BuildUE4、Package、原生启动、CARLA/AirSim 连通、auto_traffic、record_vehicle、record_walker、record_drone、demo_director、MP4 输出、v0.1.6 JSON 兼容。
- 不能用 Epic 官方 UE4.26，必须用 CARLA fork UE4.26。
- 不能把 v0.1.6 Linux 包里的 cooked PostProcessingMaterials 直接拿来用于 Windows 构建。
- 发行目录是运行版，不是源码树。

请先做：
1. 阅读 D:\Code\CarlaAir\WINDOWS_PORTING_HANDOFF_CN.md
2. 检查 D:\Code\CarlaAir\CarlaAir-v0.1.7 的 git status
3. 检查 BuildWindows.ps1、CarlaAir.ps1、env_setup/SetupEnv.ps1、env_setup/TestEnv.ps1
4. 如果目标是升级到 CarlaAir 新版本，请以当前 v0.1.7 Windows 工作树为补丁来源，按“脚本入口 -> 构建链 -> 源码兼容 -> 资源资产 -> 验证”的顺序迁移

你的目标不是重新发明构建链，而是基于现有 Windows 适配成果，最小成本把新版本恢复到可编译、可打包、可运行状态。
```

## 14. 最后结论

当前最重要的事实只有两条：

- Windows 版不是理论方案，而是已经成功落地的工作区。
- 未来升级时，最省时间的方式不是重头排障，而是以当前 `v0.1.7` Windows 工作树为补丁基线，有控制地迁移到新版本。
