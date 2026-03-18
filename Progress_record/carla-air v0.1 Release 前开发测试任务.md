# carla-air v0.1 Release 前开发测试任务

> **文档用途**：本文档供 AI Code Agent 阅读执行，描述 carla-air 项目在 v0.1 正式 release 前需要完成的 Bug 修复与功能测试任务。请按照本文档逐项执行，并按第四节要求格式输出交付物。
>
> **最后更新**：2026-03-10

---

## 一、项目背景

### 1.1 项目简介

**carla-air** 是一个空地一体的联合仿真平台，通过将 CARLA 0.9.16（基于 Unreal Engine 4 的自动驾驶仿真器）与 AirSim（微软无人机仿真器）进行联合编译，使两套系统在同一个 UE4 进程中同时运行，用户可以同时使用 CARLA Python API 和 AirSim Python API。

CARLA 原生不支持无人机，AirSim 原生没有车辆和行人。carla-air 打通了两者，实现了无人机、地面车辆、行人在同一城市场景中的完整共存，支持空地一体仿真、自动驾驶、无人机导航、social navigation、多模态数据生产等任务。

### 1.2 技术架构

- **引擎**：Unreal Engine 4.26，单进程同时运行 CARLA 和 AirSim
- **核心解决方案**：创建统一 GameMode `ASimWorldGameMode`（继承自 `ACarlaGameModeBase`，集成 AirSim 引导逻辑），解决了 UE4 单 World 只允许一个 GameMode 的根本冲突
- **AirSim 接入方式**：`ASimModeBase` 作为普通 Actor 由 GameMode 在 BeginPlay 中 Spawn，不竞争 GameMode 槽位
- **API 端口**：CARLA RPC 端口 `2000`，AirSim 端口 `41451`
- **硬件环境**：NVIDIA RTX A4000（16GB VRAM），Ubuntu Linux

### 1.3 关键路径

| 名称 | 路径 |
|------|------|
| CARLA 源码根目录 | `/mnt/data1/tianle/carla_source/` |
| UE4 4.26 引擎 | `/mnt/data1/tianle/carla_ue4/` |
| 独立包目录 | `/mnt/data1/tianle/carla_source/Dist/CARLA_Shipping_1ae5356-dirty/LinuxNoEditor/` |
| AirSim 配置文件 | `/home/tianle/Documents/AirSim/settings.json` |
| Conda 环境 | `simworld`（已安装 `airsim` 和 `carla` Python 包） |
| 一键启动脚本（源码版） | `/mnt/data1/tianle/carla_source/carlaAir.sh` |
| 一键启动脚本（独立包） | `<独立包目录>/SimWorld.sh` |
| 示例脚本目录 | `/mnt/data1/tianle/carla_source/examples/` |
| 核心应用脚本目录 | `/mnt/data1/tianle/carla_source/test_script/` |
| 已有测试脚本 | `comprehensive_api_test.py`（89 项）、`test_package.py`（17 项） |

### 1.4 编译命令

修复 Bug 后需要重新编译，使用以下命令（**不要使用 `make CarlaUE4Editor`**，会因 `features.h` 交叉编译错误失败）：

```bash
export UE4_ROOT=/mnt/data1/tianle/carla_ue4

# 修改了 CARLA 相关文件后使用（约 290 秒）
${UE4_ROOT}/Engine/Build/BatchFiles/Linux/Build.sh CarlaUE4Editor Linux Development \
  -project="/mnt/data1/tianle/carla_source/Unreal/CarlaUE4/CarlaUE4.uproject" \
  -module=Carla

# 修改了 AirSim 相关文件后使用（约 100 秒）
${UE4_ROOT}/Engine/Build/BatchFiles/Linux/Build.sh CarlaUE4Editor Linux Development \
  -project="/mnt/data1/tianle/carla_source/Unreal/CarlaUE4/CarlaUE4.uproject" \
  -module=AirSim

# Bug 修复后更新独立包
./Util/BuildTools/Package.sh --config=Shipping --no-zip
```

### 1.5 当前功能基线

截至本文档撰写时，89 项自动化测试全部通过，覆盖以下功能：

| 功能分类 | 已验证功能 |
|----------|-----------|
| CARLA 基础 | 连接、14 张地图加载、14 种天气控制、220 个蓝图 |
| CARLA Actor | 41 种车辆、52 种行人、10 种传感器（RGB/深度/语义/LiDAR/雷达/IMU/GNSS 等） |
| CARLA 导航 | OpenDRIVE、路点、拓扑、Traffic Manager、Spectator |
| AirSim 飞行 | 起飞/降落/悬停、8 种飞行控制方式 |
| AirSim 传感器 | RGB（1280×960）、深度、语义分割、红外、GPS、IMU、气压计、磁力计 |
| 联合功能 | 交通+无人机同时运行、飞行中切换天气、双系统传感器同时采集 |

---

## 二、待修复 Bug

### Bug #1：Walker AI Controller 与 AirSim 冲突

| 字段 | 内容 |
|------|------|
| **严重程度** | 高 |
| **现象** | 调用 `walker_controller.go_to_location()` 后，进程触发四元数归一化异常（`length >= 2.0f * epsilon()`）并 segfault 崩溃 |
| **根本原因** | Walker AI Controller 的 UE4 NavMesh 导航与 AirSim 插件的物理计算之间存在底层冲突 |
| **当前临时方案** | 行人只作为静态障碍物存在，不启用 AI Controller |
| **期望修复目标** | `go_to_location()` 可正常调用，行人能够自主导航，不崩溃 |
| **影响范围** | social navigation 场景、行人轨迹采集 |
| **相关代码** | `Plugins/AirSim/Source/SimMode/SimModeBase.cpp`，UE4 NavMesh 相关模块 |

### Bug #2：大量 Autopilot 车辆导致崩溃

| 字段 | 内容 |
|------|------|
| **严重程度** | 高 |
| **现象** | AirSim 运行时，超过约 10 辆 autopilot 车辆会概率性触发与 Bug #1 相同的四元数归一化错误并 segfault |
| **根本原因** | 推测与 Bug #1 同源，大量车辆物理计算与 AirSim 物理引擎产生干扰 |
| **当前临时方案** | 使用 AirSim 时限制 autopilot 车辆数量 ≤ 8 辆 |
| **期望修复目标** | 支持 30+ 辆 autopilot 车辆与 AirSim 同时稳定运行 |
| **影响范围** | 大规模交通仿真、数据生产场景 |

### Bug #3：`simSetCameraPose` 在 Shipping 包中崩溃

| 字段 | 内容 |
|------|------|
| **严重程度** | 中 |
| **现象** | 在 Shipping 独立包中调用 `airsim.MultirotorClient().simSetCameraPose()` 触发 C++ abort，错误同为四元数 epsilon 检查 |
| **根本原因** | AirSim 在 Shipping 模式下的内部 C++ 断言行为与 Development 模式不同 |
| **当前临时方案** | 所有脚本中禁止调用此 API |
| **期望修复目标** | `simSetCameraPose` 在 Shipping 包中可正常调用 |
| **影响范围** | 无人机相机姿态动态调整 |

### Bug #4：快速连续 enable/disable AirSim API 崩溃

| 字段 | 内容 |
|------|------|
| **严重程度** | 低 |
| **现象** | 同一脚本中连续调用 `enableApiControl(True/False)` 可能触发 "Actor not in registry" 崩溃 |
| **当前临时方案** | 两次调用之间添加 `time.sleep(0.5)` |
| **期望修复目标** | 增加内部状态保护，使快速切换不崩溃 |

---

## 三、测试任务清单

### T1：Bug 修复验证

对上述每个 Bug，完成修复后需验证：
- **Bug #1**：生成行人并调用 `go_to_location()`，持续运行 30 秒不崩溃，行人正常移动
- **Bug #2**：AirSim 运行时生成 30 辆 autopilot 车辆，持续运行 60 秒不崩溃
- **Bug #3**：在 Shipping 独立包中调用 `simSetCameraPose()`，不崩溃，相机姿态正确更新
- **Bug #4**：快速连续切换 `enableApiControl`，不崩溃

若某 Bug 无法修复，需记录尝试过的方案和失败原因，并给出建议的 release notes 说明文字。

### T2：ROS 支持测试

CARLA 官方提供 `carla-ros-bridge`，AirSim 官方提供 `airsim_ros_pkgs`，需验证联合编译后两套 ROS 接口均可正常工作。测试分三个层次：

**T2-A：CARLA ROS Bridge 连通性**
安装并启动 `carla-ros-bridge`，连接到 carla-air（端口 2000），验证核心话题正常发布（世界信息、Actor 列表、时钟、传感器图像），并通过 ROS 接口生成车辆和传感器。

**T2-B：AirSim ROS Wrapper 连通性**
安装并启动 `airsim_ros_pkgs`，连接到 carla-air（端口 41451），验证无人机传感器话题正常发布（IMU、GPS、相机图像），并通过 ROS 话题发布指令控制无人机飞行。

**T2-C：双 ROS 接口同时运行**
在同一 ROS master 下同时运行 `carla-ros-bridge` 和 `airsim_ros_pkgs`，验证话题命名空间无冲突，两套接口数据均正常，可同时订阅 CARLA 车辆传感器话题和 AirSim 无人机传感器话题。

### T3：Clean Install Test

在一台**未安装 UE4 和 CARLA/AirSim 源码**的全新 Ubuntu 机器上，仅使用 19GB 独立包（`LinuxNoEditor/` 目录）和 `pip install carla airsim`，验证：
- `SimWorld.sh` 可正常启动
- `test_package.py` 17 项测试全部通过
- `comprehensive_api_test.py` 89 项测试全部通过

### T4：多地图稳定性测试

对 Town01、Town03、Town05 三张地图，各自验证以下功能正常且不崩溃：地图加载、车辆生成、行人生成、天气切换、无人机起飞与降落。

### T5：长时间稳定性测试

持续运行 3 小时，场景为：车辆和行人持续生成/销毁循环，无人机持续悬停飞行。每 30 分钟记录一次 GPU 显存占用（`nvidia-smi`），验证不出现 `VK_ERROR_OUT_OF_DEVICE_MEMORY`，显存占用无持续增长趋势。

---

## 四、交付物要求

### 4.1 Bug 修复报告

对每个 Bug 输出：修复状态（已修复 / 部分修复 / 未修复）、修复方案（修改了哪些文件及内容）、修复后验证结果；若未修复，说明尝试过的方案和失败原因，并给出建议的 Known Issues 说明文字。

### 4.2 测试结果汇总表

```markdown
| 任务 | 测试项 | 结果 | 备注 |
|------|--------|------|------|
| T1-A | Walker AI Controller 修复验证 | PASS / FAIL | |
| T1-B | Autopilot 车辆压力测试（最大稳定数量：X 辆） | PASS / FAIL | |
| T1-C | simSetCameraPose 修复验证 | PASS / FAIL | |
| T1-D | 快速 enable/disable 修复验证 | PASS / FAIL | |
| T2-A | CARLA ROS Bridge 连通性 | PASS / FAIL | |
| T2-B | AirSim ROS Wrapper 连通性 | PASS / FAIL | |
| T2-C | 双 ROS 接口同时运行 | PASS / FAIL | |
| T3   | Clean Install Test（17+89 项） | PASS / FAIL | |
| T4   | 多地图稳定性（Town01/03/05） | PASS / FAIL | |
| T5   | 长时间稳定性（3 小时） | PASS / FAIL | |
```

### 4.3 Release Readiness 判定

在测试报告末尾给出判定，格式如下：

```markdown
## Release Readiness 判定

**Hard Requirements 完成情况**（以下全部通过方可 release）：
- [ ] T2-A：CARLA ROS Bridge 连通性
- [ ] T2-B：AirSim ROS Wrapper 连通性
- [ ] T2-C：双 ROS 接口同时运行
- [ ] T3：Clean Install Test
- [ ] T4：多地图稳定性

**当前判定**：Ready for Release / Not Ready（列出未通过项）

**随 release 发布的 Known Issues**：
（列出未修复的 Bug 及其临时解决方案，供写入 release notes）
```

### 4.4 更新后的开发进度记录

将本次工作结果追加到 `/mnt/data1/tianle/carla_source/Progress_record/开发进度记录.md`，包括：新增的开发时间线条目、更新后的功能状态总览、更新后的已知限制列表。

---

## 五、注意事项

| 注意事项 | 说明 |
|----------|------|
| 编译方式 | 不要使用 `make CarlaUE4Editor`，使用第一节中的分模块编译命令 |
| Bug #1/#2 测试风险 | 测试前备份状态，崩溃后重启 `SimWorld.sh` 即可恢复 |
| `simSetCameraPose` | 修复前禁止在 Shipping 包中调用，会触发 C++ abort |
| AirSim API 切换 | 两次 `enableApiControl` 调用之间至少等待 0.5 秒 |
| 地图切换 | 切换地图后等待 5 秒再执行 API 调用 |
| Python 环境 | 所有脚本在 `conda activate simworld` 后运行 |
