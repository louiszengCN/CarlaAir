<p align="center">
  <img src="logo_upload.png" alt="CarlaAir Logo" width="120"/>
</p>

<h1 align="center">CarlaAir v0.1.7</h1>

<p align="center">
  <b>Fly drones inside a CARLA world.</b><br/>
  在 CARLA 世界里飞无人机。
</p>

<p align="center">
  <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7"><img src="https://img.shields.io/badge/version-v0.1.7-blue" alt="Version"/></a>
  <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/>
  <img src="https://img.shields.io/badge/python-3.8+-blue" alt="Python 3.8+"/>
  <img src="https://img.shields.io/badge/CARLA-0.9.16-green" alt="CARLA 0.9.16"/>
  <img src="https://img.shields.io/badge/AirSim-1.8.1-orange" alt="AirSim 1.8.1"/>
  <img src="https://img.shields.io/badge/platform-Ubuntu%2020.04%20%7C%2022.04-lightgrey" alt="Platform"/>
</p>

<p align="center">
  <a href="README.md">English README</a> | <a href="README_CN.md">简体中文 README</a>
</p>

---

## 📋 Overview / 概述

**CarlaAir v0.1.7** focuses on **stability, developer experience, and recording tools**. This release resolves long-standing rendering and traffic issues, introduces a one-click environment setup, and ships a complete drone recording and demo playback toolkit.

**CarlaAir v0.1.7** 聚焦于**稳定性、开发者体验与录制工具链**。本次发布修复了长期存在的渲染与交通问题，引入一键环境配置，并提供完整的无人机录制与演示回放工具包。

---

## 🔥 What's New / 更新内容

### 🐛 Bug Fixes / 问题修复

#### Fullscreen VSync Tearing / 全屏 VSync 撕裂修复

Enabled VSync and frame rate smoothing (60 fps cap). Fixed `FullscreenMode` config and added the `-UseVSync` launch parameter. Screen tearing in fullscreen mode is now resolved.

启用 VSync 与帧率平滑（60fps 上限），修复 `FullscreenMode` 配置并添加 `-UseVSync` 启动参数。全屏模式下的画面撕裂问题已彻底解决。

#### Traffic Stability / 交通系统稳定性

Pedestrians and vehicles no longer freeze after extended simulation runs. Added 10-second health checks: stalled walkers receive new random destinations, and frozen vehicles automatically re-enable autopilot.

行人与车辆在长时间仿真后不再冻结。新增 10 秒健康检查机制：卡住的行人会获得新随机目的地，冻结的车辆会自动重新启用自动驾驶。

#### Launcher Portability / 启动器可移植性

`CarlaAir.sh` no longer hardcodes conda paths. It now uses the current `python3` in `$PATH` and prints a clear error message if the `carla` module is not found.

`CarlaAir.sh` 不再硬编码 conda 路径。现在使用 `$PATH` 中的 `python3`，如果找不到 `carla` 模块则输出清晰的错误信息。

#### I Key Camera Toggle / I 键视角切换修复

Pressing `I` now correctly cycles between first-person and default drone view. Previously, the camera would get stuck in first-person mode with no way to return.

按下 `I` 键现在可以正确地在第一人称和默认无人机视角之间切换。此前相机会卡在第一人称视角无法返回。

---

### ✨ New Features / 新功能

#### One-Click Environment Setup / 一键环境配置

`env_setup/setup_env.sh` — Creates a `carlaAir` conda environment, installs all dependencies, and deploys the pre-built `carla` Python module from `carla_python_module.tar.gz` (~35 MB). This solves the notorious ABI mismatch between pip-installed `carla` and the custom-built CarlaAir server.

`env_setup/setup_env.sh` — 创建 `carlaAir` conda 环境，安装所有依赖，并从 `carla_python_module.tar.gz`（约 35 MB）部署预编译的 `carla` Python 模块。彻底解决了 pip 安装的 `carla` 与 CarlaAir 定制服务器之间的 ABI 不匹配问题。

#### Environment Auto-Test / 环境自动检测

`env_setup/test_env.sh` — Validates Python module imports (`carla`, `airsim`, `pygame`, `numpy`) and server connectivity (CARLA port 2000, AirSim port 41451). Outputs a clear PASS / FAIL / WARN report.

`env_setup/test_env.sh` — 验证 Python 模块导入（`carla`、`airsim`、`pygame`、`numpy`）及服务器连接（CARLA 端口 2000、AirSim 端口 41451），输出清晰的 PASS / FAIL / WARN 报告。

#### Drone Recording Toolkit / 无人机录制工具

`record_drone.py` — Terminal-based drone trajectory recorder that reads position data from the CARLA side only, with zero AirSim intrusion (no control loss, no crashes). Supports multi-segment recording with ghost vehicle loop and playback controls.

`record_drone.py` — 基于终端的无人机轨迹录制工具，仅从 CARLA 侧读取位置数据，完全不接触 AirSim（不丢失控制、不坠机）。支持多段录制、幽灵车循环与回放控制。

#### Demo Director / 演示导演

`demo_director.py` — Replays vehicle, drone, and walker trajectories simultaneously with free-fly camera, weather presets, sensor overlay panels, and MP4 recording output.

`demo_director.py` — 同时回放车辆、无人机和行人轨迹，支持自由飞行相机、天气预设、传感器叠加面板以及 MP4 录制输出。

#### Coordinate System Documentation / 坐标系文档

`examples_record_demo/COORDINATE_SYSTEMS.md` — Complete CARLA-to-AirSim coordinate conversion reference, including calibration code, per-script coordinate table, and the exact offset formula for Town10HD.

`examples_record_demo/COORDINATE_SYSTEMS.md` — 完整的 CARLA↔AirSim 坐标转换参考，包含标定代码、各脚本坐标表以及 Town10HD 精确偏移公式。

---

### 🎮 Help Overlay Updates (H Key) / 帮助菜单更新

| Key / 按键 | Description / 说明 |
|---|---|
| `1` / `2` / `3` | Sensor view selection / 传感器视图切换 |
| `I` | First-person toggle (documented) / 第一人称切换（已标注） |
| `B` | FPV yaw mode (expert-only warning) / FPV 偏航模式（标注为专家功能） |

Version string bumped to **v0.1.7** in the overlay.

帮助菜单版本号已更新为 **v0.1.7**。

---

## ⚠️ Breaking Changes / 破坏性变更

**None.** This release is fully backward-compatible with v0.1.6 scripts and configurations.

**无。** 本版本与 v0.1.6 的脚本和配置完全向后兼容。

---

## 🚀 Quick Start / 快速开始

### 1. Download and extract / 下载并解压

```bash
tar xzf CarlaAir-v0.1.7.tar.gz
cd CarlaAir-v0.1.7
```

### 2. Set up the environment (first time only) / 配置环境（仅首次）

```bash
bash env_setup/setup_env.sh
```

This will:
- Create a `carlaAir` conda environment (Python 3.10)
- Install `pygame`, `airsim`, `numpy`, `Pillow`
- Remove any incompatible pip-installed `carla` package
- Deploy the pre-built CarlaAir `carla` module

完成以下操作：
- 创建 `carlaAir` conda 环境（Python 3.10）
- 安装 `pygame`、`airsim`、`numpy`、`Pillow`
- 移除不兼容的 pip 安装的 `carla` 包
- 部署预编译的 CarlaAir `carla` 模块

### 3. Verify the environment / 验证环境

```bash
bash env_setup/test_env.sh
```

### 4. Launch the simulator / 启动仿真器

```bash
conda activate carlaAir
./CarlaAir.sh Town10HD
```

### 5. Test Dual API / 测试双 API

```bash
# In another terminal / 在另一个终端
conda activate carlaAir
python3 -c "import carla; c=carla.Client('localhost',2000); print(c.get_world().get_map().name)"
python3 -c "import airsim; c=airsim.MultirotorClient(port=41451); c.confirmConnection()"
```

---

## ⬆️ Upgrade Guide / 升级指南

### From v0.1.6 / 从 v0.1.6 升级

1. **Download** the v0.1.7 release tarball.
2. **Extract** to a new directory (do not overwrite v0.1.6 in-place).
3. **Re-run environment setup** to update the `carla` Python module:
   ```bash
   cd CarlaAir-v0.1.7
   bash env_setup/setup_env.sh
   ```
4. **Verify** with `bash env_setup/test_env.sh`.
5. **Copy** any custom scripts or JSON recordings from your v0.1.6 directory — they are fully compatible.

### 从 v0.1.6 升级步骤

1. **下载** v0.1.7 发布包。
2. **解压** 到新目录（不要覆盖 v0.1.6）。
3. **重新运行环境配置** 以更新 `carla` Python 模块：
   ```bash
   cd CarlaAir-v0.1.7
   bash env_setup/setup_env.sh
   ```
4. 运行 `bash env_setup/test_env.sh` **验证**环境。
5. 将自定义脚本或 JSON 录制文件从 v0.1.6 目录**复制**过来——完全兼容。

---

## 🔎 Known Issues / 已知问题

| Issue / 问题 | Status / 状态 | Workaround / 临时方案 |
|---|---|---|
| Coordinate offsets are map-specific (calibrated for Town10HD) | By design | Re-calibrate using the code in `COORDINATE_SYSTEMS.md` when switching maps / 切换地图时使用文档中的标定代码重新校准 |
| Connecting an AirSim Python client causes the drone to lose keyboard control | Upstream AirSim behavior | Use `record_drone.py` (CARLA-side only) for recording / 使用 `record_drone.py`（仅 CARLA 侧）进行录制 |
| `cv2` (OpenCV) not installed by default | Optional | `pip install opencv-python` if you need MP4 recording in `demo_director.py` / 如需 `demo_director.py` 的 MP4 录制功能请手动安装 |

---

## 📦 Assets / 附件

| File / 文件 | Description / 说明 |
|---|---|
| `CarlaAir-v0.1.7.tar.gz` | Full binary release (Linux x86_64) / 完整二进制发布包 |
| `env_setup/carla_python_module.tar.gz` | Pre-built CARLA Python module (~35 MB) / 预编译 CARLA Python 模块 |

---

## 🙏 Acknowledgments / 致谢

CarlaAir is built on [CARLA](https://carla.org/) and [AirSim](https://github.com/microsoft/AirSim). We thank both communities for their foundational work in open-source simulation.

CarlaAir 基于 [CARLA](https://carla.org/) 和 [AirSim](https://github.com/microsoft/AirSim) 构建。感谢两个社区在开源仿真领域的基础性工作。

---

<p align="center">
  <b>Full Changelog:</b> <a href="CHANGELOG.md">CHANGELOG.md</a> &nbsp;|&nbsp;
  <b>完整变更日志：</b> <a href="CHANGELOG.md">CHANGELOG.md</a>
</p>
