<table>
  <tr>
    <td width="160" valign="middle" align="center">
      <img src="logo_upload.png" alt="CarlaAir Logo" width="140"/>
    </td>
    <td valign="middle" align="left">
      <h1>CarlaAir</h1>
      <p><b>在 CARLA 世界里飞无人机。</b><br/>
      将 CARLA 与 AirSim 融合为一——地面车辆与空中无人机，同一世界，同一脚本。</p>
      <p>
        <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7"><img src="https://img.shields.io/badge/版本-v0.1.7-blue" alt="Version"/></a>
        <img src="https://img.shields.io/badge/许可证-MIT-yellow.svg" alt="License: MIT"/>
        <img src="https://img.shields.io/badge/python-3.8+-blue" alt="Python 3.8+"/>
        <img src="https://img.shields.io/badge/CARLA-0.9.16-green" alt="CARLA 0.9.16"/>
        <img src="https://img.shields.io/badge/AirSim-1.8.1-orange" alt="AirSim 1.8.1"/>
        <img src="https://img.shields.io/badge/平台-Ubuntu%2020.04%20%7C%2022.04-lightgrey" alt="Platform"/>
        <img src="https://img.shields.io/badge/arXiv-即将发布-b31b1b" alt="arXiv"/>
      </p>
      <p>
        <a href="README.md">English</a> | <a href="README_CN.md">简体中文</a>
      </p>
    </td>
  </tr>
</table>

<p align="center">
  <img src="teaser_upload.gif" alt="CarlaAir Teaser" width="100%"/>
</p>

---

## 🔥 最新动态

- **[2026-03]** `v0.1.7` 发布 — VSync 全屏修复、稳定交通系统、一键环境配置、无人机录制工具、坐标系文档
- **[2026-03]** `v0.1.6` 发布 — 自动交通生成、UE4 原生 Sweep 碰撞、地面夹紧系统
- **[2026-03]** `v0.1.5` 发布 — 12 方向碰撞系统、双语帮助菜单（`H`）
- **[2026-03]** `v0.1.4` 发布 — ROS2 验证（63 个话题）、首个官方二进制发布

---

**CarlaAir** 是一个开源的空地联合仿真平台。它通过在底层 C++ 将全球领先的自动驾驶仿真器（CARLA）与机器人仿真器（AirSim）合并为单一的 `ASimWorldGameMode`，实现了真正的帧级传感器同步、统一的物理引擎，以及无缝的双 Python API 交互。

## ✨ 核心亮点

- 🚀 **单进程深度集成**：拒绝跨进程通信桥接（Bridge），无延迟损耗。CARLA 与 AirSim 共享同一个 UE4 世界、天气系统与物理引擎。
- 🎯 **绝对坐标对齐**：彻底解决 CARLA（左手系）与 AirSim（NED）之间的坐标转换问题，误差精确控制在 `0.0000m`。
- 🚁 **内置 FPS 无人机控制**：无需编写任何代码，在仿真窗口内直接使用 `WASD` + 鼠标 即可像玩 FPS 游戏一样丝滑驾驶无人机。
- 🚦 **开箱即用的城市交通**：启动器默认自动生成 30 辆背景车辆与 50 个行人，立即构建逼真城市场景。
- 📸 **18路同步传感器**：支持在地面车辆和空中无人机上同时挂载 RGB、激光雷达、深度图、语义分割、IMU 和 GNSS，数据严格对齐。
- 🐍 **双 API 无缝协作**：在同一个 Python 脚本中，同时调用 `carla.Client`（端口 2000）和 `airsim.MultirotorClient`（端口 41451）。

---

## 🎮 快速开始

### 选项 A：使用预编译版本（推荐）

```bash
# 1. 下载并解压 CarlaAir-v0.1.7
tar xzf CarlaAir-v0.1.7.tar.gz
cd CarlaAir-v0.1.7

# 2. 一键环境配置（仅首次需要）
bash env_setup/setup_env.sh      # 创建 conda 环境，安装依赖，部署 carla 模块
conda activate carlaAir
bash env_setup/test_env.sh        # 验证环境：应全部显示 PASS

# 3. 启动仿真器（自动生成交通流）
./CarlaAir.sh Town10HD

# 4. 运行展示脚本！（另开一个终端）
conda activate carlaAir
python3 examples/quick_start_showcase.py
```

> **你将看到：** 一辆特斯拉在城市中自动巡航，无人机从空中追踪。4 分屏同时展示 **RGB · 深度图 · 语义分割 · LiDAR 鸟瞰** — 全部实时同步。天气自动轮换。

### 选项 B：从源码编译

如果您需要修改底层 C++ 代码，请参考 [源码编译指南](CarlaAir_Release/source/BUILD_GUIDE.md)，了解如何使用 UE4.26 编译 CarlaAir。

---

## 🐍 一个脚本，两个世界

两套 API 共享**同一个仿真世界** — 无桥接、无同步烦恼。

```python
import carla, airsim

carla_client = carla.Client("localhost", 2000)
air_client   = airsim.MultirotorClient(port=41451)
world = carla_client.get_world()

# 一次天气调用，影响所有传感器 — 地面和空中
world.set_weather(carla.WeatherParameters.HardRainSunset)

# 生成一辆汽车，自动驾驶
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)

# 无人机起飞 — 同一个世界，同一场雨，同一套物理
air_client.takeoffAsync().join()
air_client.moveToPositionAsync(80, 30, -25, 5)
```

**6 个演示脚本** — 逐个试试：

```bash
python3 examples/quick_start_showcase.py   # 🎬 4分屏传感器 + 无人机追踪 + 天气轮换
python3 examples/drive_vehicle.py          # 🚗 WASD 驾驶特斯拉
python3 examples/walk_pedestrian.py        # 🚶 鼠标+键盘 城市漫步
python3 examples/switch_maps.py            # 🗺️  自动飞越全部 13 张地图
python3 examples/sensor_gallery.py         # 📸 单车 6 传感器网格展示
python3 examples/air_ground_sync.py        # 🔄 车+无人机分屏：同一场雨，同一世界
```

**录制工具** — 录制车辆、无人机、行人轨迹，然后用导演相机回放：

```bash
python3 examples/recording/record_vehicle.py     # 🚗 键盘驾驶并录制车辆轨迹
python3 examples/recording/record_drone.py       # 🚁 飞行并录制无人机轨迹（零侵入）
python3 examples/recording/record_walker.py      # 🚶 行走并录制行人轨迹
python3 examples/recording/demo_director.py \    # 🎬 多轨迹回放 + 自由相机 + MP4 录制
    trajectories/vehicle_*.json trajectories/drone_*.json
```

> **文档：** [坐标系换算 (CARLA↔AirSim)](CarlaAir_Release/guide/COORDINATE_SYSTEMS.md) · [快速入门指南](CarlaAir_Release/guide/Quick-Start.md) · [常见问题](CarlaAir_Release/guide/FAQ.md)

---

## ⌨️ 飞行控制说明

在 CarlaAir 窗口中，使用内置 FPS 控制器直接驾驶无人机（无需代码）：

| 按键 | 功能 |
|-----|--------|
| `W` / `A` / `S` / `D` | 前进 / 左移 / 后退 / 右移 |
| `Space` / `Shift` | 上升 / 下降 |
| `Mouse` | 偏航转向 |
| `Scroll` | 调节速度 |
| `N` | 切换天气 |
| `P` | 碰撞模式切换 |
| `H` | 帮助菜单 |
| `1` / `2` / `3` | 传感器画面 |

---

## 📚 文档与示例

**完整文档：**
- [快速入门指南](CarlaAir_Release/guide/Quick-Start.md)
- [技术架构详解](CarlaAir_Release/source/ARCHITECTURE.md)
- [坐标系换算](examples_record_demo/COORDINATE_SYSTEMS.md)
- [上游代码修改清单](CarlaAir_Release/source/MODIFICATIONS.md)

---

## 📜 许可证与致谢

CarlaAir 站在巨人的肩膀上。我们诚挚感谢以下开源项目的开发者：
- [CARLA Simulator](https://github.com/carla-simulator/carla) (MIT License)
- [Microsoft AirSim](https://github.com/microsoft/AirSim) (MIT License)
- [Unreal Engine](https://www.unrealengine.com/)

CarlaAir 特有的代码基于 **MIT 许可证** 开源。CARLA 相关的资产遵循 CC-BY 许可证。
