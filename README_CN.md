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
        <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.6"><img src="https://img.shields.io/badge/版本-v0.1.6-blue" alt="Version"/></a>
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
# 1. 下载并解压 CarlaAir-v0.1.6
tar xzf CarlaAir-v0.1.6.tar.gz
cd CarlaAir-v0.1.6

# 2. 启动仿真器（自动生成交通流）
./CarlaAir.sh Town10HD

# 3. 在另一个终端测试双 API 连接
python3 -c "import carla; c=carla.Client('localhost',2000); print(c.get_world().get_map().name)"
python3 -c "import airsim; c=airsim.MultirotorClient(port=41451); c.confirmConnection()"
```

### 选项 B：从源码编译

如果您需要修改底层 C++ 代码，请参考 [源码编译指南](CarlaAir_Release/source/BUILD_GUIDE.md)，了解如何使用 UE4.26 编译 CarlaAir。

---

## ⌨️ 飞行控制说明

当仿真器运行时，点击窗口内部捕获鼠标，即可使用内置的 FPS 控制器驾驶无人机：

| 按键 | 功能 |
|-----|--------|
| `W` / `A` / `S` / `D` | 前进 / 左移 / 后退 / 右移 |
| `Space` / `Shift` | 垂直上升 / 垂直下降 |
| `Mouse` (鼠标) | 偏航转向 (Yaw) |
| `Scroll Wheel` (滚轮) | 调节飞行速度 |
| `N` | 循环切换天气预设 (晴天、雨天、大雾、夜晚等) |
| `P` | 切换碰撞模式 (物理碰撞 / 无敌穿墙模式) |
| `H` | 显示/隐藏 屏幕帮助菜单 |
| `Tab` | 释放 / 捕获 鼠标 |

---

## 📚 文档与示例脚本

我们精选了 **6 个核心示例脚本**，展示空地协同仿真的关键能力：

| 脚本 | 说明 |
|------|------|
| `demo_drive_and_fly.py` | 同时控制地面车辆与无人机 |
| `drone_car_chase.py` | 无人机视觉追踪移动的地面车辆 |
| `aerial_surveillance.py` | 无人机城市巡航与图像捕获 |
| `data_collector.py` | 多传感器同步数据采集 |
| `city_tour.py` | 空地双视角城市巡游 |
| `fly_drone_keyboard.py` | 交互式键盘控制无人机飞行 |

**完整文档：**
- [快速入门指南](CarlaAir_Release/guide/Quick-Start.md)
- [技术架构详解](CarlaAir_Release/source/ARCHITECTURE.md)
- [上游代码修改清单](CarlaAir_Release/source/MODIFICATIONS.md)

---

## 📜 许可证与致谢

CarlaAir 站在巨人的肩膀上。我们诚挚感谢以下开源项目的开发者：
- [CARLA Simulator](https://github.com/carla-simulator/carla) (MIT License)
- [Microsoft AirSim](https://github.com/microsoft/AirSim) (MIT License)
- [Unreal Engine](https://www.unrealengine.com/)

CarlaAir 特有的代码基于 **MIT 许可证** 开源。CARLA 相关的资产遵循 CC-BY 许可证。
