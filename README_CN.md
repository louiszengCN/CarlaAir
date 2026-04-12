<h1 align="center">CARLA-Air：在 CARLA 世界里飞无人机</h1>
<h3 align="center">空地一体具身智能统一仿真基础设施</h3>

<p align="center">
  <a href="https://www.bilibili.com/video/BV1pTQzBkES7/">
    <img src="docs/images/teaser_video.gif" alt="CARLA-Air 演示 — 点击观看完整视频" width="100%"/>
  </a>
</p>

**CARLA-Air** 是一个开源仿真基础设施，在单一 Unreal Engine 进程中统一了高保真城市驾驶与物理精确的多旋翼飞行，为空地一体具身智能研究提供了实用的仿真基础。  

👉 **无需编译，下载即用的预构建可执行版本：**  
[百度网盘](https://pan.baidu.com/s/1RguWqwKrN-3KEgyKvWiiug?pwd=d5ai) | [Hugging Face](https://huggingface.co/tianlezeng/CarlaAIr-v0.1.7)

<div align="center">
  <a href="https://huggingface.co/papers/2603.28032"><img src="https://img.shields.io/badge/%F0%9F%8F%86%20HF%20Daily%20Papers-%231%20Paper%20of%20the%20Day-FFD700" alt="#1 Paper of the Day"/></a>
  <a href="docs/pdf/CarlaAir.pdf"><img src="https://img.shields.io/badge/Paper-PDF-red" alt="Paper PDF"/></a>
  <a href="https://arxiv.org/abs/2603.28032"><img src="https://img.shields.io/badge/arXiv-2603.28032-b31b1b?logo=arxiv&logoColor=white" alt="arXiv"/></a>
  <a href="https://github.com/louiszengCN/CarlaAir/stargazers"><img src="https://img.shields.io/github/stars/louiszengCN/CarlaAir?style=social" alt="GitHub Stars"/></a>
  <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7"><img src="https://img.shields.io/badge/version-v0.1.7-blue" alt="Version"/></a>
  <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/>
  <img src="https://img.shields.io/badge/python-3.8+-blue" alt="Python 3.8+"/>
  <img src="https://img.shields.io/badge/CARLA-0.9.16-green" alt="CARLA 0.9.16"/>
  <img src="https://img.shields.io/badge/AirSim-1.8.1-orange" alt="AirSim 1.8.1"/>
  <img src="https://img.shields.io/badge/platform-Ubuntu%2020.04%20%7C%2022.04-lightgrey" alt="Platform"/>
</div>

<br>

<p align="center">
  <a href="README.md">English</a> | <b>简体中文</b> &nbsp;&nbsp;|&nbsp;&nbsp;
  📄 <a href="docs/pdf/CarlaAir.pdf"><b>论文</b></a> &nbsp;|&nbsp;
  🌐 <a href="#"><b>项目主页</b></a> <i>(即将上线)</i> &nbsp;|&nbsp;
  📖 <a href="CarlaAir_Release/guide/Quick-Start.md"><b>文档</b></a> &nbsp;|&nbsp;
  🎬 <a href="https://www.bilibili.com/video/BV1pTQzBkES7/"><b>视频</b></a>
</p>

<p align="center">
  <a href="https://pan.baidu.com/s/1RguWqwKrN-3KEgyKvWiiug?pwd=d5ai"><img src="https://img.shields.io/badge/百度网盘-下载-2932E1?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyNCIgaGVpZ2h0PSIyNCIgdmlld0JveD0iMCAwIDI0IDI0Ij48cGF0aCBmaWxsPSJ3aGl0ZSIgZD0iTTEyIDJDNi40OCAyIDIgNi40OCAyIDEyczQuNDggMTAgMTAgMTAgMTAtNC40OCAxMC0xMFMxNy41MiAyIDEyIDJ6Ii8+PC9zdmc+" alt="Baidu Pan"/></a>
  <a href="https://huggingface.co/tianlezeng/CarlaAIr-v0.1.7"><img src="https://img.shields.io/badge/HuggingFace-下载-FFD21E?logo=huggingface&logoColor=black" alt="HuggingFace"/></a>
  <a href="docs/images/wxg.jpg?v=20260419"><img src="https://img.shields.io/badge/微信-交流群-07C160?logo=wechat&logoColor=white" alt="WeChat Group"/></a>
</p>

## 📌 目录

- [🔥 最新动态](#最新动态)
- [✨ 核心亮点](#核心亮点)
- [🏆 平台对比](#平台对比) — 15 款仿真器横向对比
- [🎮 快速开始](#快速开始) — 4 步上手
- [🐍 一个脚本，两个世界](#一个脚本两个世界) — 双 API 代码示例
- [🔬 研究方向与工作流](#研究方向与工作流) — W1–W5 验证工作流
- [⌨️ 飞行控制说明](#飞行控制说明)
- [📚 文档与教程](#文档与教程) — 8 个渐进式教程
- [🗺️ 路线图](#路线图)
- [📝 引用](#引用)
- [📜 许可证与致谢](#许可证与致谢)
- [⭐ Star History](#star-history)

## 🔥 最新动态

- **[2026-04-10]** [![Windows 源码分支](https://img.shields.io/badge/Windows-源码分支-0078D6?logo=windows&logoColor=white)](https://github.com/louiszengCN/CarlaAir/tree/windows/v0.1.7-win11-x86_64) [Windows 源码分支已发布](https://github.com/louiszengCN/CarlaAir/tree/windows/v0.1.7-win11-x86_64) -- Windows 构建与运行支持现已在独立分支中提供
- **[2026-04-01]** 🏆 $\color{red}{\textbf{\\#1 Paper of the Day}}$ **登顶 [Hugging Face Daily Papers](https://huggingface.co/papers/2603.28032)！**
- **[2026-03-30]** 📄 技术报告发布 -- [阅读论文](docs/pdf/CarlaAir.pdf)
- **[2026-03-31]** 🚀 即将上线 -- 项目主页、教程文档及开箱即用的二进制发布包，敬请期待！
- **[2026-03]** `v0.1.7` 发布 -- VSync 修复、稳定交通系统、一键环境配置、无人机录制工具、坐标系文档
- **[2026-03]** `v0.1.6` 发布 -- 自动交通生成、UE4 原生 Sweep 碰撞、地面夹紧系统
- **[2026-03]** `v0.1.5` 发布 -- 12 方向碰撞系统、双语帮助菜单（`H`）
- **[2026-03]** `v0.1.4` 发布 -- ROS2 验证（63 个话题）、首个官方二进制发布

---

## ✨ 核心亮点

| | |
|---|---|
| 🏗️ **单进程组合式集成** | `CARLAAirGameMode` 继承 CARLA 并组合 AirSim。仅修改上游 2 个文件（约 35 行）。无桥接，无延迟。 |
| 🎯 **绝对坐标对齐** | CARLA（左手系）与 AirSim（NED）坐标系之间精确 `0.0000 m` 误差。 |
| 📸 **多达 18 种传感器模态** | RGB、深度图、语义分割、实例分割、LiDAR、雷达、表面法线、IMU、GNSS、气压计 -- 空地传感器逐帧对齐。 |
| 🔄 **零修改代码迁移** | 现有 CARLA 和 AirSim Python 脚本及 ROS 2 节点可直接在 CARLA-Air 上运行，无需任何代码改动。89/89 CARLA API 测试全部通过。 |
| ⚡ **联合负载约 20 FPS** | 中等联合配置（车辆 + 无人机 + 8 个传感器）稳定运行在 19.8 +/- 1.1 FPS。通信开销 < 0.5 ms（对比桥接联合仿真的 1--5 ms）。 |
| 🛡️ **3 小时稳定性验证** | 357 次生成/销毁循环，零崩溃，零内存累积（R² = 0.11）。 |
| 🚁 **内置 FPS 无人机控制** | 在视口中使用 WASD + 鼠标直接驾驶无人机 -- 无需编写 Python 脚本。 |
| 🚦 **逼真城市交通** | 规则驱动的交通流、具有社会行为的行人、13 张城市地图。 |
| 🧩 **可扩展资产导入管线** | 支持导入自定义机器人平台、无人机配置、车辆模型和环境地图。 |

<p align="center">
  <img src="docs/images/teaser_final.jpg" alt="CARLA-Air 架构概览" width="95%"/>
</p>

---

## 🏆 平台对比

CARLA-Air 与 14 个现有仿真平台的全面对比（基于[技术报告](docs/pdf/CarlaAir.pdf)表 1）。

<table>
  <thead>
    <tr>
      <th>类别</th>
      <th>平台</th>
      <th>城市交通</th>
      <th>行人</th>
      <th>无人机飞行</th>
      <th>单进程</th>
      <th>共享渲染器</th>
      <th>原生 API</th>
      <th>联合传感器</th>
      <th>预编译包</th>
      <th>测试套件</th>
      <th>自定义资产</th>
      <th>开源</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="5"><i>自动驾驶</i></td>
      <td>CARLA</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>LGSVL</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>SUMO</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>MetaDrive</td>
      <td>✓</td><td>~</td><td>✗</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>VISTA</td>
      <td>~</td><td>✗</td><td>✗</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td rowspan="6"><i>空中 / 无人机</i></td>
      <td>AirSim</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>Flightmare</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>FlightGoggles</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>Gazebo/RotorS</td>
      <td>~</td><td>~</td><td>✓</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>OmniDrones</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>gym-pybullet-drones</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td rowspan="3"><i>联合 / 协同仿真</i></td>
      <td>TranSimHub</td>
      <td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>—</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>CARLA+SUMO</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✗</td><td>—</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>AirSim+Gazebo</td>
      <td>~</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>~</td><td>~</td><td>—</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td rowspan="5"><i>具身 AI 与 RL</i></td>
      <td>Isaac Lab</td>
      <td>✗</td><td>✗</td><td>~</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>Isaac Gym</td>
      <td>✗</td><td>✗</td><td>~</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>Habitat</td>
      <td>✗</td><td>~</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>SAPIEN</td>
      <td>✗</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>RoboSuite</td>
      <td>✗</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr style="background-color: #f0f7ff;">
      <td><b>本项目</b></td>
      <td><b>CARLA-Air</b></td>
      <td><b>✓</b></td><td><b>✓†</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td>
    </tr>
  </tbody>
</table>

<p><sup>✓ = 支持；~ = 部分或受限支持；✗ = 不支持；— = 不适用。<br/>
† 行人 AI 继承自 CARLA，功能完整；联合场景下高密度 Actor 的行为是当前工程优化目标。</sup></p>

---

## 🎮 快速开始

### 选项 A：预编译版本（推荐）

```bash
# 1. 下载并解压 CARLA-Air v0.1.7
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

> **你将看到：** 一辆特斯拉在城市中自动巡航，无人机从空中追踪。4 分屏同时展示 **RGB、深度图、语义分割和 LiDAR 鸟瞰** -- 全部实时同步。天气自动轮换。

### 选项 B：从源码编译

请参考[源码编译指南](CarlaAir_Release/source/BUILD_GUIDE.md)，了解如何使用 UE4.26 编译 CARLA-Air。

---

## 🐍 一个脚本，两个世界

两套 API 共享**同一个仿真世界** -- 无桥接，无同步烦恼。

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

**6 个演示脚本** -- 逐个体验：

```bash
python3 examples/quick_start_showcase.py   # 🎬 4 分屏传感器 + 无人机追踪 + 天气轮换
python3 examples/drive_vehicle.py          # 🚗 WASD 驾驶特斯拉
python3 examples/walk_pedestrian.py        # 🚶 鼠标视角城市漫步
python3 examples/switch_maps.py            # 🗺️  自动飞越全部 13 张地图
python3 examples/sensor_gallery.py         # 📸 单车 6 传感器网格展示
python3 examples/air_ground_sync.py        # 🔄 车 + 无人机分屏：同一场雨，同一世界
```

**录制工具** -- 录制车辆、无人机、行人轨迹，然后用导演相机回放：

```bash
python3 examples/recording/record_vehicle.py     # 🚗 键盘驾驶并录制车辆轨迹
python3 examples/recording/record_drone.py       # 🚁 飞行并录制无人机轨迹（零侵入）
python3 examples/recording/record_walker.py      # 🚶 行走并录制行人轨迹
python3 examples/recording/demo_director.py \    # 🎬 多轨迹回放 + 自由相机 + MP4 录制
    trajectories/vehicle_*.json trajectories/drone_*.json
```

> **文档：** [坐标系换算（CARLA 到 AirSim）](CarlaAir_Release/guide/COORDINATE_SYSTEMS.md) | [快速入门指南](CarlaAir_Release/guide/Quick-Start.md) | [常见问题](CarlaAir_Release/guide/FAQ.md)

---

## 🔬 研究方向与工作流

CARLA-Air 旨在支持空地一体具身智能的四大研究方向：

1. **空地协同** -- 异构空地智能体在共享城市环境中协作。
2. **具身导航（VLN/VLA）** -- 视觉语言驱动的导航与动作执行，基于高保真城市场景。
3. **多模态感知与数据集** -- 多样条件下的空地同步传感器数据采集。
4. **强化学习策略训练** -- 空地联合交互的闭环强化学习训练。

平台提供五个覆盖上述方向的参考工作流：

| | 工作流 | 研究方向 | 关键成果 |
|---|---|---|---|
| W1 | 空地协同 | 空地协同 | 实时跨域协调控制 |
| W2 | VLN/VLA 任务 | 具身导航 | 跨视角 VLN 数据管线 |
| W3 | 多模态数据集采集 | 感知与数据集 | 12 路同步，1-tick 对齐 |
| W4 | 跨视角感知 | 感知与数据集 | 14/14 天气预设验证通过 |
| W5 | RL 训练环境 | 强化学习策略训练 | 357 次重置循环，0 次崩溃 |

<table>
  <tr>
    <td align="center" width="50%">
      <img src="docs/gifs/W1.gif" alt="W1：空地协同" width="100%"/><br/>
      <b>W1：空地协同</b>
    </td>
    <td align="center" width="50%">
      <img src="docs/gifs/W2.gif" alt="W2：VLN/VLA 数据生成" width="100%"/><br/>
      <b>W2：VLN/VLA 任务</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/gifs/W3.gif" alt="W3：多模态数据集采集" width="100%"/><br/>
      <b>W3：多模态数据集采集</b>
    </td>
    <td align="center">
      <img src="docs/gifs/W4.gif" alt="W4：跨视角感知" width="100%"/><br/>
      <b>W4：跨视角感知</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/RL.jpg" alt="W5：RL 训练环境" width="100%"/><br/>
      <b>W5：RL 训练环境</b>
    </td>
    <td align="center">
      <img src="docs/gifs/customAsset.gif" alt="自定义资产导入" width="100%"/><br/>
      <b>自定义资产导入</b>
    </td>
  </tr>
  <tr>
    <td align="center" colspan="2">
      <img src="docs/gifs/ROS_demo.gif" alt="ROS 2 支持" width="100%"/><br/>
      <b>ROS 2 集成 — 63 个 Topic 覆盖双仿真后端</b>
    </td>
  </tr>
</table>

---

## ⌨️ 飞行控制说明

仿真器运行时，点击窗口内部捕获鼠标：

| 按键 | 功能 |
|-----|--------|
| `W` / `A` / `S` / `D` | 前进 / 左移 / 后退 / 右移 |
| `Space` / `Shift` | 上升 / 下降 |
| `Mouse` | 偏航转向（左右旋转） |
| `Scroll Wheel` | 调节飞行速度 |
| `N` | 切换天气预设（晴天、雨天、雾天、夜晚等） |
| `P` | 切换碰撞模式（物理碰撞 vs. 穿墙/无敌） |
| `H` | 显示/隐藏屏幕帮助菜单 |
| `Tab` | 释放 / 捕获鼠标 |

---

## 📚 文档与教程

我们提供了 **6 个精选 Python 示例**，展示核心空地协同能力：

| 示例 | 说明 |
|---------|-------------|
| `quick_start_showcase.py` | 4 分屏传感器 + 无人机追踪 + 天气轮换 |
| `drive_vehicle.py` | WASD 键盘驾驶特斯拉 |
| `walk_pedestrian.py` | 鼠标视角城市漫步 |
| `switch_maps.py` | 自动飞越全部 13 张地图 |
| `sensor_gallery.py` | 单车 6 传感器网格展示 |
| `air_ground_sync.py` | 车 + 无人机分屏：同一场雨，同一世界 |

**分步教程**（8 个脚本位于 `CarlaAir_Release/guide/examples/`，适合初学者）：

| # | 教程 | 学习内容 |
|---|----------|---------------------|
| 01 | `01_hello_world.py` | 连接双 API，验证环境配置 |
| 02 | `02_weather_control.py` | 实时更改天气参数 |
| 03 | `03_spawn_traffic.py` | 生成车辆与行人 |
| 04 | `04_sensor_capture.py` | 挂载与读取传感器 |
| 05 | `05_drone_takeoff.py` | 基础无人机飞行指令 |
| 06 | `06_drone_sensors.py` | 空中传感器配置 |
| 07 | `07_combined_demo.py` | 空地联合操作 |
| 08 | `08_full_showcase.py` | 全平台能力展示 |

**完整文档：**
- [快速入门指南](CarlaAir_Release/guide/Quick-Start.md)
- [坐标系换算（CARLA 到 AirSim）](CarlaAir_Release/guide/COORDINATE_SYSTEMS.md)
- [技术架构详解](CarlaAir_Release/source/ARCHITECTURE.md)
- [上游代码修改清单](CarlaAir_Release/source/MODIFICATIONS.md)
- [常见问题](CarlaAir_Release/guide/FAQ.md)

---

## 🗺️ 路线图

- [x] CARLA + AirSim 单进程集成（UE4.26）
- [x] FPS 无人机控制（WASD + 鼠标）
- [x] 自动交通生成（车辆 + 行人）
- [x] 18 路同步传感器
- [x] 双 Python API（CARLA + AirSim）
- [x] ROS2 验证（63 个话题）
- [x] 一键环境配置
- [x] 录制工具（车辆、无人机、行人轨迹）
- [x] 技术报告（[PDF](docs/pdf/CarlaAir.pdf)）
- [ ] 项目主页（开发中）
- [ ] 教程文档
- [ ] 3DGS 渲染管线集成
- [ ] World Model 集成
- [ ] 多无人机支持

---

## 📝 引用

如果 CARLA-Air 对您的研究有所帮助，请考虑引用我们的论文：

```bibtex
@article{zeng2026carla,
  title={CARLA-Air: Fly Drones Inside a CARLA World--A Unified Infrastructure for Air-Ground Embodied Intelligence},
  author={Zeng, Tianle and Chen, Hanxuan and Wen, Yanci and Zhang, Hong},
  journal={arXiv preprint arXiv:2603.28032},
  year={2026}
}
```

---

## 📜 许可证与致谢

CARLA-Air 站在巨人的肩膀上。我们诚挚感谢以下项目的开发者：
- [CARLA Simulator](https://github.com/carla-simulator/carla) (MIT License)
- [Microsoft AirSim](https://github.com/microsoft/AirSim) (MIT License)
- [Unreal Engine](https://www.unrealengine.com/)

CARLA-Air 特有的代码基于 **MIT License** 分发。CARLA 相关的资产遵循 CC-BY License。

---

## ⭐ Star History

<a href="https://star-history.com/#louiszengCN/CarlaAir&Date">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=louiszengCN/CarlaAir&type=Date&theme=dark" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=louiszengCN/CarlaAir&type=Date" />
   <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=louiszengCN/CarlaAir&type=Date" />
 </picture>
</a>
