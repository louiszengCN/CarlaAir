# CarlaAir

[English](README.md) | [中文](README_CN.md)

**空地一体联合仿真平台**，基于 CARLA 0.9.16 + AirSim，在单一 Unreal Engine 4.26 实例中提供统一的 FPS 无人机控制、真实城市交通和双 API 接口。

---

## 特性

- **统一仿真** — CARLA 地面仿真（车辆、行人、天气、传感器）与 AirSim 空中仿真（多旋翼无人机、相机）在同一 UE4 进程中运行
- **FPS 无人机控制** — 在 3D 视口中使用 WASD + 鼠标飞行无人机，滚轮调节速度
- **物理碰撞** — 基于 UE4 原生 Sweep 碰撞检测，无人机撞到建筑物和地形时会停住
- **自动交通** — 启动时自动生成 30 辆车 + 50 个行人，营造真实城市场景
- **双 API** — CARLA Python API（端口 2000）+ AirSim Python API（端口 41451），可同时访问
- **天气系统** — 按 N 键循环切换天气预设（晴天、雨天、雾天、夜晚）
- **24 个示例脚本** — 涵盖空中监控、城市巡游、数据采集、轨迹记录等场景

## 系统要求

| 组件 | 最低要求 |
|------|---------|
| 操作系统 | Ubuntu 20.04 / 22.04 |
| CPU | Intel i7 9代 或 AMD Ryzen 7 |
| 内存 | 32 GB |
| 显卡 | NVIDIA RTX 3070（8 GB 显存）|
| 磁盘 | 100 GB 可用空间（源码编译）/ 30 GB（二进制版）|
| 驱动 | NVIDIA 525+ 且支持 Vulkan |

## 快速开始

### 方式 A：二进制发布版（推荐）

```bash
# 下载并解压 CarlaAir-v0.1.6
tar xzf CarlaAir-v0.1.6.tar.gz
cd CarlaAir-v0.1.6

# 启动（自动生成交通）
./CarlaAir.sh

# 在另一个终端测试连接
python3 -c "import carla; c=carla.Client('localhost',2000); print(c.get_world().get_map().name)"
python3 -c "import airsim; c=airsim.MultirotorClient(port=41451); c.confirmConnection()"
```

### 方式 B：从源码编译

```bash
# 1. 克隆 UE4（CARLA 分支）
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/carla_ue4
cd ~/carla_ue4 && ./Setup.sh && ./GenerateProjectFiles.sh && make

# 2. 克隆 CarlaAir 源码
git clone <本仓库> ~/carla_source
cd ~/carla_source

# 3. 编译依赖
./Util/BuildTools/Setup.sh
./Util/BuildTools/BuildLibCarla.sh
./Util/BuildTools/BuildPythonAPI.sh
./Util/BuildTools/BuildCarlaUE4.sh --build

# 4. 以 Editor 模式启动
./carlaAir.sh
```

详见 [BUILD_GUIDE.md](CarlaAir_Release/source/BUILD_GUIDE.md)。

## 操控键位

| 按键 | 功能 |
|------|------|
| W / A / S / D | 前进 / 左移 / 后退 / 右移 |
| 空格 / Shift | 上升 / 下降 |
| 鼠标 | 偏航（左右转向）|
| 滚轮 | 调节无人机速度 |
| N | 切换天气预设 |
| P | 切换物理/无敌模式 |
| H | 显示/隐藏帮助面板 |
| Tab | 切换鼠标捕获 |

## 架构概览

```
┌──────────────────────────────────────────────────────┐
│                 Unreal Engine 4.26                    │
│  ┌────────────────────────────────────────────────┐  │
│  │       ASimWorldGameMode（核心 GameMode）        │  │
│  │  ┌──────────────────┐ ┌─────────────────────┐ │  │
│  │  │  CARLA 子系统     │ │  AirSim 子系统      │ │  │
│  │  │  - Episode       │ │  - SimModeBase      │ │  │
│  │  │  - Weather       │ │  - HUD Widget       │ │  │
│  │  │  - TrafficMgr    │ │  - 物理引擎         │ │  │
│  │  │  - RPC :2000     │ │  - RPC :41451       │ │  │
│  │  └──────────────────┘ └─────────────────────┘ │  │
│  └────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────┘
        │                          │
        ▼                          ▼
  CARLA Python API          AirSim Python API
  (端口 2000)               (端口 41451)
```

## 示例脚本

| 脚本 | 说明 |
|------|------|
| `fly_drone_keyboard.py` | 交互式键盘控制无人机 |
| `demo_drive_and_fly.py` | 地面车辆 + 无人机同时演示 |
| `demo_flight_city.py` | 自动城市飞行巡游 |
| `aerial_surveillance.py` | 无人机空中监控与拍摄 |
| `city_tour.py` | 城市导览路线 |
| `data_collector.py` | 多传感器数据采集 |
| `drone_car_chase.py` | 无人机跟踪地面车辆 |
| `drone_traj_col.py` | 无人机轨迹录制 |
| `drone_traj_playback.py` | 轨迹回放 |
| `showcase_traffic.py` | 交通可视化演示 |

完整列表见 [`examples/`](examples/) 目录。

## 项目结构

```
CarlaAir/
├── carlaAir.sh              # Editor 模式启动脚本
├── auto_traffic.py          # 自动交通生成器
├── Unreal/CarlaUE4/         # UE4 项目 + 插件
│   └── Plugins/
│       ├── AirSim/          # AirSim 插件（无人机、HUD、物理）
│       └── Carla/           # CARLA 插件（交通、传感器、RPC）
├── PythonAPI/               # Python 客户端库
├── LibCarla/                # C++ 客户端库
├── examples/                # 示例脚本（24个）
├── test_script/             # 测试脚本
├── CarlaAir_Release/        # 发布文档
│   ├── guide/               # 快速开始、FAQ
│   ├── install/             # 二进制安装指南
│   └── source/              # 源码编译指南、架构文档
├── Content/Blueprints/      # 自定义 UE4 蓝图
├── Util/BuildTools/         # 编译工具链
└── Progress_record/         # 开发记录
```

## 文档

- [快速开始](CarlaAir_Release/guide/Quick-Start.md)
- [常见问题](CarlaAir_Release/guide/FAQ.md)
- [技术架构](CarlaAir_Release/source/ARCHITECTURE.md)
- [编译指南](CarlaAir_Release/source/BUILD_GUIDE.md)
- [修改说明](CarlaAir_Release/source/MODIFICATIONS.md)
- [更新日志](CHANGELOG.md)

## 许可证

CARLA 代码采用 MIT 许可证，CARLA 资产采用 CC-BY 许可证，AirSim 采用 MIT 许可证。

详见 [LICENSE](LICENSE)。

## 致谢

CarlaAir 基于以下开源项目构建：
- [CARLA](https://github.com/carla-simulator/carla) — 开源自动驾驶仿真器
- [AirSim](https://github.com/microsoft/AirSim) — 微软开源机器人仿真平台
- [Unreal Engine 4.26](https://www.unrealengine.com/) — Epic Games
