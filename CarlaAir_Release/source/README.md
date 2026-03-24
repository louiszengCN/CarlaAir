# CarlaAir (SimWorld) — 完整源码与开发文档

> **目标受众**: 工程 LLM / 开发者，用于在 CarlaAir 基础上引入 3DGS 和 World Model 渲染管线

## 项目概述

CarlaAir (SimWorld) 是一个 **空地一体联合仿真平台**，将 CARLA 0.9.16 的地面仿真能力（车辆、行人、天气、传感器、交通管理）与 AirSim 的无人机飞行仿真能力统一在同一个 Unreal Engine 4.26 世界中。

### 核心成就
- 统一的 `ASimWorldGameMode`：同时启用所有 CARLA 和 AirSim 功能
- 双 API 服务器：CARLA RPC (port 2000) + AirSim RPC (port 41451) 同时运行
- 30+ 地面车辆 + 无人机在同一世界中协同运行
- 13 张地图、340 个 AirSim 资源文件、完整的传感器套件

---

## 目录结构

```
source/
├── README.md                  # 本文件：项目总览和开发指南
├── ARCHITECTURE.md            # 详细技术架构（类层次、数据流、坐标系）
├── BUILD_GUIDE.md             # 从源码编译的完整步骤
├── MODIFICATIONS.md           # 所有相对于原版 CARLA/AirSim 的修改清单
├── FUTURE_DEVELOPMENT.md      # 3DGS / World Model 集成点和扩展指南
├── core_source/               # 核心 C++ 源文件（新建 + 修改的）
│   ├── SimWorldGameMode.h     # [新建] 统一 GameMode 头文件
│   ├── SimWorldGameMode.cpp   # [新建] 统一 GameMode 实现（548行）
│   ├── CarlaGameModeBase.h    # [修改] WeatherClass/ActorFactories → protected
│   ├── CarlaEpisode.h         # [修改] 添加 friend class ASimWorldGameMode
│   └── SimModeBase.cpp        # [修改] 移除 SetNewWorldOrigin() 调用
├── config/
│   ├── DefaultGame.ini        # UE4 打包配置（地图、AirSim 资源）
│   └── settings.json          # AirSim 设置模板
├── build_scripts/
│   ├── carlaAir.sh            # Editor 模式启动脚本
│   └── BuildCarlaUE4.sh       # UE4 编译脚本
├── python_api/
│   ├── examples/              # 14 个示例脚本（交互/展示/联合）
│   └── DEMO/                  # 13 个自动录制视频的 Demo 脚本
└── project_docs/
    ├── 开发进度记录.md         # 完整开发日志
    ├── 测试教程.md             # 功能测试教程
    ├── SimWorld_Technical_Document.md  # 技术文档（论文级）
    └── v0.1_release_test_report.md    # v0.1 测试报告
```

---

## 关键架构

### 类继承关系

```
AGameModeBase (UE4)
  └── ACarlaGameModeBase (CARLA 插件)
        ├── Episode, Weather, Traffic, Recorder, ActorFactories
        └── ASimWorldGameMode (CarlaAir 核心)  ← core_source/
              ├── 继承所有 CARLA 功能
              ├── 在 BeginPlay() 中引导 AirSim
              ├── 生成 ASimModeBase 作为普通 Actor（非 GameMode）
              └── 管理 AirSim HUD Widget、输入绑定、子窗口
```

### 坐标系统

| 坐标系 | X+ 方向 | Y+ 方向 | Z+ 方向 |
|--------|---------|---------|---------|
| UE4 / CARLA | 前 (East) | 右 (South) | 上 |
| AirSim NED | 前 (North) | 右 (East) | **下** |

**关键映射**: NED x = CARLA x, NED y = CARLA y, NED z = **-CARLA z**

NED 原点 = PlayerStart 位置（Town10HD 中靠近海岸，UE4 x≈0）

### 关键设计决策

1. **DefaultPawnClass = nullptr**: AirSim 需要自己控制 Pawn possession，UE4 自动生成的 DefaultPawn 会破坏无人机控制
2. **手动创建 SpectatorPawn**: 不 possess 该 Pawn（否则破坏 AirSim 输入），直接注册到 CARLA Episode
3. **SimMode 作为普通 Actor**: 避免 GameMode 槽位冲突，通过 `GetWorld()->SpawnActor()` 生成
4. **移除 SetNewWorldOrigin()**: 防止世界原点偏移导致 CARLA 道路碰撞失效

---

## 核心源文件说明

### SimWorldGameMode.h / .cpp（新建，核心文件）

**路径**: `Plugins/AirSim/Source/SimWorldGameMode.h/.cpp`

这是整个 CarlaAir 项目的核心。继承 `ACarlaGameModeBase`，在其上引导 AirSim 子系统。

**构造函数** (`SimWorldGameMode.cpp:66-123`):
- `DefaultPawnClass = nullptr` — 禁止 UE4 自动生成 Pawn
- 加载 `BP_Weather` 蓝图类 → `WeatherClass`
- 注册 8 个 ActorFactory（5 个 C++类 + 3 个蓝图类）
- 初始化 AirSim 日志系统
- 预加载 ImageWrapper 模块
- 加载 AirSim HUD Widget 蓝图

**BeginPlay()** (`SimWorldGameMode.cpp:127-186`):
1. `Super::BeginPlay()` → 完整 CARLA 初始化
2. 手动创建并注册 `ASpectatorPawn`
3. 初始化 AirSim Settings
4. 设置 UE4 渲染参数（关闭运动模糊、启用 CustomDepth）
5. 创建 SimMode Actor（多旋翼/汽车/计算视觉）
6. 创建 AirSim HUD Widget
7. 设置输入绑定
8. 启动 AirSim API 服务器

**Tick()**: CARLA Recorder tick + AirSim debug report 更新

**EndPlay()**: 停止 AirSim API → 销毁 Widget → 销毁 SimMode → CARLA 清理

### CarlaGameModeBase.h（修改）

**修改内容**: 将 `WeatherClass` 和 `ActorFactories` 从 `private` 移到 `protected`

**原因**: `ASimWorldGameMode` 继承 `ACarlaGameModeBase`，需要在构造函数中设置天气蓝图和 ActorFactory 列表。原来这些成员是 `private` 的，子类无法访问。

### CarlaEpisode.h（修改）

**修改内容**: 添加 `friend class ASimWorldGameMode;` 声明

**原因**: SimWorldGameMode 需要在 BeginPlay() 中直接设置 `Episode->Spectator` 私有成员。

### SimModeBase.cpp（修改）

**修改内容**: 注释掉 `SetNewWorldOrigin()` 调用（约第 119-125 行）

**原因**: `SetNewWorldOrigin()` 将 UE4 世界原点偏移到 PlayerStart 位置，这会使 CARLA 的 landscape/road 碰撞几何失效，导致所有地面车辆穿过地面下沉约 30 米。修复后车辆正常行驶（z≈0.00），AirSim 无人机不受影响。

---

## 全部依赖

### 引擎和插件

| 组件 | 版本 | 路径 |
|------|------|------|
| Unreal Engine | 4.26 (CARLA fork) | `/mnt/data1/tianle/carla_ue4/` |
| CARLA | 0.9.16 (ue4-dev branch) | `/mnt/data1/tianle/carla_source/` |
| AirSim | Colosseum fork | `Plugins/AirSim/` (集成在 CARLA 项目中) |

### Python 依赖

```
carla==0.9.16          # CARLA Python API
airsim                  # AirSim Python API (pip install airsim)
numpy
opencv-python (cv2)
pygame                  # 仅交互式脚本需要
```

### 系统依赖

- Ubuntu 18.04+ (tested on 22.04)
- NVIDIA GPU + 驱动 (Vulkan 支持)
- Python 3.7+
- Conda 环境: `simworld` (`/mnt/data1/tianle/miniconda3/envs/simworld/`)

---

## 编译指南（快速版）

```bash
# 设置环境变量
export UE4_ROOT=/mnt/data1/tianle/carla_ue4

# 编译 AirSim 模块（~100秒）
cd /mnt/data1/tianle/carla_source
./Util/BuildTools/Build.sh ... -module=AirSim

# 编译 CARLA 模块（~290秒）
./Util/BuildTools/Build.sh ... -module=Carla

# 打包 Shipping 构建（编译~805秒 + Cook~2小时）
./Util/BuildTools/Package.sh --config=Shipping --no-zip

# 输出目录
ls Dist/CARLA_Shipping_*/LinuxNoEditor/
```

详细编译步骤见 `BUILD_GUIDE.md`。

---

## 开发入口点（3DGS / World Model 集成）

如果你要在此基础上集成新的渲染管线，关键入口点是：

1. **渲染管线替换**: `SimModeBase.cpp` → `setupPhysicsLoopPeriod()` 之后可以插入自定义渲染初始化
2. **传感器数据流**: `Plugins/AirSim/Source/Vehicles/` → 各传感器的数据获取接口
3. **地图资源**: `Content/Carla/Maps/` → UE4 地图资源，可以替换为 3DGS 场景
4. **相机系统**: `Plugins/AirSim/Source/PIPCamera.cpp` → AirSim 相机实现，可以替换渲染后端
5. **物理系统**: `SimModeBase.cpp` → `setupPhysicsLoopPeriod()` 控制物理更新频率

详见 `FUTURE_DEVELOPMENT.md`。
