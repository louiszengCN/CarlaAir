# CarlaAir 详细技术架构

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    Unreal Engine 4.26                            │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │              ASimWorldGameMode (核心)                      │  │
│  │  ┌─────────────────────┐  ┌────────────────────────────┐  │  │
│  │  │  CARLA 子系统        │  │  AirSim 子系统              │  │  │
│  │  │  (继承自父类)        │  │  (BeginPlay 中引导)         │  │  │
│  │  │                     │  │                            │  │  │
│  │  │  ■ CarlaEpisode     │  │  ■ ASimModeBase (Actor)    │  │  │
│  │  │  ■ Weather          │  │  ■ SimHUDWidget            │  │  │
│  │  │  ■ Traffic Manager  │  │  ■ AirSim Settings         │  │  │
│  │  │  ■ ActorDispatcher  │  │  ■ API Server (:41451)     │  │  │
│  │  │  ■ SensorManager    │  │  ■ PIPCamera System        │  │  │
│  │  │  ■ ActorFactories   │  │  ■ Physics Engine          │  │  │
│  │  │  ■ CarlaRecorder    │  │  ■ NedTransform            │  │  │
│  │  │  ■ RPC Server(:2000)│  │                            │  │  │
│  │  └─────────────────────┘  └────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                 │
│  ┌──────────────────┐  ┌──────────────────┐                     │
│  │  Town10HD Map     │  │  AirSim Assets   │                     │
│  │  (Roads, Buildings│  │  (Drone model,   │                     │
│  │   Traffic Lights) │  │   Weather, HUD)  │                     │
│  └──────────────────┘  └──────────────────┘                     │
└─────────────────────────────────────────────────────────────────┘
         │                              │
         ▼                              ▼
  ┌──────────────┐              ┌──────────────┐
  │  CARLA Python │              │ AirSim Python │
  │  Client API   │              │ Client API    │
  │  (port 2000)  │              │ (port 41451)  │
  └──────────────┘              └──────────────┘
```

## 初始化序列

```
UE4 Engine Start
  │
  ├── 1. ASimWorldGameMode::Constructor()
  │     ├── Super() → ACarlaGameModeBase 初始化 Episode, Recorder, Delegates
  │     ├── DefaultPawnClass = nullptr
  │     ├── 加载 BP_Weather → WeatherClass
  │     ├── 注册 8 个 ActorFactory
  │     ├── 初始化 AirSim 日志
  │     └── 加载 AirSim HUD Widget 蓝图
  │
  ├── 2. ASimWorldGameMode::BeginPlay()
  │     ├── Super::BeginPlay()
  │     │     ├── CARLA Episode 初始化
  │     │     ├── Weather Actor 生成
  │     │     ├── ActorFactory 实例化
  │     │     ├── Traffic Manager 启动
  │     │     └── CARLA RPC Server 启动 (port 2000)
  │     │
  │     ├── 创建 SpectatorPawn（不 possess）
  │     │     └── 注册到 Episode（可通过 CARLA API 移动 Spectator）
  │     │
  │     ├── AirSim 引导
  │     │     ├── InitializeAirSimSettings() — 读取 settings.json
  │     │     ├── SetUnrealEngineSettings() — 关闭 MotionBlur, 启用 CustomDepth
  │     │     ├── CreateSimMode() — SpawnActor<ASimModeWorldMultiRotor>
  │     │     │     └── SimModeBase::BeginPlay()
  │     │     │           ├── 获取 PlayerStart Transform
  │     │     │           ├── 初始化 NedTransform（不移动世界原点!）
  │     │     │           ├── 创建 WorldSimApi
  │     │     │           └── 创建无人机 Pawn
  │     │     ├── CreateAirSimWidget() — HUD Widget 添加到 Viewport
  │     │     ├── SetupAirSimInputBindings() — R/T/1/2/3/0 键绑定
  │     │     └── SimMode->startApiServer() — AirSim RPC Server 启动 (port 41451)
  │     │
  │     └── 完成！两个 API 服务器同时运行
  │
  ├── 3. Tick Loop (每帧)
  │     ├── Super::Tick() — CARLA Recorder tick
  │     └── AirSim Debug Report Widget 更新
  │
  └── 4. EndPlay()
        ├── SimMode->stopApiServer()
        ├── 销毁 Widget, SimMode
        └── Super::EndPlay() — CARLA 清理
```

## 关键数据流

### 车辆生成流程
```
Python: world.spawn_actor(vehicle_bp, transform)
  → CARLA RPC Server
    → CarlaEpisode::SpawnActor()
      → ActorDispatcher → VehicleFactory
        → UE4 World SpawnActor()
          → 物理引擎接管 (Chaos/PhysX)
```

### 无人机控制流程
```
Python: client.moveToPositionAsync(x, y, z, speed)
  → AirSim RPC Server
    → SimModeWorldMultiRotor
      → SimpleFlight Controller
        → NedTransform 坐标转换 (NED → UE4)
          → Drone Pawn 移动
```

### 传感器数据流
```
CARLA 传感器:
  Camera/Lidar/Radar → SensorManager → Raw Data → RPC → Python numpy array

AirSim 传感器:
  PIPCamera → RenderTarget → simGetImages() → RPC → Python numpy array
```

## 文件修改详情

### 修改清单（共 3 个文件修改，2 个新文件）

| 文件 | 修改类型 | 修改行数 | 目的 |
|------|---------|---------|------|
| `SimWorldGameMode.h` | **新建** | 75 行 | 统一 GameMode 类声明 |
| `SimWorldGameMode.cpp` | **新建** | 548 行 | 统一 GameMode 完整实现 |
| `CarlaGameModeBase.h` | 修改 2 处 | ~4 行 | `WeatherClass`/`ActorFactories` → protected |
| `CarlaEpisode.h` | 修改 1 处 | 1 行 | 添加 `friend class ASimWorldGameMode` |
| `SimModeBase.cpp` | 修改 1 处 | ~6 行 | 注释掉 `SetNewWorldOrigin()` |

### 每个修改的精确位置

**CarlaGameModeBase.h**:
```cpp
// 原始: private 区域
// 修改: 移到 protected 区域
protected:
    UPROPERTY(EditAnywhere)
    TSubclassOf<AWeather> WeatherClass;

    UPROPERTY(EditAnywhere)
    TArray<TSubclassOf<ACarlaActorFactory>> ActorFactories;
```

**CarlaEpisode.h** (~第 338 行):
```cpp
// 添加:
friend class ASimWorldGameMode;
```

**SimModeBase.cpp** (~第 119 行):
```cpp
// 原始:
// this->GetWorld()->SetNewWorldOrigin(FIntVector(player_loc) + FIntVector(0, 0, 0));

// 修改为注释 + 说明:
// NOTE: Do NOT call SetNewWorldOrigin() here. In CarlaAir...
global_ned_transform_.reset(new NedTransform(player_start_transform,
                                             UAirBlueprintLib::GetWorldToMetersScale(this)));
```

## Town10HD 地图布局

```
                    Y=-69
                     │
    Ocean (x<20)     │    Inland City (x>55)
                     │
  ┌──────────────────┼──────────────────────────┐
  │                  │                           │
  │   Coast/Beach    │    ┌──────────────────┐   │
  │                  │    │  City Center     │   │
  │   PlayerStart    │    │  (80, 30)        │   │
  │   (x≈0, NED     │    │                  │   │
  │    origin)       │    │  Buildings,      │   │  Y=141
  │                  │    │  Roads,          │   │
  │                  │    │  Intersections   │   │
  │                  │    └──────────────────┘   │
  │                  │                           │
  └──────────────────┼──────────────────────────┘
  X=-115             │                     X=110

  NED 原点在 PlayerStart (x≈0)
  城市中心在 NED (80, 0, -30) = CARLA (80, 0, 30m高)
```

## 已知限制和约束

| 约束 | 原因 | 解决方案 |
|------|------|---------|
| Walker 不能使用 AI Controller | `go_to_location()` 触发四元数错误导致 segfault | 使用静态 Walker |
| `simSetCameraPose` 在 Shipping 包中崩溃 | C++ abort in Shipping build | 不调用，使用无人机轨道 + yaw tracking |
| AirSim 活动时最多 ~8 个 autopilot 车辆 | 概率性四元数崩溃 | 批量启用 autopilot，延迟 0.5s |
| 纯 CARLA 脚本可安全处理 50+ 车辆 | 无 AirSim 干扰 | 展示交通时不启动无人机 |
