# CarlaAir 所有修改清单

本文档列出了 CarlaAir 相对于原版 CARLA 0.9.16 和 AirSim 的所有代码修改，方便开发者精确了解改动范围。

## 修改统计

| 类型 | 数量 | 说明 |
|------|------|------|
| **新建文件** | 2 | SimWorldGameMode.h, SimWorldGameMode.cpp |
| **修改文件** | 3 | CarlaGameModeBase.h, CarlaEpisode.h, SimModeBase.cpp |
| **配置修改** | 1 | DefaultGame.ini (添加 AirSim 资源条目) |
| **新建脚本** | 30+ | carlaAir.sh, 示例脚本, Demo 脚本 |

## 详细修改

### 1. [新建] SimWorldGameMode.h

**路径**: `Unreal/CarlaUE4/Plugins/AirSim/Source/SimWorldGameMode.h`
**行数**: 75 行
**放置在 AirSim 插件目录的原因**: 需要引用 AirSim 头文件 (SimHUDWidget.h, SimModeBase.h, PIPCamera.h, AirSimSettings.hpp)

**类声明**:
```cpp
UCLASS()
class AIRSIM_API ASimWorldGameMode : public ACarlaGameModeBase
```

**公共接口**:
- 构造函数、BeginPlay、Tick、EndPlay（override）
- 8 个输入事件处理函数（Recording, Report, Help, Trace, SubWindow 0/1/2, All）

**私有成员**:
- `USimHUDWidget* Widget_` — AirSim HUD 控件
- `ASimModeBase* SimMode_` — AirSim 模拟模式 Actor
- `APIPCamera* SubwindowCameras_[3]` — 子窗口相机

### 2. [新建] SimWorldGameMode.cpp

**路径**: `Unreal/CarlaUE4/Plugins/AirSim/Source/SimWorldGameMode.cpp`
**行数**: 548 行

**关键实现**:

| 函数 | 行号 | 功能 |
|------|------|------|
| 构造函数 | 66-123 | DefaultPawnClass=nullptr, 加载BP_Weather, 注册8个Factory |
| BeginPlay | 127-186 | CARLA初始化 → 创建SpectatorPawn → AirSim引导 |
| Tick | 190-198 | CARLA Recorder + AirSim Widget更新 |
| EndPlay | 202-224 | 停止API → 销毁Widget/SimMode → CARLA清理 |
| InitializeAirSimSettings | 228-244 | 读取settings.json |
| SetUnrealEngineSettings | 246-259 | 关闭MotionBlur, 启用CustomDepth |
| CreateSimMode | 263-286 | 根据SimMode类型SpawnActor |
| CreateAirSimWidget | 290-327 | 创建并初始化HUD Widget |
| SetupAirSimInputBindings | 362-375 | 绑定 R/;/F1/T/1/2/3/0 键 |
| GetSettingsText | 497-503 | 搜索settings.json (4个位置) |

### 3. [修改] CarlaGameModeBase.h

**路径**: `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.h`
**修改量**: 2 处，约 4 行

**修改前** (在 `private:` 区域):
```cpp
private:
    UPROPERTY(EditAnywhere)
    TSubclassOf<AWeather> WeatherClass;

    UPROPERTY(EditAnywhere)
    TArray<TSubclassOf<ACarlaActorFactory>> ActorFactories;
```

**修改后** (移到 `protected:` 区域):
```cpp
protected:
    UPROPERTY(EditAnywhere)
    TSubclassOf<AWeather> WeatherClass;

    UPROPERTY(EditAnywhere)
    TArray<TSubclassOf<ACarlaActorFactory>> ActorFactories;
```

**影响**: 仅影响访问权限，不改变运行时行为。子类 ASimWorldGameMode 可以在构造函数中设置这些属性。

### 4. [修改] CarlaEpisode.h

**路径**: `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaEpisode.h`
**修改量**: 1 处，1 行

**添加位置**: 类声明内部，约第 338 行

```cpp
friend class ASimWorldGameMode;
```

**影响**: 允许 SimWorldGameMode 直接访问 Episode 的私有成员（主要是 `Spectator` 和 `ActorDispatcher`），用于注册手动创建的 SpectatorPawn。

### 5. [修改] SimModeBase.cpp

**路径**: `Unreal/CarlaUE4/Plugins/AirSim/Source/SimMode/SimModeBase.cpp`
**修改量**: 1 处，约 6 行

**修改前** (~第 119 行):
```cpp
this->GetWorld()->SetNewWorldOrigin(FIntVector(player_loc) + FIntVector(0, 0, 0));
global_ned_transform_.reset(new NedTransform(player_start_transform,
                                             UAirBlueprintLib::GetWorldToMetersScale(this)));
```

**修改后**:
```cpp
// NOTE: Do NOT call SetNewWorldOrigin() here. In CarlaAir (CARLA + AirSim integration),
// shifting the world origin breaks CARLA's landscape/road collision, causing all ground
// vehicles to fall through the map (~30m underground). The NedTransform is initialized
// with the original player_start_transform, which correctly serves as the NED origin
// for AirSim coordinate conversion without moving the world.
global_ned_transform_.reset(new NedTransform(player_start_transform,
                                             UAirBlueprintLib::GetWorldToMetersScale(this)));
```

**影响**: 车辆不再穿过地面（z≈0.00 正常行驶），AirSim 无人机坐标系不受影响（NedTransform 仍然使用 PlayerStart 作为原点）。

### 6. [修改] DefaultGame.ini

**路径**: `Unreal/CarlaUE4/Config/DefaultGame.ini`
**修改量**: 新增约 8 行

**新增内容**:
```ini
# AirSim content directories to cook
+DirectoriesToAlwaysCook=(Path="/AirSim/Blueprints")
+DirectoriesToAlwaysCook=(Path="/AirSim/HUDAssets")
+DirectoriesToAlwaysCook=(Path="/AirSim/Models")
+DirectoriesToAlwaysCook=(Path="/AirSim/Weather")
+DirectoriesToAlwaysCook=(Path="/AirSim/StarterContent")
+DirectoriesToAlwaysCook=(Path="/AirSim/VehicleAdv")

# AirSim assets map
+MapsToCook=(FilePath="/AirSim/AirSimAssets")
```

**影响**: 打包时包含所有 AirSim 资源（无人机模型、HUD、天气特效等），共 340 个文件。

## 未修改的关键文件（供参考）

以下文件是原版 CARLA/AirSim 的，未做修改但对理解系统重要：

- `CarlaGameModeBase.cpp` — CARLA GameMode 实现
- `CarlaEpisode.cpp` — Episode 管理
- `SimModeWorldMultiRotor.cpp` — 多旋翼飞行器模式
- `SimpleFlight/SimpleFlightApi.cpp` — 简单飞行控制器
- `PIPCamera.cpp` — AirSim 相机实现
- `AirBlueprintLib.cpp` — AirSim UE4 工具库
