# CarlaAir 未来开发指南：3DGS 和 World Model 集成

## 概述

本文档为后续开发者（或工程 LLM）提供在 CarlaAir 基础上集成 **3D Gaussian Splatting (3DGS)** 和 **World Model (WM)** 渲染管线的详细入口点和技术指南。

## 当前渲染管线

CarlaAir 当前使用 UE4 4.26 的标准延迟渲染管线：

```
场景几何 → Deferred Shading → Post-Processing → 输出帧
  │                                    │
  ├── CARLA 传感器: sensor.camera.rgb/depth/seg
  │     └── UE4 SceneCapture2D → RenderTarget → Raw Data → RPC
  │
  └── AirSim 传感器: simGetImages()
        └── APIPCamera → UE4 SceneCapture2D → RenderTarget → API
```

## 3DGS 集成入口点

### 方案 A: 替换 UE4 渲染后端（深度集成）

**入口文件**: 无需修改 CarlaAir 代码，在 UE4 引擎层替换

1. **UE4 Renderer 替换**:
   - 路径: `/mnt/data1/tianle/carla_ue4/Engine/Source/Runtime/Renderer/`
   - 在 `DeferredShadingRenderer.cpp` 的 Render() 函数中插入 3DGS 渲染通道
   - 或创建新的 `F3DGSSceneRenderer` 继承 `FSceneRenderer`

2. **SceneCapture 替换**:
   - 当前 CARLA 和 AirSim 传感器都通过 `USceneCaptureComponent2D` 获取渲染结果
   - 可以创建自定义的 `U3DGSCaptureComponent` 替换标准 SceneCapture

### 方案 B: 传感器级别替换（轻量集成）

**入口文件**: `Plugins/AirSim/Source/PIPCamera.cpp`

1. **PIPCamera 修改**:
   ```cpp
   // PIPCamera.cpp 中的 getCameraImage() 函数
   // 当前: 从 UE4 RenderTarget 读取像素
   // 修改: 调用 3DGS 渲染器获取图像
   ```

2. **CARLA 传感器修改**:
   - 路径: `Plugins/Carla/Source/Carla/Sensor/SceneCaptureSensor.cpp`
   - `FPixelReader::SendPixelsInRenderThread()` 是传感器数据输出点
   - 可以在此注入 3DGS 渲染结果

### 方案 C: Python 端后处理（最轻量）

不修改 C++ 代码，在 Python 客户端端进行 3DGS 重建和渲染：

```python
import carla
import airsim
import torch  # 3DGS framework

# 1. 从 CarlaAir 获取传感器数据
rgb = carla_camera.get_image()
depth = carla_depth.get_image()
pose = spectator.get_transform()

# 2. 使用 3DGS 渲染
gs_renderer = GaussianSplatRenderer(point_cloud)
rendered = gs_renderer.render(pose, intrinsics)

# 3. 融合或对比
comparison = np.hstack([rgb, rendered])
```

## World Model 集成入口点

### 方案 A: 替换物理引擎预测

**入口文件**: `Plugins/AirSim/Source/SimMode/SimModeBase.cpp`

```cpp
// SimModeBase.cpp: setupPhysicsLoopPeriod()
// 当前: UE4 PhysX/Chaos 物理引擎驱动
// 可以在此插入 World Model 预测步骤

void SimModeBase::setupPhysicsLoopPeriod() {
    // 原始物理设置...

    // [新增] World Model 初始化
    // world_model_ = std::make_unique<WorldModel>(config);
    // world_model_->loadCheckpoint("path/to/checkpoint");
}
```

### 方案 B: 并行运行 World Model

在 Python 端并行运行 World Model，使用 CARLA/AirSim 作为 ground truth：

```python
# 1. CarlaAir 提供当前状态
state = get_simulation_state(carla_world, airsim_client)

# 2. World Model 预测下一帧
predicted_state = world_model.predict(state, action)

# 3. CarlaAir 执行动作，获取真实下一帧
carla_world.tick()
actual_state = get_simulation_state(carla_world, airsim_client)

# 4. 对比和训练
loss = compute_loss(predicted_state, actual_state)
```

### 方案 C: 替换渲染输出

**入口文件**: `Plugins/AirSim/Source/SimMode/SimModeBase.cpp`

在 `Tick()` 中插入 World Model 渲染：

```cpp
void SimModeBase::Tick(float DeltaSeconds) {
    Super::Tick(DeltaSeconds);

    // [新增] 如果启用了 World Model 渲染
    if (bUseWorldModelRenderer) {
        // 获取当前车辆/无人机状态
        auto vehicle_states = getVehicleStates();
        // World Model 预测渲染
        auto rendered_frame = world_model_->render(vehicle_states, camera_pose);
        // 替换 PIPCamera 的输出
        updateCameraOutputs(rendered_frame);
    }
}
```

## 关键文件路径速查

| 功能 | 文件路径 |
|------|---------|
| GameMode 入口 | `Plugins/AirSim/Source/SimWorldGameMode.cpp` |
| CARLA 传感器基类 | `Plugins/Carla/Source/Carla/Sensor/Sensor.h` |
| CARLA 相机传感器 | `Plugins/Carla/Source/Carla/Sensor/SceneCaptureSensor.cpp` |
| AirSim 相机 | `Plugins/AirSim/Source/PIPCamera.cpp` |
| AirSim 图像获取 | `Plugins/AirSim/Source/SimMode/SimModeBase.cpp` → `getImages()` |
| 物理循环 | `SimModeBase.cpp` → `setupPhysicsLoopPeriod()` |
| 无人机控制器 | `Plugins/AirSim/Source/Vehicles/Multirotor/` |
| 坐标转换 | `Plugins/AirSim/Source/NedTransform.h/.cpp` |
| UE4 渲染器 | `/mnt/data1/tianle/carla_ue4/Engine/Source/Runtime/Renderer/` |
| 地图资源 | `Unreal/CarlaUE4/Content/Carla/Maps/Town10HD/` |
| Python CARLA API | `PythonAPI/carla/source/` |

## 推荐开发顺序

1. **第一阶段**: 方案 C（Python 端后处理）— 不修改 C++ 代码，快速验证
2. **第二阶段**: 方案 B（传感器级替换）— 修改 PIPCamera 和 SceneCaptureSensor
3. **第三阶段**: 方案 A（引擎级替换）— 修改 UE4 渲染器

每个阶段都可以独立发布和测试，不影响其他功能。
