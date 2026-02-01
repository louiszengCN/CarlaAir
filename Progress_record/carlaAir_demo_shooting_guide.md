# CARLA-Air Demo 拍摄工程化指南

**文档用途**：供工程师参考，编写 Python 脚本录制 30 个 Demo 视频  
**核心原则**：所有内容均来自 CARLA-Air 原生仿真，不依赖后期特效或字幕叠加  
**通用技术规范**：见文末附录  
**日期**：2026-03-12（v1.1 更新：新增无人机第三方可见性强制要求）

---

## ⚠️ 强制要求：无人机必须在画面中可见（第三方可见性）

> **这是本次拍摄最重要的单条规范，适用于所有涉及无人机的 Demo（共 25 个）。**

### 为什么这是强制要求

CARLA-Air 的核心卖点是**空地一体**——无人机和地面车辆真实地共存于同一个仿真世界中。如果画面中只有无人机挂载的相机视角（即"从天上往下看"），观众无法区分这和"把一个普通相机架在高处"有什么本质区别。**只有当无人机的三维模型出现在画面中，观众才能真正感受到"有一架无人机在飞"这件事。**

具体来说，以下两种画面呈现方式的效果截然不同：

| 错误做法（禁止） | 正确做法（要求） |
|---|---|
| 只有无人机俯视 RGB 相机的画面，画面中看不到无人机本身 | 至少有一路画面能看到无人机的三维模型在空中飞行 |
| 用 Spectator 相机悬停在高空模拟"无人机视角" | 用挂载在车辆或独立固定点的外部相机，拍摄无人机在空中运动的全景 |
| 所有画面都是第一人称视角（车内/机载） | 必须有第三方视角，画面中同时出现无人机和地面车辆/行人 |

### 实现原理：AirSim 无人机模型的可见性

AirSim 中的无人机是一个完整的三维网格体（Mesh），在 UE4 场景中真实存在。CARLA 的相机传感器（`sensor.camera.rgb`）能够拍摄到 UE4 场景中的所有物体，包括 AirSim 的无人机模型。因此，**只要将 CARLA 相机放置在合适的位置，就能拍摄到无人机在空中飞行的画面**，不需要任何特殊配置。

关键前提：AirSim 和 CARLA 必须运行在同一个 UE4 实例中（即 CARLA-Air 的单进程架构），这样 CARLA 相机才能"看到"AirSim 的无人机。这正是 CARLA-Air 相比分离运行方案的核心优势之一。

### 三种标准的第三方可见相机方案

工程师可以根据每个 Demo 的具体场景选择以下方案之一，或组合使用：

**方案 A：车载后置高位相机（最推荐，最能体现空地一体）**

将相机挂载在地面车辆的后方高位，相机朝向前上方，使得画面中同时出现：车辆后部/车顶（前景）+ 无人机在空中飞行（中景）+ 城市环境（背景）。这个视角最直观地传达了"无人机跟着车在飞"的信息。

```python
# 车载后置高位相机：挂载在车辆后方，高度 4m，朝向前上方
# 画面中：车顶在画面下方，无人机在画面中央偏上
cam_rear_high = carla.Transform(
    carla.Location(x=-4.0, y=0.0, z=4.0),   # 车辆后方4m，高4m
    carla.Rotation(pitch=-20, yaw=0, roll=0)  # 略向下俯视，确保车顶和无人机都在画面中
)
camera_rear = world.spawn_actor(camera_bp, cam_rear_high, attach_to=vehicle)
# 注意：attach_to=vehicle 使相机随车辆移动，始终保持相对位置
# pitch=-20° 的设计逻辑：
#   - 无人机在车辆正上方 20m → 相机需要向上看约 arctan(20/4) ≈ 78°
#   - 但我们希望画面中同时看到车顶和无人机，所以相机朝向略向下
#   - 实际 pitch 值需要根据无人机飞行高度微调（高度越高，pitch 越接近 0°）
```

**方案 B：独立跟随相机（Orbit / Chase，适合展示全局关系）**

使用 CARLA Spectator 或独立 Actor 相机，以车辆和无人机的中间位置为中心进行 Orbit 运动。画面中同时出现无人机（上方）和车辆（下方），展示两者的空间关系。

```python
# 以车辆位置为中心，相机在后方斜上方，同时拍摄车辆和无人机
def get_air_ground_chase_transform(vehicle_transform, drone_location,
                                    distance=15.0, height=8.0):
    """
    计算能同时看到车辆和无人机的相机位置。
    相机位于车辆后方，高度介于车辆和无人机之间。
    """
    forward = vehicle_transform.get_forward_vector()
    veh_loc = vehicle_transform.location
    
    # 相机位于车辆正后方，高度为无人机高度的 40%
    drone_height = drone_location.z - veh_loc.z  # 无人机相对高度
    cam_height = drone_height * 0.4 + veh_loc.z  # 相机高度：车辆和无人机之间偏下
    
    cam_loc = carla.Location(
        x=veh_loc.x - forward.x * distance,
        y=veh_loc.y - forward.y * distance,
        z=cam_height
    )
    
    # 相机朝向：指向车辆和无人机的中间位置
    mid_z = (veh_loc.z + drone_location.z) / 2
    mid_point = carla.Location(veh_loc.x, veh_loc.y, mid_z)
    
    dx = mid_point.x - cam_loc.x
    dy = mid_point.y - cam_loc.y
    dz = mid_point.z - cam_loc.z
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(dz, math.sqrt(dx**2 + dy**2)))
    
    return carla.Transform(cam_loc, carla.Rotation(pitch=pitch, yaw=yaw))
```

**方案 C：地面固定机位（适合展示无人机飞越/降落等特定动作）**

在场景中放置一个固定的 CARLA 相机 Actor（不挂载在任何车辆上），选择一个能同时看到道路和空中的角度。无人机飞过时，画面中能看到无人机从镜头前飞过，地面车辆和行人在下方。

```python
# 固定机位相机：放置在路边，低角度仰拍
# 画面中：前景是行人/车辆，背景是天空，无人机从画面中飞过
cam_fixed_transform = carla.Transform(
    carla.Location(x=50.0, y=-5.0, z=1.8),   # 路边，人眼高度
    carla.Rotation(pitch=15, yaw=90, roll=0)   # 略向上仰拍，捕捉空中无人机
)
camera_fixed = world.spawn_actor(camera_bp, cam_fixed_transform)
# 注意：不 attach_to 任何 actor，相机固定在世界坐标中
```

### 每个 Demo 的相机方案选择指引

| Demo | 推荐方案 | 说明 |
|------|----------|------|
| D01 空地协同追踪 | 方案 A + 方案 B | 车载后置相机展示跟随关系，Orbit 展示全局 |
| D04 无人机追踪行人 | 方案 B + 方案 C | Orbit 展示无人机和行人的空间关系，固定机位展示无人机俯冲 |
| D05 十字路口监视 | 方案 C | 固定机位从路边仰拍，无人机悬停在路口上方 |
| D06 多机编队 | 方案 B | Orbit 展示三架无人机的编队队形 |
| D07 低空穿越行人 | 方案 C | 固定机位在街道侧方，拍摄无人机从行人中穿过 |
| D08 夜间巡逻 | 方案 A | 车载后置相机，夜间无人机指示灯清晰可见 |
| D09 雨天应急 | 方案 B | Orbit 展示雨中无人机和车辆 |
| D10 LiDAR 扫描 | 方案 A | 车载后置相机，无人机在车辆上方清晰可见 |
| D11 空中深度感知 | 方案 C | 固定机位拍摄无人机沿建筑物立面飞行 |
| D13 环绕建筑检测 | 方案 C | 固定机位拍摄无人机螺旋环绕建筑物 |
| D14 车辆紧急制动 | 方案 A | 车载后置相机，无人机在制动瞬间从上方俯冲 |
| D15 行人密集广场 | 方案 C | 固定机位仰拍，无人机从低到高上升 |
| D16 双车道超车 | 方案 A | 车载后置相机，无人机跟随超车过程 |
| D17 FPV 穿楼追车 | 方案 C | 固定机位拍摄无人机从建筑物之间穿过 |
| D19 空地双视角 | 方案 A + 方案 B | 最重要的 Demo，两种方案都用 |
| D22 红外夜间检测 | 方案 A | 车载后置相机，夜间无人机轮廓清晰 |
| D23 搜救场景 | 方案 B | Orbit 展示无人机搜索和车辆救援的协同 |
| D25 高速伴飞 | 方案 A | 车载后置相机，无人机在侧方伴飞清晰可见 |
| D26 城市峡谷 | 方案 C | 固定机位在峡谷底部仰拍，无人机在建筑物之间飞行 |
| D27 双 LiDAR | 方案 A | 车载后置相机，空中无人机和地面 LiDAR 同框 |
| D28 编队队形变换 | 方案 B | Orbit 展示三架无人机的队形变换 |
| D29 全天候循环 | 方案 B | Orbit 展示不同光照下无人机和车辆 |
| D30 综合压力测试 | 方案 B + 方案 C | 多视角展示最大规模场景 |

---

## 通用录制规范（所有 Demo 适用）

### 画面规格

| 参数 | 值 |
|------|----|
| 录制分辨率 | 2560 × 1440（2K） |
| 输出分辨率 | 1920 × 1080（1080p） |
| 帧率 | 30 FPS（CARLA 同步模式，`world.tick()` 驱动） |
| 同步模式 | 必须开启 `synchronous_mode = True`，确保所有传感器帧号一致 |
| HUD | 关闭所有调试信息（`world.debug` 不调用，`spectator` 相机不显示 UI） |

### 相机通用参数

```python
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '2560')
camera_bp.set_attribute('image_size_y', '1440')
camera_bp.set_attribute('fov', '90')          # 标准广角，大多数 Demo 使用
camera_bp.set_attribute('fov', '60')          # 长焦压缩感，追踪类 Demo 使用
camera_bp.set_attribute('motion_blur_intensity', '0.45')  # 运动模糊，增加电影感
camera_bp.set_attribute('motion_blur_max_distortion', '0.35')
camera_bp.set_attribute('lens_flare_intensity', '0.1')    # 轻微镜头光晕
```

### 第三人称跟随相机的通用实现

```python
# 跟随车辆的第三人称相机（后方高位视角）
def get_third_person_transform(vehicle_transform, distance=8.0, height=3.5):
    forward = vehicle_transform.get_forward_vector()
    location = vehicle_transform.location
    cam_location = carla.Location(
        x=location.x - forward.x * distance,
        y=location.y - forward.y * distance,
        z=location.z + height
    )
    cam_rotation = carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw)
    return carla.Transform(cam_location, cam_rotation)
```

### Spectator 相机（自由视角）的运动实现

```python
# 在每个 tick 中平滑移动 spectator 相机（Orbit 环绕运动）
import math

def orbit_camera(world, center, radius, height, angle_deg, pitch=-30):
    angle_rad = math.radians(angle_deg)
    loc = carla.Location(
        x=center.x + radius * math.cos(angle_rad),
        y=center.y + radius * math.sin(angle_rad),
        z=center.z + height
    )
    yaw = math.degrees(math.atan2(center.y - loc.y, center.x - loc.x))
    rot = carla.Rotation(pitch=pitch, yaw=yaw)
    world.get_spectator().set_transform(carla.Transform(loc, rot))
```

### 录制方式

使用 OBS Studio 录制屏幕，或直接用 CARLA 的 `sensor.camera.rgb` 将帧保存为图片序列，再用 FFmpeg 合成视频：

```bash
ffmpeg -framerate 30 -i frame_%06d.png -c:v libx264 -crf 18 -pix_fmt yuv420p output.mp4
```

---

## Demo 总览表

| # | Demo 名称 | 核心 Feature | 智能体 | 地图 | 时长 | 优先级 |
|---|-----------|-------------|--------|------|------|--------|
| D01 | 空地协同追踪·单车单机 | 空地同步 | 无人机+车+人 | Town10HD | 60s | P0 |
| D02 | 天气切换同步 | 单进程天气一致性 | 无人机+车+人 | Town10HD | 45s | P0 |
| D03 | 多模态传感器六格分屏 | 18路传感器同步 | 无人机+车 | Town03 | 60s | P0 |
| D04 | 无人机追踪行人·城市街道 | 行人控制+空中追踪 | 无人机+人 | Town10HD | 60s | P0 |
| D05 | 十字路口交通监视 | 航拍+语义分割 | 无人机+车+人 | Town10HD | 45s | P0 |
| D06 | 多机编队·异构目标追踪 | 多无人机协同 | 3无人机+车+人 | Town10HD | 60s | P1 |
| D07 | 无人机低空穿越行人街道 | 社会导航场景 | 无人机+人 | Town10HD | 45s | P0 |
| D08 | 夜间城市巡逻 | 夜间光照+多智能体 | 无人机+车+人 | Town10HD | 45s | P1 |
| D09 | 雨天应急响应 | 极端天气传感器 | 无人机+车+人 | Town10HD | 45s | P1 |
| D10 | LiDAR 点云城市扫描 | 地面LiDAR可视化 | 无人机+车+人 | Town03 | 45s | P0 |
| D11 | 空中深度感知·建筑轮廓 | 无人机深度相机 | 无人机+人 | Town10HD | 45s | P1 |
| D12 | 语义分割全景·城市理解 | 语义分割多视角 | 无人机+车+人 | Town10HD | 45s | P1 |
| D13 | 无人机环绕建筑物检测 | 自主飞行轨迹 | 无人机+人 | Town10HD | 45s | P1 |
| D14 | 车辆紧急制动·空中见证 | 碰撞传感器+俯视 | 无人机+车+人 | Town10HD | 30s | P1 |
| D15 | 行人密集广场·空中统计 | 行人群体控制 | 无人机+人 | Town10HD | 45s | P1 |
| D16 | 双车道超车·空地同步 | 多车控制+追踪 | 无人机+多车+人 | Town04 | 45s | P1 |
| D17 | 无人机 FPV 穿楼追车 | 动态飞行路径 | 无人机+车+人 | Town10HD | 30s | P1 |
| D18 | 地面语义导航·行人避让 | 地面感知+行人 | 车+人 | Town10HD | 45s | P1 |
| D19 | 空地双视角·同一事件 | 视角对比叙事 | 无人机+车+人 | Town10HD | 45s | P0 |
| D20 | 多天气序列·传感器对比 | 14种天气遍历 | 无人机+车 | Town10HD | 60s | P1 |
| D21 | 无人机悬停·GPS漂移测试 | IMU/GPS传感器 | 无人机+人 | Town10HD | 30s | P2 |
| D22 | 红外相机·夜间行人检测 | 无人机红外传感器 | 无人机+人 | Town10HD | 45s | P1 |
| D23 | 空地协同·搜救场景 | 多智能体任务协同 | 无人机+车+人 | Town03 | 60s | P1 |
| D24 | 行人轨迹预测·数据采集 | Walker AI控制 | 无人机+人 | Town10HD | 45s | P1 |
| D25 | 高速公路·无人机伴飞 | 高速场景+追踪 | 无人机+车 | Town04 | 45s | P1 |
| D26 | 城市峡谷·GPS遮蔽仿真 | 城市峡谷场景 | 无人机+车+人 | Town01 | 45s | P2 |
| D27 | 空地双LiDAR·点云融合 | 双平台LiDAR对比 | 无人机+车+人 | Town03 | 45s | P1 |
| D28 | 无人机编队·队形变换 | 多机协同轨迹 | 3无人机 | Town10HD | 45s | P2 |
| D29 | 全天候循环·24小时仿真 | 光照时间变化 | 无人机+车+人 | Town10HD | 60s | P1 |
| D30 | 综合压力测试·最大规模 | 平台极限能力 | 多无人机+多车+多人 | Town10HD | 60s | P1 |

---

# PART 1：Demo D01–D15 详细拍摄指南

---

## D01：空地协同追踪·单车单机

**核心 Feature**：无人机与地面车辆在同一世界实时协同运动，三个视角同步呈现同一物理事件  
**导演意图**：这是整套 Demo 的"基础款"，用最清晰的方式证明"两个仿真器在同一个世界里"。三分屏的设计让观众同时看到三个视角，空间关系一目了然。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon（晴天正午，光线最清晰） |
| 时长 | 60 秒 |
| 车辆 | Tesla Model 3（`vehicle.tesla.model3`），自动驾驶模式 |
| 无人机 | AirSim 默认四旋翼，跟随车辆正上方 20m |
| 行人 | 10–15 名行人在道路两侧人行道自然行走 |

### 相机配置（四路）

> **✅ 第三方可见性要求：相机 D 是本 Demo 的核心新增相机，必须确保画面中能看到无人机的三维模型在车辆上方飞行。**

**相机 A：无人机俯视 RGB**（画面左上）
```python
# 挂载在 AirSim 无人机机体正下方，朝向垂直向下
# AirSim 相机配置（settings.json 中设置）
{
  "CameraName": "bottom_center",
  "CaptureSettings": [{"ImageType": 0, "Width": 1280, "Height": 720}],
  "X": 0, "Y": 0, "Z": 0.1,
  "Pitch": -90, "Roll": 0, "Yaw": 0
}
```

**相机 B：车辆第一人称**（画面左下）
```python
# 挂载在车辆前保险杠上方，模拟驾驶员视角
cam_b_transform = carla.Transform(
    carla.Location(x=2.0, y=0.0, z=1.4),
    carla.Rotation(pitch=-5, yaw=0, roll=0)
)
camera_b = world.spawn_actor(camera_bp, cam_b_transform, attach_to=vehicle)
```

**相机 C：全局第三人称 Orbit**（画面右侧，占右半屏）
```python
# 以车辆为中心，半径 25m，高度 12m，缓慢顺时针 Orbit
# 每 tick 旋转 0.3°，60秒旋转约 54°（不超过半圈，保持方向感）
orbit_radius = 25.0
orbit_height = 12.0
orbit_speed = 0.3  # 度/帧
# 重要：Orbit 相机的高度设置为 12m，无人机在 20m 高度
# 因此 Orbit 画面中：车辆在下方，无人机在上方，两者同时在画面中
```

**相机 D：车载后置高位相机（画面右下角小窗，占 30% 宽度）——第三方可见性关键相机**
```python
# 挂载在车辆后方，高度 4m，朝向前上方
# 画面中：车顶在下方，无人机在画面中央，城市天空在背景
cam_d_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),   # 车辆后方5m，高4m
    carla.Rotation(pitch=-15, yaw=0, roll=0)  # 略向下，确保车顶和无人机都在画面内
)
camera_d = world.spawn_actor(camera_bp, cam_d_transform, attach_to=vehicle)
# 工程师注意：
# 1. attach_to=vehicle 使相机随车辆移动，始终保持相对位置
# 2. pitch=-15° 是初始值，需要根据无人机实际飞行高度调整：
#    - 无人机在 20m 高度时：pitch 设为 -10° 到 -15°（车顶和无人机都在画面中）
#    - 无人机在 10m 高度时：pitch 设为 -20° 到 -25°（无人机不会超出画面上边）
# 3. 如果无人机不在车辆正上方而是偶尔偏移，需要小幅调整 yaw 值
```

### 运镜设计

全程 60 秒分为三个阶段：

**阶段一（0–20s）：建立场景**  
Orbit 相机从车辆正后方开始，缓慢向右旋转。车辆在 Town10HD 主干道直行，无人机保持正上方 20m 匀速跟随。行人在两侧人行道自然行走。这 20 秒的目的是让观众建立空间感——无人机在哪，车在哪，人在哪。

**阶段二（20–45s）：核心事件——转弯**  
车辆在十字路口左转。这是整个 Demo 的关键时刻：无人机需要同步调整航向跟随车辆转弯，Orbit 相机也跟随调整中心点。三个视角同时呈现"转弯"这一物理事件：左上俯视看到车辆转弯轨迹，左下第一人称看到路口视角，右侧全局看到无人机和车辆同步转向。

**阶段三（45–60s）：拉高收尾**  
转弯完成后，Orbit 相机缓慢拉高（从 12m 到 25m），同时扩大 Orbit 半径（从 25m 到 40m），逐渐展示更大范围的城市环境。无人机也同步上升到 35m 高度，俯视画面中城市街道全景展开。最后 5 秒静止，画面稳定。

### Python 实现要点

```python
import carla
import airsim
import time
import math

# 1. 连接两个仿真器
carla_client = carla.Client('localhost', 2000)
carla_world = carla_client.get_world()
airsim_client = airsim.MultirotorClient()
airsim_client.confirmConnection()

# 2. 开启同步模式
settings = carla_world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 1.0 / 30.0  # 30 FPS
carla_world.apply_settings(settings)

# 3. 生成车辆并开启自动驾驶
blueprint_library = carla_world.get_blueprint_library()
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_point = carla_world.get_map().get_spawn_points()[0]
vehicle = carla_world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)

# 4. 生成行人（10名，在车辆附近人行道）
walker_bps = blueprint_library.filter('walker.pedestrian.*')
for i in range(10):
    walker_bp = random.choice(walker_bps)
    walker_spawn = carla.Transform(
        carla.Location(x=spawn_point.location.x + random.uniform(-20, 20),
                       y=spawn_point.location.y + random.uniform(5, 10),
                       z=spawn_point.location.z + 0.5)
    )
    walker = carla_world.try_spawn_actor(walker_bp, walker_spawn)
    if walker:
        walker_controller_bp = blueprint_library.find('controller.ai.walker')
        controller = carla_world.spawn_actor(walker_controller_bp, carla.Transform(), attach_to=walker)
        controller.start()
        controller.go_to_location(carla_world.get_random_location_from_navigation())

# 5. 无人机起飞并跟随车辆
airsim_client.enableApiControl(True)
airsim_client.armDisarm(True)
airsim_client.takeoffAsync().join()

# 6. 主循环
orbit_angle = 180.0  # 从车辆正后方开始
for tick in range(60 * 30):  # 60秒 × 30帧
    carla_world.tick()
    
    # 获取车辆当前位置
    vehicle_transform = vehicle.get_transform()
    vx = vehicle_transform.location.x
    vy = vehicle_transform.location.y
    vz = vehicle_transform.location.z
    
    # 无人机跟随车辆（AirSim 坐标系注意转换）
    target_altitude = -20.0  # AirSim Z轴向下为正，-20表示上方20m
    airsim_client.moveToPositionAsync(
        vx, vy, target_altitude,
        velocity=5.0,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=vehicle_transform.rotation.yaw)
    )
    
    # Orbit 相机更新
    orbit_radius = 25.0 + max(0, tick - 45*30) * 0.01  # 最后15秒扩大半径
    orbit_height = 12.0 + max(0, tick - 45*30) * 0.03  # 最后15秒拉高
    orbit_angle += 0.3  # 每帧旋转0.3度
    orbit_camera(carla_world, vehicle_transform.location, orbit_radius, orbit_height, orbit_angle)
    
    time.sleep(0.001)  # 避免 CPU 过载
```

### 分屏合成说明

三路相机分别保存为独立图片序列，后期用 FFmpeg 合成三分屏：
```bash
# 左上（无人机俯视）：640×360，左上角
# 左下（车辆第一人称）：640×360，左下角
# 右侧（全局 Orbit）：1280×720，右半屏
ffmpeg -i drone_view_%06d.png -i car_view_%06d.png -i orbit_view_%06d.png \
  -filter_complex "[0]scale=640:360[a];[1]scale=640:360[b];[2]scale=1280:720[c];
                   [a][b]vstack[left];[left][c]hstack" \
  -c:v libx264 -crf 18 D01_output.mp4
```

---

## D02：天气切换同步

**核心 Feature**：单进程架构保证天气变化同时影响地面和空中所有传感器，物理上完全一致  
**导演意图**：相机固定不动，让天气变化本身成为主角。左右两侧同步出现雨滴的那一帧，是整套 Demo 中最有力的单帧画面。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 时长 | 45 秒（5 种天气各 9 秒） |
| 车辆 | 3 辆车在主干道缓慢行驶（速度 20 km/h） |
| 无人机 | 悬停在十字路口正上方 25m，固定不动 |
| 行人 | 8 名行人在人行道行走 |

### 相机配置（两路，均固定不动）

**相机 A：地面固定机位**（画面左侧）
```python
# 固定在十字路口一角，45°仰角拍摄对角线方向
cam_a_transform = carla.Transform(
    carla.Location(x=-45.0, y=-30.0, z=3.0),  # 根据 Town10HD 实际坐标调整
    carla.Rotation(pitch=-10, yaw=45, roll=0)
)
# 这个机位应该能同时看到：道路、行人、远处建筑、天空
```

**相机 B：无人机俯视固定**（画面右侧）
```python
# 挂载在悬停的无人机正下方，垂直俯视
# 俯视画面中应能看到：十字路口、车辆、行人、道路积水（雨天）
```

### 天气序列与 Python 实现

```python
weather_presets = [
    carla.WeatherParameters.ClearNoon,          # 晴天正午
    carla.WeatherParameters.CloudyNoon,         # 多云
    carla.WeatherParameters.WetNoon,            # 湿润（刚下过雨）
    carla.WeatherParameters.MidRainyNoon,       # 中雨
    carla.WeatherParameters.HardRainNoon,       # 暴雨
]

# 每种天气持续 9 秒 = 270 帧
frames_per_weather = 270

for weather_idx, weather in enumerate(weather_presets):
    carla_world.set_weather(weather)
    for frame in range(frames_per_weather):
        carla_world.tick()
        # 保存两路相机帧
```

### 关键拍摄细节

天气切换应该是**瞬间切换**（硬切），不要渐变。这样观众能清晰看到"切换前"和"切换后"的对比，视觉冲击力更强。暴雨状态下，地面相机应能看到：路面积水反光、雨滴打在镜头上的效果（CARLA 原生支持）、行人撑伞或加快脚步（Walker AI 行为）。

```python
# 开启雨滴打在相机镜头上的效果
camera_bp.set_attribute('lens_circle_falloff', '5.0')
# CARLA 的 HardRainNoon 天气预设已包含镜头雨滴效果，无需额外设置
```

---

## D03：多模态传感器六格分屏

**核心 Feature**：18 路传感器同步采集，帧号完全一致，面向数据集构建研究者  
**导演意图**：六格分屏本身就是一个强有力的视觉论据——观众看到 6 个画面同时运动，且明显是同一个世界的不同"感知方式"，会立刻理解这个平台的数据价值。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03（LiDAR 效果在 Town03 的建筑密度下最好看） |
| 天气 | ClearNoon |
| 时长 | 60 秒 |
| 车辆 | 1 辆主车（携带传感器），2 辆配角车辆从 LiDAR 扫描范围内经过 |
| 无人机 | 悬停在主车正上方 15m，携带空中传感器组 |
| 行人 | 5 名行人，其中 2 名从 LiDAR 扫描范围内经过 |

### 六格分屏布局

```
┌─────────────┬─────────────┬─────────────┐
│  地面 RGB   │  地面 深度  │ 地面语义分割 │
├─────────────┼─────────────┼─────────────┤
│  无人机 RGB │ 无人机 深度 │ LiDAR 点云  │
└─────────────┴─────────────┴─────────────┘
```

### 传感器配置

```python
# 地面传感器组（挂载在车辆前方）
sensors_ground = {
    'rgb': blueprint_library.find('sensor.camera.rgb'),
    'depth': blueprint_library.find('sensor.camera.depth'),
    'semantic': blueprint_library.find('sensor.camera.semantic_segmentation'),
}

# 无人机传感器组（AirSim settings.json 配置）
# RGB: ImageType=0, Depth: ImageType=1, Segmentation: ImageType=5

# LiDAR（挂载在车辆顶部）
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('points_per_second', '1120000')
lidar_bp.set_attribute('rotation_frequency', '20')
lidar_bp.set_attribute('range', '50')
lidar_bp.set_attribute('upper_fov', '15')
lidar_bp.set_attribute('lower_fov', '-25')
```

### LiDAR 点云可视化

```python
import open3d as o3d
import numpy as np

def visualize_lidar(lidar_data):
    points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4)
    xyz = points[:, :3]
    
    # 按高度着色（彩虹色谱，z 值映射到颜色）
    z_normalized = (xyz[:, 2] - xyz[:, 2].min()) / (xyz[:, 2].max() - xyz[:, 2].min() + 1e-6)
    colors = plt.cm.rainbow(z_normalized)[:, :3]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd
```

### 关键时刻设计

在第 30 秒左右，安排一辆配角车辆从主车 LiDAR 扫描范围内缓慢经过。此时 LiDAR 点云格（右下格）中会清晰出现车辆的三维轮廓，同时 RGB 格（左上格）中也能看到同一辆车。这个"同一物体在不同传感器中的对应关系"是整个 Demo 最有说服力的时刻。

---

## D04：无人机追踪行人·城市街道

**核心 Feature**：无人机在有行人的城市环境中追踪特定目标行人  
**导演意图**：这是整套 Demo 中最能体现"人"这一维度的核心 Demo。目标行人穿越人群时，无人机始终锁定目标，这个画面对具身智能、安防、搜救领域的研究者有极强的共鸣。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearAfternoon（下午光线，阴影更有层次感） |
| 时长 | 60 秒 |
| 目标行人 | 1 名（穿红色上衣，`walker.pedestrian.0001`，颜色通过蓝图属性设置） |
| 背景行人 | 20 名，随机游走 |
| 车辆 | 3 辆车在道路上正常行驶（增加场景真实感） |
| 无人机 | 从高处俯冲，锁定目标后保持 5m 高度跟随 |

### 目标行人路径设计

```python
# 目标行人的路径点（在 Town10HD 中手动设置，确保路径有趣）
# 路径应包含：直行→转弯→穿越人群→再次转弯
waypoints = [
    carla.Location(x=100, y=50, z=0.5),   # 起点
    carla.Location(x=100, y=80, z=0.5),   # 直行
    carla.Location(x=120, y=80, z=0.5),   # 右转
    carla.Location(x=120, y=110, z=0.5),  # 穿越人群密集区
    carla.Location(x=140, y=110, z=0.5),  # 再次右转
]

# 控制目标行人按路径行走
target_walker_control = carla.WalkerControl()
target_walker_control.speed = 1.4  # 正常步行速度 1.4 m/s
target_walker_control.direction = compute_direction(current_pos, next_waypoint)
target_walker.apply_control(target_walker_control)
```

### 无人机追踪逻辑

```python
# 无人机追踪目标行人
def track_pedestrian(airsim_client, target_walker, carla_world):
    walker_location = target_walker.get_location()
    
    # AirSim 坐标系与 CARLA 坐标系的转换
    # CARLA: X前Y右Z上；AirSim: X前Y右Z下（NED坐标系）
    target_x = walker_location.x
    target_y = walker_location.y
    target_z = -5.0  # 目标行人上方 5m（AirSim Z向下为正，-5表示上方5m）
    
    airsim_client.moveToPositionAsync(
        target_x, target_y, target_z,
        velocity=8.0,  # 追踪速度，足够跟上步行
        timeout_sec=0.1  # 每帧更新一次目标
    )
```

### 相机配置（三路）

> **✅ 第三方可见性要求：相机 C 是本 Demo 的第三方可见性相机，必须确保画面中同时看到无人机模型和目标行人。**

**相机 A：无人机第一人称 RGB**（画面左侧）
```python
# 挂载在无人机前方，朝向略向下（pitch=-30°）
# 追踪时，目标行人应始终在画面中心附近
# 可以用 AirSim 的 simGetCameraInfo 实时调整相机朝向，使其始终对准目标
```

**相机 B：全局第三人称 Orbit**（画面右上）
```python
# 以目标行人为中心，半径 20m，高度 15m，缓慢 Orbit
# Orbit 高度介于无人机（追踪时在 5m）和全局视角之间，画面中同时看到无人机和行人
orbit_center = target_walker.get_location()
orbit_radius = 20.0
orbit_height = 15.0
```

**相机 C：地面固定机位（画面右下）——第三方可见性关键相机**
```python
# 固定在街道侧方，人眼高度，略向上仰拍
# 画面中：前景是行人群体，中景是目标行人，背景是无人机在空中追踪
cam_c_transform = carla.Transform(
    carla.Location(x=110.0, y=45.0, z=1.8),  # 路边固定点，靠近行人路径
    carla.Rotation(pitch=20, yaw=0, roll=0)   # 向上仰拍，捕捉空中无人机
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform)
# 工程师注意：
# 1. 这个相机不跟随任何车辆，固定在世界坐标中
# 2. 相机位置需要在目标行人路径附近，确保行人和无人机都能入镜
# 3. 无人机在低空追踪时（5m 高度），从这个视角看：
#    - 行人在前景，无人机在行人上方不远处，画面层次感很强
# 4. 如果无人机俯冲阶段（从 50m 降到 5m），这个相机能拍到无人机从高处俗冲而下的全过程
```

### 运镜设计

**0–10s**：无人机从高处（50m）俯视整个场景，目标行人在人群中不易辨认。Orbit 相机展示全局。  
**10–20s**：无人机开始俯冲，高度从 50m 降到 5m，逐渐锁定目标行人。这个俯冲过程是视觉高潮之一。  
**20–50s**：无人机保持 5m 高度跟随，目标行人穿越人群、转弯，无人机始终跟随。  
**50–60s**：无人机缓慢上升到 20m，展示全局，目标行人在人群中越来越小，最终淡出。

---

## D05：十字路口交通监视

**核心 Feature**：无人机悬停在十字路口上方进行航拍交通监视，配合语义分割  
**导演意图**：这个 Demo 最直观地展示了"无人机作为空中传感器节点"的应用价值。交通灯变绿时多辆车同时启动的瞬间，是整个 Demo 的视觉高潮。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD（选择有大型十字路口的区域） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 车辆 | 8 辆车，在十字路口等待红灯，绿灯后同时启动 |
| 无人机 | 悬停在十字路口正上方 30m，固定不动 |
| 行人 | 10 名行人在斑马线上等待和穿越 |

### 相机配置（两路）

**相机 A：无人机 RGB 俯视**（画面左侧）
```python
# 垂直向下，FOV=90，覆盖整个十字路口
# 0–20s：45°斜向俯视（pitch=-45°），展示十字路口全貌和远处城市
# 20–45s：缓慢旋转到垂直俯视（pitch=-90°），展示"上帝视角"
```

**相机 B：无人机语义分割俯视**（画面右侧）
```python
# 与相机 A 同位置，但使用语义分割相机
# 颜色映射：车辆=蓝色，行人=红色，道路=灰色，建筑=棕色，植被=绿色
semantic_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
# CARLA 语义分割默认颜色映射，无需额外设置
```

### 交通灯控制

```python
# 找到十字路口的交通灯，手动控制红绿灯时序
traffic_lights = carla_world.get_actors().filter('traffic.traffic_light')

# 在第 15 秒（450帧）时，将所有红灯切换为绿灯
# 这样观众能看到：等待（红灯）→ 启动（绿灯）的完整过程
for tl in traffic_lights:
    if is_at_target_intersection(tl):
        tl.set_state(carla.TrafficLightState.Red)  # 前15秒红灯
        # 第15秒后切换
        tl.set_state(carla.TrafficLightState.Green)
```

### 运镜设计

**0–15s**：无人机相机从 45° 斜向俯视，展示十字路口全貌。8 辆车在路口等待红灯，行人在斑马线上等待。  
**15s**：交通灯变绿，8 辆车同时启动。这是视觉高潮——语义分割画面中，8 个蓝色方块同时开始移动。  
**15–35s**：相机缓慢旋转到垂直俯视，展示"上帝视角"下的交通流。  
**35–45s**：相机保持垂直俯视，行人穿越斑马线，语义分割画面中红色小点（行人）和蓝色方块（车辆）交织移动。

---

## D06：多机编队·异构目标追踪

**核心 Feature**：多架无人机同时在城市上空飞行，各自追踪不同类型的目标（车辆和行人）  
**导演意图**：这是视觉冲击力最强的 Demo 之一。全局俯视画面中，3 架无人机和它们各自的目标同时在画面中，展示了平台的多智能体协同能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 60 秒 |
| 无人机 | 3 架（需确认 CARLA-Air 多无人机实例化支持） |
| 目标 | 无人机1追踪车辆A，无人机2追踪车辆B，无人机3追踪目标行人 |
| 背景 | 5 辆额外车辆，15 名背景行人 |

### 多无人机初始化

```python
# AirSim 多无人机配置（settings.json）
{
  "Vehicles": {
    "Drone1": {"VehicleType": "SimpleFlight", "X": 0, "Y": 0, "Z": -2},
    "Drone2": {"VehicleType": "SimpleFlight", "X": 5, "Y": 0, "Z": -2},
    "Drone3": {"VehicleType": "SimpleFlight", "X": 10, "Y": 0, "Z": -2}
  }
}

# 分别获取三个无人机的控制客户端
drone1 = airsim.MultirotorClient()
drone1.enableApiControl(True, "Drone1")
drone2 = airsim.MultirotorClient()
drone2.enableApiControl(True, "Drone2")
drone3 = airsim.MultirotorClient()
drone3.enableApiControl(True, "Drone3")
```

### 相机配置（四路）

**相机 A：全局高空俯视**（画面上半部分，占 60% 高度）
```python
# 固定在场景正上方 80m，垂直俯视
# 三架无人机和它们的目标都在画面中
# 可以用不同颜色的 debug 标记区分（但不显示在最终画面中）
```

**相机 B/C/D：三架无人机各自第一人称**（画面下半部分，三等分）
```python
# 每架无人机各一路 RGB 相机，朝向略向下（pitch=-30°）
# 追踪时相机始终对准目标
```

### 运镜设计

**0–20s**：全局俯视展示三架无人机从停机位起飞，分散到各自目标上方。  
**20–50s**：三分屏下方展示各自第一人称视角，上方全局视角持续。其中一个目标（目标行人）突然改变方向，对应无人机3立刻调整航向。  
**50–60s**：三架无人机同时上升到 50m，全局视角展示三架无人机在城市上空的编队位置，收尾。

---

## D07：无人机低空穿越行人街道

**核心 Feature**：无人机在行人密集的城市街道低空飞行，展示社会导航场景  
**导演意图**：这个 Demo 的戏剧张力来自于"无人机和人的距离感"。2–3m 的低空飞行，无人机和行人几乎在同一高度，这个画面对社会导航研究者有极强的代入感。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD（选择人行道宽阔的街道） |
| 天气 | ClearAfternoon |
| 时长 | 45 秒 |
| 无人机 | 沿预设路径低空飞行，高度 2.5m |
| 行人 | 25 名，在无人机飞行路径附近密集分布 |
| 车辆 | 2 辆车在道路上正常行驶（增加场景层次） |

### 无人机飞行路径

```python
# 预设飞行路径点（沿人行道飞行，高度 2.5m）
flight_path = [
    airsim.Vector3r(0, 0, -2.5),      # 起点
    airsim.Vector3r(20, 0, -2.5),     # 直行
    airsim.Vector3r(20, 10, -2.5),    # 右转（绕过行人群）
    airsim.Vector3r(40, 10, -2.5),    # 继续直行
    airsim.Vector3r(40, 0, -2.5),     # 左转回到主路
    airsim.Vector3r(60, 0, -2.5),     # 直行到终点
]

# 按路径飞行
for point in flight_path:
    airsim_client.moveToPositionAsync(
        point.x_val, point.y_val, point.z_val,
        velocity=3.0  # 低速飞行，3 m/s，增加紧张感
    ).join()
```

### 相机配置（三路）

> **✅ 第三方可见性要求：相机 B 和相机 C 必须确保无人机的三维模型在画面中可见。本 Demo 是最能体现“无人机和人居于同一空间”的 Demo，第三方视角至关重要。**

**相机 A：无人机第一人称 FPV**（画面左侧）
```python
# 挂载在无人机前方，FOV=100（广角，增加临场感）
# pitch=0°（水平向前），让观众感受到"在人群中穿行"的视角
```

**相机 B：侧方跟随相机**（画面右上）——第三方可见性关键相机
```python
# 固定在街道侧方，高度 1.5m（人眼高度），朝向无人机飞行方向
# 画面中：行人在前景，无人机在行人中间穿过（高度相近，仅 2.5m）
# 这是整个 Demo 中最能体现"无人机和人居于同一空间"的画面
cam_b_transform = carla.Transform(
    carla.Location(x=10.0, y=-4.0, z=1.5),   # 街道左侧人行道边缘
    carla.Rotation(pitch=5, yaw=0, roll=0)    # 略向上，确保无人机入镜
)
camera_b = world.spawn_actor(camera_bp, cam_b_transform)
# 工程师注意：
# 1. 无人机在 2.5m 高度飞行，人眼高度相机在 1.5m
#    两者高度差只有1m，画面中无人机和行人几乎平辈，这种近距离感极具冲击力
# 2. 相机需要安放在无人机飞行路径的垂直方向，确保无人机从相机正前方飞过
# 3. 相机位置需要在飞行路径开始前就设置好，不随无人机移动
```

**相机 C：后方跟随相机**（画面右下）——第三方可见性辅助相机
```python
# 跟随无人机后方，高度 3m，拍摄无人机在行人中穿过的背面视角
# 画面中：无人机在画面中心，行人在无人机两侧和前方
cam_c_transform = carla.Transform(
    carla.Location(x=-3.0, y=0.0, z=3.0),  # 无人机后方3m，高3m
    carla.Rotation(pitch=-10, yaw=0, roll=0)
)
# 工程师注意：这个相机需要实时跟随无人机位置更新
# 可以用 simSetVehiclePose 每帧更新相机位置，或者将相机 attach 到无人机上
```

### 关键拍摄细节

行人的 Walker AI 应该设置为正常行走速度（1.2–1.5 m/s），不要让行人主动避让无人机（CARLA 的 Walker AI 默认不感知无人机）。这样画面中会出现无人机"穿越"行人的视觉效果，增加戟剧张力。在第一人称视角中，行人会从画面两侧"掠过"，制造强烈的速度感和临场感。相机 B 的侧方视角中，行人和无人机几乎处于同一高度，这个画面是整个 Demo 的视觉高潮。

---

## D08：夜间城市巡逻

**核心 Feature**：夜间光照条件下的多智能体协同，展示平台的光照仿真能力  
**导演意图**：夜间场景的视觉美感是这个 Demo 的核心。路灯、车灯、无人机指示灯在黑暗中的光影效果，是白天场景无法替代的视觉体验。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | `ClearNight`（晴朗夜晚，路灯全亮） |
| 时长 | 45 秒 |
| 车辆 | 2 辆车，开启车灯，在主干道巡逻式行驶 |
| 无人机 | 在车辆上方 20m 跟随，AirSim 无人机默认有指示灯效果 |
| 行人 | 5 名行人，在路灯下行走 |

### 天气设置

```python
night_weather = carla.WeatherParameters(
    cloudiness=10.0,
    precipitation=0.0,
    sun_altitude_angle=-90.0,  # 太阳在地平线以下，完全夜晚
    fog_density=0.0,
    wetness=0.0
)
carla_world.set_weather(night_weather)
```

### 相机配置

> **✅ 第三方可见性要求：相机 A 的 Orbit 高度必须介于车辆和无人机之间，确保画面中同时看到车辆和无人机。夜间无人机指示灯是识别无人机存在的关键视觉线索。**

**相机 A：全局第三人称 Orbit**（主画面，占 70% 宽度）——第三方可见性关键相机
```python
# 夜间 Orbit 相机，高度 12m，半径 25m
# 重要：Orbit 高度 12m < 无人机高度 20m
# 因此 Orbit 画面中：车辆在下方，无人机在上方，两者同时在画面中
# 夜间无人机的指示灯（红绿闪烁）在黑暗中极为显眼，是识别无人机的关键特征
# FOV=75°，略窄于白天，增加夜间压迫感
orbit_radius = 25.0
orbit_height = 12.0  # 必须小于无人机高度 20m
```

**相机 B：车载后置高位相机**（画面右侧小窗，占 30% 宽度）——第三方可见性辅助相机
```python
# 挂载在车辆后方，高度 4m，朝向前上方
# 画面中：车顶在下方，无人机指示灯在上方闪烁，夜间光影极具美感
cam_b_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-10, yaw=0, roll=0)
)
camera_b = world.spawn_actor(camera_bp, cam_b_transform, attach_to=vehicle)
# 工程师注意：夜间无人机的指示灯在黑暗中极为显眼
# 即使无人机较高（如 30m），指示灯仍然清晰可见
# 这个画面将车辆、路灯、无人机指示灯全部纳入一个画面
```

### 运镜设计

**0–15s**：Orbit 相机从低角度（pitch=-5°）开始，展示夜间城市街道全景。路灯、车灯、建筑灯光构成夜景。无人机指示灯在黑暗中清晰可见，这是识别无人机存在的第一个视觉线索。  
**15–35s**：Orbit 相机缓慢上升（从 12m 到 25m），同时 Orbit 半径扩大。注意：上升过程中无人机会从画面上方逐渐进入中心，这个运动过程本身就很具观赏性。  
**35–45s**：Orbit 相机稳定在高位，俯视夜间城市。无人机指示灯在黑暗中清晰可见，车灯在道路上留下光迹。车载后置相机同时展示车顶和无人机指示灯的局部特写。

---

## D09：雨天应急响应

**核心 Feature**：极端天气（暴雨）下的多智能体协同，传感器数据质量变化  
**导演意图**：暴雨场景的视觉效果极为震撼——雨滴打在镜头上、地面积水反光、能见度降低。这个 Demo 展示了平台在极端条件下的仿真能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | `HardRainNoon`（暴雨正午） |
| 时长 | 45 秒 |
| 车辆 | 2 辆车，速度降低（雨天驾驶行为） |
| 无人机 | 在低空（10m）飞行，模拟应急响应场景 |
| 行人 | 8 名行人，部分撑伞，部分快速奔跑 |

### 雨天传感器效果

```python
# 暴雨天气设置
heavy_rain = carla.WeatherParameters(
    cloudiness=100.0,
    precipitation=100.0,          # 最大降雨量
    precipitation_deposits=80.0,  # 地面积水
    wind_intensity=50.0,          # 风力（影响雨滴方向）
    sun_altitude_angle=45.0,
    fog_density=20.0,             # 轻微雾气
    wetness=100.0                 # 路面完全湿润
)
carla_world.set_weather(heavy_rain)
```

### 相机配置（三路）

> **✅ 第三方可见性要求：相机 C 必须确保无人机模型在雨中可见。雨中无人机的画面是最能证明平台极端天气仿真能力的视觉证据。**

**相机 A：地面车辆第一人称**（画面左侧）：雨滴打在挡风玻璃上，雨刷摆动（CARLA 原生效果）  
**相机 B：无人机俯视**（画面右上）：雨天地面积水反光，能见度降低  
**相机 C：车载后置高位相机**（画面右下）——第三方可见性关键相机
```python
# 挂载在车辆后方，高度 4m，朝向前上方
# 画面中：车顶在下方，无人机在雨中在上方飞行，雨滴打在镜头上
cam_c_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-10, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=vehicle)
# 工程师注意：
# 1. 雨天无人机在低空（10m）飞行，相比晴天更容易入镜
# 2. 雨滴打在镜头上的效果加上无人机在雨中飞行，是整个 Demo 的视觉高潮
# 3. 雨天无人机的主轴旋转器运动模糊在雨天背景下更加明显
```

---

## D10：LiDAR 点云城市扫描

**核心 Feature**：地面 LiDAR 对城市环境的实时三维扫描，点云中清晰呈现行人和车辆轮廓  
**导演意图**：LiDAR 点云的视觉效果本身就很有科技感。按高度彩虹着色的点云，加上行人和车辆的三维轮廓，是最能打动数据集研究者的画面。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03（建筑密度适中，LiDAR 效果最佳） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 携带 64 线 LiDAR，缓慢在街道行驶（15 km/h） |
| 无人机 | 在主车上方 20m 跟随，携带空中 RGB 相机 |
| 行人 | 8 名行人，其中 3 名会穿越 LiDAR 扫描范围 |
| 配角车 | 2 辆，会从 LiDAR 扫描范围内经过 |

### LiDAR 配置

```python
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', '64')           # 64线，高密度
lidar_bp.set_attribute('points_per_second', '1120000')
lidar_bp.set_attribute('rotation_frequency', '20') # 20Hz
lidar_bp.set_attribute('range', '50')              # 50m 范围
lidar_bp.set_attribute('upper_fov', '15')
lidar_bp.set_attribute('lower_fov', '-25')
lidar_bp.set_attribute('atmosphere_attenuation_rate', '0.004')
```

### 相机配置（两路）

> **✅ 第三方可见性要求：必须增加相机 C（车载后置高位），确保画面中能看到无人机模型在车辆上方飞行。**

**相机 A：LiDAR 点云可视化**（画面左侧，Open3D 渲染）  
**相机 B：无人机俯视 RGB**（画面右上）：提供空中视角对照，让观众理解点云对应的真实场景  
**相机 C：车载后置高位相机**（画面右下）——第三方可见性关键相机
```python
# 挂载在主车后方，高度 4m，朝向前上方
# 画面中：车顶在下方，无人机在上方飞行，两者同时入镜
cam_c_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-10, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=vehicle)
```

### 运镜设计

**0–15s**：主车在直道行驶，LiDAR 扫描建筑物立面，点云中建筑轮廓清晰。  
**15–30s**：一名行人从主车前方穿越，LiDAR 点云中出现行人的三维轮廓（约 1000–2000 个点）。  
**30–45s**：一辆配角车辆从侧方经过，LiDAR 点云中出现车辆的完整三维轮廓，远比行人轮廓更大更密集。

---

## D11：空中深度感知·建筑轮廓

**核心 Feature**：无人机深度相机对城市建筑的三维感知  
**导演意图**：深度图的视觉效果（近处深色、远处浅色的灰度图）与 RGB 图形成强烈对比，展示了同一场景在不同感知模态下的差异。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 无人机 | 沿建筑物立面飞行，高度 15–30m |
| 行人 | 10 名行人在建筑物下方行走 |
| 车辆 | 2 辆车在道路上行驶 |

### 相机配置（两路）

> **✅ 第三方可见性要求：必须增加相机 C（地面固定机位），仿拍无人机沿建筑物立面飞行的全景。地面上的行人和车辆应在画面中同时可见。**

**相机 A：无人机 RGB**（画面左侧）  
**相机 B：无人机深度相机**（画面右上）  
**相机 C：地面固定机位**（画面右下）——第三方可见性关键相机
```python
# 固定在建筑物对面的街道上，仿拍无人机沿建筑物立面飞行
# 画面中：建筑物在背景，无人机在中景，行人和车辆在前景
cam_c_transform = carla.Transform(
    carla.Location(x=0.0, y=-20.0, z=2.0),
    carla.Rotation(pitch=10, yaw=90, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform)
```

```python
depth_bp = blueprint_library.find('sensor.camera.depth')
depth_bp.set_attribute('image_size_x', '1280')
depth_bp.set_attribute('image_size_y', '720')
depth_bp.set_attribute('fov', '90')

# 深度图后处理：将原始深度值转换为可视化的灰度图
def process_depth(depth_image):
    array = np.frombuffer(depth_image.raw_data, dtype=np.float32)
    array = array.reshape((depth_image.height, depth_image.width, 4))
    # CARLA 深度图编码：R + G*256 + B*256*256，单位 cm
    depth_meters = (array[:,:,0] + array[:,:,1]*256 + array[:,:,2]*256*256) / 1000.0
    # 归一化到 0-255 用于可视化
    depth_normalized = np.clip(depth_meters / 100.0, 0, 1) * 255
    return depth_normalized.astype(np.uint8)
```

### 运镜设计

无人机从建筑物一端飞向另一端，保持与建筑立面约 10m 的距离。深度图中，建筑立面呈现均匀的灰度，而建筑前方的行人和车辆因为更近而呈现更深的颜色，形成清晰的层次感。

---

## D12：语义分割全景·城市理解

**核心 Feature**：多视角语义分割，展示平台对城市环境的完整语义理解能力  
**导演意图**：语义分割图的颜色鲜艳，视觉效果强烈。当行人（红色）、车辆（蓝色）、道路（灰色）、建筑（棕色）同时出现在画面中，观众能直观感受到"这个平台理解这个世界"。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 车辆 | 5 辆，在主干道行驶 |
| 无人机 | 悬停在十字路口上方 20m |
| 行人 | 15 名，在人行道和斑马线上 |

### 相机配置（三路）

> **✅ 第三方可见性要求：必须增加相机 D（车载后置高位），确保无人机模型在画面中可见。语义分割画面中，无人机应被识别为独立的语义类别或至少在 RGB 画面中可见。**

**相机 A：地面 RGB**（画面左上）  
**相机 B：地面语义分割**（画面左下）  
**相机 C：无人机语义分割俯视**（画面右上，占右半屏）  
**相机 D：车载后置高位相机**（画面右下小窗）——第三方可见性关键相机
```python
# 挂载在地面车辆后方，高度 4m，拍摄无人机悬停在路口上方
cam_d_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_d = world.spawn_actor(camera_bp, cam_d_transform, attach_to=vehicle)
```

```python
# CARLA 语义分割标签颜色（官方定义）
semantic_colors = {
    0: (0, 0, 0),       # None
    1: (70, 70, 70),    # Building
    2: (100, 40, 40),   # Fence
    3: (55, 90, 80),    # Other
    4: (220, 20, 60),   # Pedestrian（红色，最显眼）
    5: (153, 153, 153), # Pole
    6: (157, 234, 50),  # RoadLine
    7: (128, 64, 128),  # Road
    8: (244, 35, 232),  # SideWalk
    9: (107, 142, 35),  # Vegetation
    10: (0, 0, 142),    # Vehicle（蓝色）
    11: (102, 102, 156),# Wall
    12: (220, 220, 0),  # TrafficSign
}
```

---

## D13：无人机环绕建筑物检测

**核心 Feature**：无人机沿预设轨迹自主飞行，对建筑物进行环绕检测  
**导演意图**：这个 Demo 展示了无人机的自主飞行能力和对建筑物的感知能力。环绕飞行的轨迹本身就很有视觉美感。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 无人机 | 以目标建筑物为中心，半径 15m，高度从 5m 到 25m 螺旋上升 |
| 行人 | 8 名行人在建筑物周围行走 |
| 车辆 | 2 辆车在附近道路行驶 |

### 螺旋飞行轨迹实现

```python
import math

def spiral_orbit(airsim_client, center_x, center_y, 
                 start_radius=15, end_radius=15,
                 start_height=5, end_height=25,
                 total_frames=1350, speed=3.0):
    """
    螺旋环绕飞行：从低处开始，螺旋上升到高处
    total_frames: 45秒 × 30帧 = 1350帧
    """
    for frame in range(total_frames):
        progress = frame / total_frames  # 0.0 到 1.0
        
        # 角度：完整转两圈（720°）
        angle = math.radians(progress * 720)
        
        # 高度线性增加
        height = -(start_height + (end_height - start_height) * progress)
        
        # 半径保持不变（也可以设计为变化）
        radius = start_radius
        
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        airsim_client.simSetVehiclePose(
            airsim.Pose(airsim.Vector3r(x, y, height),
                       airsim.to_quaternion(0, 0, angle + math.pi)),  # 朝向建筑物
            ignore_collision=True,
            vehicle_name="Drone1"
        )
```

### 相机配置（两路）

> **✅ 第三方可见性要求：相机 B 必须是能看到无人机环绕建筑物飞行的全景。建筑物周围的行人和车辆应在画面中同时可见。**

**相机 A：无人机第一人称 RGB**（画面左侧）：始终朝向建筑物，随无人机旋转  
**相机 B：全局固定机位**（画面右侧）：从远处拍摄无人机环绕建筑物的全景
```python
# 固定机位应放置在建筑物对面街道，能同时看到：
# 1. 无人机在建筑物周围飞行（中景）
# 2. 建筑物本身（背景）
# 3. 建筑物周围的行人和车辆（前景）
cam_b_transform = carla.Transform(
    carla.Location(x=0.0, y=-30.0, z=5.0),
    carla.Rotation(pitch=-5, yaw=90, roll=0)
)
camera_b = world.spawn_actor(camera_bp, cam_b_transform)
```

---

## D14：车辆紧急制动·空中见证

**核心 Feature**：碰撞传感器 + 无人机俯视，展示地面事件的空中见证能力  
**导演意图**：这个 Demo 展示了"空中视角作为地面事件见证者"的应用价值。紧急制动的瞬间，无人机俯视画面中能清晰看到车辆的刹车痕迹（轮胎烟雾）。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 30 秒 |
| 主车 | 以 60 km/h 行驶，在第 15 秒触发紧急制动 |
| 无人机 | 在主车正上方 15m 跟随 |
| 行人 | 1 名行人突然出现在主车前方（触发制动的原因） |
| 配角车 | 2 辆在附近行驶 |

### 碰撞传感器配置

```python
collision_bp = blueprint_library.find('sensor.other.collision')
collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)

def on_collision(event):
    print(f"Collision detected: {event.other_actor.type_id}")
    # 触发录制标记

collision_sensor.listen(on_collision)
```

### 紧急制动实现

```python
# 在第 15 秒（450帧）触发紧急制动
for tick in range(30 * 30):
    carla_world.tick()
    if tick == 450:
        # 关闭自动驾驶，手动施加最大制动力
        vehicle.set_autopilot(False)
        control = carla.VehicleControl()
        control.brake = 1.0
        control.throttle = 0.0
        vehicle.apply_control(control)
```

### 相机配置

> **✅ 第三方可见性要求：必须增加车载后置高位相机，确保制动瞬间画面中能看到无人机从上方俗冲而下的动作。这是整个 Demo 的视觉高潮。**

**相机 A：车载后置高位相机**（主画面）——第三方可见性关键相机
```python
# 挂载在车辆后方，高度 4m，朝向前上方
# 制动时：车辆在下方突然停止，无人机因惯性从上方向前俗冲
cam_a_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_a = world.spawn_actor(camera_bp, cam_a_transform, attach_to=vehicle)
```

**相机 B：无人机俯视 RGB**（画面右上）：制动时俯视车辆刹车痕迹（轮胎烟雾）  
**相机 C：车辆第一人称**（画面右下）：展示驾驶员视角下的行人突然出现

---

## D15：行人密集广场·空中统计

**核心 Feature**：无人机对行人密集区域的俯视监控，展示 CARLA 行人控制能力的上限  
**导演意图**：30 名行人同时在广场上活动，从空中俯视的画面极为壮观。这个 Demo 展示了 CARLA 行人仿真的规模能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD（选择有广场的区域） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 行人 | 30 名，在广场上随机游走，形成自然的人群流动 |
| 无人机 | 从低处（5m）缓慢上升到高处（40m），展示不同高度的俯视效果 |
| 车辆 | 3 辆车在广场周围道路行驶 |

### 行人生成

```python
# 批量生成 30 名行人
walkers = []
walker_controllers = []

spawn_points_walkers = []
for _ in range(30):
    spawn_point = carla.Transform()
    loc = carla_world.get_random_location_from_navigation()
    if loc:
        spawn_point.location = loc
        spawn_points_walkers.append(spawn_point)

# 批量生成（更高效）
batch = []
for spawn_point in spawn_points_walkers:
    walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
    batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

results = carla_client.apply_batch_sync(batch, True)
walker_ids = [r.actor_id for r in results if not r.error]
walkers = carla_world.get_actors(walker_ids)

# 为每个行人添加 AI 控制器
controller_bp = blueprint_library.find('controller.ai.walker')
batch_controllers = []
for walker in walkers:
    batch_controllers.append(
        carla.command.SpawnActor(controller_bp, carla.Transform(), walker.id)
    )
controller_results = carla_client.apply_batch_sync(batch_controllers, True)
```

### 相机配置

> **✅ 第三方可见性要求：必须增加地面固定机位，仿拍无人机从低处上升的全过程。广场上的行人和无人机应在画面中同时可见。**

**相机 A：无人机俯视 RGB**（主画面）  
**相机 B：地面固定机位**（画面右侧）——第三方可见性关键相机
```python
# 固定在广场边缘，人眼高度，略向上仰拍
# 画面中：行人在前景，无人机在行人上方不断上升
cam_b_transform = carla.Transform(
    carla.Location(x=-10.0, y=0.0, z=1.8),
    carla.Rotation(pitch=20, yaw=0, roll=0)
)
camera_b = world.spawn_actor(camera_bp, cam_b_transform)
# 工程师注意：
# 1. 无人机从 5m 上升到 40m，这个相机能拍到全程
# 2. 在 5m 高度时，无人机和行人几乎处于同一高度，画面层次感极强
# 3. 随着无人机上升，它在画面中逐渐变小，最终变成天空中的一个小点
```

### 运镜设计

**0–20s**：无人机从 5m 高度开始，缓慢上升。在低高度时，行人几乎和无人机等高，画面中行人的面部和服装细节清晰可见。地面固定相机画面中，无人机就在行人上方不远处，两者同时入镜。  
**20–40s**：无人机上升到 20m，行人变成小点，但人群的整体流动模式开始清晰——人群的密度分布、流动方向、聚集和分散。  
**40–45s**：无人机到达 40m 高度，俯视整个广场，30 名行人形成的人群图案一览无余。


---

# PART 2：Demo D16–D30 详细拍摄指南

---

## D16：双车道超车·空地同步

**核心 Feature**：多车辆控制 + 无人机追踪，展示复杂交通场景下的空地协同  
**导演意图**：超车是自动驾驶研究中的经典场景。无人机在上方追踪超车全过程，地面和空中视角同时呈现，让观众感受到"空地联合感知"的价值。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town04（有长直道，适合超车场景） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 以 60 km/h 行驶，在第 15 秒执行超车 |
| 被超车辆 | 以 40 km/h 行驶，保持在右车道 |
| 无人机 | 在主车上方 20m 跟随，超车时跟随主车变道 |
| 行人 | 4 名行人在路边人行道行走（增加场景真实感） |

### 超车控制实现

```python
# 超车场景控制逻辑
class OvertakeController:
    def __init__(self, vehicle, slow_vehicle):
        self.vehicle = vehicle
        self.slow_vehicle = slow_vehicle
        self.phase = 'follow'  # follow -> change_left -> overtake -> change_right
        self.phase_timer = 0
    
    def update(self, tick):
        if tick < 450:  # 前15秒：跟随慢车
            self.phase = 'follow'
            control = carla.VehicleControl(throttle=0.5, steer=0.0)
        elif tick < 600:  # 15-20秒：变道到左车道
            self.phase = 'change_left'
            control = carla.VehicleControl(throttle=0.6, steer=-0.15)
        elif tick < 900:  # 20-30秒：超车（加速）
            self.phase = 'overtake'
            control = carla.VehicleControl(throttle=0.8, steer=0.0)
        else:  # 30秒后：变回右车道
            self.phase = 'change_right'
            control = carla.VehicleControl(throttle=0.6, steer=0.1)
        
        self.vehicle.apply_control(control)
```

### 相机配置（四路）

> **✅ 第三方可见性要求：必须增加车载后置高位相机，确保超车全过程中无人机在画面中可见。这是最能体现“无人机跟踪车辆变道”的画面。**

**相机 A：主车第一人称**（画面左侧）：展示超车时的驾驶员视角，被超车辆从右侧后视镜消失  
**相机 B：无人机俯视**（画面右上）：俯视超车全过程，两辆车的相对位置变化清晰  
**相机 C：侧方固定机位**（画面右下）：从路边拍摄超车全景，行人在前景  
**相机 D：车载后置高位相机**（画面右下小窗）——第三方可见性关键相机
```python
# 挂载在主车后方，高度 4m，朝向前上方
# 超车时：车顶在下方，无人机在上方跟随车辆变道，画面中车辆和无人机同时入镜
cam_d_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_d = world.spawn_actor(camera_bp, cam_d_transform, attach_to=vehicle)
```

### 运镜设计

Orbit 相机以两辆车的中点为中心，在超车过程中缓慢向前移动（跟随车辆），保持两辆车都在画面中。超车完成后，Orbit 相机拉高展示全局，无人机和两辆车同时在画面中。车载后置相机画面是最能体现“无人机跟随车辆变道”的视角，必须确保超车全过程中无人机始终在画面中。

---

## D17：无人机 FPV 穿楼追车

**核心 Feature**：无人机高速动态飞行路径，展示 AirSim 飞行物理的精确性  
**导演意图**：FPV（第一人称视角）穿越飞行是无人机视频中最有视觉冲击力的形式。无人机在建筑物之间穿梭追踪车辆，画面极具动感。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD（建筑物密集，适合穿越飞行） |
| 天气 | ClearNoon |
| 时长 | 30 秒 |
| 无人机 | 高速飞行（15 m/s），在建筑物之间穿梭 |
| 目标车辆 | 以 40 km/h 在街道行驶，无人机追踪 |
| 行人 | 10 名行人在街道上，增加穿越难度感 |

### FPV 飞行路径设计

```python
# FPV 穿越路径（需要根据 Town10HD 实际地图手动设计，避免碰撞建筑物）
# 路径设计原则：
# 1. 包含至少 2 次急转弯（增加戏剧感）
# 2. 至少 1 次从建筑物缝隙穿过（宽度 > 5m 安全）
# 3. 高度变化：从 20m 俯冲到 5m，再拉高到 15m

fpv_waypoints = [
    (0, 0, -20),      # 起点，高空
    (30, 0, -5),      # 俯冲到低空
    (30, 20, -5),     # 急转弯（建筑物缝隙）
    (60, 20, -10),    # 拉高
    (80, 10, -8),     # 追上目标车辆
]

# 使用 simSetVehiclePose 直接设置位置（比 moveToPositionAsync 更精确）
for i, (x, y, z) in enumerate(fpv_waypoints):
    # 计算朝向（指向下一个路径点）
    if i < len(fpv_waypoints) - 1:
        next_x, next_y, next_z = fpv_waypoints[i+1]
        yaw = math.degrees(math.atan2(next_y - y, next_x - x))
        pitch = math.degrees(math.atan2(next_z - z, math.sqrt((next_x-x)**2 + (next_y-y)**2)))
    
    pose = airsim.Pose(
        airsim.Vector3r(x, y, z),
        airsim.to_quaternion(math.radians(pitch), 0, math.radians(yaw))
    )
    airsim_client.simSetVehiclePose(pose, ignore_collision=False)
```

### 相机配置（两路）

> **✅ 第三方可见性要求：必须增加地面固定机位，拍摄无人机在建筑物之间高速穿越的全景。地面车辆和行人应在画面中同时可见。**

**相机 A：无人机 FPV 第一人称**（主画面）：FOV=120°（超广角，增加速度感），motion_blur 强度提高到 0.8  
**相机 B：地面固定机位**（画面右侧）——第三方可见性关键相机
```python
# 固定在建筑物缝隙处，仿拍无人机高速穿过
# 画面中：建筑物在两侧，无人机从远处飞来，车辆和行人在下方
cam_b_transform = carla.Transform(
    carla.Location(x=30.0, y=-8.0, z=3.0),  # 建筑物缝隙处
    carla.Rotation(pitch=0, yaw=0, roll=0)   # 水平朝向无人机飞行方向
)
camera_b = world.spawn_actor(camera_bp, cam_b_transform)
# 工程师注意：
# 1. 相机位置需要在无人机飞行路径的垂直方向
# 2. 无人机高速飞过时，这个相机能拍到无人机从画面一侧高速飞过
# 3. 运动模糊在高速无人机上极为显眼，这个画面极具冲击力

---

## D18：地面语义导航·行人避让

**核心 Feature**：地面车辆在有行人的街道自主导航，语义分割辅助感知  
**导演意图**：这个 Demo 聚焦在地面车辆视角，展示 CARLA 自动驾驶 + 行人交互的场景。无人机作为"空中见证者"提供第三视角。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearAfternoon |
| 时长 | 45 秒 |
| 主车 | 自动驾驶，在有行人的街道行驶 |
| 行人 | 15 名，其中 3 名会在主车前方穿越马路 |
| 无人机 | 悬停在主车前方 30m 上空，俯视整个场景 |

### 行人穿越马路控制

```python
# 控制特定行人在主车接近时穿越马路
class CrossingPedestrian:
    def __init__(self, walker, trigger_distance=20.0):
        self.walker = walker
        self.trigger_distance = trigger_distance
        self.has_crossed = False
    
    def update(self, vehicle_location):
        if self.has_crossed:
            return
        
        walker_location = self.walker.get_location()
        distance = walker_location.distance(vehicle_location)
        
        if distance < self.trigger_distance:
            # 触发穿越：控制行人向道路对面走
            control = carla.WalkerControl()
            control.speed = 1.8  # 稍快的步行速度（过马路时人会走快）
            control.direction = carla.Vector3D(1, 0, 0)  # 向道路方向
            self.walker.apply_control(control)
            self.has_crossed = True
```

### 相机配置（三路）

> **✅ 第三方可见性要求：无人机俯视画面中必须能看到无人机模型本身。建议在无人机下方加载一个向前下方值班的相机，这样无人机模型的機身和旋转的桨叶会在画面上方入镜。同时必须增加车载后置高位相机作为第四路。**

**相机 A：车辆第一人称 RGB**（画面左上）  
**相机 B：车辆语义分割**（画面左下）：行人（红色）在车辆前方穿越时，语义分割画面中红色方块突然出现  
**相机 C：无人机俯视 RGB**（画面右上）：展示车辆和行人的空间关系  
**相机 D：车载后置高位相机**（画面右下）——第三方可见性关键相机
```python
# 挂载在车辆后方，高度 4m，拍摄无人机在车辆前方上方悬停
cam_d_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_d = world.spawn_actor(camera_bp, cam_d_transform, attach_to=vehicle)
```

---

## D19：空地双视角·同一事件

**核心 Feature**：同一物理事件从空中和地面两个视角同时呈现，展示视角互补性  
**导演意图**：这是整套 Demo 中叙事性最强的一个。选择一个戏剧性事件（如行人突然出现、车辆急转弯），同时从空中和地面两个视角记录，让观众感受到"两个视角看同一个世界"的震撼。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 核心事件 | 一名行人突然从小巷跑出，穿越主干道 |
| 主车 | 以 50 km/h 行驶，紧急制动 |
| 无人机 | 在主车上方 15m 跟随 |
| 背景 | 5 名其他行人，3 辆其他车辆 |

### 相机配置（两路，左右分屏）

> **✅ 第三方可见性要求：必须增加车载后置高位相机作为第三路，确保左右分屏之外还有一路画面能同时看到无人机和车辆。这个第三视角是证明“两者在同一个世界”的关键画面。**

**相机 A：无人机俯视 RGB**（画面左侧）：从空中看到行人从小巷跑出，主车制动  
**相机 B：主车第一人称 RGB**（画面右侧）：从地面看到行人突然出现，制动感受  
**相机 C：车载后置高位相机**（画面小窗）——第三方可见性关键相机
```python
# 挂载在车辆后方，高度 4m，同时拍到车顶和上方的无人机
cam_c_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=vehicle)
```

### 关键时刻设计

在第 20 秒，行人从小巷跑出。此时左侧画面（无人机俯视）中，行人的出现是"预期之内"的——因为从空中可以看到小巷，知道行人会从那里出来。而右侧画面（车辆第一人称）中，行人的出现是"突然的"——因为小巷被建筑物遮挡，直到行人跑出来才看到。这个"空中视角的预知性 vs 地面视角的局限性"的对比，是这个 Demo 最有价值的叙事时刻。

```python
# 行人从小巷跑出的控制
def spawn_running_pedestrian(world, alley_exit_location):
    walker_bp = blueprint_library.find('walker.pedestrian.0001')
    spawn_transform = carla.Transform(alley_exit_location)
    walker = world.spawn_actor(walker_bp, spawn_transform)
    
    # 控制行人快速跑过马路
    control = carla.WalkerControl()
    control.speed = 3.5  # 跑步速度
    control.direction = carla.Vector3D(0, 1, 0)  # 穿越方向
    walker.apply_control(control)
    return walker

# 在第 20 秒（600帧）触发
if tick == 600:
    running_walker = spawn_running_pedestrian(world, alley_exit)
```

---

## D20：多天气序列·传感器对比

**核心 Feature**：14 种天气预设的完整遍历，展示平台天气仿真的广度  
**导演意图**：这个 Demo 是一个"能力展示"型视频。快速切换 14 种天气，每种天气停留 4 秒，让观众在 60 秒内看到平台的天气仿真范围。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 时长 | 60 秒（14 种天气 × 约 4 秒） |
| 车辆 | 2 辆车，固定在十字路口 |
| 无人机 | 悬停在十字路口上方 20m，固定不动 |
| 行人 | 8 名行人，固定在人行道上 |

### 天气序列

```python
weather_sequence = [
    ("ClearNoon", carla.WeatherParameters.ClearNoon),
    ("CloudyNoon", carla.WeatherParameters.CloudyNoon),
    ("WetNoon", carla.WeatherParameters.WetNoon),
    ("WetCloudyNoon", carla.WeatherParameters.WetCloudyNoon),
    ("MidRainyNoon", carla.WeatherParameters.MidRainyNoon),
    ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
    ("SoftRainNoon", carla.WeatherParameters.SoftRainNoon),
    ("ClearSunset", carla.WeatherParameters.ClearSunset),
    ("CloudySunset", carla.WeatherParameters.CloudySunset),
    ("WetSunset", carla.WeatherParameters.WetSunset),
    ("WetCloudySunset", carla.WeatherParameters.WetCloudySunset),
    ("MidRainSunset", carla.WeatherParameters.MidRainSunset),
    ("HardRainSunset", carla.WeatherParameters.HardRainSunset),
    ("SoftRainSunset", carla.WeatherParameters.SoftRainSunset),
]

frames_per_weather = int(60 * 30 / len(weather_sequence))  # 约 128 帧

for name, weather in weather_sequence:
    carla_world.set_weather(weather)
    for frame in range(frames_per_weather):
        carla_world.tick()
```

### 相机配置（两路，左右分屏）

> **✅ 第三方可见性要求：必须增加车载后置高位相机作为第三路，确保天气切换时画面中无人机模型可见。这是证明“天气在两个体系中同步变化”的关键画面。**

**相机 A：地面固定机位 RGB**（画面左侧）：固定角度，让天气变化成为主角  
**相机 B：无人机俯视 RGB**（画面右上）：同步展示空中视角的天气变化  
**相机 C：车载后置高位相机**（画面右下）——第三方可见性关键相机
```python
# 挂载在地面车辆后方，高度 4m，拍摄无人机悬停在上方
# 天气切换时：车顶在下方，无人机在上方，两者同时在同一天气环境中
cam_c_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=vehicle)
```

---

## D21：无人机悬停·GPS 与 IMU 数据流

**核心 Feature**：无人机传感器数据（GPS、IMU、气压计）的实时可视化  
**导演意图**：这个 Demo 面向工程师受众，展示平台的传感器数据质量。画中画设计：主画面是仿真器，小窗口是实时数据流。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 30 秒 |
| 无人机 | 悬停在 20m 高度，执行小幅机动（前后左右各移动 2m） |
| 行人 | 5 名行人在下方行走 |

### 传感器数据可视化

```python
# AirSim IMU 数据读取
imu_data = airsim_client.getImuData()
print(f"Linear Accel: {imu_data.linear_acceleration}")
print(f"Angular Vel: {imu_data.angular_velocity}")

# GPS 数据读取
gps_data = airsim_client.getGpsData()
print(f"Lat: {gps_data.gnss.geo_point.latitude:.6f}")
print(f"Lon: {gps_data.gnss.geo_point.longitude:.6f}")
print(f"Alt: {gps_data.gnss.geo_point.altitude:.2f}m")

# 气压计数据
barometer_data = airsim_client.getBarometerData()
print(f"Altitude (baro): {barometer_data.altitude:.2f}m")

# 将数据实时绘制为折线图（matplotlib），作为画中画显示
```

### 相机配置（主画面 + 画中画）

> **✅ 第三方可见性要求：全局 Orbit 相机必须能看到无人机模型本身。无人机在执行小幅机动时，機身的小幅摇摇应在画面中可见。这是证明“这是真实的无人机在飞”而不是相机在飞的关键画面。**

**主画面**：全局第三人称 Orbit，展示无人机悬停和机动  
**画中画**（右下角 30% 宽度）：matplotlib 实时折线图，显示 IMU 加速度和 GPS 高度数据

```python
# Orbit 相机设置建议：以无人机为中心，半径 15m，高度 5m，缓慢环绕
# 这样无人机始终在画面中心，機身清晰可见
# 无人机下方应有行人和车辆，增加场景层次感
```

---

## D22：红外相机·夜间行人检测

**核心 Feature**：无人机红外相机在夜间对行人的热成像感知  
**导演意图**：红外相机的视觉效果独特——行人在黑暗中呈现为明亮的热源，与冷色调的背景形成强烈对比。这个 Demo 展示了 AirSim 红外传感器的独特能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNight |
| 时长 | 45 秒 |
| 无人机 | 在街道上空 15m 缓慢飞行 |
| 行人 | 12 名行人在街道上行走 |
| 车辆 | 2 辆车，开启车灯行驶 |

### 红外相机配置（AirSim）

```python
# AirSim 红外相机配置（settings.json）
{
  "CameraName": "infrared",
  "CaptureSettings": [
    {
      "ImageType": 7,  # Infrared
      "Width": 1280,
      "Height": 720
    }
  ],
  "X": 0, "Y": 0, "Z": 0.1,
  "Pitch": -45, "Roll": 0, "Yaw": 0
}

# 读取红外图像
responses = airsim_client.simGetImages([
    airsim.ImageRequest("infrared", airsim.ImageType.Infrared, False, False)
])
infrared_data = responses[0]
```

### 相机配置（两路，左右分屏）

> **✅ 第三方可见性要求：必须增加地面固定机位作为第三路，在夜间环境中拍摄无人机在建筑物上方飞行的全景。夜间无人机的小灯光（AirSim 默认开启）应在画面中可见，这是夜间场景中证明无人机存在的最直观的方式。**

**相机 A：无人机 RGB**（画面左侧）：夜间 RGB，路灯下行人轮廓模糊  
**相机 B：无人机红外**（画面右上）：行人呈现为明亮热源，车辆引擎也呈现为热源  
**相机 C：地面固定机位**（画面右下）——第三方可见性关键相机
```python
# 固定在建筑物对面的人行道上，高度 2m，略向上仰拍
# 夜间画面中：路灯光圈在前景，无人机的小灯光在上方闪烁
cam_c_transform = carla.Transform(
    carla.Location(x=0.0, y=-15.0, z=2.0),
    carla.Rotation(pitch=20, yaw=90, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform)
```

---

## D23：空地协同·搜救场景

**核心 Feature**：无人机和地面车辆协同执行搜救任务，展示平台的任务协同能力  
**导演意图**：搜救场景是无人机最重要的实际应用之一。这个 Demo 用叙事性的方式展示：无人机从空中发现目标，地面车辆前往救援。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03（有较多开阔区域） |
| 天气 | CloudyNoon（多云，增加紧张感） |
| 时长 | 60 秒 |
| 目标行人 | 1 名"需要救援"的行人，静止在偏僻位置 |
| 背景行人 | 10 名正常行走的行人 |
| 无人机 | 从高处搜索，发现目标后悬停标记 |
| 救援车辆 | 1 辆车，接收到无人机信号后前往目标位置 |

### 任务流程实现

```python
# 搜救任务状态机
class SearchRescueTask:
    def __init__(self):
        self.phase = 'search'  # search -> found -> navigate -> arrive
    
    def update(self, tick, drone_pos, target_pos, vehicle):
        if self.phase == 'search':
            # 无人机在区域内搜索飞行（螺旋扩展路径）
            self.spiral_search(tick)
            # 检测是否发现目标（距离 < 10m）
            if drone_pos.distance(target_pos) < 10:
                self.phase = 'found'
                print("Target found! Hovering...")
        
        elif self.phase == 'found':
            # 无人机悬停在目标上方，等待 5 秒
            self.hover_above_target(target_pos)
            if tick > self.found_tick + 150:  # 5秒
                self.phase = 'navigate'
                # 发送目标坐标给地面车辆
                vehicle.set_target_location(target_pos)
        
        elif self.phase == 'navigate':
            # 地面车辆导航到目标位置
            vehicle.set_autopilot(True)
            # 检测车辆是否到达
            if vehicle.get_location().distance(target_pos) < 5:
                self.phase = 'arrive'
```

### 相机配置（三路）

> **✅ 第三方可见性要求：全局 Orbit 相机必须能同时看到无人机模型和地面车辆。当无人机悬停在目标上方标记时，地面车辆应在画面中同时可见。这是搜救场景中“空地协同”的最直观的视觉证据。**

**相机 A：无人机俯视 RGB**（画面左上）：搜索阶段展示大范围俯视  
**相机 B：全局第三人称 Orbit**（画面右侧）：展示无人机和地面车辆的协同关系，两者必须同时入镜  
**相机 C：地面车辆第一人称**（画面左下）：展示车辆前往目标的过程

---

## D24：行人轨迹预测·数据采集

**核心 Feature**：无人机从空中采集行人轨迹数据，用于训练轨迹预测模型  
**导演意图**：这个 Demo 面向数据集构建研究者，展示平台作为数据采集工具的价值。无人机俯视画面中，行人轨迹以可视化方式呈现。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 行人 | 20 名，在广场和街道上自然行走 |
| 无人机 | 悬停在 30m 高度，俯视整个场景 |
| 车辆 | 3 辆车在周围道路行驶 |

### 轨迹数据采集与可视化

```python
# 实时记录所有行人的位置
walker_trajectories = {walker.id: [] for walker in walkers}

for tick in range(45 * 30):
    carla_world.tick()
    
    for walker in walkers:
        loc = walker.get_location()
        walker_trajectories[walker.id].append((loc.x, loc.y, tick))

# 在无人机俯视画面上叠加轨迹线（使用 CARLA debug 绘制，但仅在开发时使用）
# 注意：最终视频中不显示 debug 信息，轨迹可视化通过后期处理实现
for walker_id, traj in walker_trajectories.items():
    for i in range(1, len(traj)):
        carla_world.debug.draw_line(
            carla.Location(traj[i-1][0], traj[i-1][1], 0.5),
            carla.Location(traj[i][0], traj[i][1], 0.5),
            thickness=0.1,
            color=carla.Color(255, 0, 0),
            life_time=0.1
        )
```

### 相机配置（两路）

> **✅ 第三方可见性要求：必须增加地面固定机位作为第三路，拍摄无人机悬停在上方的全景。地面上的行人和无人机应在画面中同时可见。**

**相机 A：无人机俯视 RGB**（画面左侧）：展示行人的自然行走行为  
**相机 B：无人机语义分割俯视**（画面右上）：行人（红色）在语义分割中清晰可见，便于后期轨迹提取  
**相机 C：地面固定机位**（画面右下）——第三方可见性关键相机
```python
# 固定在广场边缘，仿拍无人机悬停在上方的全景
# 画面中：行人在前景，无人机在行人上方不远处悬停
cam_c_transform = carla.Transform(
    carla.Location(x=-15.0, y=0.0, z=2.0),
    carla.Rotation(pitch=25, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform)
```

---

## D25：高速公路·无人机伴飞

**核心 Feature**：高速公路场景下的无人机伴飞，展示平台对高速运动的支持  
**导演意图**：高速公路场景的视觉感受与城市街道完全不同——宽阔的道路、高速行驶的车辆、无人机在侧方伴飞。这个画面极具动感。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town04（有高速公路） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 以 120 km/h 在高速公路行驶 |
| 无人机 | 在主车侧方 10m、高度 5m 伴飞（不是正上方，而是侧方） |
| 配角车 | 4 辆车，在不同车道以不同速度行驶 |
| 行人 | 无（高速公路无行人，符合真实场景） |

### 侧方伴飞实现

```python
# 无人机保持在车辆侧方 10m、高度 5m 的位置伴飞
def side_escort_position(vehicle_transform, side_offset=10.0, height=5.0):
    right_vector = vehicle_transform.get_right_vector()
    location = vehicle_transform.location
    
    drone_location = carla.Location(
        x=location.x + right_vector.x * side_offset,
        y=location.y + right_vector.y * side_offset,
        z=location.z + height
    )
    return drone_location

# 注意：高速行驶时，无人机需要更高的跟随速度
# AirSim 的 moveToPositionAsync 速度参数需要设置为 40+ m/s
```

### 相机配置（两路）

> **✅ 第三方可见性要求：必须增加车载后置高位相机作为第三路，拍摄无人机在车辆侧方伴飞的全景。这是整个 Demo 最关键的画面：车辆在下方高速行驶，无人机在侧方伴飞，两者同时入镜。**

**相机 A：无人机侧视 RGB**（画面左侧）：无人机相机朝向主车，拍摄主车高速行驶的侧面  
**相机 B：主车第一人称**（画面右上）：高速行驶的驾驶员视角，道路向前延伸  
**相机 C：车载后置高位相机**（画面右下）——第三方可见性关键相机
```python
# 挂载在主车后方，高度 4m，朝向前上方
# 侧方伴飞时：车顶在下方，无人机在车辆右侧上方伴飞，两者同时入镜
# 高速行驶时，这个画面是最能体现“无人机伴飞”概念的视角
cam_c_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=vehicle)
```

### 运镜设计

**0–15s**：无人机从后方追上主车，逐渐移动到侧方伴飞位置。  
**15–35s**：稳定伴飞，无人机相机缓慢从侧视转向前视，拍摄主车和前方道路。  
**35–45s**：无人机缓慢上升到 20m，从侧方高位俯视主车和整条高速公路。

---

## D26：城市峡谷·多路径规划

**核心 Feature**：高楼密集的城市峡谷场景，展示无人机在复杂城市环境中的导航能力  
**导演意图**：城市峡谷是无人机导航的最难场景之一。高楼遮挡 GPS 信号，气流复杂。这个 Demo 展示了平台对这类极端城市场景的仿真能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town01（高楼密集区域） |
| 天气 | CloudyNoon（多云，增加峡谷压迫感） |
| 时长 | 45 秒 |
| 无人机 | 在高楼之间的峡谷中低空飞行（高度 10–20m） |
| 行人 | 15 名行人在峡谷底部街道行走 |
| 车辆 | 3 辆车在峡谷底部行驶 |

### 峡谷飞行路径

```python
# 峡谷飞行路径：沿建筑物之间的街道飞行
# 高度保持在建筑物高度的 1/3 处（约 10-15m）
# 路径设计要点：
# 1. 包含至少一次 90° 转弯（在十字路口处）
# 2. 飞行时建筑物在画面两侧形成"峡谷感"
# 3. 偶尔抬头（pitch=-30°）拍摄建筑物顶部和天空

canyon_path = [
    (0, 0, -12),
    (50, 0, -12),    # 直行穿越峡谷
    (50, 30, -15),   # 转弯，略微上升
    (100, 30, -12),  # 继续直行
]
```

### 相机配置（两路）

> **✅ 第三方可见性要求：全局侧方固定机位必须能看到无人机在建筑物之间穿越的全景。地面车辆和行人应在画面中同时可见。**

**相机 A：无人机 FPV**（画面左侧）：FOV=100°，展示峡谷的压迫感  
**相机 B：全局侧方固定机位**（画面右侧）：从峡谷侧面拍摄无人机穿越的全景，行人在前景
```python
# 全局侧方固定机位应放置在峡谷入口处，能同时看到：
# 1. 无人机在建筑物之间穿越（中景）
# 2. 建筑物在两侧形成峡谷感（背景）
# 3. 地面车辆和行人在下方（前景）
cam_b_transform = carla.Transform(
    carla.Location(x=-20.0, y=0.0, z=5.0),
    carla.Rotation(pitch=-5, yaw=0, roll=0)
)
camera_b = world.spawn_actor(camera_bp, cam_b_transform)
```

---

## D27：空地双 LiDAR·点云融合

**核心 Feature**：地面 LiDAR 和空中 LiDAR 同时工作，展示双平台点云的互补性  
**导演意图**：地面 LiDAR 扫描范围广但有遮挡盲区，空中 LiDAR 俯视无遮挡但密度低。两者融合的点云覆盖了单一平台无法覆盖的区域，这个视觉对比极具说服力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03 |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 携带地面 LiDAR，缓慢行驶 |
| 无人机 | 在主车上方 20m，携带空中 LiDAR |
| 行人 | 8 名行人，部分在建筑物遮挡区域（地面 LiDAR 盲区） |

### 双 LiDAR 配置

```python
# 地面 LiDAR（水平扫描，360°）
ground_lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
ground_lidar_bp.set_attribute('channels', '64')
ground_lidar_bp.set_attribute('range', '50')
ground_lidar_bp.set_attribute('upper_fov', '15')
ground_lidar_bp.set_attribute('lower_fov', '-25')

# 空中 LiDAR（向下扫描）
aerial_lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
aerial_lidar_bp.set_attribute('channels', '32')
aerial_lidar_bp.set_attribute('range', '30')
aerial_lidar_bp.set_attribute('upper_fov', '0')
aerial_lidar_bp.set_attribute('lower_fov', '-45')  # 向下扫描
```

### 点云融合可视化

```python
# 将两个 LiDAR 的点云合并，用不同颜色区分来源
# 地面 LiDAR：蓝色
# 空中 LiDAR：橙色
# 融合点云：按高度彩虹着色

def merge_point_clouds(ground_points, aerial_points):
    # 转换坐标系到统一坐标系
    ground_xyz = ground_points[:, :3]
    aerial_xyz = aerial_points[:, :3]
    
    # 颜色区分
    ground_colors = np.tile([0.2, 0.4, 1.0], (len(ground_xyz), 1))  # 蓝色
    aerial_colors = np.tile([1.0, 0.5, 0.1], (len(aerial_xyz), 1))  # 橙色
    
    merged_xyz = np.vstack([ground_xyz, aerial_xyz])
    merged_colors = np.vstack([ground_colors, aerial_colors])
    
    return merged_xyz, merged_colors
```

### 相机配置（三路）

> **✅ 第三方可见性要求：必须增加车载后置高位相机作为第四路，确保画面中能看到无人机在车辆上方飞行。点云可视化画面是技术展示，但必须配合一路能看到真实无人机模型的画面，才能证明“这个点云是真实的无人机采集的”。**

**相机 A：地面 LiDAR 点云**（画面左上）：蓝色点云，有建筑物遗挡盲区  
**相机 B：空中 LiDAR 点云**（画面左下）：橙色点云，俯视无遗挡  
**相机 C：融合点云**（画面右上）：蓝色+橙色融合，覆盖更完整  
**相机 D：车载后置高位相机**（画面右下）——第三方可见性关键相机
```python
# 挂载在主车后方，高度 4m，拍摄无人机在车辆上方飞行
cam_d_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_d = world.spawn_actor(camera_bp, cam_d_transform, attach_to=vehicle)
```

---

## D28：无人机编队·队形变换

**核心 Feature**：多架无人机执行协同队形变换，展示多智能体协同控制能力  
**导演意图**：队形变换是多无人机协同中最有视觉美感的场景。从 V 形到横排再到菱形，队形变换的过程本身就是一种视觉艺术。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 无人机 | 3 架，执行队形变换 |
| 行人 | 10 名行人在下方广场（增加场景层次） |
| 车辆 | 2 辆车在周围道路 |

### 队形变换实现

```python
# 三种队形的相对位置（以编队中心为原点）
formations = {
    'V_shape': [
        (0, 0, 0),      # 领队
        (-5, -5, 0),    # 左翼
        (5, -5, 0),     # 右翼
    ],
    'line': [
        (-8, 0, 0),     # 左
        (0, 0, 0),      # 中
        (8, 0, 0),      # 右
    ],
    'diamond': [
        (0, 5, 0),      # 前
        (-5, 0, 0),     # 左
        (5, 0, 0),      # 右
        # 第4架：(0, -5, 0) 后（如果有4架）
    ],
}

def interpolate_formation(from_formation, to_formation, progress):
    """在两种队形之间平滑插值"""
    result = []
    for i in range(len(from_formation)):
        x = from_formation[i][0] + (to_formation[i][0] - from_formation[i][0]) * progress
        y = from_formation[i][1] + (to_formation[i][1] - from_formation[i][1]) * progress
        z = from_formation[i][2] + (to_formation[i][2] - from_formation[i][2]) * progress
        result.append((x, y, z))
    return result
```

### 相机配置（两路）

> **✅ 第三方可见性要求：两路相机必须都能看到无人机模型。全局高空俯视相机中，三架无人机的機身和旋转桨叶应清晰可见。地面上的行人和车辆应在画面中同时可见，体现“空地一体”。**

**相机 A：全局高空俯视**（画面左侧）：从 50m 高度俯视，三架无人机的队形变换清晰可见  
**相机 B：全局侧方 Orbit**（画面右侧）：从侧方 30m 高度 Orbit，展示队形的三维结构
```python
# Orbit 相机设置建议：以编队中心为原点，半径 25m，高度 10m
# 这样三架无人机都在画面中，队形变换的三维结构清晰可见
# 地面上的行人和车辆应在画面下方，体现空地一体
```

---

## D29：全天候循环·光照变化

**核心 Feature**：24 小时光照循环仿真，展示平台的时间和光照变化能力  
**导演意图**：从日出到正午到日落到夜晚的完整光照变化，是视觉上最美的 Demo 之一。城市在不同光照条件下呈现完全不同的视觉质感。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 时长 | 60 秒（模拟 24 小时，每秒约 24 分钟） |
| 车辆 | 2 辆车，固定在主干道缓慢行驶 |
| 无人机 | 悬停在十字路口上方 25m，固定不动 |
| 行人 | 8 名行人 |

### 光照循环实现

```python
# 通过连续改变太阳高度角来模拟 24 小时光照循环
# sun_altitude_angle: -90（深夜）→ 0（日出）→ 90（正午）→ 0（日落）→ -90（深夜）

total_frames = 60 * 30  # 1800帧

for tick in range(total_frames):
    progress = tick / total_frames  # 0.0 到 1.0
    
    # 太阳高度角：从 -90 到 90 再回到 -90（一个完整周期）
    sun_angle = -90 + 180 * math.sin(progress * math.pi)
    
    # 云量随时间变化（日出和日落时云量增加，增加戏剧感）
    cloudiness = 20 + 30 * abs(math.sin(progress * math.pi))
    
    weather = carla.WeatherParameters(
        sun_altitude_angle=sun_angle,
        cloudiness=cloudiness,
        precipitation=0.0,
        fog_density=max(0, 10 * (1 - abs(sun_angle) / 90))  # 清晨有轻雾
    )
    carla_world.set_weather(weather)
    carla_world.tick()
```

### 相机配置（两路）

> **✅ 第三方可见性要求：必须增加车载后置高位相机作为第三路。全天候光照循环中，无人机在日出、正午、日落、夜晚四种光照下的外观应在画面中可见。夜晚时无人机的灯光在黑暗中尤为显眼。**

**相机 A：全局 Orbit**（画面左侧）：展示城市在不同光照下的整体氛围  
**相机 B：无人机俯视**（画面右上）：展示地面在不同光照下的阴影变化  
**相机 C：车载后置高位相机**（画面右下）——第三方可见性关键相机
```python
# 挂载在地面车辆后方，高度 4m，拍摄无人机在上方悬停
# 全天候光照循环中：日出时无人机被橙红色晶射光渲染，日落时被金色光渲染，夜晚时灯光闪烁
cam_c_transform = carla.Transform(
    carla.Location(x=-5.0, y=0.0, z=4.0),
    carla.Rotation(pitch=-15, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=vehicle)
```

### 关键时刻

**日出（约第 15 秒）**：橙红色光线从地平线升起，建筑物投下长长的阴影。  
**正午（约第 30 秒）**：强烈的顶光，阴影最短，城市最明亮。  
**日落（约第 45 秒）**：金色光线，建筑物被染成暖色，是整个 Demo 中视觉最美的时刻。  
**夜晚（约第 55 秒）**：路灯亮起，城市进入夜间模式。

---

## D30：综合压力测试·最大规模场景

**核心 Feature**：平台在最大规模场景下的稳定性，展示 CARLA-Air 的极限能力  
**导演意图**：这是整套 Demo 的"压轴"。最多的车辆、最多的行人、最多的无人机，同时在同一个场景中运行，展示平台的规模能力和稳定性。这个 Demo 的叙事是"这个平台能承受多大的规模"。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 60 秒 |
| 车辆 | 20 辆，全部自动驾驶 |
| 行人 | 50 名，全部 AI 控制 |
| 无人机 | 3 架（最大支持数量），各自执行不同任务 |

### 大规模场景生成

```python
# 批量生成 20 辆车
vehicle_bps = blueprint_library.filter('vehicle.*')
spawn_points = carla_world.get_map().get_spawn_points()

batch_vehicles = []
for i in range(min(20, len(spawn_points))):
    bp = random.choice(vehicle_bps)
    if bp.has_attribute('color'):
        bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
    batch_vehicles.append(carla.command.SpawnActor(bp, spawn_points[i]))

vehicle_results = carla_client.apply_batch_sync(batch_vehicles, True)
vehicles = [carla_world.get_actor(r.actor_id) for r in vehicle_results if not r.error]

# 开启所有车辆自动驾驶
for v in vehicles:
    v.set_autopilot(True)

# 批量生成 50 名行人（参考 D15 的实现）
```

### 相机配置（三路）

> **✅ 第三方可见性要求：所有三路相机必须都能看到无人机模型。全局高空俯视画面中，3 架无人机的機身应清晰可见。无人机俯视画面中，建议在无人机下方加载一个向前下方值班的相机，这样无人机機身和桨叶会在画面上方入镜。这是整套 Demo 的压轴，必须确保无人机清晰可见。**

**相机 A：全局高空俯视（60m）**（画面上方，占 50% 高度）：展示整个场景的规模，20 辆车、50 名行人、3 架无人机同时可见  
**相机 B：无人机1俯视**（画面左下）：追踪车辆，无人机機身在画面上方入镜  
**相机 C：无人机2俯视**（画面右下）：追踪行人，无人机機身在画面上方入镜

### 运镜设计

**0–20s**：全局高空俯视，缓慢 Orbit，展示场景的整体规模。  
**20–45s**：切换到三分屏，展示三架无人机各自的任务视角。  
**45–60s**：回到全局高空俯视，Orbit 相机缓慢拉高到 100m，最终展示整个 Town10HD 城市全景，所有智能体都在画面中。

### 性能监控

```python
# 在录制过程中监控帧率，确保 30 FPS 稳定
import time

frame_times = []
for tick in range(60 * 30):
    start_time = time.time()
    carla_world.tick()
    end_time = time.time()
    frame_times.append(end_time - start_time)
    
    # 如果帧时间超过 40ms（低于 25 FPS），记录警告
    if end_time - start_time > 0.04:
        print(f"Warning: Frame {tick} took {(end_time-start_time)*1000:.1f}ms")

print(f"Average FPS: {1.0 / np.mean(frame_times):.1f}")
```

---

# 附录：通用工程规范

## 坐标系转换（CARLA ↔ AirSim）

```python
def carla_to_airsim(carla_location):
    """CARLA 坐标系转换为 AirSim NED 坐标系"""
    # CARLA: X前 Y右 Z上（右手系）
    # AirSim: X前 Y右 Z下（NED，Z轴反向）
    return airsim.Vector3r(
        x_val=carla_location.x,
        y_val=carla_location.y,
        z_val=-carla_location.z  # Z轴取反
    )

def airsim_to_carla(airsim_vector):
    """AirSim NED 坐标系转换为 CARLA 坐标系"""
    return carla.Location(
        x=airsim_vector.x_val,
        y=airsim_vector.y_val,
        z=-airsim_vector.z_val  # Z轴取反
    )
```

## 同步模式下的帧同步

```python
# 确保 CARLA 和 AirSim 在同一帧推进
# CARLA 使用 world.tick() 推进
# AirSim 在同步模式下使用 simPause/simContinueForFrames

settings = airsim_client.getSimulatorSettings()
# AirSim 同步模式配置见 settings.json:
# "ClockSpeed": 1.0
# "LocalHostIp": "127.0.0.1"

# 主循环中的同步方式
for tick in range(total_frames):
    carla_world.tick()  # CARLA 推进一帧
    # AirSim 会自动跟随（因为共享同一个 UE4 GameLoop）
    time.sleep(1.0 / 30.0)  # 控制帧率
```

## 视频输出规范

```bash
# 将图片序列合成为视频
ffmpeg -framerate 30 \
       -i frame_%06d.png \
       -c:v libx264 \
       -crf 18 \
       -preset slow \
       -pix_fmt yuv420p \
       -movflags +faststart \
       output_1080p.mp4

# 从 2K 下采样到 1080p（提高画质）
ffmpeg -i output_2k.mp4 \
       -vf scale=1920:1080 \
       -c:v libx264 -crf 16 \
       output_1080p_hq.mp4
```

## 常见问题排查

| 问题 | 原因 | 解决方法 |
|------|------|----------|
| 帧率不稳定 | 场景复杂度过高 | 减少行人/车辆数量，或降低 LiDAR 点数 |
| AirSim 无人机位置与 CARLA 不同步 | 坐标系转换错误 | 检查 Z 轴方向转换 |
| 相机画面有撕裂感 | 未开启同步模式 | 确认 `synchronous_mode = True` |
| 行人穿越建筑物 | Navigation mesh 问题 | 使用 `get_random_location_from_navigation()` 生成行人位置 |
| 录制帧丢失 | 磁盘写入速度不足 | 使用 SSD，或先录制到内存再写入 |

---

*文档版本：v1.0 | 日期：2026-03-12 | 作者：CARLA-Air 导演组*
