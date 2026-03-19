# CARLA-Air Demo 拍摄工程指南 v5.0

> **版本说明**：本版本在 v4.0 基础上精简为 15 个核心 Demo，核心变化：
> 1. 无人机全程匀速单向飞行，禁止悬停等待
> 2. 车辆速度根据无人机速度配速，保持相对位置稳定
> 3. CAM-B 独立自由相机 + EMA 滤波，彻底消除抖动
> 4. 所有 Demo 均保证四格构图中无人机可见

---

## 一、通用运动规范（所有 Demo 必须遵守）

### 1.1 核心运动原则

> **无人机必须始终保持单向匀速飞行，严禁悬停等待车辆。**

传统做法是让无人机悬停在车辆上方等待，这会导致两个严重问题：第一，无人机在悬停时 PID 控制器会持续微调姿态，产生高频抖动，CAM-A 和 CAM-B 画面都会剧烈晃动；第二，悬停画面在视觉上是静止的，完全没有运动感，作为展示视频极其无聊。

**正确做法**：无人机以固定速度沿直线飞行，车辆跟随无人机配速，两者保持相对静止。这样无人机处于稳定的匀速飞行状态，PID 控制器不需要频繁修正，画面最稳定。

```python
# ── 核心运动控制逻辑 ──────────────────────────────────────────────

DRONE_SPEED   = 8.0   # 无人机匀速飞行速度（m/s），约 29 km/h
VEHICLE_SPEED = 8.0   # 车辆目标速度（m/s），与无人机保持一致
BACK_OFFSET   = 4.0   # 无人机落后车辆的距离（m）
SIDE_OFFSET   = 1.5   # 无人机侧向偏移（m），略偏右，构图更自然
DRONE_HEIGHT  = 7.0   # 无人机飞行高度（m），城市场景推荐 6–8m

# 无人机飞行路径：提前规划一条直线航点序列
# 不要用 moveToPositionAsync 实时跟随（会产生加减速抖动）
# 而是用 moveOnPathAsync 沿预规划路径匀速飞行

import airsim, math

def plan_straight_path(start_x, start_y, height, yaw_deg, 
                        total_distance=200.0, step=5.0):
    """
    规划一条直线飞行路径（航点序列）。
    start_x, start_y: 起点坐标（CARLA 坐标系，单位 m）
    height:           飞行高度（m）
    yaw_deg:          飞行方向（度，CARLA 坐标系）
    total_distance:   总飞行距离（m）
    step:             航点间距（m），越小路径越平滑
    """
    yaw_rad = math.radians(yaw_deg)
    path = []
    d = 0.0
    while d <= total_distance:
        x = start_x + d * math.cos(yaw_rad)
        y = start_y + d * math.sin(yaw_rad)
        path.append(airsim.Vector3r(x_val=x, y_val=y, z_val=-height))
        d += step
    return path

# 启动无人机匀速飞行（非阻塞）
drone_path = plan_straight_path(
    start_x=drone_start_x, start_y=drone_start_y,
    height=DRONE_HEIGHT, yaw_deg=road_direction_deg,
    total_distance=300.0
)
airsim_client.moveOnPathAsync(
    drone_path,
    velocity=DRONE_SPEED,
    drivetrain=airsim.DrivetrainType.ForwardOnly,
    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=road_direction_deg)
)

# 车辆配速：使用 CARLA 的 autopilot + 速度控制
# 或者直接设置车辆速度（更精确）
vehicle_control = carla.VehicleControl()
vehicle_control.throttle = 0.5  # 初始油门，后续通过 PID 维持目标速度

def maintain_vehicle_speed(vehicle, target_speed_ms):
    """简单的速度维持控制器，保持车辆与无人机同速。"""
    current_v = vehicle.get_velocity()
    current_speed = math.sqrt(current_v.x**2 + current_v.y**2)
    error = target_speed_ms - current_speed
    control = carla.VehicleControl()
    control.throttle = max(0.0, min(1.0, 0.5 + error * 0.1))
    control.brake    = max(0.0, min(1.0, -error * 0.05)) if error < 0 else 0.0
    vehicle.apply_control(control)
```

### 1.2 标准四格构图

```
┌─────────────────┬─────────────────┐
│                 │                 │
│   CAM-A         │   CAM-B         │
│   无人机第一视角 │   无人机第三视角 │
│  （机腹前视）   │  （独立自由相机）│
│                 │                 │
├─────────────────┼─────────────────┤
│                 │                 │
│   CAM-C         │   CAM-D         │
│   车辆第一视角  │   车辆第三视角   │
│  （挡风玻璃后） │  （车辆后上方）  │
│                 │                 │
└─────────────────┴─────────────────┘
```

**FFmpeg 四格合成命令：**

```bash
ffmpeg \
  -i cam_a.mp4 -i cam_b.mp4 -i cam_c.mp4 -i cam_d.mp4 \
  -filter_complex "
    [0:v]scale=960:540[a];
    [1:v]scale=960:540[b];
    [2:v]scale=960:540[c];
    [3:v]scale=960:540[d];
    [a][b]hstack[top];
    [c][d]hstack[bottom];
    [top][bottom]vstack[out]
  " \
  -map "[out]" -c:v libx264 -crf 18 output_4grid.mp4
```

### 1.3 四路相机配置标准

#### CAM-A：无人机第一视角（attach 到无人机，允许运动感）

```python
# AirSim settings.json 中配置
{
    "CameraName": "cam_a_fpv",
    "CaptureSettings": [{"ImageType": 0, "Width": 1920, "Height": 1080, "FOV_Degrees": 90}],
    "X": 0.1, "Y": 0.0, "Z": 0.05,
    "Pitch": -30, "Roll": 0, "Yaw": 0
}
# Pitch=-30°：向前下方倾斜，既能看到前方道路，也能看到正下方的车辆/行人
# 不拍到螺旋桨：相机在桨盘平面以下
```

#### CAM-B：无人机第三视角（独立自由相机，每帧手动更新）

> ⚠️ **绝对不能 attach_to=drone**，否则无人机姿态抖动会直接传递到画面。

```python
import math

class SmoothFollowCamera:
    """
    独立自由相机，跟随无人机但不 attach。
    每帧手动计算目标位置，EMA 低通滤波消除抖动。
    """
    def __init__(self, world, camera_bp, alpha=0.10,
                 offset_back=5.0, offset_up=3.0):
        self.alpha       = alpha        # EMA 系数，0.10 推荐
        self.offset_back = offset_back  # 后退距离（m）
        self.offset_up   = offset_up    # 上升高度（m）
        self.smooth_x = self.smooth_y = self.smooth_z = None
        init_tf = carla.Transform(carla.Location(x=0, y=0, z=100))
        self.camera = world.spawn_actor(camera_bp, init_tf)

    def update(self, drone_loc: carla.Location, flight_yaw_deg: float):
        """
        drone_loc:      无人机在 CARLA 坐标系中的位置
        flight_yaw_deg: 无人机飞行方向（度），用于计算"后方"
        """
        yaw_rad  = math.radians(flight_yaw_deg)
        target_x = drone_loc.x - self.offset_back * math.cos(yaw_rad)
        target_y = drone_loc.y - self.offset_back * math.sin(yaw_rad)
        target_z = drone_loc.z + self.offset_up

        if self.smooth_x is None:
            self.smooth_x, self.smooth_y, self.smooth_z = target_x, target_y, target_z
        else:
            a = self.alpha
            self.smooth_x = a * target_x + (1-a) * self.smooth_x
            self.smooth_y = a * target_y + (1-a) * self.smooth_y
            self.smooth_z = a * target_z + (1-a) * self.smooth_z

        # 相机始终朝向无人机当前位置
        dx = drone_loc.x - self.smooth_x
        dy = drone_loc.y - self.smooth_y
        dz = drone_loc.z - self.smooth_z
        dist_h = math.sqrt(dx**2 + dy**2) + 1e-6
        pitch  = math.degrees(math.atan2(-dz, dist_h))
        yaw    = math.degrees(math.atan2(dy, dx))

        self.camera.set_transform(carla.Transform(
            carla.Location(x=self.smooth_x, y=self.smooth_y, z=self.smooth_z),
            carla.Rotation(pitch=pitch, yaw=yaw, roll=0)
        ))

# 使用方式（主循环中每帧调用）：
cam_b = SmoothFollowCamera(world, camera_bp, alpha=0.10,
                            offset_back=5.0, offset_up=3.0)
for tick in range(total_frames):
    world.tick()
    drone_loc_carla = get_drone_location_in_carla(airsim_client)
    cam_b.update(drone_loc_carla, flight_yaw_deg=road_direction_deg)
```

#### CAM-C：车辆第一视角（attach 到车辆）

```python
cam_c = world.spawn_actor(
    camera_bp,
    carla.Transform(
        carla.Location(x=0.5, y=0.0, z=1.4),
        carla.Rotation(pitch=-5, yaw=0, roll=0)
    ),
    attach_to=vehicle
)
# 驾驶员视角，pitch=-5° 略向下，视野自然
# 画面内容：前方道路、行人、交通灯、其他车辆
```

#### CAM-D：车辆第三视角（attach 到车辆）

```python
cam_d = world.spawn_actor(
    camera_bp,
    carla.Transform(
        carla.Location(x=-8.0, y=0.0, z=5.0),
        carla.Rotation(pitch=-15, yaw=0, roll=0)
    ),
    attach_to=vehicle
)
# 车辆后上方，pitch=-15° 向下看
# 画面内容：主车在画面中下方，无人机在画面上方（因无人机飞在车辆斜后上方）
# 注意：无人机飞行高度 7m，CAM-D 高度 5m，无人机在画面上方约 1/3 处
```

### 1.4 无人机飞行位置标准

无人机相对于车辆的标准偏移：

| 参数 | 值 | 说明 |
|------|----|------|
| 落后距离 | **4m**（约 1 个车身） | 从 CAM-D 看：无人机在车辆斜前上方，两者同框 |
| 侧向偏移 | **1.5m**（略偏右） | 避免正后方透视压缩，构图更自然 |
| 飞行高度 | **6–8m**（城市） / **5–7m**（追踪行人） | 低空贴近，目标清晰，无人机在 CAM-D 中显眼 |

```python
def get_drone_target_position(vehicle_transform,
                               back=4.0, side=1.5, height=7.0):
    """
    计算无人机目标位置（CARLA 坐标系）。
    无人机飞在车辆斜后上方，落后 back 米，侧偏 side 米，高度 height 米。
    返回 airsim.Vector3r（AirSim 坐标系，Z 轴取反）。
    """
    loc = vehicle_transform.location
    yaw = math.radians(vehicle_transform.rotation.yaw)
    x = loc.x - back * math.cos(yaw) - side * math.sin(yaw)
    y = loc.y - back * math.sin(yaw) + side * math.cos(yaw)
    z = loc.z + height
    return airsim.Vector3r(x_val=x, y_val=y, z_val=-z)
```

### 1.5 坐标系转换（AirSim ↔ CARLA）

```python
def airsim_to_carla_location(airsim_pos) -> carla.Location:
    """AirSim NED 坐标系 → CARLA 右手坐标系"""
    return carla.Location(
        x= airsim_pos.x_val,
        y= airsim_pos.y_val,
        z=-airsim_pos.z_val   # Z 轴方向相反
    )

def carla_to_airsim_location(carla_loc) -> airsim.Vector3r:
    """CARLA 右手坐标系 → AirSim NED 坐标系"""
    return airsim.Vector3r(
        x_val= carla_loc.x,
        y_val= carla_loc.y,
        z_val=-carla_loc.z    # Z 轴方向相反
    )
```

### 1.6 基础参数

| 参数 | 值 |
|------|----|
| 分辨率 | 1920×1080（每路相机） |
| 帧率 | 30 FPS，CARLA 同步模式 |
| 编码 | H.264，CRF=18 |
| 同步模式 | `synchronous_mode=True`，`fixed_delta_seconds=1/30` |
| 无人机速度 | 8 m/s（约 29 km/h），可按场景调整 |
| 车辆速度 | 与无人机一致，8 m/s |

---

## 二、Demo 精选总览（15 个）

精选逻辑：覆盖所有核心 Feature（空地协同、天气同步、传感器、行人、多机协同、VLN），去除重复度高的变体，保留视觉冲击力最强的版本。

| # | Demo 名称 | 核心 Feature | 地图 | 时长 | 优先级 |
|---|-----------|-------------|------|------|--------|
| D01 | 空地协同追踪·城市主干道 | 基础空地同框，平台名片 | Town10HD | 40s | **P0** |
| D02 | 天气实时切换·空地同步 | 单进程同步天气，最强视觉论据 | Town10HD | 45s | **P0** |
| D03 | 四模态传感器·同帧对比 | RGB+深度+语义+LiDAR 同步 | Town10HD | 40s | **P0** |
| D04 | 无人机追踪行人·低空近景 | 无人机低空追踪行人，Human-UAV | Town03 | 35s | **P0** |
| D05 | 空地双视角·同一事件 | 空中预知 vs 地面局限，叙事最强 | Town10HD | 40s | **P0** |
| D06 | 夜间红外·行人热成像 | 夜间多传感器，极端光照仿真 | Town10HD | 40s | **P0** |
| D07 | 高速公路·无人机侧方伴飞 | 高速运动下的空地协同 | Town04 | 40s | P1 |
| D08 | 多车协同·无人机俯视调度 | 多辆车+无人机，城市交叉路口 | Town05 | 45s | P1 |
| D09 | 无人机搜救·引导车辆 | 无人机发现行人，引导车辆救援 | Town03 | 50s | **P0** |
| D10 | 空地 LiDAR·点云融合 | 地面+空中 LiDAR 互补覆盖 | Town05 | 40s | P1 |
| D11 | 多无人机编队·异构协同 | 3 架无人机追踪不同目标 | Town10HD | 45s | P1 |
| D12 | VLN 导航·语言指令驾驶 | 语言指令引导空地导航 | Town01 | 40s | P1 |
| D13 | 全天候光照·传感器对比 | 14 种天气下传感器性能变化 | Town10HD | 50s | P1 |
| D14 | 城市峡谷·低空穿越 | 无人机高楼间穿越，视觉冲击 | Town01 | 35s | P1 |
| D15 | 压轴·最大规模场景 | 多车+多人+多机同时运行 | Town10HD | 55s | **P0** |

---

## 三、Demo 详细拍摄指南

---

### D01：空地协同追踪·城市主干道

**导演意图**：这是整套 Demo 的"名片"，用最干净的画面展示 CARLA-Air 最核心的能力——无人机和车辆在同一个仿真世界里同时运行，四格画面让观众一眼看清空地一体的全貌。

**运动设计**：无人机沿 Town10HD 主干道方向匀速飞行，车辆配速跟随，全程直线，无转弯，无悬停。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon（最高画质，阴影清晰） |
| 无人机速度 | 8 m/s |
| 车辆速度 | 8 m/s（与无人机同速） |
| 无人机高度 | 7m |
| 无人机偏移 | 落后 4m，右偏 1.5m |
| 行人 | 15 名，人行道自然行走 |
| 配角车 | 5 辆，周围车道正常行驶 |
| 时长 | 40 秒 |

**相机配置：**

- **CAM-A（左上）**：无人机机腹前视，pitch=-30°，FOV=90°。画面：前方道路，主车在画面下方，行人在人行道两侧。
- **CAM-B（右上）**：独立自由相机，offset_back=5m，offset_up=3m，alpha=0.10。画面：无人机机身在上方 1/3，主车在中下方，城市街道为背景。
- **CAM-C（左下）**：车辆挡风玻璃后，pitch=-5°。画面：前方道路，行人，交通灯，其他车辆。
- **CAM-D（右下）**：车辆后方 8m，高 5m，pitch=-15°。画面：主车在中下方，无人机在上方（因无人机飞在车辆斜后上方 7m 处）。

**运镜节奏（40 秒）：**

- 0–8s：四格画面同时入场，无人机和车辆已在运动中，避免起步加速的抖动
- 8–30s：匀速巡航，展示城市街道全貌，行人在 CAM-A 和 CAM-C 中穿行
- 30–40s：车辆通过一个有行人的路口，CAM-A 中行人近距离出现，CAM-D 中无人机和车辆同框最清晰

```python
# D01 场景初始化
world = client.load_world('Town10HD')
weather = carla.WeatherParameters.ClearNoon
world.set_weather(weather)

# 选择一条笔直的主干道（Town10HD 中推荐使用中央大道）
# 车辆起点：主干道起始位置
vehicle_spawn = carla.Transform(
    carla.Location(x=-50, y=0, z=0.5),
    carla.Rotation(yaw=0)  # 朝 X 轴正方向行驶
)
vehicle = world.spawn_actor(vehicle_bp, vehicle_spawn)

# 无人机起点：车辆起点的斜后上方
drone_start_x = vehicle_spawn.location.x - 4.0   # 落后 4m
drone_start_y = vehicle_spawn.location.y + 1.5    # 右偏 1.5m
drone_start_z = 7.0                                # 高度 7m

airsim_client.takeoffAsync().join()
airsim_client.moveToPositionAsync(
    drone_start_x, drone_start_y, -drone_start_z, velocity=5
).join()

# 规划直线路径并开始匀速飞行
path = plan_straight_path(
    start_x=drone_start_x, start_y=drone_start_y,
    height=drone_start_z, yaw_deg=0,  # 朝 X 轴正方向
    total_distance=350.0
)
airsim_client.moveOnPathAsync(path, velocity=8.0,
    drivetrain=airsim.DrivetrainType.ForwardOnly,
    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0))

# 同步启动车辆
# 使用 autopilot 或手动控制维持 8 m/s
vehicle.set_autopilot(True)
# 建议：关闭 autopilot，改用手动速度控制以精确配速
```

---

### D02：天气实时切换·空地同步

**导演意图**：这是整套 Demo 中最有力的单个视觉论据。在同一个画面里，天气从晴天切换到暴雨，四格画面同时变化，没有任何延迟——这一帧直接证明了"单进程同步"的核心技术价值。

**运动设计**：与 D01 相同的直线匀速运动，天气切换在运动过程中进行，不停车不悬停。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气序列 | ClearNoon → CloudyNoon → WetNoon → HardRainNoon → ClearSunset |
| 每段天气时长 | 约 8 秒 |
| 无人机速度 | 8 m/s |
| 车辆速度 | 8 m/s |
| 无人机高度 | 7m |
| 时长 | 45 秒 |

**关键拍摄细节**：天气切换使用 CARLA 的 `world.set_weather()` 逐帧插值，不要硬切，要有 2–3 秒的渐变过渡。暴雨时路面会出现积水反光，CAM-C（车辆第一视角）中雨滴打在挡风玻璃上的效果是最强的视觉时刻。

```python
import numpy as np

# 天气序列定义
weather_sequence = [
    carla.WeatherParameters.ClearNoon,
    carla.WeatherParameters.CloudyNoon,
    carla.WeatherParameters.WetNoon,
    carla.WeatherParameters.HardRainNoon,
    carla.WeatherParameters.ClearSunset,
]
frames_per_weather = 8 * 30   # 8 秒 × 30 FPS = 240 帧
transition_frames  = 2 * 30   # 2 秒渐变过渡

def lerp_weather(w1, w2, t):
    """在两个天气参数之间线性插值，t ∈ [0, 1]"""
    def lerp(a, b): return a + (b - a) * t
    return carla.WeatherParameters(
        cloudiness            = lerp(w1.cloudiness,             w2.cloudiness),
        precipitation         = lerp(w1.precipitation,          w2.precipitation),
        precipitation_deposits= lerp(w1.precipitation_deposits, w2.precipitation_deposits),
        wind_intensity        = lerp(w1.wind_intensity,         w2.wind_intensity),
        sun_azimuth_angle     = lerp(w1.sun_azimuth_angle,      w2.sun_azimuth_angle),
        sun_altitude_angle    = lerp(w1.sun_altitude_angle,     w2.sun_altitude_angle),
        fog_density           = lerp(w1.fog_density,            w2.fog_density),
        wetness               = lerp(w1.wetness,                w2.wetness),
    )

# 主录制循环中的天气控制
current_weather_idx = 0
frame_in_weather    = 0

for tick in range(total_frames):
    world.tick()
    frame_in_weather += 1

    if frame_in_weather <= transition_frames and current_weather_idx > 0:
        # 渐变过渡阶段
        t = frame_in_weather / transition_frames
        w = lerp_weather(
            weather_sequence[current_weather_idx - 1],
            weather_sequence[current_weather_idx],
            t
        )
        world.set_weather(w)
    else:
        world.set_weather(weather_sequence[current_weather_idx])

    if frame_in_weather >= frames_per_weather:
        current_weather_idx = min(current_weather_idx + 1, len(weather_sequence) - 1)
        frame_in_weather = 0
```

---

### D03：四模态传感器·同帧对比

**导演意图**：展示 CARLA-Air 的传感器能力——同一帧画面，四个格子分别显示 RGB、深度图、语义分割、LiDAR 点云投影，让研究者直观看到多模态数据的同步性。

**运动设计**：直线匀速，与 D01 相同。四格布局改为传感器对比布局（非标准四格）。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 传感器布局 | 左上=RGB，右上=深度图，左下=语义分割，右下=LiDAR 点云 |
| 无人机高度 | 8m（稍高，LiDAR 扫描范围更广） |
| 时长 | 40 秒 |

**相机/传感器配置：**

```python
# 四种传感器均挂载在无人机上（机腹朝前下方）
# 1. RGB 相机
rgb_bp = world.get_blueprint_library().find('sensor.camera.rgb')
rgb_bp.set_attribute('image_size_x', '960')
rgb_bp.set_attribute('image_size_y', '540')
rgb_bp.set_attribute('fov', '90')

# 2. 深度相机
depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
depth_bp.set_attribute('image_size_x', '960')
depth_bp.set_attribute('image_size_y', '540')
depth_bp.set_attribute('fov', '90')

# 3. 语义分割相机
seg_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
seg_bp.set_attribute('image_size_x', '960')
seg_bp.set_attribute('image_size_y', '540')
seg_bp.set_attribute('fov', '90')

# 4. LiDAR（挂载在无人机下方，向下扫描）
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('range', '50')
lidar_bp.set_attribute('points_per_second', '1000000')
lidar_bp.set_attribute('rotation_frequency', '30')

# 所有传感器使用相同的挂载变换（机腹前视）
sensor_transform = carla.Transform(
    carla.Location(x=0.1, y=0, z=0.05),
    carla.Rotation(pitch=-30, yaw=0, roll=0)
)
# 注意：以上传感器挂载在 CARLA 中对应的无人机 actor 上
# 如果使用 AirSim 控制无人机，需要将 CARLA actor 绑定到 AirSim 无人机位置

# LiDAR 点云转图像（用于四格显示）
def lidar_to_image(point_cloud, img_width=960, img_height=540):
    """将 LiDAR 点云投影到 BEV（鸟瞰图）图像"""
    import numpy as np
    points = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4)
    # BEV 投影：x→水平，y→垂直，intensity→颜色
    bev = np.zeros((img_height, img_width, 3), dtype=np.uint8)
    scale = 5.0  # 每像素代表 5m
    cx, cy = img_width // 2, img_height // 2
    for p in points:
        px = int(cx + p[0] / scale * (img_width / 2))
        py = int(cy - p[1] / scale * (img_height / 2))
        if 0 <= px < img_width and 0 <= py < img_height:
            intensity = min(255, int(p[3] * 255))
            bev[py, px] = [intensity, intensity, 255]
    return bev
```

---

### D04：无人机追踪行人·低空近景

**导演意图**：展示 CARLA-Air 的 Human-UAV Interaction 能力。无人机在 5–7m 的极低高度追踪一名行人，CAM-A 中行人占画面 1/3 以上，清晰可见面部和动作。同时车辆在旁边道路行驶，保持空地三者同框。

**运动设计**：行人沿人行道直线行走（1.8 m/s），无人机以相同速度跟随，高度 6m。车辆在相邻车道以 8 m/s 行驶。三者均为单向直线运动。

| 参数 | 值 |
|------|----|
| 地图 | Town03（有宽阔人行道的城市广场区域） |
| 天气 | ClearNoon |
| 无人机速度 | 1.8 m/s（与行人同速） |
| 无人机高度 | **6m**（极低，行人清晰） |
| 行人速度 | 1.8 m/s，直线行走 |
| 车辆速度 | 8 m/s（相邻车道） |
| 行人数量 | 目标行人 1 名（主角）+ 背景行人 10 名 |
| 时长 | 35 秒 |

**关键拍摄细节**：

无人机追踪行人时，偏移量需要调整（不再是落后车辆，而是落后行人）：

```python
def drone_follow_pedestrian(pedestrian_transform,
                              back=3.0,    # 落后行人 3m（比追车近）
                              side=0.5,    # 侧偏 0.5m
                              height=6.0): # 极低高度
    loc = pedestrian_transform.location
    yaw = math.radians(pedestrian_transform.rotation.yaw)
    x = loc.x - back * math.cos(yaw) - side * math.sin(yaw)
    y = loc.y - back * math.sin(yaw) + side * math.cos(yaw)
    z = loc.z + height
    return airsim.Vector3r(x_val=x, y_val=y, z_val=-z)

# 行人控制：让行人沿直线行走
pedestrian_control = carla.WalkerControl()
pedestrian_control.speed     = 1.8
pedestrian_control.direction = carla.Vector3D(x=1.0, y=0.0, z=0.0)
pedestrian.apply_control(pedestrian_control)

# CAM-B 偏移参数调整（追踪行人时相机更近）
cam_b = SmoothFollowCamera(world, camera_bp, alpha=0.10,
                            offset_back=4.0, offset_up=2.5)
```

**CAM-D 调整**：追踪行人时，车辆在相邻车道，CAM-D 需要侧向偏移以同时拍到行人、无人机和车辆：

```python
# CAM-D 侧向偏移，拍到相邻车道的车辆
cam_d = world.spawn_actor(
    camera_bp,
    carla.Transform(
        carla.Location(x=-6.0, y=-3.0, z=4.0),  # 向左偏 3m，拍到相邻车道
        carla.Rotation(pitch=-15, yaw=15, roll=0) # 略向右看
    ),
    attach_to=vehicle
)
```

---

### D05：空地双视角·同一事件

**导演意图**：展示空地协同的核心价值——无人机从高处看到了车辆看不到的信息。具体场景：前方路口有行人突然横穿，无人机 CAM-A 提前 3 秒看到，车辆 CAM-C 此时还看不到（被前车遮挡）。这是最有叙事张力的 Demo。

**运动设计**：直线匀速行驶至路口，行人在路口横穿触发事件。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD（选择有前车遮挡的路口） |
| 天气 | ClearNoon |
| 无人机高度 | 10m（稍高，视野更广，能提前看到路口） |
| 前车 | 1 辆，在主车前方 15m 行驶，遮挡 CAM-C 视线 |
| 横穿行人 | 1 名，在路口突然横穿 |
| 时长 | 40 秒 |

**关键叙事时刻**（需要精确控制时间）：

```python
# 行人横穿触发时机：
# - 无人机 CAM-A 在 t=20s 时看到行人进入路口
# - 车辆 CAM-C 在 t=23s 时才能看到行人（被前车遮挡了 3 秒）
# 这 3 秒的差异是整个 Demo 的核心叙事时刻

# 控制行人在正确时机进入路口
pedestrian_spawn_time = 20 * 30  # 第 600 帧（t=20s）行人开始横穿

for tick in range(total_frames):
    world.tick()
    if tick == pedestrian_spawn_time:
        # 行人开始横穿
        ped_control = carla.WalkerControl()
        ped_control.speed = 1.5
        ped_control.direction = carla.Vector3D(x=0, y=1, z=0)  # 横向穿越
        pedestrian.apply_control(ped_control)
```

---

### D06：夜间红外·行人热成像

**导演意图**：展示极端光照条件下的传感器能力。左上=RGB（漆黑一片，几乎看不到），右上=红外热成像（行人清晰可见），左下=车辆 RGB，右下=车辆红外。对比效果极其震撼。

**运动设计**：直线匀速，夜间场景，街灯提供点状光源。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | `ClearNight`（自定义：sun_altitude_angle=-90，无月光） |
| 无人机高度 | 8m |
| 行人 | 20 名，部分在街灯下，部分在阴影中 |
| 时长 | 40 秒 |

```python
# 夜间天气配置
night_weather = carla.WeatherParameters(
    cloudiness=0,
    precipitation=0,
    sun_altitude_angle=-90,   # 太阳在地平线以下，完全夜间
    sun_azimuth_angle=0,
    fog_density=0,
    wetness=0,
)
world.set_weather(night_weather)

# 红外相机（使用语义分割相机模拟热成像效果）
# CARLA 没有原生红外相机，使用深度相机 + 后处理模拟
# 或使用 AirSim 的 InfraRed 相机类型（ImageType=5）
# AirSim settings.json 中配置红外相机：
{
    "CameraName": "infrared",
    "CaptureSettings": [{"ImageType": 5,  # InfraRed
                          "Width": 1920, "Height": 1080}],
    "X": 0.1, "Y": 0.0, "Z": 0.05,
    "Pitch": -30, "Roll": 0, "Yaw": 0
}
```

---

### D07：高速公路·无人机侧方伴飞

**导演意图**：展示高速运动场景下的空地协同。无人机不在车辆正后方，而是在车辆右侧方 3m 处伴飞，CAM-B 从侧面拍摄无人机和车辆同框，高速运动感极强。

**运动设计**：高速公路直线，车辆 25 m/s（90 km/h），无人机同速侧方伴飞。

| 参数 | 值 |
|------|----|
| 地图 | Town04（高速公路段） |
| 天气 | ClearNoon |
| 无人机速度 | **25 m/s**（90 km/h） |
| 车辆速度 | 25 m/s |
| 无人机位置 | 车辆右侧 3m，高度 4m（与车辆几乎同高） |
| 时长 | 40 秒 |

**侧方伴飞的相机调整**：

```python
# 侧方伴飞时，CAM-B 需要从侧面拍摄
# 相机位于无人机右后方，朝向左前方（拍到无人机和车辆）
cam_b = SmoothFollowCamera(world, camera_bp, alpha=0.12,  # 高速场景稍提高响应
                            offset_back=4.0, offset_up=2.0)
# 注意：侧方伴飞时 flight_yaw_deg 不变，但 cam_b 的朝向会自动对准无人机

# 无人机侧方位置
def drone_side_escort(vehicle_transform, side=3.0, height=4.0):
    loc = vehicle_transform.location
    yaw = math.radians(vehicle_transform.rotation.yaw)
    # 右侧伴飞：沿车辆右侧方向偏移
    x = loc.x - side * math.sin(yaw)  # 右侧
    y = loc.y + side * math.cos(yaw)
    z = loc.z + height
    return airsim.Vector3r(x_val=x, y_val=y, z_val=-z)
```

---

### D08：多车协同·无人机俯视调度

**导演意图**：展示多智能体场景。无人机飞在路口上方 20m，俯视 4 辆车通过路口，CAM-A 中可以看到完整的路口全貌和车辆轨迹。同时 CAM-D（其中一辆车的后视）中可以看到无人机悬停在路口上方。

**运动设计**：无人机在路口上方缓慢平移（2 m/s），保持对路口的俯视。车辆正常通过路口。

| 参数 | 值 |
|------|----|
| 地图 | Town05（大型十字路口） |
| 天气 | ClearNoon |
| 无人机高度 | **20m**（俯视路口全貌） |
| 无人机速度 | 2 m/s（缓慢平移，保持俯视稳定） |
| 车辆数量 | 4 辆，从不同方向进入路口 |
| 行人 | 10 名，在路口人行横道等待或通过 |
| 时长 | 45 秒 |

```python
# 无人机在路口上方缓慢平移（不悬停，保持单向运动）
intersection_center = carla.Location(x=0, y=0, z=0)  # 路口中心坐标
# 无人机从路口一侧缓慢飞过，全程保持对路口的俯视
drone_path = plan_straight_path(
    start_x=intersection_center.x - 30,
    start_y=intersection_center.y,
    height=20.0,
    yaw_deg=0,
    total_distance=60.0,
    step=2.0
)
airsim_client.moveOnPathAsync(drone_path, velocity=2.0,
    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0))

# CAM-D 挂载在其中一辆车上，高度调整以拍到 20m 高的无人机
cam_d = world.spawn_actor(
    camera_bp,
    carla.Transform(
        carla.Location(x=-8.0, y=0.0, z=8.0),  # 更高，以拍到 20m 高的无人机
        carla.Rotation(pitch=-20, yaw=0, roll=0)
    ),
    attach_to=vehicle
)
```

---

### D09：无人机搜救·引导车辆

**导演意图**：展示空地协同的应用价值。无人机在街道上空飞行，发现一名倒地行人，随后引导地面车辆前往救援。叙事分三幕：无人机发现目标 → 无人机悬停标记位置 → 车辆抵达。

**运动设计**：第一幕无人机直线飞行搜索，第二幕无人机在目标上方缓慢盘旋（小圆圈，保持运动），第三幕车辆直线驶向目标。

| 参数 | 值 |
|------|----|
| 地图 | Town03 |
| 天气 | CloudyNoon（稍阴，增加紧张感） |
| 无人机高度 | 搜索阶段 12m，发现目标后降至 6m |
| 目标行人 | 1 名，静止倒地（speed=0） |
| 背景行人 | 8 名，正常行走 |
| 时长 | 50 秒 |

```python
# 第二幕：无人机在目标上方缓慢盘旋（半径 5m，保持运动感）
import math

def generate_circle_path(center_x, center_y, height,
                          radius=5.0, speed=2.0, fps=30):
    """生成圆形飞行路径（保持运动，避免悬停抖动）"""
    circumference = 2 * math.pi * radius
    total_time    = circumference / speed
    total_frames  = int(total_time * fps)
    path = []
    for i in range(total_frames):
        angle = 2 * math.pi * i / total_frames
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        path.append(airsim.Vector3r(x_val=x, y_val=y, z_val=-height))
    return path

# 发现目标后，无人机切换到盘旋模式
target_loc = pedestrian.get_location()
circle_path = generate_circle_path(
    center_x=target_loc.x, center_y=target_loc.y,
    height=6.0, radius=5.0, speed=2.0
)
airsim_client.moveOnPathAsync(circle_path, velocity=2.0,
    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=30))  # 缓慢旋转偏航
```

---

### D10：空地 LiDAR·点云融合

**导演意图**：展示空地双 LiDAR 的互补覆盖。左侧显示地面车辆 LiDAR（水平扫描，建筑物侧面清晰，但屋顶盲区大），右侧显示无人机 LiDAR（俯视扫描，屋顶和地面清晰，但建筑物侧面盲区大），最后两者融合成完整点云。

**运动设计**：直线匀速，与 D01 相同。

| 参数 | 值 |
|------|----|
| 地图 | Town05（建筑物密集，LiDAR 效果明显） |
| 天气 | ClearNoon |
| 无人机高度 | 15m（LiDAR 俯视覆盖更广） |
| 传感器布局 | 左上=车辆 LiDAR BEV，右上=无人机 LiDAR BEV，下方=融合点云 |
| 时长 | 40 秒 |

```python
# 车辆 LiDAR（水平扫描）
vehicle_lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
vehicle_lidar_bp.set_attribute('channels', '64')
vehicle_lidar_bp.set_attribute('range', '50')
vehicle_lidar_bp.set_attribute('upper_fov', '10')
vehicle_lidar_bp.set_attribute('lower_fov', '-30')
vehicle_lidar = world.spawn_actor(
    vehicle_lidar_bp,
    carla.Transform(carla.Location(x=0, y=0, z=2.0)),
    attach_to=vehicle
)

# 无人机 LiDAR（俯视扫描）
drone_lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
drone_lidar_bp.set_attribute('channels', '64')
drone_lidar_bp.set_attribute('range', '50')
drone_lidar_bp.set_attribute('upper_fov', '0')
drone_lidar_bp.set_attribute('lower_fov', '-90')  # 全部向下扫描
# 无人机 LiDAR 挂载在 CARLA 中对应的无人机 actor 上
```

---

### D11：多无人机编队·异构协同

**导演意图**：视觉冲击力最强的 Demo。3 架无人机同时飞行：无人机 1 追踪一辆轿车，无人机 2 追踪一名行人，无人机 3 在更高处俯视全局。CAM-A 显示无人机 3 的俯视全景，其他三格显示无人机 1、2 和地面视角。

**运动设计**：三架无人机均匀速单向飞行，各自追踪不同目标，目标也均为直线运动。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 无人机 1 | 追踪轿车，高度 7m，速度 8 m/s |
| 无人机 2 | 追踪行人，高度 6m，速度 1.8 m/s |
| 无人机 3 | 全局俯视，高度 25m，速度 3 m/s（缓慢平移） |
| 时长 | 45 秒 |

```python
# AirSim settings.json 中配置 3 架无人机
# 需要在 settings.json 的 Vehicles 字段中添加 3 个 SimpleFlight 实例
{
  "Vehicles": {
    "Drone1": {"VehicleType": "SimpleFlight", "X": 0, "Y": 0, "Z": -2},
    "Drone2": {"VehicleType": "SimpleFlight", "X": 5, "Y": 0, "Z": -2},
    "Drone3": {"VehicleType": "SimpleFlight", "X": 10, "Y": 0, "Z": -2}
  }
}

# 分别控制三架无人机
client1 = airsim.MultirotorClient()
client1.enableApiControl(True, "Drone1")
client2 = airsim.MultirotorClient()
client2.enableApiControl(True, "Drone2")
client3 = airsim.MultirotorClient()
client3.enableApiControl(True, "Drone3")

# 四格布局调整（多无人机场景）
# 左上=无人机 3 俯视全景（看到三架无人机和所有目标）
# 右上=无人机 1 追踪轿车的 CAM-B
# 左下=无人机 2 追踪行人的 CAM-A
# 右下=地面车辆 CAM-D（看到无人机 1 在上方）
```

---

### D12：VLN 导航·语言指令驾驶

**导演意图**：接轨 2026 年最热的研究方向。画面中显示语言指令字幕（如 "Turn left at the next intersection, avoid the pedestrian crossing"），车辆和无人机按照指令执行。这个 Demo 不需要真正跑 VLN 模型，只需要预先设计好路线并配上字幕。

**运动设计**：车辆按预设路线行驶（包含一次左转），无人机在车辆斜后上方跟随。

| 参数 | 值 |
|------|----|
| 地图 | Town01（小镇，街道标识清晰） |
| 天气 | ClearNoon |
| 无人机高度 | 7m |
| 路线 | 直行 → 左转 → 直行（包含路口决策） |
| 时长 | 40 秒 |

```python
# 预设路线（航点序列）
waypoints = [
    carla.Location(x=0,   y=0,   z=0),  # 起点
    carla.Location(x=50,  y=0,   z=0),  # 直行 50m
    carla.Location(x=50,  y=-30, z=0),  # 左转
    carla.Location(x=100, y=-30, z=0),  # 继续直行
]

# 后期在视频上叠加字幕（FFmpeg）
# 字幕文件 subtitles.srt：
# 1
# 00:00:05,000 --> 00:00:10,000
# "Drive straight for 50 meters"
#
# 2
# 00:00:15,000 --> 00:00:20,000
# "Turn left at the intersection"
#
# ffmpeg -i output_4grid.mp4 -vf subtitles=subtitles.srt output_vln.mp4
```

---

### D13：全天候光照·传感器对比

**导演意图**：展示 CARLA-Air 的全天候仿真能力。画面快速切换 5 种典型天气（晴天、阴天、雨天、雾天、夜晚），每种天气持续约 8 秒，四格画面同步变化。

**运动设计**：直线匀速，天气在运动中切换。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气序列 | ClearNoon → CloudyNoon → HardRainNoon → FoggyNoon → ClearNight |
| 每段时长 | 8 秒 |
| 无人机高度 | 8m |
| 时长 | 50 秒 |

（天气切换代码与 D02 相同，使用 `lerp_weather` 函数平滑过渡。）

---

### D14：城市峡谷·低空穿越

**导演意图**：视觉冲击力最强的单机 Demo。无人机在 Town01 的狭窄街道中低速穿越，两侧是紧贴的欧式建筑，高度仅 5m，CAM-A 中建筑物从两侧快速掠过，极具临场感。车辆在同一条街道上行驶，无人机从车辆上方飞过。

**运动设计**：无人机沿街道中心线直线飞行，速度 5 m/s（低速，保持画面清晰）。

| 参数 | 值 |
|------|----|
| 地图 | Town01（狭窄欧式街道） |
| 天气 | ClearNoon（阴影在建筑间形成强烈对比） |
| 无人机高度 | **5m**（极低，建筑物在两侧近距离掠过） |
| 无人机速度 | 5 m/s |
| 车辆速度 | 5 m/s（与无人机同速） |
| 时长 | 35 秒 |

```python
# Town01 狭窄街道坐标（需根据实际地图调整）
# 选择一条两侧有建筑物、宽度约 8m 的街道
# 无人机飞行高度 5m，建筑物高度约 10–15m，无人机在建筑物高度的 1/3 处

# CAM-A FOV 调整为 120°（更宽视角，建筑物从两侧掠过效果更强）
{
    "CameraName": "cam_a_fpv",
    "CaptureSettings": [{"ImageType": 0, "Width": 1920, "Height": 1080, 
                          "FOV_Degrees": 120}],  # 宽角，增强临场感
    "Pitch": -15,  # 稍微向下，但主要看前方
}
```

---

### D15：压轴·最大规模场景

**导演意图**：整套 Demo 的收尾。最大规模场景：3 架无人机 + 10 辆车 + 30 名行人同时运行，展示 CARLA-Air 的系统承载能力。CAM-A（无人机 3 俯视）中可以看到整个城市街道的全貌，车辆和行人密集运动。

**运动设计**：所有智能体均为直线运动，无人机 3 在高处缓慢平移，无人机 1 和 2 追踪各自目标。

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 无人机数量 | 3 架（配置同 D11） |
| 车辆数量 | 10 辆（autopilot） |
| 行人数量 | 30 名（随机行走） |
| 无人机 3 高度 | 30m（俯视全局） |
| 时长 | 55 秒 |

```python
# 批量生成车辆和行人
import random

# 生成 10 辆车
vehicle_bps = world.get_blueprint_library().filter('vehicle.*')
spawn_points = world.get_map().get_spawn_points()
vehicles = []
for i in range(10):
    bp = random.choice(vehicle_bps)
    sp = random.choice(spawn_points)
    v  = world.try_spawn_actor(bp, sp)
    if v:
        v.set_autopilot(True)
        vehicles.append(v)

# 生成 30 名行人
walker_bp  = world.get_blueprint_library().find('walker.pedestrian.0001')
walkers    = []
for i in range(30):
    spawn_loc = world.get_random_location_from_navigation()
    if spawn_loc:
        w = world.try_spawn_actor(walker_bp,
                                   carla.Transform(spawn_loc))
        if w:
            # 随机行走控制器
            walker_controller_bp = world.get_blueprint_library().find(
                'controller.ai.walker')
            wc = world.spawn_actor(walker_controller_bp,
                                    carla.Transform(), attach_to=w)
            wc.start()
            wc.go_to_location(world.get_random_location_from_navigation())
            wc.set_max_speed(1.5)
            walkers.append((w, wc))
```

---

## 四、录制优先级与执行顺序

### 第一批（P0，最先录制，用于开源发布）

| 顺序 | Demo | 理由 |
|------|------|------|
| 1 | **D01** | 平台名片，最简单，先验证四格构图 |
| 2 | **D02** | 天气同步，最强视觉论据，GitHub README 首图 |
| 3 | **D06** | 夜间红外，对比效果震撼，传播性强 |
| 4 | **D04** | 行人追踪，覆盖 Human-UAV 受众 |
| 5 | **D05** | 叙事最强，适合作为长视频的高潮段落 |
| 6 | **D09** | 搜救场景，应用价值最直观 |
| 7 | **D15** | 压轴大场景，整套 Demo 的收尾 |

### 第二批（P1，用于丰富 Demo 页面）

D03 → D07 → D08 → D10 → D11 → D12 → D13 → D14

---

## 五、常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| CAM-B 画面抖动 | attach_to=drone | 改用 SmoothFollowCamera，不 attach |
| 无人机加减速抖动 | 使用 moveToPositionAsync 实时跟随 | 改用 moveOnPathAsync 预规划路径 |
| 四格画面不同步 | CARLA 未开启同步模式 | `synchronous_mode=True`，`fixed_delta_seconds=1/30` |
| 行人在画面中太小 | 无人机飞行高度过高 | 追踪行人时高度降至 5–7m |
| 无人机在 CAM-D 中不可见 | 无人机飞在车辆正上方 | 调整为落后 4m + 侧偏 1.5m 的斜后上方位置 |
| 天气切换画面跳变 | 使用硬切 | 改用 lerp_weather 函数 2–3 秒渐变过渡 |
| AirSim 和 CARLA 坐标不对齐 | Z 轴方向相反 | 使用 airsim_to_carla_location 转换函数 |
