# CARLA-Air Demo 拍摄工程化指南 v4.0

> **版本说明**：本版本根据导演审片意见全面重构相机方案，核心变化包括：统一四格构图标准、重新设计无人机相机挂载位置、优化人物追踪镜头、扩充传感器协同感知 Demo、增加地图多样性。

---

## 一、通用录制规范

### 1.1 基础参数

| 参数 | 值 |
|------|----|
| 分辨率 | 1920×1080（每路相机） |
| 帧率 | 30 FPS（CARLA 同步模式） |
| 编码 | H.264，CRF=18 |
| 色彩空间 | sRGB |
| 同步模式 | `synchronous_mode = True`，`fixed_delta_seconds = 1/30` |

### 1.2 标准四格构图（所有 Demo 默认布局）

```
┌─────────────────┬─────────────────┐
│                 │                 │
│   CAM-A         │   CAM-B         │
│   无人机第一视角 │   无人机第三视角 │
│   （机腹朝前下） │   （后上方跟随） │
│                 │                 │
├─────────────────┼─────────────────┤
│                 │                 │
│   CAM-C         │   CAM-D         │
│   车辆第一视角  │   车辆第三视角   │
│   （挡风玻璃后）│   （后上方跟随） │
│                 │                 │
└─────────────────┴─────────────────┘
```

**四格合成命令（FFmpeg）：**

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

### 1.3 无人机相机挂载标准（重要）

**CAM-A：无人机第一视角（机腹前视）**

相机挂载在无人机机腹，略向前下方倾斜，确保：
- 画面中看不到螺旋桨（相机在桨盘下方）
- 画面中能看到地面的车辆和行人
- 飞行时有明显的运动感

```python
# AirSim settings.json 中配置
{
    "CameraName": "front_bottom",
    "CaptureSettings": [{"ImageType": 0, "Width": 1920, "Height": 1080, "FOV_Degrees": 90}],
    "X": 0.1, "Y": 0.0, "Z": 0.05,   # 机腹前方
    "Pitch": -30, "Roll": 0, "Yaw": 0  # 向前下方倾斜 30°
}
```

**CAM-B：无人机第三视角（后上方跟随）**

相机挂载在无人机后上方，能同时看到：
- 无人机机身和旋转的桨叶（画面上方）
- 无人机正在追踪的车辆或行人（画面中下方）

```python
# AirSim settings.json 中配置
{
    "CameraName": "rear_top",
    "CaptureSettings": [{"ImageType": 0, "Width": 1920, "Height": 1080, "FOV_Degrees": 100}],
    "X": -1.5, "Y": 0.0, "Z": -0.8,   # 后方 1.5m，上方 0.8m
    "Pitch": -20, "Roll": 0, "Yaw": 180  # 朝向无人机前方（即向前下方看）
}
# 注意：Yaw=180 使相机朝向无人机飞行方向，Pitch=-20 使画面中无人机机身在上，地面目标在下
```

**CAM-C：车辆第一视角（挡风玻璃后）**

```python
cam_c_transform = carla.Transform(
    carla.Location(x=0.5, y=0.0, z=1.4),   # 驾驶员眼睛位置
    carla.Rotation(pitch=-5, yaw=0, roll=0)  # 略向下看，视野自然
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=vehicle)
```

**CAM-D：车辆第三视角（后上方跟随）**

相机挂载在车辆后上方，能同时看到：
- 车辆（画面中下方）
- 在车辆上方飞行的无人机（画面上方）

```python
cam_d_transform = carla.Transform(
    carla.Location(x=-8.0, y=0.0, z=5.0),   # 后方 8m，上方 5m
    carla.Rotation(pitch=-15, yaw=0, roll=0)  # 略向下，车辆在画面中，无人机在上方
)
camera_d = world.spawn_actor(camera_bp, cam_d_transform, attach_to=vehicle)
```

> **关键设计原则**：CAM-D 的高度（z=5.0）和后退距离（x=-8.0）需要根据无人机飞行高度动态调整。若无人机飞行高度为 15m，则 CAM-D 的 z 可调整至 6–8m，确保无人机始终在画面上方 1/3 区域内可见。

### 1.4 地图分配方案

| 地图 | 特点 | 分配 Demo |
|------|------|-----------|
| **Town10HD** | 欧式大城市，高楼密集，最高画质 | D01 D02 D03 D05 D08 D09 D15 D20 D28 D29 D30 |
| **Town03** | 中等城市，有立交桥和环形路，场景多样 | D04 D06 D11 D23 D27 |
| **Town04** | 高速公路 + 郊区，开阔视野 | D07 D25 |
| **Town05** | 方格路网，大型十字路口，适合多智能体 | D12 D13 D16 D24 |
| **Town06** | 密歇根左转，高速公路匝道，美式郊区 | D14 D26 |
| **Town01** | 小镇，狭窄街道，欧式建筑，适合低空穿越 | D17 D18 D19 D22 |
| **Town02** | 小型居民区，行人密集 | D10 D21 |

### 1.5 行人拍摄规范（重要）

- 无人机飞行高度：追踪行人时控制在 **8–12m**（而非 20–30m），确保行人在画面中清晰可见
- 无人机 CAM-A（机腹前视）的 FOV 设置为 **90°**，行人在画面中占比不低于 1/4
- 行人移动速度建议 **1.5–2.0 m/s**（正常步行），避免过快导致画面模糊
- 建议在行人密集区域（广场、人行道）拍摄，而非空旷道路

---

## 二、Demo 总览表

| # | Demo 名称 | 核心 Feature | 地图 | 时长 | 优先级 | 传感器 |
|---|-----------|-------------|------|------|--------|--------|
| D01 | 空地协同追踪·城市主干道 | 无人机跟随车辆，空地同框 | Town10HD | 45s | P0 | RGB×4 |
| D02 | 天气一致性·实时切换 | 单进程同步天气，最强视觉论据 | Town10HD | 50s | P0 | RGB×4 |
| D03 | 传感器融合·四模态同步 | RGB+深度+语义+LiDAR 同帧 | Town10HD | 45s | P0 | 多模态 |
| D04 | 无人机追踪行人·广场场景 | 无人机低空追踪行人 | Town03 | 40s | P0 | RGB×4 |
| D05 | 空地双视角·同一事件 | 空中预知性 vs 地面局限性 | Town10HD | 45s | P0 | RGB×4 |
| D06 | 多无人机编队·异构协同 | 3 架无人机各自追踪不同目标 | Town03 | 50s | P1 | RGB×6 |
| D07 | 高速公路·无人机侧方伴飞 | 高速运动下的空地协同 | Town04 | 45s | P1 | RGB×4 |
| D08 | 夜间红外·行人热成像 | 夜间传感器能力展示 | Town10HD | 45s | P0 | RGB+IR |
| D09 | 全天候光照循环 | 24 小时光照仿真 | Town10HD | 60s | P1 | RGB×4 |
| D10 | 深度相机·三维重建 | 空地双深度相机点云融合 | Town02 | 45s | P1 | Depth+RGB |
| D11 | 语义分割·多类别识别 | 空地双语义分割对比 | Town03 | 40s | P1 | Seg×4 |
| D12 | 多车协同·交叉路口 | 多辆车+无人机协同通过路口 | Town05 | 45s | P1 | RGB×4 |
| D13 | 无人机搜索·螺旋扩展 | 无人机自主搜索行人 | Town05 | 50s | P1 | RGB×4 |
| D14 | 紧急制动·空地感知互补 | 无人机提前感知障碍，车辆制动 | Town06 | 40s | P0 | RGB×4 |
| D15 | 大规模行人·人群仿真 | 50 名行人的密集场景 | Town10HD | 45s | P1 | RGB×4 |
| D16 | 空地 LiDAR·点云融合 | 地面+空中 LiDAR 互补 | Town05 | 45s | P1 | LiDAR×2+RGB |
| D17 | 城市峡谷·低空穿越 | 无人机在高楼间低空飞行 | Town01 | 40s | P1 | RGB×4 |
| D18 | VLN 导航·语言指令驾驶 | 语言指令引导空地导航 | Town01 | 45s | P1 | RGB×4 |
| D19 | 雷达感知·障碍物检测 | 无人机雷达+车辆雷达协同 | Town01 | 40s | P1 | Radar+RGB |
| D20 | 14 种天气·传感器对比 | 天气变化下传感器性能对比 | Town10HD | 60s | P1 | RGB+Seg |
| D21 | IMU/GPS 数据流可视化 | 传感器数据实时展示 | Town02 | 30s | P2 | IMU+GPS |
| D22 | 夜间 RGB+红外·双模对比 | 夜间双模传感器对比 | Town01 | 45s | P1 | RGB+IR |
| D23 | 搜救协同·无人机引导车辆 | 无人机发现目标，引导车辆救援 | Town03 | 60s | P0 | RGB×4 |
| D24 | 行人轨迹·数据采集 | 无人机俯视采集行人轨迹 | Town05 | 45s | P1 | RGB+Seg |
| D25 | 光流传感器·无人机导航 | 光流传感器辅助无人机定位 | Town04 | 40s | P2 | OptFlow+RGB |
| D26 | 多传感器融合·自动驾驶 | 车辆多传感器融合感知 | Town06 | 45s | P1 | Camera+LiDAR+Radar |
| D27 | 无人机编队·队形变换 | 3 架无人机队形变换 | Town10HD | 45s | P1 | RGB×6 |
| D28 | 跨地图·场景多样性 | 快速切换 5 种地图场景 | 多地图 | 60s | P1 | RGB×4 |
| D29 | 全传感器·综合展示 | 所有传感器类型一次性展示 | Town10HD | 60s | P0 | 全类型 |
| D30 | 压轴·最大规模场景 | 20 车+50 人+3 机同时运行 | Town10HD | 60s | P0 | RGB×4 |

---

## 三、Demo 详细拍摄指南

---

## D01：空地协同追踪·城市主干道

**核心 Feature**：无人机实时跟随地面车辆，空地同框，展示 CARLA-Air 最基础也最核心的能力。  
**导演意图**：这是整套 Demo 的"名片"。四格画面同时展示无人机视角、无人机第三视角、车辆视角、车辆第三视角，让观众在 45 秒内完整感受空地一体的视觉冲击。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 以 40 km/h 在主干道行驶，自动驾驶 |
| 无人机 | 在主车正上方 12m 跟随飞行 |
| 行人 | 15 名，在人行道自然行走 |
| 配角车 | 5 辆，在周围车道行驶 |

### 相机配置（标准四格）

**CAM-A（左上）：无人机第一视角**
- 挂载位置：机腹，pitch=-30°，FOV=90°
- 画面内容：前方道路、主车在画面下方、行人在人行道

**CAM-B（右上）：无人机第三视角**
- 挂载位置：无人机后上方 x=-1.5m, z=-0.8m，pitch=-20°，Yaw=180°
- 画面内容：无人机机身在画面上方，主车在画面中下方，城市街道为背景

**CAM-C（左下）：车辆第一视角**
- 挂载位置：驾驶位，x=0.5m, z=1.4m，pitch=-5°
- 画面内容：前方道路，行人，交通灯，其他车辆

**CAM-D（右下）：车辆第三视角**
- 挂载位置：车辆后方 x=-8m, z=5m，pitch=-15°
- 画面内容：主车在画面中下方，无人机在画面上方 1/3 处，城市天际线为背景

```python
# 无人机跟随主车的位置控制
def drone_follow_vehicle(vehicle_transform, height=12.0):
    loc = vehicle_transform.location
    drone_target = airsim.Vector3r(
        x=loc.x,
        y=loc.y,
        z=-(loc.z + height)  # AirSim Z 轴向下取负
    )
    airsim_client.moveToPositionAsync(
        drone_target.x_val, drone_target.y_val, drone_target.z_val,
        velocity=15,  # m/s，足以跟上 40 km/h 的车辆
        lookahead=5,
        adaptive_lookahead=1
    )

# CAM-D 高度动态调整（确保无人机始终在画面上方）
# 无人机飞行高度 12m，CAM-D 挂载高度 5m，差值 7m，无人机在画面上方约 1/3 处
```

### 运镜设计

**0–10s**：四格同时开始。CAM-B 展示无人机从远处追上主车的过程，主车从画面远处逐渐变大。  
**10–30s**：稳定跟随阶段。四格画面同步展示空地协同状态，CAM-D 中无人机和主车同框。  
**30–45s**：运镜变化——CAM-D 的车辆第三视角相机缓慢向上移动（通过插值调整 z 值），逐渐从"车辆后方"变成"高空俯视"，最终在画面中同时看到无人机和主车以及整条街道。

---

## D02：天气一致性·实时切换

**核心 Feature**：CARLA 和 AirSim 在同一进程内同步切换天气，是证明单进程架构的最强视觉论据。  
**导演意图**：天气切换的那一帧是整套 Demo 中最有力的画面。四格画面同时从晴天变为暴雨，没有任何延迟，这一帧画面本身就完成了对"完全同步"的证明。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气序列 | ClearNoon → HardRainNoon → WetCloudySunset → ClearNight（各停留约 12s） |
| 时长 | 50 秒 |
| 主车 | 以 30 km/h 在主干道缓慢行驶 |
| 无人机 | 在主车上方 12m 跟随 |
| 行人 | 10 名，在人行道行走 |

### 天气切换实现

```python
weather_sequence = [
    ("ClearNoon",       carla.WeatherParameters.ClearNoon,       12),
    ("HardRainNoon",    carla.WeatherParameters.HardRainNoon,     12),
    ("WetCloudySunset", carla.WeatherParameters.WetCloudySunset,  12),
    ("ClearNight",      carla.WeatherParameters.ClearNight,       14),
]

# 天气切换：CARLA 和 AirSim 在同一帧内同步切换
for weather_name, weather_params, duration_sec in weather_sequence:
    # CARLA 切换天气
    carla_world.set_weather(weather_params)
    # AirSim 切换天气（通过 simSetWeatherParameter）
    airsim_client.simSetWeatherParameter(airsim.WeatherParameter.Rain,
        0.8 if "Rain" in weather_name else 0.0)
    airsim_client.simSetWeatherParameter(airsim.WeatherParameter.Fog,
        0.3 if "Night" in weather_name else 0.0)
    
    for frame in range(duration_sec * 30):
        carla_world.tick()
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，机腹前视，能看到雨滴打在镜头上的效果  
**CAM-B（右上）**：无人机第三视角，后上方，能同时看到无人机和主车在雨中行驶  
**CAM-C（左下）**：车辆第一视角，雨天时雨刮器工作，路面反光  
**CAM-D（右下）**：车辆第三视角，后上方，能同时看到无人机和主车，天气变化时两者同时被淋雨

> **关键时刻**：天气切换的瞬间，四格画面同时变化。建议在后期剪辑时，在切换帧前后各保留 0.5 秒的静止画面，让观众清晰感受到"同步"。

---

## D03：传感器融合·四模态同步

**核心 Feature**：RGB、深度图、语义分割、LiDAR 四种传感器在同一帧内同步输出，展示平台的传感器生态。  
**导演意图**：这个 Demo 面向数据集研究者。四格画面分别展示四种模态，同一场景在不同传感器下的呈现对比，是这个 Demo 的核心价值。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 以 30 km/h 缓慢行驶，路过行人密集区域 |
| 无人机 | 在主车上方 10m 跟随 |
| 行人 | 20 名，在人行道和斑马线区域 |

### 四模态传感器配置

```python
# 挂载在无人机上的四种传感器（均朝向前下方）
sensor_configs = [
    {"name": "rgb",    "type": airsim.ImageType.Scene,           "color": None},
    {"name": "depth",  "type": airsim.ImageType.DepthPerspective, "color": None},
    {"name": "seg",    "type": airsim.ImageType.Segmentation,     "color": None},
]

# CARLA 车辆上的 LiDAR
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('range', '50')
lidar_bp.set_attribute('points_per_second', '1000000')
lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2.5))
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

# 同步采集：在同一个 tick 内采集所有传感器数据
def sync_collect(tick):
    carla_world.tick()
    images = airsim_client.simGetImages([
        airsim.ImageRequest("front_bottom", airsim.ImageType.Scene),
        airsim.ImageRequest("front_bottom", airsim.ImageType.DepthPerspective),
        airsim.ImageRequest("front_bottom", airsim.ImageType.Segmentation),
    ])
    # LiDAR 数据通过 callback 异步收集，但帧号与 CARLA tick 对齐
    return images
```

### 相机配置（四格，替代标准布局）

**CAM-A（左上）**：无人机 RGB，机腹前视，行人清晰可见  
**CAM-B（右上）**：无人机深度图，行人轮廓清晰，远近关系明显  
**CAM-C（左下）**：无人机语义分割，行人（红色）、车辆（蓝色）、道路（灰色）清晰区分  
**CAM-D（右下）**：车辆 LiDAR 点云可视化（俯视角），行人和车辆呈现为点云簇

> **注意**：本 Demo 的四格均为传感器数据，不包含车辆第一/第三视角。建议在视频开头用 2 秒的全局 Orbit 镜头（单屏）展示场景全貌，再切入四格传感器画面。

---

## D04：无人机追踪行人·广场场景

**核心 Feature**：无人机低空追踪特定行人，展示平台对行人控制和无人机追踪的支持。  
**导演意图**：这是整套 Demo 中最能体现"人"这一维度的视频。无人机飞行高度控制在 8–10m，确保行人在画面中清晰可见。行人在广场上行走，路线有转弯和加速，增加追踪的戏剧性。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03（有较大广场区域） |
| 天气 | ClearAfternoon |
| 时长 | 40 秒 |
| 目标行人 | 1 名，在广场上行走，路线有转弯 |
| 背景行人 | 20 名，在广场自然行走 |
| 无人机 | 在目标行人上方 8–10m 追踪 |
| 车辆 | 3 辆，在广场周围道路行驶 |

### 行人路径设计

```python
# 目标行人的路径：直行 → 左转 → 加速 → 右转 → 停止
target_waypoints = [
    carla.Location(x=100, y=50, z=0),   # 起点
    carla.Location(x=150, y=50, z=0),   # 直行 50m
    carla.Location(x=150, y=100, z=0),  # 左转
    carla.Location(x=200, y=100, z=0),  # 直行（加速段）
    carla.Location(x=200, y=130, z=0),  # 右转
    carla.Location(x=200, y=130, z=0),  # 停止
]

# 行人速度控制
walker_speeds = [1.5, 1.5, 2.5, 2.5, 1.5, 0.0]  # m/s，加速段速度提升

# 无人机追踪：保持在行人上方 9m，水平偏移不超过 2m
def track_pedestrian(walker_location, height=9.0):
    airsim_client.moveToPositionAsync(
        x=walker_location.x,
        y=walker_location.y,
        z=-(walker_location.z + height),
        velocity=5,  # 低速追踪，保持稳定
    )
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，机腹前视，pitch=-35°（比标准更陡，确保 8m 高度时行人清晰）  
- 目标行人应在画面中心，背景行人在周围，广场地面纹理清晰可见

**CAM-B（右上）**：无人机第三视角，后上方  
- 无人机机身在画面上方，目标行人在画面中下方，广场全景为背景

**CAM-C（左下）**：目标行人视角（固定跟随行人的第三人称相机，非车辆）  
```python
# 跟随行人的第三人称相机（挂载在行人后上方）
cam_c_transform = carla.Transform(
    carla.Location(x=-3.0, y=0.0, z=2.5),
    carla.Rotation(pitch=-20, yaw=0, roll=0)
)
camera_c = world.spawn_actor(camera_bp, cam_c_transform, attach_to=target_walker)
```
- 画面内容：行人背影在画面中，无人机在行人上方可见，广场背景

**CAM-D（右下）**：全局广场固定机位  
- 放置在广场边缘高处（z=8m），俯角约 30°，能同时看到无人机、目标行人、背景行人

> **关键设计**：CAM-C 挂载在行人身上，这样无人机始终在画面上方，行人在画面中，完美体现"无人机追踪行人"的空地关系。

---

## D05：空地双视角·同一事件

**核心 Feature**：同一物理事件（行人突然穿越马路）从空中和地面两个视角同时呈现，展示视角互补性。  
**导演意图**：空中视角有"预知性"（能看到行人从小巷跑出），地面视角有"局限性"（行人突然出现）。这个对比是整套 Demo 中叙事性最强的时刻。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 核心事件 | 行人从小巷跑出，穿越主干道 |
| 主车 | 以 50 km/h 行驶，紧急制动 |
| 无人机 | 在主车上方 12m 跟随 |
| 背景 | 5 名行人，3 辆配角车 |

### 事件触发实现

```python
# 在第 20 秒（600帧）触发行人从小巷跑出
if tick == 600:
    walker_bp = blueprint_library.find('walker.pedestrian.0001')
    alley_exit = carla.Location(x=200, y=80, z=0)
    walker = world.spawn_actor(walker_bp, carla.Transform(alley_exit))
    
    control = carla.WalkerControl()
    control.speed = 3.5   # 跑步速度
    control.direction = carla.Vector3D(0, 1, 0)
    walker.apply_control(control)

# 主车紧急制动（在行人出现后 1 秒触发）
if tick == 630:
    vehicle_control = carla.VehicleControl()
    vehicle_control.brake = 1.0
    vehicle_control.throttle = 0.0
    vehicle.apply_control(vehicle_control)
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，能提前看到小巷中的行人（空中视角的预知性）  
**CAM-B（右上）**：无人机第三视角，能同时看到无人机、主车、行人三者的空间关系  
**CAM-C（左下）**：车辆第一视角，行人突然出现（地面视角的局限性），制动时画面前倾  
**CAM-D（右下）**：车辆第三视角，能看到主车制动、无人机在上方、行人在前方

---

## D06：多无人机编队·异构协同

**核心 Feature**：3 架无人机各自追踪不同目标（1 架追车，1 架追行人，1 架全局监控），展示多智能体异构协同。  
**导演意图**：这是视觉冲击力最强的 Demo 之一。3 架无人机同时在画面中，各自执行不同任务，体现了平台的多智能体扩展能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03 |
| 天气 | ClearNoon |
| 时长 | 50 秒 |
| 无人机 | 3 架：Drone1（追车）、Drone2（追行人）、Drone3（全局监控，悬停 30m） |
| 主车 | 1 辆，Drone1 追踪 |
| 目标行人 | 1 名，Drone2 追踪 |
| 背景 | 5 辆车，15 名行人 |

### 多无人机 settings.json 配置

```json
{
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": -5
    },
    "Drone2": {
      "VehicleType": "SimpleFlight",
      "X": 10, "Y": 0, "Z": -5
    },
    "Drone3": {
      "VehicleType": "SimpleFlight",
      "X": 20, "Y": 0, "Z": -5
    }
  }
}
```

```python
# 三架无人机各自的控制客户端
client1 = airsim.MultirotorClient()
client1.enableApiControl(True, "Drone1")

client2 = airsim.MultirotorClient()
client2.enableApiControl(True, "Drone2")

client3 = airsim.MultirotorClient()
client3.enableApiControl(True, "Drone3")

# Drone3 悬停在场景中心 30m 高度
client3.moveToPositionAsync(center_x, center_y, -30, velocity=5, vehicle_name="Drone3")
```

### 相机配置（六格，特殊布局）

```
┌──────────┬──────────┬──────────┐
│ Drone1   │ Drone2   │ Drone3   │
│ 第一视角 │ 第一视角 │ 全局俯视 │
├──────────┴──────────┴──────────┤
│                                 │
│     全局 Orbit（宽幅，占下半）  │
│     3架无人机+车+人同框         │
└─────────────────────────────────┘
```

```bash
# 六格合成（上三格 + 下宽幅）
ffmpeg \
  -i drone1_fpv.mp4 -i drone2_fpv.mp4 -i drone3_top.mp4 -i global_orbit.mp4 \
  -filter_complex "
    [0:v]scale=640:360[a];
    [1:v]scale=640:360[b];
    [2:v]scale=640:360[c];
    [a][b][c]hstack=inputs=3[top];
    [3:v]scale=1920:540[bottom];
    [top][bottom]vstack[out]
  " \
  -map "[out]" output_6grid.mp4
```

---

## D07：高速公路·无人机侧方伴飞

**核心 Feature**：高速公路场景下无人机在车辆侧方伴飞，展示高速运动下的空地协同。  
**导演意图**：高速公路的视觉感受与城市街道完全不同——宽阔道路、高速行驶、无人机在侧方伴飞。CAM-D（车辆第三视角）在这个 Demo 中是最精彩的画面：车辆在下方高速行驶，无人机在侧方伴飞，两者同时入镜。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town04（有高速公路） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 以 120 km/h 在高速公路行驶 |
| 无人机 | 在主车右侧 8m、高度 5m 伴飞 |
| 配角车 | 4 辆，不同车道不同速度 |

### 侧方伴飞实现

```python
def side_escort_position(vehicle_transform, side_offset=8.0, height=5.0):
    right_vector = vehicle_transform.get_right_vector()
    location = vehicle_transform.location
    
    drone_x = location.x + right_vector.x * side_offset
    drone_y = location.y + right_vector.y * side_offset
    drone_z = -(location.z + height)  # AirSim Z 轴向下取负
    
    return drone_x, drone_y, drone_z

# 高速跟随需要更高的速度参数
airsim_client.moveToPositionAsync(
    drone_x, drone_y, drone_z,
    velocity=40,  # 高速公路场景需要 40 m/s 以上
    lookahead=10,
    adaptive_lookahead=1
)
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，机腹前视，朝向主车行驶方向，能看到前方高速公路  
**CAM-B（右上）**：无人机第三视角，后上方，无人机机身在上，主车在右侧下方伴飞  
- 注意：侧方伴飞时，CAM-B 的 Yaw 需要调整，使相机朝向主车方向
```python
# 侧方伴飞时，CAM-B 需要朝向主车（向左旋转 90°）
# 在 settings.json 中：Yaw=90（朝向主车侧面）
```
**CAM-C（左下）**：车辆第一视角，高速行驶，前方道路延伸  
**CAM-D（右下）**：车辆第三视角，后上方，**这是本 Demo 最精彩的画面**：主车在画面中，无人机在右侧上方伴飞，高速公路向前延伸

### 运镜设计

**0–15s**：无人机从后方追上主车，CAM-B 展示追赶过程。  
**15–35s**：稳定侧方伴飞，四格同步展示。CAM-D 中无人机和主车同框，高速行驶的动感最强。  
**35–45s**：无人机缓慢上升到 20m，CAM-D 中无人机逐渐升高，最终从侧上方俯视整条高速公路。

---

## D08：夜间红外·行人热成像

**核心 Feature**：无人机红外相机在夜间对行人的热成像感知，展示 AirSim 红外传感器能力。  
**导演意图**：红外画面的视觉效果极具冲击力——行人在黑暗中呈现为明亮热源。四格画面中，左侧是 RGB（黑暗），右侧是红外（行人发光），对比强烈。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNight |
| 时长 | 45 秒 |
| 无人机 | 在街道上空 10m 缓慢飞行（低空确保行人清晰） |
| 行人 | 15 名，在街道和人行道行走 |
| 车辆 | 3 辆，开启车灯行驶 |

### 红外相机配置

```python
# AirSim settings.json 红外相机配置
{
    "CameraName": "infrared",
    "CaptureSettings": [
        {"ImageType": 7, "Width": 1920, "Height": 1080}  # ImageType 7 = Infrared
    ],
    "X": 0.1, "Y": 0.0, "Z": 0.05,
    "Pitch": -35, "Roll": 0, "Yaw": 0  # 夜间低空，角度更陡
}

# 读取红外图像
responses = airsim_client.simGetImages([
    airsim.ImageRequest("infrared", airsim.ImageType.Infrared, False, False)
])
```

### 相机配置（四格，特殊布局）

**CAM-A（左上）**：无人机 RGB 第一视角，夜间路灯下行人轮廓模糊  
**CAM-B（右上）**：无人机红外第一视角，行人呈现为明亮热源，车辆引擎也发热  
**CAM-C（左下）**：无人机第三视角（后上方），夜间无人机灯光在黑暗中闪烁，地面路灯为背景  
**CAM-D（右下）**：地面固定机位（街道侧方，z=2m，仰角 20°），展示无人机在夜间街道上空飞行的全景

> **关键视觉时刻**：当行人走过路灯下方时，RGB 画面中行人短暂清晰，红外画面中行人始终明亮。这个对比在第 20–25 秒最为明显。


---

## D09：全天候光照循环

**核心 Feature**：24 小时光照循环仿真，展示平台的时间和光照变化能力。  
**导演意图**：从日出到正午到日落到夜晚，城市在不同光照下呈现完全不同的视觉质感。CAM-D（车辆第三视角）中无人机在不同光照下的外观变化是最美的画面。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 时长 | 60 秒（模拟 24 小时） |
| 主车 | 以 20 km/h 缓慢行驶 |
| 无人机 | 在主车上方 12m 跟随 |
| 行人 | 10 名 |

### 光照循环实现

```python
import math

total_frames = 60 * 30  # 1800帧

for tick in range(total_frames):
    progress = tick / total_frames
    sun_angle = -90 + 180 * math.sin(progress * math.pi)
    cloudiness = 20 + 30 * abs(math.sin(progress * math.pi))
    fog_density = max(0, 15 * (1 - abs(sun_angle) / 90))  # 清晨有轻雾
    
    weather = carla.WeatherParameters(
        sun_altitude_angle=sun_angle,
        cloudiness=cloudiness,
        fog_density=fog_density,
        precipitation=0.0
    )
    carla_world.set_weather(weather)
    carla_world.tick()
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，日出时橙红色光线，正午强烈顶光，日落金色，夜晚路灯  
**CAM-B（右上）**：无人机第三视角，无人机机身在不同光照下的外观变化（日出时被橙色渲染，夜晚灯光闪烁）  
**CAM-C（左下）**：车辆第一视角，前方道路在不同光照下的阴影变化  
**CAM-D（右下）**：车辆第三视角，**最美画面**：无人机在上方，主车在下方，背后是随时间变化的天空

---

## D10：深度相机·三维重建

**核心 Feature**：空地双深度相机同步采集，展示深度感知的互补性。  
**导演意图**：地面深度相机有遮挡盲区，空中深度相机俯视无遮挡。两者的深度图对比，直观展示了空地协同感知的价值。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town02（有较多建筑遮挡） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 以 20 km/h 缓慢行驶 |
| 无人机 | 在主车上方 10m 跟随 |
| 行人 | 12 名，部分在建筑物遮挡区域 |

### 深度相机配置

```python
# 车辆前向深度相机（CARLA）
depth_bp = blueprint_library.find('sensor.camera.depth')
depth_bp.set_attribute('image_size_x', '1920')
depth_bp.set_attribute('image_size_y', '1080')
depth_bp.set_attribute('fov', '90')
depth_cam = world.spawn_actor(depth_bp,
    carla.Transform(carla.Location(x=0.5, z=1.4)), attach_to=vehicle)

# 无人机俯视深度相机（AirSim）
# 在 settings.json 中配置 ImageType=1（DepthPerspective）
# Pitch=-35°，确保行人在画面中清晰

# 深度图可视化（将深度值映射到彩虹色）
def colorize_depth(depth_array):
    depth_normalized = (depth_array / depth_array.max() * 255).astype(np.uint8)
    return cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
```

### 相机配置（四格）

**CAM-A（左上）**：无人机深度图（俯视），行人轮廓清晰，无建筑遮挡  
**CAM-B（右上）**：无人机 RGB 第三视角，无人机机身在上，主车在下，行人可见  
**CAM-C（左下）**：车辆前向深度图，建筑物遮挡导致部分行人不可见（体现地面视角局限性）  
**CAM-D（右下）**：车辆第三视角，无人机在上方，主车在下方

---

## D11：语义分割·多类别识别

**核心 Feature**：空地双语义分割同步，展示平台的语义感知能力。  
**导演意图**：语义分割的视觉效果鲜艳，行人（红色）、车辆（蓝色）、道路（灰色）、建筑（棕色）清晰区分。空地双视角的语义分割对比，展示了不同高度下的语义感知差异。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03 |
| 天气 | ClearNoon |
| 时长 | 40 秒 |
| 主车 | 以 30 km/h 行驶 |
| 无人机 | 在主车上方 10m 跟随 |
| 行人 | 20 名，在人行道和斑马线 |

### 语义分割配置

```python
# CARLA 车辆语义分割相机
seg_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
seg_bp.set_attribute('image_size_x', '1920')
seg_bp.set_attribute('image_size_y', '1080')
seg_cam = world.spawn_actor(seg_bp,
    carla.Transform(carla.Location(x=0.5, z=1.4)), attach_to=vehicle)

# AirSim 无人机语义分割
# settings.json 中配置 ImageType=5（Segmentation）
# 颜色映射：行人=红色，车辆=蓝色，道路=灰色，建筑=棕色，植被=绿色

# CARLA 语义标签转彩色
CARLA_SEMANTIC_COLORS = {
    0: (0, 0, 0),        # Unlabeled
    4: (220, 20, 60),    # Pedestrian（红色）
    10: (0, 0, 142),     # Vehicle（蓝色）
    7: (128, 64, 128),   # Road（灰色）
    2: (70, 70, 70),     # Building（棕色）
    8: (107, 142, 35),   # Vegetation（绿色）
}
```

### 相机配置（四格）

**CAM-A（左上）**：无人机语义分割（俯视），行人（红色）在道路（灰色）上清晰可见  
**CAM-B（右上）**：无人机 RGB 第三视角，无人机机身在上，主车在下  
**CAM-C（左下）**：车辆语义分割（前视），行人（红色）在斑马线处清晰  
**CAM-D（右下）**：车辆第三视角，无人机在上，主车在下

---

## D12：多车协同·交叉路口

**核心 Feature**：多辆车辆协同通过交叉路口，无人机从空中监控全局，展示多智能体协同。  
**导演意图**：交叉路口是自动驾驶最复杂的场景。无人机从空中俯视，能看到整个路口的车流；地面车辆视角展示局部感知。空地视角的互补在这个场景中最为直观。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town05（有大型十字路口） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 1 辆，通过路口 |
| 配角车 | 5 辆，从不同方向通过路口 |
| 无人机 | 悬停在路口上方 20m，俯视全局 |
| 行人 | 10 名，在人行道等待或穿越 |

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角（机腹俯视），路口全景，所有车辆和行人清晰可见  
**CAM-B（右上）**：无人机第三视角，无人机悬停，路口全景为背景，无人机机身在画面上方  
**CAM-C（左下）**：主车第一视角，通过路口时的局部感知视角  
**CAM-D（右下）**：主车第三视角，主车在画面中，无人机在上方悬停，路口全景为背景

---

## D13：无人机搜索·螺旋扩展

**核心 Feature**：无人机执行螺旋扩展搜索路径，寻找目标行人，展示无人机自主搜索能力。  
**导演意图**：螺旋搜索路径在 CAM-A（俯视）中呈现为优美的几何图案，同时 CAM-B（第三视角）展示无人机在城市上空的飞行姿态。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town05 |
| 天气 | CloudyNoon |
| 时长 | 50 秒 |
| 目标行人 | 1 名，静止在搜索区域某处 |
| 背景行人 | 15 名，自然行走 |
| 无人机 | 从 25m 高度执行螺旋扩展搜索 |
| 车辆 | 3 辆，在周围道路行驶 |

### 螺旋搜索路径实现

```python
import math

def spiral_search_path(center_x, center_y, height, num_loops=4, step=8.0):
    """生成螺旋扩展搜索路径"""
    waypoints = []
    for i in range(num_loops * 36):  # 每圈 36 个点（每 10° 一个点）
        angle = math.radians(i * 10)
        radius = step * (i / 36.0 + 0.5)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        waypoints.append((x, y, -height))
    return waypoints

# 执行搜索
waypoints = spiral_search_path(center_x=150, center_y=150, height=25)
for wp in waypoints:
    airsim_client.moveToPositionAsync(wp[0], wp[1], wp[2], velocity=8).join()
    # 检测目标行人（通过位置距离判断）
    if detect_target(airsim_client.getMultirotorState().kinematics_estimated.position):
        print("Target found!")
        break
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，俯视搜索区域，螺旋路径在地面投影清晰可见  
**CAM-B（右上）**：无人机第三视角，展示无人机在城市上空的飞行姿态，地面车辆和行人可见  
**CAM-C（左下）**：全局固定高空俯视（50m），展示螺旋搜索路径的全貌  
**CAM-D（右下）**：目标行人附近的地面固定机位，展示被发现时的行人视角

---

## D14：紧急制动·空地感知互补

**核心 Feature**：无人机提前感知前方障碍物，通过通信让地面车辆提前制动，展示空地感知互补。  
**导演意图**：这个 Demo 有最强的叙事张力——无人机"看到"了车辆看不到的障碍，并"告知"了车辆。CAM-A（无人机俯视）中障碍物清晰可见，CAM-C（车辆第一视角）中障碍物被建筑遮挡，直到制动前才出现。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town06（有弯道和遮挡） |
| 天气 | ClearNoon |
| 时长 | 40 秒 |
| 主车 | 以 60 km/h 行驶，在弯道处紧急制动 |
| 障碍物 | 1 辆故障车辆，停在弯道后方（地面视角被遮挡） |
| 无人机 | 在主车前方 30m、上方 15m 飞行（提前看到障碍） |
| 行人 | 5 名，在路边 |

### 感知互补实现

```python
# 无人机提前检测到障碍物（通过位置距离判断）
def drone_detects_obstacle(drone_pos, obstacle_pos, detection_range=25.0):
    distance = math.sqrt(
        (drone_pos.x - obstacle_pos.x)**2 +
        (drone_pos.y - obstacle_pos.y)**2
    )
    return distance < detection_range

# 当无人机检测到障碍物时，触发车辆制动
if drone_detects_obstacle(drone_pos, obstacle_pos):
    # 模拟通信延迟（0.5秒）
    time.sleep(0.5)
    vehicle_control = carla.VehicleControl()
    vehicle_control.brake = 0.8  # 紧急制动
    vehicle_control.throttle = 0.0
    vehicle.apply_control(vehicle_control)
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，提前看到弯道后方的障碍物  
**CAM-B（右上）**：无人机第三视角，无人机在主车前方飞行，主车在画面中下方  
**CAM-C（左下）**：车辆第一视角，弯道遮挡障碍物，直到制动前才看到  
**CAM-D（右下）**：车辆第三视角，制动时主车在画面中，无人机在前上方

---

## D15：大规模行人·人群仿真

**核心 Feature**：50 名行人的密集场景，无人机低空追踪特定行人，展示平台的大规模行人仿真能力。  
**导演意图**：密集人群从无人机俯视角度看极具视觉冲击力。无人机飞行高度控制在 8m，确保每个行人在画面中清晰可见。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD（主广场区域） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 行人 | 50 名，在广场自然行走 |
| 无人机 | 在广场上空 8m，追踪特定行人 |
| 车辆 | 3 辆，在广场周围道路行驶 |

### 大规模行人生成

```python
# 批量生成 50 名行人
walker_bps = blueprint_library.filter('walker.pedestrian.*')
spawn_points = []

# 在广场范围内随机生成行人位置
for i in range(50):
    loc = carla.Location(
        x=random.uniform(100, 200),
        y=random.uniform(100, 200),
        z=0.5
    )
    spawn_points.append(carla.Transform(loc))

batch_walkers = []
for sp in spawn_points:
    bp = random.choice(walker_bps)
    batch_walkers.append(carla.command.SpawnActor(bp, sp))

walker_results = carla_client.apply_batch_sync(batch_walkers, True)
walkers = [world.get_actor(r.actor_id) for r in walker_results if not r.error]

# 为每个行人设置 AI 控制器
walker_controllers = []
controller_bp = blueprint_library.find('controller.ai.walker')
for walker in walkers:
    controller = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
    controller.start()
    controller.go_to_location(world.get_random_location_from_navigation())
    controller.set_max_speed(random.uniform(1.0, 2.0))
    walker_controllers.append(controller)
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，8m 低空俯视，50 名行人清晰可见，目标行人在画面中心  
**CAM-B（右上）**：无人机第三视角，无人机机身在上，密集人群在下，广场全景为背景  
**CAM-C（左下）**：目标行人第三视角（挂载在行人身上），无人机在上方追踪  
**CAM-D（右下）**：广场边缘固定机位（z=10m，俯角 45°），展示无人机和人群的全景


---

## D16：空地 LiDAR·点云融合

**核心 Feature**：地面 LiDAR（水平扫描）和空中 LiDAR（俯视扫描）同步采集，点云融合后覆盖更完整。  
**导演意图**：点云可视化本身视觉效果强烈，蓝色（地面）和橙色（空中）两种点云融合的画面直观展示了空地协同感知的互补价值。CAM-B（无人机第三视角）作为"真实感锚点"，证明这是真实无人机采集的数据。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town05 |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 主车 | 携带地面 LiDAR，以 20 km/h 缓慢行驶 |
| 无人机 | 在主车上方 15m，携带空中 LiDAR |
| 行人 | 10 名，部分在建筑遮挡区域 |

### 双 LiDAR 配置

```python
# 地面 LiDAR（水平 360° 扫描）
ground_lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
ground_lidar_bp.set_attribute('channels', '64')
ground_lidar_bp.set_attribute('range', '50')
ground_lidar_bp.set_attribute('upper_fov', '15')
ground_lidar_bp.set_attribute('lower_fov', '-25')
ground_lidar_bp.set_attribute('points_per_second', '1000000')
ground_lidar = world.spawn_actor(ground_lidar_bp,
    carla.Transform(carla.Location(x=0, z=2.5)), attach_to=vehicle)

# 空中 LiDAR（向下扫描，AirSim 通过 CARLA actor 挂载）
aerial_lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
aerial_lidar_bp.set_attribute('channels', '32')
aerial_lidar_bp.set_attribute('range', '30')
aerial_lidar_bp.set_attribute('upper_fov', '0')
aerial_lidar_bp.set_attribute('lower_fov', '-45')

# 点云融合可视化
def merge_and_colorize(ground_pts, aerial_pts):
    ground_colors = np.tile([0.2, 0.4, 1.0], (len(ground_pts), 1))  # 蓝色
    aerial_colors = np.tile([1.0, 0.5, 0.1], (len(aerial_pts), 1))  # 橙色
    merged_xyz = np.vstack([ground_pts[:, :3], aerial_pts[:, :3]])
    merged_colors = np.vstack([ground_colors, aerial_colors])
    return merged_xyz, merged_colors
```

### 相机配置（四格，传感器特殊布局）

**CAM-A（左上）**：地面 LiDAR 点云（蓝色），有建筑遮挡盲区  
**CAM-B（右上）**：无人机第三视角（后上方），无人机机身在上，主车在下，**作为真实感锚点**  
**CAM-C（左下）**：空中 LiDAR 点云（橙色），俯视无遮挡  
**CAM-D（右下）**：融合点云（蓝色+橙色），覆盖更完整

---

## D17：城市峡谷·低空穿越

**核心 Feature**：无人机在高楼密集的城市峡谷中低空穿越，展示复杂城市环境下的飞行能力。  
**导演意图**：城市峡谷是无人机导航的最难场景。CAM-A（FPV）展示穿越时的压迫感，CAM-B（第三视角）展示无人机在高楼之间的飞行姿态，地面车辆和行人在画面下方，体现空地一体。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town01（狭窄欧式街道） |
| 天气 | CloudyNoon |
| 时长 | 40 秒 |
| 无人机 | 在高楼之间 12m 高度穿越，包含 90° 转弯 |
| 行人 | 15 名，在峡谷底部街道行走 |
| 车辆 | 3 辆，在峡谷底部行驶 |

### 峡谷飞行路径

```python
# 峡谷飞行路径：直行 → 90° 转弯 → 继续直行
# 高度保持在建筑物高度的 1/3（约 10-15m）
# 路径设计：建筑物在画面两侧形成"峡谷感"
canyon_waypoints = [
    (0,   0,  -12),   # 起点
    (60,  0,  -12),   # 直行穿越峡谷
    (60,  0,  -15),   # 转弯前略微上升（增加戏剧感）
    (60,  50, -12),   # 90° 转弯
    (120, 50, -12),   # 继续直行
]

for wp in canyon_waypoints:
    airsim_client.moveToPositionAsync(wp[0], wp[1], wp[2], velocity=10).join()
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机 FPV（机腹前视，FOV=100°），峡谷压迫感，建筑在两侧，行人在下方  
**CAM-B（右上）**：无人机第三视角（后上方），无人机机身在上，高楼在两侧，地面车辆和行人可见  
**CAM-C（左下）**：峡谷入口固定机位（z=5m，仰角 15°），展示无人机从远处飞来穿越的全景  
**CAM-D（右下）**：地面车辆第三视角，主车在下，无人机在上方飞过

---

## D18：VLN 导航·语言指令驾驶

**核心 Feature**：语言指令引导空地协同导航，展示平台对 Vision-Language Navigation 研究的支持。  
**导演意图**：这个 Demo 面向 2026 年最活跃的 VLN/VLA 研究者。画面中叠加语言指令字幕（如"Fly to the red building and track the pedestrian below"），无人机和车辆根据指令执行任务。字幕是后期添加的，不需要真正运行 VLN 模型。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town01 |
| 天气 | ClearAfternoon |
| 时长 | 45 秒 |
| 无人机 | 根据语言指令飞向目标建筑，追踪目标行人 |
| 主车 | 根据语言指令驶向目标路口 |
| 行人 | 10 名，目标行人在红色建筑附近 |

### 语言指令字幕设计（后期添加）

```
第 0–5s：  "Initialize aerial-ground navigation system..."
第 5–15s： "Command: Drone → fly to red building, altitude 10m"
第 15–25s："Command: Vehicle → navigate to intersection B3"
第 25–35s："Command: Drone → track pedestrian below, maintain 8m"
第 35–45s："Task complete. Aerial-ground coordination achieved."
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角，飞向目标建筑，目标行人在画面中逐渐变大  
**CAM-B（右上）**：无人机第三视角，无人机飞行姿态，目标建筑和行人在画面中  
**CAM-C（左下）**：车辆第一视角，驶向目标路口  
**CAM-D（右下）**：车辆第三视角，无人机在上方，主车在下方，两者向各自目标运动

---

## D19：雷达感知·障碍物检测

**核心 Feature**：无人机雷达和车辆雷达同步工作，展示雷达传感器的协同感知能力。  
**导演意图**：雷达数据可视化（极坐标图）与 RGB 画面对比，展示雷达在恶劣天气下的感知优势。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town01 |
| 天气 | HardRainNoon（雨天体现雷达优势） |
| 时长 | 40 秒 |
| 主车 | 携带前向雷达，以 40 km/h 行驶 |
| 无人机 | 在主车上方 12m，携带雷达 |
| 障碍物 | 2 辆停放车辆，在雨中 RGB 视觉模糊 |
| 行人 | 8 名 |

### 雷达配置

```python
# 车辆前向雷达（CARLA）
radar_bp = blueprint_library.find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov', '30')
radar_bp.set_attribute('vertical_fov', '10')
radar_bp.set_attribute('range', '50')
radar_bp.set_attribute('points_per_second', '1500')
radar = world.spawn_actor(radar_bp,
    carla.Transform(carla.Location(x=2.0, z=1.0),
                    carla.Rotation(pitch=5)), attach_to=vehicle)

# 雷达数据可视化（极坐标图）
def visualize_radar(radar_data):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(6, 6))
    for detect in radar_data:
        ax.scatter(detect.azimuth, detect.depth,
                   c='lime', s=10, alpha=0.8)
    ax.set_facecolor('black')
    return fig
```

### 相机配置（四格，传感器特殊布局）

**CAM-A（左上）**：无人机 RGB 第一视角，雨中能见度低  
**CAM-B（右上）**：无人机第三视角，雨中无人机飞行，雨滴效果明显  
**CAM-C（左下）**：车辆雷达极坐标可视化，障碍物清晰显示  
**CAM-D（右下）**：车辆第三视角，无人机在上，主车在下，雨中行驶

---

## D20：14 种天气·传感器对比

**核心 Feature**：14 种天气预设的完整遍历，每种天气下 RGB 和语义分割同步展示，体现天气对传感器的影响。  
**导演意图**：快速切换 14 种天气，每种停留约 4 秒，让观众在 60 秒内感受平台的天气仿真广度，同时语义分割画面展示不同天气下感知性能的变化。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 时长 | 60 秒（14 种天气 × 约 4s） |
| 主车 | 以 20 km/h 缓慢行驶 |
| 无人机 | 在主车上方 12m 跟随 |
| 行人 | 10 名 |

### 天气序列

```python
weather_sequence = [
    ("ClearNoon",       carla.WeatherParameters.ClearNoon),
    ("CloudyNoon",      carla.WeatherParameters.CloudyNoon),
    ("WetNoon",         carla.WeatherParameters.WetNoon),
    ("WetCloudyNoon",   carla.WeatherParameters.WetCloudyNoon),
    ("MidRainyNoon",    carla.WeatherParameters.MidRainyNoon),
    ("HardRainNoon",    carla.WeatherParameters.HardRainNoon),
    ("SoftRainNoon",    carla.WeatherParameters.SoftRainNoon),
    ("ClearSunset",     carla.WeatherParameters.ClearSunset),
    ("CloudySunset",    carla.WeatherParameters.CloudySunset),
    ("WetSunset",       carla.WeatherParameters.WetSunset),
    ("WetCloudySunset", carla.WeatherParameters.WetCloudySunset),
    ("MidRainSunset",   carla.WeatherParameters.MidRainSunset),
    ("HardRainSunset",  carla.WeatherParameters.HardRainSunset),
    ("SoftRainSunset",  carla.WeatherParameters.SoftRainSunset),
]
frames_per_weather = int(60 * 30 / len(weather_sequence))  # 约 128 帧

for name, weather in weather_sequence:
    carla_world.set_weather(weather)
    for _ in range(frames_per_weather):
        carla_world.tick()
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角 RGB，天气变化时画面效果最直观（雨滴、雾气、光线）  
**CAM-B（右上）**：无人机第三视角，无人机在不同天气下的外观，主车在下方  
**CAM-C（左下）**：车辆第一视角语义分割，不同天气下语义分割性能变化（雨天行人识别率下降）  
**CAM-D（右下）**：车辆第三视角，无人机在上，主车在下，天气变化时两者同时被影响

---

## D21：IMU/GPS 数据流·画中画可视化

**核心 Feature**：无人机 IMU 和 GPS 传感器数据实时可视化，展示平台的传感器数据质量。  
**导演意图**：这个 Demo 面向工程师受众。主画面是无人机飞行的第三视角，右下角画中画是实时数据折线图，展示 IMU 加速度和 GPS 高度的实时变化。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town02 |
| 天气 | ClearNoon |
| 时长 | 30 秒 |
| 无人机 | 执行小幅机动（前后左右各移动 3m，上下各 2m） |
| 行人 | 8 名，在下方行走 |
| 车辆 | 2 辆 |

### 数据可视化实现

```python
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# 实时数据缓冲区
accel_x_buf = deque(maxlen=90)  # 3秒数据
altitude_buf = deque(maxlen=90)

def update_plot(frame):
    imu_data = airsim_client.getImuData()
    gps_data  = airsim_client.getGpsData()
    
    accel_x_buf.append(imu_data.linear_acceleration.x_val)
    altitude_buf.append(gps_data.gnss.geo_point.altitude)
    
    ax1.clear()
    ax1.plot(list(accel_x_buf), color='cyan', linewidth=1.5)
    ax1.set_title('IMU Accel X (m/s²)', color='white', fontsize=8)
    ax1.set_facecolor('#1a1a2e')
    
    ax2.clear()
    ax2.plot(list(altitude_buf), color='lime', linewidth=1.5)
    ax2.set_title('GPS Altitude (m)', color='white', fontsize=8)
    ax2.set_facecolor('#1a1a2e')
```

### 相机配置（主画面 + 画中画）

**主画面（全屏）**：无人机第三视角（后上方），无人机机身在上，行人和车辆在下，执行小幅机动  
**画中画（右下角 30% 宽度）**：matplotlib 实时折线图，深色背景，IMU 和 GPS 数据曲线

---

## D22：夜间 RGB+红外·双模对比

**核心 Feature**：夜间 RGB 和红外双模传感器对比，展示不同传感器在夜间的感知能力差异。  
**导演意图**：与 D08 不同，本 Demo 同时展示无人机和车辆的夜间感知，强调空地双平台的夜间协同能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town01 |
| 天气 | ClearNight |
| 时长 | 45 秒 |
| 无人机 | 在街道上空 10m 飞行 |
| 主车 | 开启车灯，以 30 km/h 行驶 |
| 行人 | 12 名，在街道行走 |

### 相机配置（四格）

**CAM-A（左上）**：无人机 RGB 第一视角，夜间，路灯下行人轮廓模糊  
**CAM-B（右上）**：无人机红外第一视角，行人呈现为明亮热源  
**CAM-C（左下）**：车辆第一视角 RGB，车灯照亮前方，行人在光圈边缘模糊  
**CAM-D（右下）**：车辆第三视角，夜间无人机灯光在上方闪烁，主车车灯在下方，行人在路灯下

---

## D23：搜救协同·无人机引导车辆

**核心 Feature**：无人机从空中发现目标行人，引导地面车辆前往救援，展示空地任务协同能力。  
**导演意图**：搜救场景有最强的叙事性。三幕结构：搜索→发现→救援，每幕对应不同的相机运镜。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town03 |
| 天气 | CloudyNoon |
| 时长 | 60 秒 |
| 目标行人 | 1 名，静止在偏僻位置 |
| 背景行人 | 10 名，正常行走 |
| 无人机 | 螺旋搜索，发现目标后悬停 |
| 救援车辆 | 1 辆，接收无人机信号后前往目标 |

### 任务状态机

```python
class SearchRescueTask:
    def __init__(self):
        self.phase = 'search'

    def update(self, tick, drone_pos, target_pos, vehicle):
        if self.phase == 'search':
            self.spiral_search(tick)
            if drone_pos.distance(target_pos) < 10:
                self.phase = 'found'
                self.found_tick = tick

        elif self.phase == 'found':
            self.hover_above_target(target_pos)
            if tick > self.found_tick + 150:  # 悬停 5 秒
                self.phase = 'navigate'

        elif self.phase == 'navigate':
            vehicle.set_autopilot(True)
            # 设置目标点（需要配合 CARLA 导航系统）
            if vehicle.get_location().distance(target_pos) < 5:
                self.phase = 'arrive'
```

### 相机配置（标准四格，三幕切换）

**搜索阶段（0–25s）**：  
- CAM-A：无人机第一视角，俯视搜索区域  
- CAM-B：无人机第三视角，螺旋飞行姿态  
- CAM-C：救援车辆第一视角（等待中）  
- CAM-D：救援车辆第三视角，无人机在远处搜索可见

**发现阶段（25–35s）**：  
- CAM-A：无人机第一视角，目标行人在画面中心  
- CAM-B：无人机第三视角，悬停在目标上方  
- CAM-C：目标行人附近固定机位，无人机从上方出现  
- CAM-D：救援车辆第三视角，开始启动

**救援阶段（35–60s）**：  
- CAM-A：无人机第一视角，引导车辆前进  
- CAM-B：无人机第三视角，车辆在下方行驶  
- CAM-C：救援车辆第一视角，前往目标  
- CAM-D：救援车辆第三视角，无人机在上方引导，目标行人在前方

---

## D24：行人轨迹·数据采集

**核心 Feature**：无人机低空俯视采集行人轨迹数据，展示平台作为数据采集工具的价值。  
**导演意图**：无人机飞行高度控制在 8m，确保行人在画面中清晰可见。轨迹可视化线条（后期叠加）在画面中呈现为彩色路径，视觉效果直观。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town05（大型广场） |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 行人 | 25 名，在广场自然行走 |
| 无人机 | 悬停在 8m 高度，俯视广场 |
| 车辆 | 3 辆，在广场周围道路行驶 |

### 轨迹数据采集

```python
walker_trajectories = {walker.id: [] for walker in walkers}

for tick in range(45 * 30):
    carla_world.tick()
    for walker in walkers:
        loc = walker.get_location()
        walker_trajectories[walker.id].append({
            'tick': tick,
            'x': loc.x, 'y': loc.y, 'z': loc.z
        })

# 后期可视化：将轨迹投影到无人机俯视画面上
# 使用 OpenCV 在每帧画面上绘制轨迹线
def draw_trajectories_on_frame(frame, trajectories, drone_pose, camera_matrix):
    for walker_id, traj in trajectories.items():
        color = WALKER_COLORS[walker_id % len(WALKER_COLORS)]
        for i in range(1, len(traj)):
            pt1 = world_to_image(traj[i-1], drone_pose, camera_matrix)
            pt2 = world_to_image(traj[i],   drone_pose, camera_matrix)
            cv2.line(frame, pt1, pt2, color, thickness=2)
    return frame
```

### 相机配置（标准四格）

**CAM-A（左上）**：无人机第一视角（8m 低空俯视），行人清晰可见，轨迹线后期叠加  
**CAM-B（右上）**：无人机第三视角，无人机悬停，广场全景，行人在下方  
**CAM-C（左下）**：广场边缘固定机位（z=3m，俯角 30°），展示无人机和行人的全景  
**CAM-D（右下）**：无人机语义分割俯视，行人（红色）清晰区分，便于轨迹提取

---

## D25：光流传感器·无人机视觉定位

**核心 Feature**：光流传感器辅助无人机在 GPS 信号弱的环境中定位，展示平台的传感器多样性。  
**导演意图**：光流数据可视化（箭头场）与 RGB 画面对比，展示光流传感器的工作原理和实用价值。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town04 |
| 天气 | CloudyNoon（模拟 GPS 信号弱的场景） |
| 时长 | 40 秒 |
| 无人机 | 在地面纹理丰富的区域低速飞行（5m 高度） |
| 主车 | 以 30 km/h 行驶 |
| 行人 | 8 名 |

### 光流传感器配置

```python
# AirSim 光流数据读取
# 光流通过连续帧之间的图像差分计算
def compute_optical_flow(prev_frame, curr_frame):
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
    flow = cv2.calcOpticalFlowFarneback(
        prev_gray, curr_gray, None,
        pyr_scale=0.5, levels=3, winsize=15,
        iterations=3, poly_n=5, poly_sigma=1.2, flags=0
    )
    return flow

# 光流可视化（箭头场）
def draw_flow_arrows(frame, flow, step=16):
    h, w = frame.shape[:2]
    y, x = np.mgrid[step//2:h:step, step//2:w:step]
    fx, fy = flow[y, x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    for (x1, y1), (x2, y2) in lines.astype(int):
        cv2.arrowedLine(frame, (x1, y1), (x2, y2), (0, 255, 0), 1, tipLength=0.3)
    return frame
```

### 相机配置（四格）

**CAM-A（左上）**：无人机 RGB 第一视角（5m 低空），地面纹理清晰  
**CAM-B（右上）**：光流可视化（箭头场叠加在 RGB 上），展示无人机运动方向  
**CAM-C（左下）**：车辆第一视角，主车在地面行驶  
**CAM-D（右下）**：车辆第三视角，无人机在上方低空飞行，主车在下方

---

## D26：多传感器融合·自动驾驶

**核心 Feature**：车辆同时使用 Camera、LiDAR、Radar 三种传感器进行感知融合，无人机提供空中辅助视角。  
**导演意图**：这个 Demo 展示了 CARLA-Air 对完整自动驾驶感知栈的支持。三种传感器数据同时可视化，展示多模态融合的全貌。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town06 |
| 天气 | WetNoon（湿路面，雷达优势明显） |
| 时长 | 45 秒 |
| 主车 | 携带 Camera+LiDAR+Radar，以 40 km/h 行驶 |
| 无人机 | 在主车上方 12m 跟随，提供空中视角 |
| 行人 | 10 名 |
| 配角车 | 4 辆 |

### 三传感器配置

```python
# Camera（已在通用规范中配置）

# LiDAR
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', '64')
lidar_bp.set_attribute('range', '50')
lidar = world.spawn_actor(lidar_bp,
    carla.Transform(carla.Location(z=2.5)), attach_to=vehicle)

# Radar
radar_bp = blueprint_library.find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov', '35')
radar_bp.set_attribute('range', '50')
radar = world.spawn_actor(radar_bp,
    carla.Transform(carla.Location(x=2.0, z=1.0),
                    carla.Rotation(pitch=5)), attach_to=vehicle)
```

### 相机配置（四格，传感器特殊布局）

**CAM-A（左上）**：车辆 RGB 第一视角  
**CAM-B（右上）**：无人机第三视角，无人机在上，主车在下，提供空中辅助视角  
**CAM-C（左下）**：车辆 LiDAR 点云可视化（侧视角），行人和车辆为点云簇  
**CAM-D（右下）**：车辆 Radar 极坐标可视化，障碍物清晰标注

---

## D27：无人机编队·队形变换

**核心 Feature**：3 架无人机执行 V 形→横排→菱形队形变换，展示多智能体协同控制。  
**导演意图**：队形变换是多无人机协同中最有视觉美感的场景。CAM-A（高空俯视）展示队形的几何美感，CAM-B（侧方 Orbit）展示三维结构，地面行人和车辆在画面下方体现空地一体。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 45 秒 |
| 无人机 | 3 架，执行队形变换 |
| 行人 | 15 名，在下方广场 |
| 车辆 | 3 辆，在周围道路 |

### 队形变换实现

```python
formations = {
    'V_shape':  [(0, 0, 0), (-6, -6, 0), (6, -6, 0)],
    'line':     [(-8, 0, 0), (0, 0, 0), (8, 0, 0)],
    'diamond':  [(0, 6, 0), (-6, 0, 0), (6, 0, 0)],
}

def interpolate_formation(from_f, to_f, progress):
    return [
        (from_f[i][0] + (to_f[i][0] - from_f[i][0]) * progress,
         from_f[i][1] + (to_f[i][1] - from_f[i][1]) * progress,
         from_f[i][2] + (to_f[i][2] - from_f[i][2]) * progress)
        for i in range(len(from_f))
    ]

# 队形变换节奏：V形(0-15s) → 横排(15-30s) → 菱形(30-45s)
```

### 相机配置（四格）

**CAM-A（左上）**：全局高空俯视（50m），三架无人机队形清晰，地面行人和车辆可见  
**CAM-B（右上）**：全局侧方 Orbit（半径 30m，高度 15m），展示队形三维结构，地面在下方  
**CAM-C（左下）**：Drone1 第三视角（后上方），能看到 Drone1 机身和其他两架无人机  
**CAM-D（右下）**：地面车辆第三视角，3 架无人机在上方编队飞行，主车在下方

---

## D28：跨地图·场景多样性

**核心 Feature**：快速切换 5 种地图场景，展示平台对不同城市环境的支持广度。  
**导演意图**：每种地图停留约 12 秒，展示不同风格的城市环境。这个 Demo 是整套 Demo 的"广度展示"，让观众感受到平台不局限于单一场景。

### 场景序列

| 时间 | 地图 | 场景特点 |
|------|------|----------|
| 0–12s | Town10HD | 欧式大城市，高楼密集 |
| 12–24s | Town04 | 高速公路，开阔视野 |
| 24–36s | Town01 | 小镇，狭窄欧式街道 |
| 36–48s | Town05 | 大型十字路口，方格路网 |
| 48–60s | Town03 | 中等城市，立交桥 |

### 场景切换实现

```python
# 注意：地图切换需要重新加载世界，建议分段录制后后期拼接
# 每段录制时，保持相同的无人机高度（12m）和车辆速度（30 km/h）
# 后期拼接时，在切换点添加 0.5 秒的黑场过渡

map_sequence = ['Town10HD', 'Town04', 'Town01', 'Town05', 'Town03']
for map_name in map_sequence:
    client.load_world(map_name)
    world = client.get_world()
    # 重新生成车辆、行人、无人机
    setup_scene(world, map_name)
    record_segment(world, duration=12)
```

### 相机配置（标准四格）

所有地图使用相同的四格布局，确保切换时构图一致：  
**CAM-A**：无人机第一视角 | **CAM-B**：无人机第三视角  
**CAM-C**：车辆第一视角 | **CAM-D**：车辆第三视角（无人机在上方可见）

---

## D29：全传感器·综合展示

**核心 Feature**：在一个视频中展示所有传感器类型（RGB、深度、语义、LiDAR、Radar、红外、光流），展示平台的传感器生态完整性。  
**导演意图**：这个 Demo 是整套 Demo 的"能力清单"。采用快速切换的方式，每种传感器停留约 8 秒，配合字幕标注传感器名称（后期添加）。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 60 秒 |
| 主车 | 以 30 km/h 行驶 |
| 无人机 | 在主车上方 10m 跟随 |
| 行人 | 15 名 |

### 传感器展示序列

| 时间 | 传感器 | 画面内容 |
|------|--------|----------|
| 0–8s | RGB（空地双视角） | 标准四格，空地协同 |
| 8–16s | 深度图（空地） | 深度彩虹色可视化 |
| 16–24s | 语义分割（空地） | 行人红色，车辆蓝色 |
| 24–32s | LiDAR 点云（地面） | 蓝色点云，360° 扫描 |
| 32–40s | LiDAR 点云（空中） | 橙色点云，俯视扫描 |
| 40–48s | Radar 极坐标图 | 障碍物检测可视化 |
| 48–56s | 红外（夜间切换） | 行人热成像 |
| 56–60s | 全传感器融合 | 四格同时展示四种模态 |

### 相机配置

本 Demo 采用动态布局，随传感器类型切换画面组合。核心原则：每个传感器展示阶段，至少有一路画面能看到无人机模型本身（通常是 CAM-B 无人机第三视角）。

---

## D30：压轴·最大规模场景

**核心 Feature**：20 辆车 + 50 名行人 + 3 架无人机同时运行，展示 CARLA-Air 的极限规模能力。  
**导演意图**：这是整套 Demo 的压轴。最大规模场景的视觉冲击力来自于"数量"——观众看到这么多智能体同时运行，直觉上就会感受到平台的规模能力。

### 场景配置

| 参数 | 值 |
|------|----|
| 地图 | Town10HD |
| 天气 | ClearNoon |
| 时长 | 60 秒 |
| 车辆 | 20 辆，全部自动驾驶 |
| 行人 | 50 名，全部 AI 控制 |
| 无人机 | 3 架：Drone1 追踪主车，Drone2 追踪目标行人，Drone3 全局监控（50m） |

### 大规模场景生成

```python
# 批量生成 20 辆车
vehicle_bps = blueprint_library.filter('vehicle.*')
spawn_points = world.get_map().get_spawn_points()

batch = [carla.command.SpawnActor(
    random.choice(vehicle_bps), spawn_points[i]
) for i in range(min(20, len(spawn_points)))]

results = carla_client.apply_batch_sync(batch, True)
vehicles = [world.get_actor(r.actor_id) for r in results if not r.error]
for v in vehicles:
    v.set_autopilot(True)

# 批量生成 50 名行人（参考 D15 实现）
# 三架无人机配置（参考 D06 实现）
```

### 相机配置（四格，特殊布局）

```
┌─────────────────────────────────┐
│                                 │
│   全局高空俯视（60m）           │
│   20车+50人+3机同框             │
│   （占上半 50% 高度）           │
│                                 │
├──────────┬──────────┬───────────┤
│ Drone1   │ Drone2   │ 车辆第三  │
│ 第三视角 │ 第三视角 │ 视角      │
│ 追踪主车 │ 追踪行人 │ 无人机可见│
└──────────┴──────────┴───────────┘
```

```bash
# 特殊布局合成
ffmpeg \
  -i global_top.mp4 -i drone1_3rd.mp4 -i drone2_3rd.mp4 -i vehicle_3rd.mp4 \
  -filter_complex "
    [0:v]scale=1920:540[top];
    [1:v]scale=640:540[b];
    [2:v]scale=640:540[c];
    [3:v]scale=640:540[d];
    [b][c][d]hstack=inputs=3[bottom];
    [top][bottom]vstack[out]
  " \
  -map "[out]" output_d30.mp4
```

### 运镜设计

**0–20s**：全局高空俯视（60m），缓慢 Orbit，展示整个场景规模。下方三格展示三路无人机视角。  
**20–45s**：全局俯视相机缓慢下降到 30m，场景细节逐渐清晰，行人和车辆的运动轨迹更明显。  
**45–60s**：全局俯视相机缓慢上升到 100m，最终展示整个 Town10HD 城市全景，所有智能体都在画面中，以此作为整套 Demo 的收尾。

### 性能监控

```python
import time
import numpy as np

frame_times = []
for tick in range(60 * 30):
    start = time.time()
    carla_world.tick()
    elapsed = time.time() - start
    frame_times.append(elapsed)
    
    if elapsed > 0.04:  # 低于 25 FPS 时警告
        print(f"Warning: Frame {tick} took {elapsed*1000:.1f}ms")

print(f"Average FPS: {1.0 / np.mean(frame_times):.1f}")
print(f"Min FPS: {1.0 / max(frame_times):.1f}")
```

---

## 四、附录：通用工程规范

### 4.1 坐标系转换（CARLA ↔ AirSim）

```python
def carla_to_airsim(carla_location):
    """CARLA 坐标转 AirSim 坐标"""
    # CARLA: X前 Y右 Z上（右手系）
    # AirSim: X前 Y右 Z下（NED 坐标系）
    return airsim.Vector3r(
        x_val=carla_location.x,
        y_val=carla_location.y,
        z_val=-carla_location.z  # Z 轴取反
    )

def airsim_to_carla(airsim_vector):
    """AirSim 坐标转 CARLA 坐标"""
    return carla.Location(
        x=airsim_vector.x_val,
        y=airsim_vector.y_val,
        z=-airsim_vector.z_val  # Z 轴取反
    )
```

### 4.2 帧同步实现

```python
# 确保 CARLA 和 AirSim 在同一帧内同步
# CARLA 使用同步模式，AirSim 通过 pause/resume 实现同步

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 1.0 / 30  # 30 FPS
world.apply_settings(settings)

for tick in range(total_frames):
    # 1. 暂停 AirSim
    airsim_client.simPause(True)
    
    # 2. CARLA 推进一帧
    world.tick()
    
    # 3. 采集 CARLA 传感器数据
    carla_data = collect_carla_sensors()
    
    # 4. 恢复 AirSim 并推进一帧
    airsim_client.simPause(False)
    airsim_client.simContinueForTime(1.0 / 30)
    
    # 5. 采集 AirSim 传感器数据
    airsim_data = collect_airsim_sensors()
    
    # 6. 保存帧数据
    save_frame(tick, carla_data, airsim_data)
```

### 4.3 视频输出规范

| 参数 | 值 |
|------|----|
| 分辨率 | 1920×1080（单格）/ 3840×2160（四格合成后可选） |
| 帧率 | 30 FPS |
| 编码 | H.264（libx264），CRF=18 |
| 音频 | 无（后期添加背景音乐） |
| 格式 | MP4 |

### 4.4 常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 四格画面时间不对齐 | 未开启同步模式 | 确保 `synchronous_mode = True` |
| 无人机在 CAM-D 中不可见 | CAM-D 高度不足 | 将 CAM-D 的 z 值提高到无人机飞行高度的 1/3 |
| 行人在 CAM-A 中太小 | 无人机飞行高度过高 | 将无人机高度降低到 8–12m |
| AirSim 相机看到螺旋桨 | 相机挂载位置在桨盘上方 | 将相机 Z 值设为正值（机腹方向） |
| 多无人机控制冲突 | 使用了同一个客户端 | 为每架无人机创建独立的 `MultirotorClient` |
| 天气切换不同步 | CARLA 和 AirSim 分别切换 | 在同一个 tick 内同时调用两者的天气 API |

