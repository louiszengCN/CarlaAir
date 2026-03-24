# CarlaAir Quick Start Guide

欢迎使用 **CarlaAir** — 空地一体联合仿真平台。本指南将帮助你在 10 分钟内完成从启动到运行第一个联合仿真脚本。

## 目录

1. [启动仿真器](#1-启动仿真器)
2. [Hello World: 连接验证](#2-hello-world)
3. [控制天气](#3-控制天气)
4. [生成交通](#4-生成交通)
5. [传感器采集](#5-传感器采集)
6. [无人机起飞](#6-无人机起飞)
7. [无人机传感器](#7-无人机传感器)
8. [联合仿真：地面+空中](#8-联合仿真)
9. [完整演示](#9-完整演示)
10. [下一步](#10-下一步)

---

## 1. 启动仿真器

```bash
# Shipping 构建（推荐）
cd /mnt/data1/tianle/carla_source/Dist/CARLA_Shipping_1ae5356-dirty/LinuxNoEditor
./carla_air.sh

# 或 Editor 构建（开发调试用）
cd /mnt/data1/tianle/carla_source
./carlaAir.sh
```

等待看到 "Ready! Both servers are running." 即可。

**端口说明**:
- CARLA API: `localhost:2000` (地面仿真：车辆、行人、天气、传感器)
- AirSim API: `localhost:41451` (空中仿真：无人机飞行、相机)

---

## 2. Hello World

运行 `examples/01_hello_world.py` 验证连接：

```python
import carla
import airsim

# 连接 CARLA（地面仿真）
client = carla.Client('localhost', 2000)
client.set_timeout(10)
world = client.get_world()
print(f"CARLA 连接成功: {world.get_map().name}")
print(f"可用生成点: {len(world.get_map().get_spawn_points())} 个")

# 连接 AirSim（空中仿真）
air = airsim.MultirotorClient(port=41451)
air.confirmConnection()
print("AirSim 连接成功: 无人机就绪")

# 查看当前天气
weather = world.get_weather()
print(f"当前天气: 太阳高度={weather.sun_altitude_angle}°, 云量={weather.cloudiness}%")

print("\nCarlaAir 已就绪！地面 + 空中 API 均可用。")
```

**预期输出**:
```
CARLA 连接成功: Carla/Maps/Town10HD/Town10HD
可用生成点: 155 个
AirSim 连接成功: 无人机就绪
当前天气: 太阳高度=45.0°, 云量=0.0%

CarlaAir 已就绪！地面 + 空中 API 均可用。
```

---

## 3. 控制天气

CarlaAir 的天气系统支持实时切换，同时影响地面和空中视角。

运行 `examples/02_weather_control.py`:

```python
import carla
import time

client = carla.Client('localhost', 2000)
world = client.get_world()

# 定义天气预设
presets = {
    "晴天":    {"sun_altitude_angle": 70, "cloudiness": 10},
    "多云":    {"sun_altitude_angle": 50, "cloudiness": 80},
    "暴雨":    {"precipitation": 100, "cloudiness": 90, "wetness": 100,
                "precipitation_deposits": 80, "wind_intensity": 50},
    "大雾":    {"fog_density": 80, "fog_distance": 10, "cloudiness": 60},
    "日落":    {"sun_altitude_angle": 5, "cloudiness": 30},
}

for name, params in presets.items():
    weather = carla.WeatherParameters()
    for k, v in params.items():
        setattr(weather, k, v)
    world.set_weather(weather)
    print(f"天气切换: {name}")
    time.sleep(3)

# 恢复晴天
world.set_weather(carla.WeatherParameters.ClearNoon)
print("天气已恢复为晴天")
```

---

## 4. 生成交通

运行 `examples/03_spawn_traffic.py`:

```python
import carla
import random
import time

client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# 在城市中心附近生成车辆
spawn_points = world.get_map().get_spawn_points()
# 选择城市中心的生成点 (x > 55, 远离海岸)
city_spawns = [sp for sp in spawn_points if sp.location.x > 55][:10]

vehicles = []
for sp in city_spawns:
    bp = random.choice(bp_lib.filter('vehicle.*'))
    if bp.has_attribute('color'):
        bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
    v = world.try_spawn_actor(bp, sp)
    if v:
        v.set_autopilot(True)
        vehicles.append(v)

print(f"已生成 {len(vehicles)} 辆自动驾驶车辆")

# 生成行人（静态，不使用 AI 控制器以避免崩溃）
walkers = []
for _ in range(15):
    loc = world.get_random_location_from_navigation()
    if loc:
        bp = random.choice(bp_lib.filter('walker.pedestrian.*'))
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'true')
        w = world.try_spawn_actor(bp, carla.Transform(loc))
        if w:
            walkers.append(w)

print(f"已生成 {len(walkers)} 个行人")

# 观察 20 秒
print("观察交通运行... (20秒)")
time.sleep(20)

# 清理
for v in vehicles: v.destroy()
for w in walkers: w.destroy()
print("清理完成")
```

---

## 5. 传感器采集

运行 `examples/04_sensor_capture.py`:

```python
import carla
import numpy as np
import cv2
import queue
import time

client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# 生成一辆带传感器的车辆
spawn_points = world.get_map().get_spawn_points()
vehicle = world.spawn_actor(
    bp_lib.find('vehicle.tesla.model3'),
    [sp for sp in spawn_points if sp.location.x > 55][0]
)
vehicle.set_autopilot(True)

# 在车辆上安装 RGB 相机
cam_bp = bp_lib.find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '1280')
cam_bp.set_attribute('image_size_y', '720')
cam_bp.set_attribute('fov', '100')

cam_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

# 采集图像
img_queue = queue.Queue()
camera.listen(lambda img: img_queue.put(img) if not img_queue.full() else None)

print("采集 5 帧图像...")
time.sleep(2)  # 等待传感器预热

for i in range(5):
    time.sleep(0.5)
    try:
        img = img_queue.get(timeout=2)
        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
        frame = arr.reshape(img.height, img.width, 4)[:, :, :3]  # BGRA -> BGR
        cv2.imwrite(f'/tmp/carla_frame_{i}.png', frame)
        print(f"  保存帧 {i}: {img.width}x{img.height}")
    except queue.Empty:
        print(f"  帧 {i}: 超时")

# 清理
camera.stop()
camera.destroy()
vehicle.destroy()
print(f"采集完成! 图像保存在 /tmp/carla_frame_*.png")
```

---

## 6. 无人机起飞

运行 `examples/05_drone_takeoff.py`:

```python
import airsim
import time

client = airsim.MultirotorClient(port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("无人机解锁，准备起飞...")

# 起飞
client.takeoffAsync().join()
print("已起飞!")

# 飞到城市中心上空 (NED坐标: x=80向内陆, y=0, z=-30即30米高)
print("飞往城市中心...")
client.moveToPositionAsync(80, 0, -30, 5).join()
print("到达城市中心上空 (高度30米)")

# 获取状态
state = client.getMultirotorState()
pos = state.kinematics_estimated.position
print(f"当前位置: NED({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
print(f"海拔高度: {abs(pos.z_val):.1f}m")

# 悬停 5 秒
print("悬停 5 秒...")
client.hoverAsync().join()
time.sleep(5)

# 正方形飞行
print("执行正方形航线...")
waypoints = [
    (90, 0, -30),   # 前方
    (90, 10, -30),   # 右方
    (80, 10, -30),   # 后方
    (80, 0, -30),    # 返回起点
]
for i, (x, y, z) in enumerate(waypoints):
    print(f"  航点 {i+1}/4: ({x}, {y}, {z})")
    client.moveToPositionAsync(x, y, z, 5).join()

# 降落
print("降落中...")
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
print("降落完成!")
```

---

## 7. 无人机传感器

运行 `examples/06_drone_sensors.py`:

```python
import airsim
import numpy as np
import cv2
import time

client = airsim.MultirotorClient(port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# 起飞并飞到城市中心
client.takeoffAsync().join()
client.moveToPositionAsync(80, 0, -25, 5).join()
print("无人机已到达城市中心上空")

# 采集三种图像: RGB, 深度, 语义分割
print("采集多模态图像...")

responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),           # RGB
    airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),  # 深度 (float)
    airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),     # 语义分割
])

# RGB
if responses[0].height > 0:
    img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    rgb = img.reshape(responses[0].height, responses[0].width, 3)
    cv2.imwrite('/tmp/drone_rgb.png', rgb)
    print(f"  RGB: {responses[0].width}x{responses[0].height}")

# 深度
if responses[1].height > 0:
    depth = airsim.list_to_2d_float_array(
        responses[1].image_data_float, responses[1].width, responses[1].height)
    depth_vis = np.clip(depth / 100.0 * 255, 0, 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    cv2.imwrite('/tmp/drone_depth.png', depth_color)
    print(f"  Depth: range [{depth.min():.1f}, {depth.max():.1f}]m")

# 语义分割
if responses[2].height > 0:
    img = np.frombuffer(responses[2].image_data_uint8, dtype=np.uint8)
    seg = img.reshape(responses[2].height, responses[2].width, 3)
    cv2.imwrite('/tmp/drone_seg.png', seg)
    print(f"  Segmentation: {responses[2].width}x{responses[2].height}")

# 降落
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
print(f"\n图像保存在 /tmp/drone_*.png")
```

---

## 8. 联合仿真

这是 CarlaAir 的核心能力：**同时操控地面车辆和空中无人机**。

运行 `examples/07_combined_demo.py`:

```python
import carla
import airsim
import time
import math

# === 连接两个 API ===
client_carla = carla.Client('localhost', 2000)
client_carla.set_timeout(10)
world = client_carla.get_world()

client_air = airsim.MultirotorClient(port=41451)
client_air.confirmConnection()
client_air.enableApiControl(True)
client_air.armDisarm(True)

print("双 API 连接成功!")

# === CARLA: 设置天气 + 生成车辆 ===
weather = carla.WeatherParameters()
weather.sun_altitude_angle = 45.0
weather.cloudiness = 20.0
world.set_weather(weather)

bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()
city_spawns = [sp for sp in spawn_points if sp.location.x > 55]

vehicles = []
for sp in city_spawns[:8]:
    bp = bp_lib.filter('vehicle.*')[len(vehicles) % 20]
    v = world.try_spawn_actor(bp, sp)
    if v:
        v.set_autopilot(True)
        vehicles.append(v)
print(f"CARLA: 已生成 {len(vehicles)} 辆车辆")

# === AirSim: 无人机起飞 ===
client_air.takeoffAsync().join()
client_air.moveToPositionAsync(80, 0, -30, 5).join()
print("AirSim: 无人机已到达城市中心上空 30m")

# === 联合运行: 无人机巡航 + 地面交通 ===
print("\n联合仿真运行中...")
print("  - 地面: 8辆自动驾驶车辆")
print("  - 空中: 无人机在城市上空巡航")

# 无人机绕城市中心飞行
orbit_duration = 15  # 秒
for i in range(orbit_duration * 2):
    t = i / (orbit_duration * 2) * 2 * math.pi
    x = 80 + 15 * math.cos(t)
    y = 0 + 15 * math.sin(t)
    client_air.moveToPositionAsync(x, y, -30, 8)

    # 打印状态
    if i % 10 == 0:
        drone_pos = client_air.getMultirotorState().kinematics_estimated.position
        car_count = len([a for a in world.get_actors().filter('vehicle.*')])
        print(f"  [{i//2}s] 无人机: ({drone_pos.x_val:.0f},{drone_pos.y_val:.0f},{drone_pos.z_val:.0f}), "
              f"地面车辆: {car_count}")

    time.sleep(0.5)

# === 清理 ===
print("\n清理中...")
client_air.landAsync().join()
client_air.armDisarm(False)
client_air.enableApiControl(False)
for v in vehicles:
    v.destroy()

print("联合仿真演示完成!")
```

---

## 9. 完整演示

运行 `examples/08_full_showcase.py` 展示所有核心功能：

```python
"""
CarlaAir 完整功能展示
演示内容: 天气切换 → 交通生成 → 传感器采集 → 无人机飞行 → 联合巡航
"""
import carla
import airsim
import numpy as np
import cv2
import time
import math

# === 初始化 ===
print("=" * 50)
print("CarlaAir 完整功能展示")
print("=" * 50)

client = carla.Client('localhost', 2000)
client.set_timeout(10)
world = client.get_world()
bp_lib = world.get_blueprint_library()

air = airsim.MultirotorClient(port=41451)
air.confirmConnection()
air.enableApiControl(True)
air.armDisarm(True)

# === 1. 天气展示 ===
print("\n[1/5] 天气系统")
for name, w in [("晴天", carla.WeatherParameters.ClearNoon),
                ("雨天", carla.WeatherParameters.HardRainNoon),
                ("日落", carla.WeatherParameters.ClearSunset)]:
    world.set_weather(w)
    print(f"  天气: {name}")
    time.sleep(2)
world.set_weather(carla.WeatherParameters.ClearNoon)

# === 2. 交通生成 ===
print("\n[2/5] 交通系统")
spawns = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
vehicles = []
for sp in spawns[:8]:
    v = world.try_spawn_actor(bp_lib.filter('vehicle.*')[len(vehicles)], sp)
    if v:
        v.set_autopilot(True)
        vehicles.append(v)
print(f"  已生成 {len(vehicles)} 辆车辆")

# === 3. 地面传感器 ===
print("\n[3/5] 地面传感器")
cam_bp = bp_lib.find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '1920')
cam_bp.set_attribute('image_size_y', '1080')
cam = world.spawn_actor(cam_bp,
    carla.Transform(carla.Location(x=80, y=30, z=25), carla.Rotation(pitch=-30)))
import queue
q = queue.Queue(10)
cam.listen(lambda img: q.put(img) if not q.full() else None)
time.sleep(1)
try:
    img = q.get(timeout=3)
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:,:,:3]
    cv2.imwrite('/tmp/showcase_ground.png', arr)
    print("  地面相机: 已保存 /tmp/showcase_ground.png")
except: pass
cam.stop(); cam.destroy()

# === 4. 无人机飞行 ===
print("\n[4/5] 无人机飞行")
air.takeoffAsync().join()
air.moveToPositionAsync(80, 0, -30, 5).join()
print("  已飞到城市中心上空 30m")

responses = air.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
if responses[0].height > 0:
    img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    frame = img.reshape(responses[0].height, responses[0].width, 3)
    cv2.imwrite('/tmp/showcase_aerial.png', frame)
    print("  航拍相机: 已保存 /tmp/showcase_aerial.png")

# === 5. 联合巡航 ===
print("\n[5/5] 联合巡航 (10秒)")
for i in range(20):
    t = i / 20 * 2 * math.pi
    air.moveToPositionAsync(80 + 12*math.cos(t), 12*math.sin(t), -30, 8)
    time.sleep(0.5)

# === 清理 ===
print("\n清理中...")
air.landAsync().join()
air.armDisarm(False)
air.enableApiControl(False)
for v in vehicles: v.destroy()

print("\n" + "=" * 50)
print("展示完成! 核心功能:")
print("  - CARLA 天气系统")
print("  - CARLA 交通管理")
print("  - CARLA 传感器采集")
print("  - AirSim 无人机飞行")
print("  - 地面+空中联合仿真")
print("=" * 50)
```

---

## 10. 下一步

### 更多示例脚本

完整的示例脚本集合在 `source/python_api/examples/` 目录中，包括：

| 脚本 | 功能 |
|------|------|
| `fly_drone_keyboard.py` | 键盘实时控制无人机 |
| `drone_car_chase.py` | 无人机追踪地面车辆 |
| `aerial_surveillance.py` | 无人机航拍巡逻 |
| `data_collector.py` | 多模态数据采集 |
| `showcase_traffic.py` | 大规模交通展示 |

### 录制演示视频

```bash
# 运行 13 个自动录制 Demo
cd /mnt/data1/tianle/carla_source
bash DEMO/scripts/run_all_v2.sh
```

### CARLA Python API 参考

- 官方文档: https://carla.readthedocs.io/en/0.9.16/
- 核心类: `carla.Client`, `carla.World`, `carla.Actor`, `carla.Transform`

### AirSim Python API 参考

- 官方文档: https://microsoft.github.io/AirSim/
- 核心类: `airsim.MultirotorClient`, `airsim.ImageRequest`

### 常见问题

| 问题 | 解决方案 |
|------|---------|
| CARLA 连接超时 | 等待服务器启动完成（首次 2-5 分钟） |
| AirSim 连接失败 | 检查 `~/Documents/AirSim/settings.json` |
| 车辆穿过地面 | 这个 Bug 已修复，确保使用最新构建 |
| Walker AI 崩溃 | 不要对 Walker 使用 `go_to_location()`，保持静态 |
| 10+ 车辆 + 无人机崩溃 | 使用 `try_spawn_actor`，限制 autopilot 车辆 ≤8 |
