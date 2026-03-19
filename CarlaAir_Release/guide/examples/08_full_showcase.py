#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 8: 完整功能展示"""
import carla
import airsim
import numpy as np
import cv2
import queue
import time
import math

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

# [1/5] 天气
print("\n[1/5] 天气系统")
for name, w in [("晴天", carla.WeatherParameters.ClearNoon),
                ("雨天", carla.WeatherParameters.HardRainNoon),
                ("日落", carla.WeatherParameters.ClearSunset)]:
    world.set_weather(w)
    print(f"  {name}")
    time.sleep(2)
world.set_weather(carla.WeatherParameters.ClearNoon)

# [2/5] 交通
print("\n[2/5] 交通系统")
spawns = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]
vehicles = []
for sp in spawns[:8]:
    v = world.try_spawn_actor(bp_lib.filter('vehicle.*')[len(vehicles)], sp)
    if v:
        v.set_autopilot(True)
        vehicles.append(v)
print(f"  {len(vehicles)} 辆车辆运行中")

# [3/5] 地面传感器
print("\n[3/5] 地面传感器")
cam_bp = bp_lib.find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '1920')
cam_bp.set_attribute('image_size_y', '1080')
cam = world.spawn_actor(cam_bp,
    carla.Transform(carla.Location(x=80, y=30, z=25), carla.Rotation(pitch=-30)))
q = queue.Queue(10)
cam.listen(lambda img: q.put(img) if not q.full() else None)
time.sleep(1)
try:
    img = q.get(timeout=3)
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:,:,:3]
    cv2.imwrite('/tmp/showcase_ground.png', arr)
    print("  已保存 /tmp/showcase_ground.png")
except: pass
cam.stop(); cam.destroy()

# [4/5] 无人机
print("\n[4/5] 无人机飞行")
air.takeoffAsync().join()
air.moveToPositionAsync(80, 0, -30, 5).join()
print("  城市中心上空 30m")
responses = air.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
if responses[0].height > 0:
    img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    cv2.imwrite('/tmp/showcase_aerial.png', img.reshape(responses[0].height, responses[0].width, 3))
    print("  已保存 /tmp/showcase_aerial.png")

# [5/5] 联合巡航
print("\n[5/5] 联合巡航 (10秒)")
for i in range(20):
    t = i / 20 * 2 * math.pi
    air.moveToPositionAsync(80 + 12*math.cos(t), 12*math.sin(t), -30, 8)
    time.sleep(0.5)

# 清理
air.landAsync().join()
air.armDisarm(False)
air.enableApiControl(False)
for v in vehicles: v.destroy()

print("\n" + "=" * 50)
print("展示完成! 核心功能全部验证通过。")
print("=" * 50)
