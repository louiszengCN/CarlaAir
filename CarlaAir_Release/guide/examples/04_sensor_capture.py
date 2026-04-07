#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 4: 传感器采集"""
import queue
import time

import cv2
import numpy as np

import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# 生成带传感器的车辆
spawn_points = world.get_map().get_spawn_points()
vehicle = world.spawn_actor(
    bp_lib.find('vehicle.tesla.model3'),
    [sp for sp in spawn_points if sp.location.x > 55][0]
)
vehicle.set_autopilot(True)

# 安装 RGB 相机
cam_bp = bp_lib.find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '1280')
cam_bp.set_attribute('image_size_y', '720')
cam_bp.set_attribute('fov', '100')
camera = world.spawn_actor(cam_bp,
    carla.Transform(carla.Location(x=1.5, z=2.0)), attach_to=vehicle)

img_queue = queue.Queue(10)
camera.listen(lambda img: img_queue.put(img) if not img_queue.full() else None)

print("采集 5 帧图像...")
time.sleep(2)

for i in range(5):
    time.sleep(0.5)
    try:
        img = img_queue.get(timeout=2)
        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
        frame = arr.reshape(img.height, img.width, 4)[:, :, :3]
        cv2.imwrite(f'/tmp/carla_frame_{i}.png', frame)
        print(f"  保存帧 {i}: {img.width}x{img.height}")
    except queue.Empty:
        print(f"  帧 {i}: 超时")

camera.stop(); camera.destroy(); vehicle.destroy()
print("采集完成! 图像: /tmp/carla_frame_*.png")
