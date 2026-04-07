#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 1: 连接验证"""
import airsim

import carla

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

weather = world.get_weather()
print(f"当前天气: 太阳高度={weather.sun_altitude_angle:.1f}°, 云量={weather.cloudiness:.1f}%")
print("\nCarlaAir 已就绪！地面 + 空中 API 均可用。")
