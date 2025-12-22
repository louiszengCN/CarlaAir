#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 7: 空地联合仿真"""
import carla
import airsim
import time
import math

# 连接双 API
client_carla = carla.Client('localhost', 2000)
client_carla.set_timeout(10)
world = client_carla.get_world()

client_air = airsim.MultirotorClient(port=41451)
client_air.confirmConnection()
client_air.enableApiControl(True)
client_air.armDisarm(True)
print("双 API 连接成功!")

# CARLA: 天气 + 车辆
world.set_weather(carla.WeatherParameters(sun_altitude_angle=45, cloudiness=20))
bp_lib = world.get_blueprint_library()
spawns = [sp for sp in world.get_map().get_spawn_points() if sp.location.x > 55]

vehicles = []
for sp in spawns[:8]:
    v = world.try_spawn_actor(bp_lib.filter('vehicle.*')[len(vehicles) % 20], sp)
    if v:
        v.set_autopilot(True)
        vehicles.append(v)
print(f"CARLA: {len(vehicles)} 辆车辆")

# AirSim: 无人机
client_air.takeoffAsync().join()
client_air.moveToPositionAsync(80, 0, -30, 5).join()
print("AirSim: 无人机就位")

# 联合运行: 无人机绕城市飞行 + 地面交通
print("\n联合仿真运行 15 秒...")
for i in range(30):
    t = i / 30 * 2 * math.pi
    client_air.moveToPositionAsync(80 + 15*math.cos(t), 15*math.sin(t), -30, 8)
    if i % 10 == 0:
        pos = client_air.getMultirotorState().kinematics_estimated.position
        print(f"  [{i//2}s] 无人机: ({pos.x_val:.0f},{pos.y_val:.0f},{pos.z_val:.0f})")
    time.sleep(0.5)

# 清理
client_air.landAsync().join()
client_air.armDisarm(False)
client_air.enableApiControl(False)
for v in vehicles: v.destroy()
print("联合仿真演示完成!")
