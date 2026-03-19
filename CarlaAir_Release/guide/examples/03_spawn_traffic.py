#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 3: 生成交通"""
import carla
import random
import time

client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# 在城市中心附近生成车辆 (x > 55, 远离海岸)
spawn_points = world.get_map().get_spawn_points()
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

# 生成静态行人
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

print("观察交通运行... (20秒)")
time.sleep(20)

for v in vehicles: v.destroy()
for w in walkers: w.destroy()
print("清理完成")
