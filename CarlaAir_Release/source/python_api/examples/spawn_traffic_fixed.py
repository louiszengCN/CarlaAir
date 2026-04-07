#!/usr/bin/env python3
"""
spawn_traffic_fixed.py — Spawn CARLA traffic that stays on the road surface.

Workaround for CarlaAir: vehicles fall through ground due to physics/collision issue.
This script continuously fixes vehicle Z positions to keep them on the road.

Usage: python spawn_traffic_fixed.py [num_vehicles]
Press Ctrl+C to stop and destroy all vehicles.
"""

import math
import random
import sys
import time

import carla

# City center spawn point indices (inland, x > 50)
CENTRAL_SPAWN_INDICES = [54, 53, 133, 121, 123, 41, 42, 81, 135, 128, 92, 152, 35]

NUM_VEHICLES = int(sys.argv[1]) if len(sys.argv) > 1 else 25


def get_inland_spawn_points(world, count):
    """Get spawn points sorted by distance to city center, inland only (x > 30)."""
    all_sps = world.get_map().get_spawn_points()

    # First use known good central indices
    result = []
    for idx in CENTRAL_SPAWN_INDICES:
        if idx < len(all_sps):
            result.append(all_sps[idx])
        if len(result) >= count:
            return result

    # Then add more inland points sorted by distance to city center
    used = set(CENTRAL_SPAWN_INDICES[:len(result)])
    remaining = [(i, sp) for i, sp in enumerate(all_sps)
                 if i not in used and sp.location.x > 30]
    remaining.sort(key=lambda x: math.sqrt(
        (x[1].location.x - 80)**2 + (x[1].location.y - 30)**2))

    for idx, sp in remaining:
        result.append(sp)
        if len(result) >= count:
            break

    return result


def main():
    print(f"Spawning {NUM_VEHICLES} vehicles at city center...")

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # Clean existing vehicles
    existing = world.get_actors().filter('vehicle.*')
    if len(existing) > 0:
        print(f"  Destroying {len(existing)} old vehicles...")
        for v in existing:
            try: v.destroy()
            except: pass
        time.sleep(1)

    # Get inland spawn points
    spawn_points = get_inland_spawn_points(world, NUM_VEHICLES)
    print(f"  Using {len(spawn_points)} inland spawn points")

    # Spawn vehicles
    vehicle_bps = list(bp_lib.filter('vehicle.*'))
    vehicles = []
    spawn_zs = {}  # Store original spawn Z for each vehicle

    for sp in spawn_points:
        bp = random.choice(vehicle_bps)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        v = world.try_spawn_actor(bp, sp)
        if v:
            vehicles.append(v)
            spawn_zs[v.id] = sp.location.z

    print(f"  Spawned {len(vehicles)} vehicles")

    # Enable autopilot (do separately to avoid quaternion crash)
    enabled = 0
    for v in vehicles:
        try:
            v.set_autopilot(True)
            enabled += 1
            time.sleep(0.05)
        except:
            pass
    print(f"  Autopilot enabled: {enabled}/{len(vehicles)}")

    print("\nTraffic running! Fixing Z positions to keep vehicles on roads...")
    print("Press Ctrl+C to stop.\n")

    fix_count = 0
    try:
        while True:
            alive = []
            for v in vehicles:
                try:
                    t = v.get_transform()
                    target_z = spawn_zs.get(v.id, 0.6)

                    # If vehicle fell below road (z < target_z - 2), fix it
                    if t.location.z < target_z - 2:
                        t.location.z = target_z + 0.5
                        v.set_transform(t)
                        fix_count += 1

                    alive.append(v)
                except:
                    pass  # Vehicle destroyed or invalid

            vehicles = alive

            # Status every 5 seconds
            moving = sum(1 for v in vehicles
                        if (v.get_velocity().x**2 + v.get_velocity().y**2)**0.5 > 0.5)
            sys.stdout.write(
                f"\r  Vehicles: {len(vehicles)} | Moving: {moving} | Z-fixes: {fix_count}    ")
            sys.stdout.flush()

            time.sleep(0.2)  # Fix Z 5 times per second

    except KeyboardInterrupt:
        print(f"\n\nDestroying {len(vehicles)} vehicles...")
        for v in vehicles:
            try: v.destroy()
            except: pass
        print("Done.")


if __name__ == '__main__':
    main()
