#!/usr/bin/env python3
"""
auto_traffic.py — CarlaAir 自动交通生成器 (v0.1.7)
启动后在异步模式下生成车辆和行人，保持后台运行。
定时健康检查：冻结的行人自动重启，车辆自动恢复。
Ctrl+C 或 SIGTERM 时自动清理所有生成的 actor。

Usage:
    python3 auto_traffic.py [--vehicles 30] [--walkers 50] [--port 2000]
"""

import carla
import random
import time
import signal
import sys
import argparse
import logging

logging.basicConfig(format='[Traffic] %(message)s', level=logging.INFO)
log = logging.info

# ── Globals for cleanup ──
vehicles_list = []
walkers_list = []   # [{id, con}, ...]
all_walker_ids = []
client = None
world = None
running = True


def signal_handler(sig, frame):
    global running
    log("Received shutdown signal, cleaning up...")
    running = False


def spawn_vehicles(world, client, count=30, tm_port=8000):
    """Spawn vehicles with autopilot via Traffic Manager (async mode)."""
    bp_lib = world.get_blueprint_library()
    vehicle_bps = bp_lib.filter('vehicle.*')
    car_bps = [bp for bp in vehicle_bps if int(bp.get_attribute('number_of_wheels')) == 4]
    if not car_bps:
        car_bps = list(vehicle_bps)

    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    tm = client.get_trafficmanager(tm_port)
    tm.set_global_distance_to_leading_vehicle(2.5)
    tm.global_percentage_speed_difference(10.0)

    batch = []
    for sp in spawn_points[:count]:
        bp = random.choice(car_bps)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        bp.set_attribute('role_name', 'autopilot')
        batch.append(
            carla.command.SpawnActor(bp, sp)
            .then(carla.command.SetAutopilot(carla.command.FutureActor, True, tm.get_port()))
        )

    results = client.apply_batch_sync(batch, False)
    spawned = 0
    for r in results:
        if not r.error:
            vehicles_list.append(r.actor_id)
            spawned += 1

    log(f"Vehicles: {spawned}/{count} spawned")
    return spawned


def spawn_walkers(world, client, count=50):
    """Spawn pedestrians with AI controllers (async mode)."""
    bp_lib = world.get_blueprint_library()
    walker_bps = bp_lib.filter('walker.pedestrian.*')
    controller_bp = bp_lib.find('controller.ai.walker')

    spawn_points = []
    for _ in range(count * 3):
        loc = world.get_random_location_from_navigation()
        if loc:
            spawn_points.append(carla.Transform(loc))
        if len(spawn_points) >= count:
            break

    batch = []
    speeds = []
    for sp in spawn_points[:count]:
        bp = random.choice(walker_bps)
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'false')
        if bp.has_attribute('speed'):
            if random.random() < 0.15:
                speeds.append(float(bp.get_attribute('speed').recommended_values[2]))
            else:
                speeds.append(float(bp.get_attribute('speed').recommended_values[1]))
        else:
            speeds.append(1.4)
        batch.append(carla.command.SpawnActor(bp, sp))

    results = client.apply_batch_sync(batch, False)
    valid_speeds = []
    for i, r in enumerate(results):
        if not r.error:
            walkers_list.append({"id": r.actor_id})
            valid_speeds.append(speeds[i])

    batch_ctrl = []
    for w in walkers_list:
        batch_ctrl.append(
            carla.command.SpawnActor(controller_bp, carla.Transform(), w["id"])
        )
    ctrl_results = client.apply_batch_sync(batch_ctrl, False)
    for i, r in enumerate(ctrl_results):
        walkers_list[i]["con"] = r.actor_id if not r.error else None

    for w in walkers_list:
        if w.get("con"):
            all_walker_ids.append(w["con"])
        all_walker_ids.append(w["id"])

    world.wait_for_tick()

    world.set_pedestrians_cross_factor(0.05)
    all_actors = world.get_actors(all_walker_ids)
    for i in range(0, len(all_walker_ids), 2):
        try:
            all_actors[i].start()
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            all_actors[i].set_max_speed(valid_speeds[i // 2] if i // 2 < len(valid_speeds) else 1.4)
        except Exception:
            pass

    log(f"Walkers: {len(walkers_list)}/{count} spawned with AI controllers")
    return len(walkers_list)


def health_check_walkers(world):
    """Reassign destinations to all walkers. Fixes frozen walkers."""
    if not all_walker_ids:
        return
    try:
        actors = world.get_actors(all_walker_ids)
        reassigned = 0
        for i in range(0, len(all_walker_ids), 2):
            try:
                ctrl = actors[i]
                loc = world.get_random_location_from_navigation()
                if loc and ctrl.is_alive:
                    ctrl.go_to_location(loc)
                    reassigned += 1
            except Exception:
                pass
        return reassigned
    except Exception:
        return 0


def health_check_vehicles(world, client, tm_port=8000):
    """Re-enable autopilot on any vehicle that lost it."""
    if not vehicles_list:
        return
    try:
        actors = world.get_actors(vehicles_list)
        restarted = 0
        for a in actors:
            try:
                vel = a.get_velocity()
                speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
                # If vehicle is nearly stopped and not at a traffic light, re-enable autopilot
                if speed < 0.01:
                    a.set_autopilot(True, tm_port)
                    restarted += 1
            except Exception:
                pass
        return restarted
    except Exception:
        return 0


def cleanup():
    """Destroy all spawned actors."""
    global client, vehicles_list, walkers_list, all_walker_ids

    if not client:
        return

    try:
        if all_walker_ids:
            all_actors = world.get_actors(all_walker_ids)
            for i in range(0, len(all_walker_ids), 2):
                try:
                    all_actors[i].stop()
                except Exception:
                    pass

        if vehicles_list:
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
            log(f"Destroyed {len(vehicles_list)} vehicles")

        if all_walker_ids:
            client.apply_batch([carla.command.DestroyActor(x) for x in all_walker_ids])
            log(f"Destroyed {len(walkers_list)} walkers")
    except Exception as e:
        log(f"Cleanup warning: {e}")


def main():
    global client, world, running

    parser = argparse.ArgumentParser(description='CarlaAir Auto Traffic Generator')
    parser.add_argument('--vehicles', '-n', type=int, default=10)
    parser.add_argument('--walkers', '-w', type=int, default=10)
    parser.add_argument('--port', '-p', type=int, default=2000)
    parser.add_argument('--tm-port', type=int, default=8000)
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Retry connection (CarlaAir may still be starting)
    for attempt in range(10):
        try:
            log(f"Connecting to CARLA (port {args.port})...")
            client = carla.Client('localhost', args.port)
            client.set_timeout(20.0)
            world = client.get_world()
            break
        except Exception as e:
            if attempt < 9:
                log(f"  Waiting for CARLA... ({e})")
                time.sleep(5)
            else:
                log(f"Failed to connect after 10 attempts.")
                sys.exit(1)

    map_name = world.get_map().name.split('/')[-1]
    log(f"Connected: {map_name}")

    # Ensure async mode
    settings = world.get_settings()
    if settings.synchronous_mode:
        log("WARNING: World is in sync mode, switching to async for traffic")
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.0
        world.apply_settings(settings)

    # Spawn traffic
    nv = spawn_vehicles(world, client, count=args.vehicles, tm_port=args.tm_port)
    nw = spawn_walkers(world, client, count=args.walkers)
    log(f"Traffic ready: {nv} vehicles + {nw} walkers on {map_name}")
    log("Running in background. Health checks every 10s.")

    # Keep alive with periodic health checks
    last_health = time.time()
    HEALTH_INTERVAL = 10.0  # seconds

    while running:
        try:
            time.sleep(0.5)

            now = time.time()
            if now - last_health >= HEALTH_INTERVAL:
                last_health = now
                rw = health_check_walkers(world)
                rv = health_check_vehicles(world, client, args.tm_port)
                # Only log if something was fixed
                if rv and rv > 5:
                    log(f"Health: restarted {rv} stalled vehicles")

        except RuntimeError:
            log("Connection interrupted, waiting to reconnect...")
            time.sleep(5)
            try:
                world = client.get_world()
                log(f"Reconnected: {world.get_map().name}")
            except Exception:
                pass
        except Exception as e:
            if running:
                log(f"Error: {e}")
                time.sleep(2)

    cleanup()
    log("Traffic generator stopped.")


if __name__ == '__main__':
    main()
