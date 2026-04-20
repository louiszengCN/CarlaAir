#!/usr/bin/env python3
"""CarlaAir integration test — connects to a running CARLA server and exercises the API."""

from __future__ import annotations

import argparse
import random
import sys
import time

import carla


def pass_(msg: str) -> None:
    print(f"  PASS  {msg}")


def fail(msg: str) -> None:
    print(f"  FAIL  {msg}", file=sys.stderr)


def poll_until(predicate, timeout: float, interval: float = 0.1):
    deadline = time.time() + timeout
    last_value = None
    while time.time() < deadline:
        last_value = predicate()
        if last_value:
            return last_value
        time.sleep(interval)
    return last_value


def wait_for_value(getter, is_ready, timeout: float, interval: float = 0.1):
    deadline = time.time() + timeout
    last_value = getter()
    while time.time() < deadline:
        last_value = getter()
        if is_ready(last_value):
            return last_value
        time.sleep(interval)
    return last_value


def lifted_spawn_point(transform: carla.Transform, lift_z: float = 0.15) -> carla.Transform:
    return carla.Transform(
        transform.location + carla.Location(z=lift_z),
        transform.rotation,
    )


def run(host: str, port: int, timeout: float) -> int:
    errors = 0

    # ── 1. Connect ────────────────────────────────────────────────────────────
    print("\n[1] Connecting to CARLA server...")
    try:
        client = carla.Client(host, port)
        client.set_timeout(timeout)
        server_ver = client.get_server_version()
        client_ver = client.get_client_version()
        pass_(f"Connected  server={server_ver}  client={client_ver}")
    except Exception as e:
        fail(f"Connection failed: {e}")
        return 1

    # ── 2. Load a proper map ──────────────────────────────────────────────────
    print("\n[2] Load Town01_Opt...")
    try:
        world = client.get_world()
        current_name = world.get_map().name
    except Exception:
        current_name = ""
    try:
        if "Town01" not in current_name:
            print("      Loading Town01_Opt (may take ~60s)...")
            client.set_timeout(120.0)
            world = client.load_world("Town01_Opt")
            client.set_timeout(timeout)
            time.sleep(3)
        map_ = world.get_map()
        pass_(f"World OK  map={map_.name}")
    except Exception as e:
        fail(f"load_world/get_map: {e}")
        errors += 1
        return errors

    # ── 3. Blueprint library ──────────────────────────────────────────────────
    print("\n[3] Blueprint library...")
    vehicles: list[carla.ActorBlueprint] = []
    sensors: list[carla.ActorBlueprint] = []
    walkers: list[carla.ActorBlueprint] = []
    try:
        bplib = world.get_blueprint_library()
        vehicles = list(bplib.filter("vehicle.*"))
        sensors = list(bplib.filter("sensor.*"))
        walkers = list(bplib.filter("walker.*"))
        pass_(f"vehicles={len(vehicles)}  sensors={len(sensors)}  walkers={len(walkers)}")
        if not vehicles:
            print("  WARN  No vehicle blueprints (BP_VehicleFactory asset needs resave in editor)")
        if len(sensors) < 5:
            fail(f"Expected >=5 sensor blueprints, got {len(sensors)}")
            errors += 1
    except Exception as e:
        fail(f"blueprint_library: {e}")
        errors += 1

    # ── 4. Spawn points ───────────────────────────────────────────────────────
    print("\n[4] Spawn points...")
    spawn_points: list[carla.Transform] = []
    try:
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            fail("No spawn points — map may not be loaded correctly")
            errors += 1
        else:
            pass_(f"{len(spawn_points)} spawn points available")
    except Exception as e:
        fail(f"get_spawn_points: {e}")
        errors += 1

    # ── 5. Spawn & destroy a vehicle ──────────────────────────────────────────
    print("\n[5] Spawn and destroy a vehicle...")
    spawned_vehicle: carla.Actor | None = None
    if not vehicles:
        print("  SKIP  No vehicle blueprints available (see warning above)")
    else:
        try:
            PREFERRED = [
                "vehicle.ue4.bmw.grantourer",
                "vehicle.ue4.audi.tt",
                "vehicle.ue4.chevrolet.impala",
                "vehicle.ue4.mercedes.ccc",
            ]
            by_id = {b.id: b for b in vehicles}
            bp = next((by_id[pid] for pid in PREFERRED if pid in by_id), None)
            if bp is None:
                bp = random.choice(vehicles)
            transform = spawn_points[0] if spawn_points else carla.Transform()
            spawned_vehicle = world.try_spawn_actor(bp, transform)
            if spawned_vehicle is None:
                fail("try_spawn_actor returned None (spawn point blocked?)")
                errors += 1
            else:
                loc = spawned_vehicle.get_location()
                pass_(f"Spawned {bp.id} at ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
        except Exception as e:
            fail(f"spawn vehicle: {e}")
            errors += 1
        finally:
            if spawned_vehicle is not None:
                spawned_vehicle.destroy()
                pass_("Vehicle destroyed OK")

    # ── 6. Weather ────────────────────────────────────────────────────────────
    print("\n[6] Weather...")
    try:
        original_weather = world.get_weather()
        weather = world.get_weather()
        weather.cloudiness = 50.0
        weather.precipitation = 10.0
        world.set_weather(weather)
        readback = wait_for_value(
            getter=world.get_weather,
            is_ready=lambda current: (
                abs(current.cloudiness - 50.0) <= 0.5
                and abs(current.precipitation - 10.0) <= 0.5
            ),
            timeout=3.0,
            interval=0.2,
        )
        cloudiness_ok = abs(readback.cloudiness - 50.0) <= 0.5
        precipitation_ok = abs(readback.precipitation - 10.0) <= 0.5
        weather_changed = (
            abs(readback.cloudiness - original_weather.cloudiness) > 0.1
            or abs(readback.precipitation - original_weather.precipitation) > 0.1
        )
        if not cloudiness_ok or not precipitation_ok or not weather_changed:
            fail(
                "Weather readback unexpected: "
                f"cloudiness={readback.cloudiness} precipitation={readback.precipitation} "
                f"original_cloudiness={original_weather.cloudiness} "
                f"original_precipitation={original_weather.precipitation}"
            )
            errors += 1
        else:
            pass_(f"Weather set+readback OK  cloudiness={readback.cloudiness}  precipitation={readback.precipitation}")
    except Exception as e:
        fail(f"weather: {e}")
        errors += 1

    # ── 7. Tick (synchronous mode) ────────────────────────────────────────────
    print("\n[7] Synchronous tick...")
    frame = 0
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        for _ in range(5):
            frame = world.tick()
        if frame == 0:
            fail("tick() returned 0 — synchronous mode may not be working")
            errors += 1
        else:
            pass_(f"5 ticks OK, last frame={frame}")
    except Exception as e:
        fail(f"synchronous tick: {e}")
        errors += 1
    finally:
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except Exception:
            pass

    # ── 8. Actor list ─────────────────────────────────────────────────────────
    print("\n[8] Actor list...")
    try:
        actors = world.get_actors()
        actor_list = list(actors)
        pass_(f"{len(actor_list)} actors in world")
        if len(actor_list) == 0:
            fail("World has no actors — something is wrong")
            errors += 1
    except Exception as e:
        fail(f"get_actors: {e}")
        errors += 1

    # ── 9. Waypoints ─────────────────────────────────────────────────────────
    print("\n[9] Waypoints...")
    try:
        if spawn_points:
            wp = map_.get_waypoint(spawn_points[0].location)
            if wp is None:
                fail("get_waypoint returned None at first spawn point")
                errors += 1
            else:
                pass_(f"Waypoint OK  road_id={wp.road_id}  lane_id={wp.lane_id}")
        else:
            print("  SKIP  No spawn points to test waypoints")
    except Exception as e:
        fail(f"waypoints: {e}")
        errors += 1

    # ── 10. Traffic lights ────────────────────────────────────────────────────
    print("\n[10] Traffic lights...")
    try:
        tls = list(world.get_actors().filter("traffic.traffic_light"))
        if len(tls) == 0:
            # Town01 has traffic lights; absence is unexpected but not fatal
            print(f"  WARN  No traffic lights found in {map_.name}")
        else:
            tl = tls[0]
            state = tl.get_state()
            pass_(f"{len(tls)} traffic lights  first_state={state}")
    except Exception as e:
        fail(f"traffic_lights: {e}")
        errors += 1

    # ── 11. Sensor spawn ──────────────────────────────────────────────────────
    print("\n[11] Sensor spawn (RGB camera)...")
    sensor_bp_list = [b for b in sensors if b.id == "sensor.camera.rgb"]
    if not sensor_bp_list:
        print("  SKIP  sensor.camera.rgb blueprint not found")
    else:
        sensor_actor: carla.Actor | None = None
        try:
            sensor_bp = sensor_bp_list[0]
            sensor_bp.set_attribute("image_size_x", "64")
            sensor_bp.set_attribute("image_size_y", "64")
            transform = spawn_points[0] if spawn_points else carla.Transform()
            sensor_actor = world.try_spawn_actor(sensor_bp, transform)
            if sensor_actor is None:
                fail("try_spawn_actor returned None for RGB camera")
                errors += 1
            else:
                pass_(f"RGB camera spawned id={sensor_actor.id}")
        except Exception as e:
            fail(f"sensor spawn: {e}")
            errors += 1
        finally:
            if sensor_actor is not None:
                sensor_actor.destroy()
                pass_("Sensor destroyed OK")

    # ── 12. Vehicle physics control ───────────────────────────────────────────
    print("\n[12] Vehicle physics control...")
    if not vehicles or not spawn_points:
        print("  SKIP  No vehicles or spawn points")
    else:
        ctrl_vehicle: carla.Actor | None = None
        original_settings = None
        try:
            PREFERRED = [
                "vehicle.ue4.bmw.grantourer",
                "vehicle.ue4.audi.tt",
                "vehicle.ue4.chevrolet.impala",
                "vehicle.ue4.mercedes.ccc",
            ]
            by_id = {b.id: b for b in vehicles}
            bp = next((by_id[pid] for pid in PREFERRED if pid in by_id), None)
            if bp is None:
                bp = random.choice(vehicles)
            base_spawn = spawn_points[0]
            sp = lifted_spawn_point(base_spawn)
            ctrl_vehicle = world.try_spawn_actor(bp, sp)
            if ctrl_vehicle is None:
                fail("try_spawn_actor returned None for control test")
                errors += 1
            else:
                original_settings = world.get_settings()
                sync_settings = world.get_settings()
                sync_settings.synchronous_mode = True
                sync_settings.fixed_delta_seconds = 0.05
                world.apply_settings(sync_settings)
                ctrl = carla.VehicleControl()
                ctrl.throttle = 0.5
                ctrl.steer = 0.1
                # Give the vehicle a few frames to settle on the road before checking control.
                for _ in range(10):
                    world.tick()
                for _ in range(20):
                    ctrl_vehicle.apply_control(ctrl)
                    world.tick()
                readback = wait_for_value(
                    getter=ctrl_vehicle.get_control,
                    is_ready=lambda current: abs(current.throttle - 0.5) <= 0.01,
                    timeout=2.0,
                    interval=0.1,
                )
                velocity = ctrl_vehicle.get_velocity()
                horizontal_speed = abs(velocity.x) + abs(velocity.y)
                if abs(readback.throttle - 0.5) > 0.01 or horizontal_speed <= 0.01:
                    fail(
                        "Control application mismatch: "
                        f"throttle={readback.throttle} steer={readback.steer} "
                        f"velocity=({velocity.x:.3f}, {velocity.y:.3f}, {velocity.z:.3f}) "
                        f"spawn=({sp.location.x:.3f}, {sp.location.y:.3f}, {sp.location.z:.3f})"
                    )
                    errors += 1
                else:
                    pass_(
                        "Vehicle control applied+readback OK  "
                        f"throttle={readback.throttle:.2f}  steer={readback.steer:.2f}  "
                        f"speed=({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f})"
                    )
        except Exception as e:
            fail(f"vehicle control: {e}")
            errors += 1
        finally:
            if original_settings is not None:
                try:
                    world.apply_settings(original_settings)
                except Exception:
                    pass
            if ctrl_vehicle is not None:
                ctrl_vehicle.destroy()
                pass_("Control vehicle destroyed OK")

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    if errors == 0:
        print("=== ALL TESTS PASSED ===")
        return 0
    else:
        print(f"=== {errors} TEST(S) FAILED ===", file=sys.stderr)
        return 1


def main() -> None:
    parser = argparse.ArgumentParser(description="CarlaAir integration test")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    sys.exit(run(args.host, args.port, args.timeout))


if __name__ == "__main__":
    main()
