#!/usr/bin/env python3
"""
drone_traj_playback_airsim.py — AirSim 轨迹回放 + CARLA 导演窗口

功能：
1) AirSim 无人机按轨迹飞行（后台线程独立连接，避免阻塞）
2) CARLA 导演视角（FPS 风格移动）
3) R 键在导演窗口中显示/隐藏轨迹（点 + 虚线）
4) 可选 CARLA 代理无人机，用于直观看到回放对象
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

try:
    import airsim
except ImportError:
    airsim = None

try:
    import carla
except ImportError:
    carla = None

try:
    import pygame
except ImportError:
    pygame = None


def lerp_angle(a0, a1, t):
    d = (a1 - a0 + 180.0) % 360.0 - 180.0
    return a0 + d * t


def parse_args():
    p = argparse.ArgumentParser(description="AirSim 轨迹回放 + CARLA 导演窗口")
    p.add_argument("traj", type=str, help="trajectory.json")
    p.add_argument("--airsim-port", type=int, default=41451)
    p.add_argument("--carla-host", type=str, default="localhost")
    p.add_argument("--carla-port", type=int, default=2000)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--velocity", type=float, default=10.0)
    p.add_argument("--interp", type=float, default=0.0, help=">0 时插值平滑")
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--no-proxy-drone", action="store_true", help="不生成 CARLA 代理无人机")
    p.add_argument("--vehicle-name", type=str, default="", help="AirSim vehicle_name (empty means default)")
    p.add_argument("--direct-carla-fly", action="store_true", help="directly move CARLA drone actor along trajectory")
    p.add_argument("--carla-drone-id", type=int, default=None, help="explicit CARLA drone actor id in direct mode")
    p.add_argument(
        "--mapping-file",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "drone_traj_output", "default_mapping.json"),
        help="mapping json from recorder; used to convert drone_pose -> airsim coords",
    )
    p.add_argument(
        "--coord-source",
        type=str,
        default="auto",
        choices=["auto", "airsim", "mapped"],
        help="use airsim_pose_ned directly, or map drone_pose with mapping-file",
    )
    return p.parse_args()


def find_carla_drone_actor(world, expected_loc=None):
    try:
        actors = world.get_actors()
    except Exception:
        return None
    cands = []
    for a in actors:
        tid = (getattr(a, "type_id", "") or "").lower()
        role = ""
        try:
            role = (a.attributes.get("role_name", "") or "").lower()
        except Exception:
            role = ""
        if ("drone" in tid) or ("drone" in role) or ("airsim" in role):
            cands.append(a)
    if not cands:
        return None
    if expected_loc is not None:
        def dist(a):
            try:
                l = a.get_transform().location
                return math.sqrt((l.x - expected_loc.x) ** 2 + (l.y - expected_loc.y) ** 2 + (l.z - expected_loc.z) ** 2)
            except Exception:
                return 1e9
        cands.sort(key=dist)
        return cands[0]
    cands.sort(key=lambda x: 0 if "drone" in ((x.attributes.get("role_name", "") or "").lower()) else 1)
    return cands[0]


def list_carla_drone_actors(world):
    out = []
    try:
        actors = world.get_actors()
    except Exception:
        return out
    for a in actors:
        tid = (getattr(a, "type_id", "") or "").lower()
        role = ""
        try:
            role = (a.attributes.get("role_name", "") or "")
        except Exception:
            role = ""
        if ("drone" in tid) or ("drone" in role.lower()) or ("airsim" in role.lower()):
            try:
                l = a.get_transform().location
                out.append((a, l, role))
            except Exception:
                pass
    return out


def draw_dashed_line(world, start, end, dash_length=0.8, gap_length=0.5,
                     color=None, thickness=0.05, life_time=0.5):
    if color is None:
        color = carla.Color(60, 120, 120)
    dist = start.distance(end)
    if dist <= 1e-3:
        return
    direction = (end - start) / dist
    step = dash_length + gap_length
    n = int(dist / step) + 1
    for i in range(n):
        p1 = start + direction * (i * step)
        p2 = p1 + direction * dash_length
        if p1.distance(start) > dist:
            break
        if p2.distance(start) > dist:
            p2 = end
        world.debug.draw_line(p1, p2, thickness=thickness, color=color, life_time=life_time)


def viz_traj(world, points, life_time=0.5, color=None):
    if color is None:
        color = carla.Color(0, 255, 255)
    prev = None
    for p in points:
        pose = p["drone_pose"]
        loc = carla.Location(float(pose["x"]), float(pose["y"]), float(pose["z"]))
        world.debug.draw_point(loc, size=0.14, color=color, life_time=life_time)
        if prev is not None:
            draw_dashed_line(world, prev, loc, life_time=life_time, color=color)
        prev = loc


def viz_polyline(world, locations, life_time=0.5, color=None):
    if color is None:
        color = carla.Color(255, 180, 0)
    prev = None
    for loc in locations:
        world.debug.draw_point(loc, size=0.11, color=color, life_time=life_time)
        if prev is not None:
            draw_dashed_line(world, prev, loc, life_time=life_time, color=color)
        prev = loc


def load_mapping(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            m = json.load(f)
        A = np.array(m["A"], dtype=float)
        off = np.array(m["offset"], dtype=float)
        z_sign = float(m.get("z_sign", -1.0))
        name = str(m.get("name", "xy"))
        return {"A": A, "offset": off, "z_sign": z_sign, "name": name}
    except Exception:
        return None


def map_drone_pose_to_airsim_ned(point, mapping):
    """Inverse mapping: CARLA drone_pose -> AirSim NED."""
    p = point["drone_pose"]
    cx, cy, cz = float(p["x"]), float(p["y"]), float(p["z"])
    if mapping is None:
        return cx, cy, -cz, float(p.get("yaw", 0.0))
    A = mapping["A"]
    off = mapping["offset"]
    z_sign = mapping["z_sign"]
    try:
        A_inv = np.linalg.inv(A)
        ax, ay = (A_inv @ np.array([cx - off[0], cy - off[1]], dtype=float)).tolist()
    except Exception:
        ax, ay = cx - off[0], cy - off[1]
    az = (cz - off[2]) / (z_sign if abs(z_sign) > 1e-6 else -1.0)
    return float(ax), float(ay), float(az), float(p.get("yaw", 0.0))


def map_airsim_ned_to_carla(ax, ay, az, mapping):
    """Forward mapping: AirSim NED -> CARLA xyz."""
    if mapping is None:
        return ax, ay, -az
    A = mapping["A"]
    off = mapping["offset"]
    z_sign = mapping["z_sign"]
    xy = A @ np.array([ax, ay], dtype=float)
    cz = z_sign * az
    return float(xy[0] + off[0]), float(xy[1] + off[1]), float(cz + off[2])


def point_to_airsim_ned(point, source_mode, mapping):
    """
    source_mode:
      - airsim: always use recorded airsim_pose_ned if available
      - mapped: always convert drone_pose via mapping
      - auto  : prefer mapping when point uses forced/manual map; else use airsim_pose_ned
    """
    if source_mode == "mapped":
        return map_drone_pose_to_airsim_ned(point, mapping)
    if source_mode == "airsim":
        if "airsim_pose_ned" in point:
            p = point["airsim_pose_ned"]
            return float(p["x"]), float(p["y"]), float(p["z"]), float(p.get("yaw", 0.0))
        if "airsim_pose" in point:
            p = point["airsim_pose"]
            return float(p["x"]), float(p["y"]), float(p["z"]), float(p.get("yaw", 0.0))
        return map_drone_pose_to_airsim_ned(point, mapping)
    # auto
    mi = point.get("mapping_info", {})
    mode = str(mi.get("mode", ""))
    if mode in ("forced_map", "manual_calibrated", "calibrated"):
        return map_drone_pose_to_airsim_ned(point, mapping)
    if "airsim_pose_ned" in point:
        p = point["airsim_pose_ned"]
        return float(p["x"]), float(p["y"]), float(p["z"]), float(p.get("yaw", 0.0))
    if "airsim_pose" in point:
        p = point["airsim_pose"]
        return float(p["x"]), float(p["y"]), float(p["z"]), float(p.get("yaw", 0.0))
    return map_drone_pose_to_airsim_ned(point, mapping)


def to_carla_tf(point):
    p = point["drone_pose"]
    return carla.Transform(
        carla.Location(float(p["x"]), float(p["y"]), float(p["z"])),
        carla.Rotation(float(p.get("pitch", 0.0)), float(p.get("yaw", 0.0)), float(p.get("roll", 0.0))),
    )


def run_airsim(points, args, running, proxy_state, mapping, telemetry):
    """后台线程：AirSim 无人机沿轨迹飞行。"""
    try:
        ac = airsim.MultirotorClient(port=args.airsim_port)
        ac.confirmConnection()
        try:
            if hasattr(ac, "listVehicles"):
                vs = ac.listVehicles()
                if vs:
                    print(f"[airsim] vehicles: {vs}")
        except Exception:
            pass
        ac.enableApiControl(True, vehicle_name=args.vehicle_name)
        ac.armDisarm(True, vehicle_name=args.vehicle_name)
        ac.takeoffAsync(vehicle_name=args.vehicle_name).join()
    except Exception as e:
        print(f"[drone_thread] startup failed: {e}")
        running[0] = False
        return

    velocity = max(1.0, args.velocity * args.speed)
    reach_dist = 2.5
    timeout_per_point = 45.0
    n = len(points)
    idx = 0

    def fly_to(px, py, pz, yaw):
        ac.moveToPositionAsync(
            px, py, pz, velocity,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(False, yaw),
            vehicle_name=args.vehicle_name,
        )
        t0 = time.time()
        while running[0] and time.time() - t0 < timeout_per_point:
            try:
                s = ac.getMultirotorState(vehicle_name=args.vehicle_name).kinematics_estimated.position
                with telemetry["lock"]:
                    telemetry["ned"] = (float(s.x_val), float(s.y_val), float(s.z_val))
                d = math.sqrt((px - s.x_val) ** 2 + (py - s.y_val) ** 2 + (pz - s.z_val) ** 2)
                if d < reach_dist:
                    break
            except Exception:
                pass
            time.sleep(0.15)

    try:
        # first point
        x0, y0, z0, yaw0 = point_to_airsim_ned(points[0], args.coord_source, mapping)
        try:
            s0 = ac.getMultirotorState(vehicle_name=args.vehicle_name).kinematics_estimated.position
            d0 = math.sqrt((x0 - s0.x_val) ** 2 + (y0 - s0.y_val) ** 2 + (z0 - s0.z_val) ** 2)
            print(f"[drone_thread] source={args.coord_source} first_target=({x0:.2f},{y0:.2f},{z0:.2f}) current=({s0.x_val:.2f},{s0.y_val:.2f},{s0.z_val:.2f}) dist={d0:.1f}m")
        except Exception:
            pass
        fly_to(x0, y0, z0, yaw0)
        with proxy_state["lock"]:
            proxy_state["idx"] = 0

        while running[0]:
            if idx >= n:
                if not args.loop:
                    break
                idx = 0
            x, y, z, yaw = point_to_airsim_ned(points[idx], args.coord_source, mapping)
            fly_to(x, y, z, yaw)
            with proxy_state["lock"]:
                proxy_state["idx"] = idx
            idx += 1

            if args.interp > 0 and idx < n:
                x1, y1, z1, yaw1 = point_to_airsim_ned(points[idx - 1], args.coord_source, mapping)
                x2, y2, z2, yaw2 = point_to_airsim_ned(points[idx], args.coord_source, mapping)
                dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
                steps = max(1, int(dist / args.interp))
                for s in range(1, steps):
                    if not running[0]:
                        break
                    t = s / steps
                    fly_to(
                        x1 + (x2 - x1) * t,
                        y1 + (y2 - y1) * t,
                        z1 + (z2 - z1) * t,
                        lerp_angle(yaw1, yaw2, t),
                    )
    except Exception as e:
        print(f"[drone_thread] runtime error: {e}")
    finally:
        try:
            ac.moveToZAsync(-3, 3, vehicle_name=args.vehicle_name).join()
            ac.landAsync(vehicle_name=args.vehicle_name).join()
            ac.armDisarm(False, vehicle_name=args.vehicle_name)
            ac.enableApiControl(False, vehicle_name=args.vehicle_name)
        except Exception:
            pass
        print("[drone_thread] finished")


def run_carla_direct(points, args, running, actor, planned_locs):
    """Directly fly a CARLA drone actor along planned CARLA trajectory."""
    if actor is None:
        print("[carla_direct] no drone actor available.")
        return
    n = len(points)
    if n == 0 or len(planned_locs) == 0:
        return
    idx = 0

    try:
        actor.set_simulate_physics(False)
    except Exception:
        pass

    speed = max(1.0, args.velocity * args.speed)
    dt = 0.03

    def smooth_move_to(target_loc, target_rot, speed_mps):
        try:
            cur_tf = actor.get_transform()
        except Exception:
            return
        cur = cur_tf.location
        dist = cur.distance(target_loc)
        if dist < 1e-3:
            actor.set_transform(carla.Transform(target_loc, target_rot))
            return
        steps = max(1, int(dist / max(0.05, speed_mps * dt)))
        for s in range(1, steps + 1):
            if not running[0]:
                return
            t = s / steps
            loc = carla.Location(
                x=cur.x + (target_loc.x - cur.x) * t,
                y=cur.y + (target_loc.y - cur.y) * t,
                z=cur.z + (target_loc.z - cur.z) * t,
            )
            rot = carla.Rotation(
                pitch=cur_tf.rotation.pitch + (target_rot.pitch - cur_tf.rotation.pitch) * t,
                yaw=lerp_angle(cur_tf.rotation.yaw, target_rot.yaw, t),
                roll=cur_tf.rotation.roll + (target_rot.roll - cur_tf.rotation.roll) * t,
            )
            actor.set_transform(carla.Transform(loc, rot))
            time.sleep(dt)
        # detect external override
        try:
            aft = actor.get_transform().location
            e = aft.distance(target_loc)
            if e > 3.0:
                print(f"[carla_direct][WARN] target not reached well (err={e:.2f}m), maybe overridden by external controller.")
        except Exception:
            pass

    try:
        # Preflight: ascend first, then go to first trajectory point
        tf0 = actor.get_transform()
        first = planned_locs[0]
        pre_alt = max(float(tf0.location.z), float(first.z)) + 8.0
        pre_loc = carla.Location(x=float(tf0.location.x), y=float(tf0.location.y), z=float(pre_alt))
        print(f"[carla_direct] preflight ascend to z={pre_alt:.1f}")
        smooth_move_to(pre_loc, tf0.rotation, speed_mps=max(2.0, speed * 0.8))

        p0 = points[0]["drone_pose"]
        first_rot = carla.Rotation(float(p0.get("pitch", 0.0)), float(p0.get("yaw", 0.0)), float(p0.get("roll", 0.0)))
        print(f"[carla_direct] move to first trajectory point ({first.x:.1f},{first.y:.1f},{first.z:.1f})")
        smooth_move_to(first, first_rot, speed_mps=speed)

        while running[0]:
            if idx >= n:
                if not args.loop:
                    break
                idx = 0
            p = points[idx]["drone_pose"]
            loc = planned_locs[idx]
            rot = carla.Rotation(float(p.get("pitch", 0.0)), float(p.get("yaw", 0.0)), float(p.get("roll", 0.0)))
            smooth_move_to(loc, rot, speed_mps=speed)
            idx += 1
    except Exception as e:
        print(f"[carla_direct] runtime error: {e}")
    finally:
        print("[carla_direct] finished")


def main():
    args = parse_args()
    if airsim is None:
        sys.exit("Need airsim")
    if carla is None:
        sys.exit("Need carla")
    if pygame is None:
        args.no_window = True

    with open(args.traj, "r", encoding="utf-8") as f:
        data = json.load(f)
    points = data.get("points", [])
    if not points:
        sys.exit("trajectory has no points")
    mapping = load_mapping(args.mapping_file)
    if mapping is not None:
        print(f"[mapping] loaded {args.mapping_file} name={mapping['name']}")
    else:
        print(f"[mapping] not found/invalid: {args.mapping_file}")

    # Build planned CARLA trajectory using calibrated mapping (for visual verification)
    planned_carla_locs = []
    for p in points:
        ax, ay, az, _ = point_to_airsim_ned(p, args.coord_source, mapping)
        cxp, cyp, czp = map_airsim_ned_to_carla(ax, ay, az, mapping)
        planned_carla_locs.append(carla.Location(cxp, cyp, czp))

    # CARLA director camera
    cc = carla.Client(args.carla_host, args.carla_port)
    cc.set_timeout(10.0)
    world = cc.get_world()
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    cam_w, cam_h = 1280, 720
    bp.set_attribute("image_size_x", str(cam_w))
    bp.set_attribute("image_size_y", str(cam_h))
    bp.set_attribute("fov", "90")

    p0 = points[0]["drone_pose"]
    cx = float(p0["x"]) - 15.0
    cy = float(p0["y"])
    cz = float(p0["z"]) + 25.0
    cam_yaw = 0.0
    cam_pitch = -45.0
    cam_speed = 25.0
    cam_alt = 15.0
    mouse_sens = 0.15

    cam = world.spawn_actor(
        bp,
        carla.Transform(
            carla.Location(cx, cy, cz),
            carla.Rotation(cam_pitch, cam_yaw, 0.0),
        ),
    )
    actors = [cam]
    frame = [None]
    lock = threading.Lock()

    def on_img(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, :3][:, :, ::-1]
        with lock:
            frame[0] = np.ascontiguousarray(arr)

    cam.listen(on_img)

    # Optional visible proxy drone in CARLA (disabled in direct mode)
    proxy_actor = None
    if (not args.no_proxy_drone) and (not args.direct_carla_fly):
        tf0 = to_carla_tf(points[0])
        lib = world.get_blueprint_library()
        for bp_id in ["static.prop.drone", "static.prop.dji_inspire"]:
            try:
                dbp = lib.find(bp_id)
                proxy_actor = world.try_spawn_actor(dbp, tf0)
                if proxy_actor is not None:
                    actors.append(proxy_actor)
                    print(f"[proxy] spawned {bp_id}")
                break
            except Exception:
                continue

    running = [True]
    proxy_state = {"idx": 0, "lock": threading.Lock()}
    telemetry = {"ned": None, "lock": threading.Lock()}
    controlled_actor = None
    if args.direct_carla_fly:
        candidates = list_carla_drone_actors(world)
        if candidates:
            print("[direct] candidate drone actors:")
            for a, l, role in candidates:
                print(f"  id={a.id} type={a.type_id} role='{role}' loc=({l.x:.1f},{l.y:.1f},{l.z:.1f})")
        if args.carla_drone_id is not None:
            controlled_actor = world.get_actor(int(args.carla_drone_id))
            if controlled_actor is None:
                print(f"[mode] --carla-drone-id={args.carla_drone_id} not found, fallback to auto pick.")
        if controlled_actor is None:
            controlled_actor = find_carla_drone_actor(world, expected_loc=planned_carla_locs[0])
        if controlled_actor is None:
            print("[mode] direct-carla-fly requested, but no CARLA drone actor found. fallback to AirSim mode.")
            t = threading.Thread(target=run_airsim, args=(points, args, running, proxy_state, mapping, telemetry), daemon=True)
        else:
            role = ""
            try:
                role = controlled_actor.attributes.get("role_name", "")
            except Exception:
                pass
            print(f"[mode] direct-carla-fly on actor id={controlled_actor.id} type={controlled_actor.type_id} role={role}")
            t = threading.Thread(target=run_carla_direct, args=(points, args, running, controlled_actor, planned_carla_locs), daemon=True)
    else:
        controlled_actor = proxy_actor if proxy_actor is not None else find_carla_drone_actor(world, expected_loc=planned_carla_locs[0])
        print(f"[mode] airsim-fly vehicle_name='{args.vehicle_name or '<default>'}'")
        t = threading.Thread(target=run_airsim, args=(points, args, running, proxy_state, mapping, telemetry), daemon=True)
    t.start()
    actual_trail = []
    last_actual = None

    show_traj = False
    shot = 0
    screen = None
    clock = None
    if not args.no_window:
        pygame.init()
        screen = pygame.display.set_mode((cam_w, cam_h))
        pygame.display.set_caption("Director FPS - WASD/EQ + Mouse, R=traj, C=screenshot, ESC=quit")
        pygame.mouse.set_visible(False)
        try:
            pygame.event.set_grab(True)
        except Exception:
            pass
        font = pygame.font.SysFont("monospace", 14)
        clock = pygame.time.Clock()

    print("[ready] playback running")
    print("[viz] R: toggle trajectory overlays (cyan=recorded drone_pose, yellow=planned mapped, magenta=actual flown)")
    try:
        while running[0]:
            dt = max(0.01, (clock.tick(30) / 1000.0) if clock else 0.05)

            if screen is not None:
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        running[0] = False
                    elif ev.type == pygame.KEYDOWN:
                        if ev.key == pygame.K_ESCAPE:
                            running[0] = False
                        elif ev.key == pygame.K_r:
                            show_traj = not show_traj
                            print(f"[viz] trajectory {'ON' if show_traj else 'OFF'}")
                        elif ev.key == pygame.K_c:
                            with lock:
                                arr = frame[0]
                            if arr is not None:
                                p = f"/tmp/director_{shot}.png"
                                s = pygame.image.frombuffer(arr.tobytes(), (arr.shape[1], arr.shape[0]), "RGB")
                                pygame.image.save(s, p)
                                shot += 1
                                print(f"[shot] {p}")
                    elif ev.type == pygame.MOUSEMOTION:
                        cam_yaw += ev.rel[0] * mouse_sens
                        cam_pitch -= ev.rel[1] * mouse_sens
                        cam_pitch = max(-89.0, min(89.0, cam_pitch))

                # FPS movement: follow mouse yaw direction
                keys = pygame.key.get_pressed()
                yaw_rad = math.radians(cam_yaw)
                fwd_x, fwd_y = math.cos(yaw_rad), math.sin(yaw_rad)
                rgt_x, rgt_y = -math.sin(yaw_rad), math.cos(yaw_rad)
                move_f = (keys[pygame.K_w] - keys[pygame.K_s]) * cam_speed * dt
                move_r = (keys[pygame.K_d] - keys[pygame.K_a]) * cam_speed * dt
                cx += fwd_x * move_f + rgt_x * move_r
                cy += fwd_y * move_f + rgt_y * move_r
                if keys[pygame.K_e]:
                    cz += cam_alt * dt
                if keys[pygame.K_q]:
                    cz -= cam_alt * dt
                cz = max(1.0, min(300.0, cz))

            cam.set_transform(
                carla.Transform(
                    carla.Location(cx, cy, cz),
                    carla.Rotation(cam_pitch, cam_yaw, 0.0),
                )
            )

            if proxy_actor is not None:
                with proxy_state["lock"]:
                    idx = max(0, min(len(points) - 1, proxy_state["idx"]))
                try:
                    if idx < len(planned_carla_locs):
                        loc = planned_carla_locs[idx]
                        proxy_actor.set_transform(
                            carla.Transform(
                                loc,
                                carla.Rotation(float(points[idx]["drone_pose"].get("pitch", 0.0)),
                                               float(points[idx]["drone_pose"].get("yaw", 0.0)),
                                               float(points[idx]["drone_pose"].get("roll", 0.0)),
                                               ),
                            )
                        )
                    else:
                        proxy_actor.set_transform(to_carla_tf(points[idx]))
                except Exception:
                    pass

            if args.direct_carla_fly and controlled_actor is not None:
                try:
                    l = controlled_actor.get_transform().location
                    world.debug.draw_string(
                        l + carla.Location(z=1.0),
                        f"CONTROLLED id={controlled_actor.id}",
                        color=carla.Color(255, 255, 0),
                        life_time=0.3,
                        draw_shadow=False,
                    )
                except Exception:
                    pass

            # Build actual flown trail
            curr = None
            if args.direct_carla_fly and controlled_actor is not None:
                try:
                    curr = controlled_actor.get_transform().location
                except Exception:
                    curr = None
            else:
                with telemetry["lock"]:
                    ned_now = telemetry["ned"]
                if ned_now is not None:
                    cxa, cya, cza = map_airsim_ned_to_carla(ned_now[0], ned_now[1], ned_now[2], mapping)
                    curr = carla.Location(cxa, cya, cza)
            if curr is not None:
                if last_actual is None or curr.distance(last_actual) > 0.8:
                    actual_trail.append(curr)
                    if len(actual_trail) > 1200:
                        actual_trail = actual_trail[-1200:]
                    last_actual = curr

            if show_traj:
                viz_traj(world, points, life_time=0.5, color=carla.Color(0, 255, 255))
                viz_polyline(world, planned_carla_locs, life_time=0.5, color=carla.Color(255, 180, 0))
                viz_polyline(world, actual_trail, life_time=0.5, color=carla.Color(255, 0, 255))

            if screen is not None:
                with lock:
                    arr = frame[0]
                if arr is not None:
                    try:
                        s = pygame.image.frombuffer(arr.tobytes(), (arr.shape[1], arr.shape[0]), "RGB")
                        screen.blit(s, (0, 0))
                    except Exception:
                        screen.fill((20, 20, 30))
                else:
                    screen.fill((20, 20, 30))
                status = f"Traj:{'ON' if show_traj else 'OFF'}  ESC Quit  R Toggle  C Shot  trail={len(actual_trail)}"
                screen.blit(font.render(status, True, (0, 255, 200)), (8, 8))
                pygame.display.flip()
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        running[0] = False
        t.join(timeout=6.0)
        try:
            cam.stop()
        except Exception:
            pass
        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass
        if screen is not None:
            try:
                pygame.event.set_grab(False)
            except Exception:
                pass
            pygame.mouse.set_visible(True)
            pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
