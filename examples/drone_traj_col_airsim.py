#!/usr/bin/env python3
"""
drone_traj_col_airsim.py — AirSim 轨迹录制（FPS 控制）+ CARLA 轨迹可视化

重点修复：
1) 坐标系排查与修复：
   - 轨迹点同时保存 AirSim 原生坐标 `airsim_pose_ned` 与 CARLA 映射坐标 `drone_pose`
   - 若能在 CARLA 中自动找到无人机 actor，则直接用其 transform 作为 `drone_pose`（最准确）
   - 否则在首次按 R 时自动用 CARLA spectator 做原点标定偏移（自动标定）
2) 控制改为 FPS 风格：
   - 鼠标左右控制 yaw，WASD 始终沿当前朝向前后左右移动
3) 按 R 记录时，即可在 CARLA 场景中看到轨迹（点 + 虚线）

Usage:
    ./SimWorld.sh Town10HD
    python3 examples/drone_traj_col_airsim.py
    python3 examples/drone_traj_col_airsim.py --carla-host localhost --carla-port 2000
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
    import pygame
except ImportError:
    pygame = None

try:
    import airsim
except ImportError:
    airsim = None

try:
    import carla
except ImportError:
    carla = None


BUILTIN_FALLBACK_MAP = {
    "name": "xy",
    "A": [[1.0, 0.0], [0.0, 1.0]],
    "z_sign": -1.0,
    # from your validated manual calibration run (err ~0.44m)
    "offset": [-172.15, 183.99, 27.44],
    "median_err": 0.44,
    "samples": 2,
}


def parse_xyz(s):
    vals = [v.strip() for v in str(s).split(",")]
    if len(vals) != 3:
        raise ValueError("xyz must be like: x,y,z")
    return float(vals[0]), float(vals[1]), float(vals[2])


class AxisCalibrator:
    """Estimate AirSim NED -> CARLA xyz mapping (axis/sign + translation)."""

    def __init__(self):
        # xy transforms: [cx, cy]^T = A * [ax, ay]^T + t
        self.candidates = [
            ("xy", np.array([[1.0, 0.0], [0.0, 1.0]]), -1.0),
            ("x- y", np.array([[1.0, 0.0], [0.0, -1.0]]), -1.0),
            ("-x y", np.array([[-1.0, 0.0], [0.0, 1.0]]), -1.0),
            ("-x -y", np.array([[-1.0, 0.0], [0.0, -1.0]]), -1.0),
            ("yx", np.array([[0.0, 1.0], [1.0, 0.0]]), -1.0),
            ("y -x", np.array([[0.0, 1.0], [-1.0, 0.0]]), -1.0),
            ("-y x", np.array([[0.0, -1.0], [1.0, 0.0]]), -1.0),
            ("-y -x", np.array([[0.0, -1.0], [-1.0, 0.0]]), -1.0),
        ]
        # each: list of offsets [tx,ty,tz], list of residuals
        self.buf = {name: {"offs": [], "errs": []} for name, _, _ in self.candidates}
        self.max_len = 200

    def update(self, airsim_ned_pose, carla_tf):
        ax = float(airsim_ned_pose["x"])
        ay = float(airsim_ned_pose["y"])
        az = float(airsim_ned_pose["z"])
        c = np.array([float(carla_tf.location.x), float(carla_tf.location.y), float(carla_tf.location.z)], dtype=float)
        axy = np.array([ax, ay], dtype=float)
        for name, A, z_sign in self.candidates:
            pred_xy = A @ axy
            pred_z = z_sign * az
            off = np.array([c[0] - pred_xy[0], c[1] - pred_xy[1], c[2] - pred_z], dtype=float)
            b = self.buf[name]
            b["offs"].append(off)
            if len(b["offs"]) > self.max_len:
                b["offs"] = b["offs"][-self.max_len:]
            med = np.median(np.stack(b["offs"], axis=0), axis=0)
            rec = np.array([pred_xy[0] + med[0], pred_xy[1] + med[1], pred_z + med[2]], dtype=float)
            err = float(np.linalg.norm(rec - c))
            b["errs"].append(err)
            if len(b["errs"]) > self.max_len:
                b["errs"] = b["errs"][-self.max_len:]

    def best(self):
        best_name = None
        best_score = 1e9
        for name, _, _ in self.candidates:
            errs = self.buf[name]["errs"]
            if len(errs) < 5:
                continue
            score = float(np.median(np.array(errs, dtype=float)))
            if score < best_score:
                best_score = score
                best_name = name
        if best_name is None:
            return None
        for name, A, z_sign in self.candidates:
            if name == best_name:
                offs = self.buf[name]["offs"]
                med = np.median(np.stack(offs, axis=0), axis=0) if offs else np.zeros(3, dtype=float)
                n = len(offs)
                return {
                    "name": name,
                    "A": A,
                    "z_sign": z_sign,
                    "offset": med,
                    "median_err": best_score,
                    "samples": n,
                }
        return None

    def map_pose(self, airsim_like_pose, airsim_ned_pose):
        b = self.best()
        if b is None:
            return dict(airsim_like_pose), {"mode": "direct", "ok": False}
        axy = np.array([float(airsim_ned_pose["x"]), float(airsim_ned_pose["y"])], dtype=float)
        out_xy = b["A"] @ axy
        out_z = b["z_sign"] * float(airsim_ned_pose["z"])
        off = b["offset"]
        mapped = {
            "x": float(out_xy[0] + off[0]),
            "y": float(out_xy[1] + off[1]),
            "z": float(out_z + off[2]),
            "pitch": float(airsim_like_pose["pitch"]),
            "yaw": float(airsim_like_pose["yaw"]),
            "roll": float(airsim_like_pose["roll"]),
        }
        ok = (b["samples"] >= 8 and b["median_err"] < 6.0)
        return mapped, {"mode": "calibrated", "ok": ok, "name": b["name"], "median_err": float(b["median_err"]), "samples": int(b["samples"])}

    def solve_from_pairs(self, pairs):
        """
        Manual calibration from correspondences:
            pairs[i] = {"airsim_ned": (ax,ay,az), "carla": (cx,cy,cz)}
        Returns best mapping dict compatible with best().
        """
        if len(pairs) < 2:
            return None
        best = None
        for name, A, z_sign in self.candidates:
            offs = []
            errs = []
            for p in pairs:
                ax, ay, az = p["airsim_ned"]
                cx, cy, cz = p["carla"]
                pred_xy = A @ np.array([ax, ay], dtype=float)
                pred_z = z_sign * az
                off = np.array([cx - pred_xy[0], cy - pred_xy[1], cz - pred_z], dtype=float)
                offs.append(off)
            med = np.median(np.stack(offs, axis=0), axis=0)
            for p in pairs:
                ax, ay, az = p["airsim_ned"]
                cx, cy, cz = p["carla"]
                pred_xy = A @ np.array([ax, ay], dtype=float)
                pred_z = z_sign * az
                rec = np.array([pred_xy[0] + med[0], pred_xy[1] + med[1], pred_z + med[2]], dtype=float)
                errs.append(float(np.linalg.norm(rec - np.array([cx, cy, cz], dtype=float))))
            med_err = float(np.median(np.array(errs, dtype=float)))
            cand = {"name": name, "A": A, "z_sign": z_sign, "offset": med, "median_err": med_err, "samples": len(pairs)}
            if best is None or cand["median_err"] < best["median_err"]:
                best = cand
        return best


def quaternion_to_euler_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(90.0, sinp)
    else:
        pitch = math.degrees(math.asin(sinp))

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    return pitch, yaw, roll


def airsim_state_to_pose(state):
    pos = state.kinematics_estimated.position
    ori = state.kinematics_estimated.orientation
    pitch, yaw, roll = quaternion_to_euler_deg(ori.x_val, ori.y_val, ori.z_val, ori.w_val)
    # AirSim NED -> ENU-like CARLA sign for Z
    carla_like = {
        "x": float(pos.x_val),
        "y": float(pos.y_val),
        "z": -float(pos.z_val),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "roll": float(roll),
    }
    ned = {
        "x": float(pos.x_val),
        "y": float(pos.y_val),
        "z": float(pos.z_val),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "roll": float(roll),
    }
    return carla_like, ned


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


def viz_traj(world, points, life_time=0.5):
    prev = None
    for p in points:
        pose = p["drone_pose"]
        loc = carla.Location(x=float(pose["x"]), y=float(pose["y"]), z=float(pose["z"]))
        world.debug.draw_point(loc, size=0.14, color=carla.Color(0, 255, 255), life_time=life_time)
        if prev is not None:
            draw_dashed_line(world, prev, loc, life_time=life_time)
        prev = loc


def find_carla_drone_actor(world):
    """自动寻找 CARLA 中可见的无人机 actor。"""
    try:
        actors = world.get_actors()
    except Exception:
        return None
    candidates = []
    for a in actors:
        tid = (getattr(a, "type_id", "") or "").lower()
        role = ""
        try:
            role = (a.attributes.get("role_name", "") or "").lower()
        except Exception:
            role = ""
        if ("drone" in tid) or ("drone" in role) or ("airsim" in role):
            candidates.append(a)
    if not candidates:
        return None
    # 优先 role_name 更像无人机控制角色
    candidates.sort(key=lambda x: 0 if "drone" in (x.attributes.get("role_name", "").lower()) else 1)
    return candidates[0]


def refind_best_drone_actor(world, prev_actor=None, expected_loc=None):
    if prev_actor is not None:
        try:
            _ = prev_actor.get_transform()
            return prev_actor
        except Exception:
            pass
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
                return math.sqrt((l.x - expected_loc[0]) ** 2 + (l.y - expected_loc[1]) ** 2 + (l.z - expected_loc[2]) ** 2)
            except Exception:
                return 1e9
        cands.sort(key=dist)
        return cands[0]
    cands.sort(key=lambda x: 0 if "drone" in ((x.attributes.get("role_name", "") or "").lower()) else 1)
    return cands[0]


def make_forced_map_from_single_pair(airsim_ned_pose, carla_tf):
    """
    Build initial mapping from one pair.
    Default assumption: CARLA xy aligns with AirSim xy, z is inverted sign.
    """
    ax = float(airsim_ned_pose["x"])
    ay = float(airsim_ned_pose["y"])
    az = float(airsim_ned_pose["z"])
    cx = float(carla_tf.location.x)
    cy = float(carla_tf.location.y)
    cz = float(carla_tf.location.z)
    # candidate fixed as "xy" + z_sign=-1 (validated in your environment)
    A = np.array([[1.0, 0.0], [0.0, 1.0]], dtype=float)
    z_sign = -1.0
    off = np.array([cx - ax, cy - ay, cz - (z_sign * az)], dtype=float)
    return {"name": "xy", "A": A, "z_sign": z_sign, "offset": off, "median_err": 0.0, "samples": 1}


def normalize_map_obj(m):
    if m is None:
        return None
    try:
        return {
            "name": str(m.get("name", "xy")),
            "A": np.array(m["A"], dtype=float),
            "z_sign": float(m.get("z_sign", -1.0)),
            "offset": np.array(m["offset"], dtype=float),
            "median_err": float(m.get("median_err", 999.0)),
            "samples": int(m.get("samples", 0)),
        }
    except Exception:
        return None


def dump_map_obj(m):
    if m is None:
        return None
    return {
        "name": m["name"],
        "A": m["A"].tolist() if hasattr(m["A"], "tolist") else m["A"],
        "z_sign": float(m["z_sign"]),
        "offset": m["offset"].tolist() if hasattr(m["offset"], "tolist") else m["offset"],
        "median_err": float(m.get("median_err", 0.0)),
        "samples": int(m.get("samples", 0)),
    }


def load_map_from_file(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        return normalize_map_obj(raw)
    except Exception:
        return None


def save_map_to_file(path, m):
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(dump_map_obj(m), f, indent=2, ensure_ascii=False)
        return True
    except Exception:
        return False


def parse_args():
    p = argparse.ArgumentParser(description="AirSim 轨迹录制（FPS 控制）")
    p.add_argument("--airsim-port", type=int, default=41451)
    p.add_argument("--output-dir", type=str, default=os.path.join(os.path.dirname(__file__), "drone_traj_output"))
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fov", type=float, default=90.0)
    p.add_argument("--carla-host", type=str, default="localhost")
    p.add_argument("--carla-port", type=int, default=2000)
    p.add_argument(
        "--mapping-file",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "drone_traj_output", "default_mapping.json"),
        help="persisted mapping json path; used when auto-calibration fails",
    )
    p.add_argument("--auto-calibrate", action="store_true", default=True, help="startup auto calibration from CARLA drone actor")
    p.add_argument("--no-auto-calibrate", action="store_false", dest="auto_calibrate")
    p.add_argument("--diag-point", type=str, default=None, help="manual calibration target in CARLA: x,y,z")
    p.add_argument("--diag-point-size", type=float, default=0.5)
    return p.parse_args()


def main():
    args = parse_args()
    if airsim is None:
        sys.exit("Need airsim: pip install airsim")
    if pygame is None:
        sys.exit("Need pygame: pip install pygame")

    base_output = os.path.abspath(args.output_dir)
    os.makedirs(base_output, exist_ok=True)

    ts = time.strftime("%Y%m%d_%H%M%S")
    trace_dir = os.path.join(base_output, f"trace_drone_airsim_{ts}")
    k = 0
    while os.path.exists(trace_dir):
        k += 1
        trace_dir = os.path.join(base_output, f"trace_drone_airsim_{ts}_{k}")
    os.makedirs(trace_dir, exist_ok=True)
    os.makedirs(os.path.join(trace_dir, "frames"), exist_ok=True)

    # AirSim control client
    ac = airsim.MultirotorClient(port=args.airsim_port)
    ac.confirmConnection()
    ac.enableApiControl(True)
    ac.armDisarm(True)
    ac.takeoffAsync().join()
    ac.moveToZAsync(-25.0, 5).join()
    time.sleep(0.4)

    # AirSim camera client (separate socket to avoid BufferError)
    ac_cam = airsim.MultirotorClient(port=args.airsim_port)
    ac_cam.confirmConnection()

    # Optional CARLA for viz + calibration
    carla_world = None
    carla_drone_actor = None
    calibrator = AxisCalibrator()
    last_actor_refind = 0.0
    last_diag_print = 0.0
    if carla is not None:
        try:
            cc = carla.Client(args.carla_host, args.carla_port)
            cc.set_timeout(5.0)
            carla_world = cc.get_world()
            carla_drone_actor = find_carla_drone_actor(carla_world)
            if carla_drone_actor is not None:
                print(f"[CARLA] found drone actor id={carla_drone_actor.id}, using actor transform for mapping.")
            else:
                print("[CARLA] connected. no drone actor found now; will keep scanning and auto-calibrate.")
        except Exception as e:
            print(f"[CARLA] not connected, trajectory viz disabled: {e}")
            carla_world = None

    recorded = []
    show_traj = False

    move_speed = 6.0
    yaw_sensitivity = 0.65
    max_yaw_rate = 360.0
    cmd_duration = 0.05

    running = [True]
    drone_img = [None]
    img_lock = threading.Lock()
    manual_pairs = []
    forced_map = None
    # Load persisted mapping first; fallback to built-in map if file absent
    fm_file = load_map_from_file(args.mapping_file)
    if fm_file is not None:
        forced_map = fm_file
        print(f"[MAP] loaded persisted mapping from {args.mapping_file}: {forced_map['name']} err={forced_map['median_err']:.2f}m")
    else:
        forced_map = normalize_map_obj(BUILTIN_FALLBACK_MAP)
        print(
            "[MAP] using built-in fallback mapping "
            f"({forced_map['name']} err={forced_map['median_err']:.2f}m). "
            "Will override if auto/manual calibration succeeds."
        )

    pygame.init()
    win_w, win_h = 640, 360
    screen = pygame.display.set_mode((win_w, win_h))
    pygame.display.set_caption("AirSim Recorder (FPS) - R=Record ENTER=Save ESC=Quit")
    pygame.mouse.set_visible(False)
    try:
        pygame.event.set_grab(True)
    except Exception:
        pass
    font = pygame.font.SysFont("monospace", 14)
    clock = pygame.time.Clock()
    mouse_dx_acc = 0.0
    diag_on = args.diag_point is not None
    diag_loc = None
    if args.diag_point is not None:
        try:
            if carla is None:
                raise RuntimeError("carla python API not available")
            x, y, z = parse_xyz(args.diag_point)
            diag_loc = carla.Location(x=x, y=y, z=z)
        except Exception as e:
            print(f"[DIAG] invalid --diag-point: {e}")
            diag_on = False

    def capture_loop():
        while running[0]:
            try:
                resp = ac_cam.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
                if resp and resp[0].width > 0:
                    r = resp[0]
                    buf = np.frombuffer(r.image_data_uint8, dtype=np.uint8).copy()
                    n = r.height * r.width
                    ch = len(buf) // n
                    img = buf.reshape(r.height, r.width, ch)
                    if ch >= 3:
                        img = img[:, :, [2, 1, 0]]
                    small = img[::2, ::2, :3]
                    with img_lock:
                        drone_img[0] = np.ascontiguousarray(small)
            except Exception:
                pass
            time.sleep(0.08)

    t_cap = threading.Thread(target=capture_loop, daemon=True)
    t_cap.start()

    def map_to_carla_pose(airsim_like_pose, airsim_ned_pose):
        nonlocal carla_drone_actor, last_actor_refind, last_diag_print, forced_map
        if forced_map is not None:
            axy = np.array([float(airsim_ned_pose["x"]), float(airsim_ned_pose["y"])], dtype=float)
            out_xy = forced_map["A"] @ axy
            out_z = forced_map["z_sign"] * float(airsim_ned_pose["z"])
            off = forced_map["offset"]
            mapped = {
                "x": float(out_xy[0] + off[0]),
                "y": float(out_xy[1] + off[1]),
                "z": float(out_z + off[2]),
                "pitch": float(airsim_like_pose["pitch"]),
                "yaw": float(airsim_like_pose["yaw"]),
                "roll": float(airsim_like_pose["roll"]),
            }
            info = {
                "mode": "forced_map",
                "ok": True,
                "name": forced_map["name"],
                "median_err": float(forced_map["median_err"]),
                "samples": int(forced_map["samples"]),
            }
            return mapped, info
        # Periodically refind actor and update calibrator
        if carla_world is not None:
            now = time.time()
            if forced_map is not None:
                axy = np.array([float(airsim_ned_pose["x"]), float(airsim_ned_pose["y"])], dtype=float)
                out_xy = forced_map["A"] @ axy
                out_z = forced_map["z_sign"] * float(airsim_ned_pose["z"])
                off = forced_map["offset"]
                expected = (float(out_xy[0] + off[0]), float(out_xy[1] + off[1]), float(out_z + off[2]))
            else:
                expected = (airsim_like_pose["x"], airsim_like_pose["y"], airsim_like_pose["z"])
            if now - last_actor_refind > 1.0:
                carla_drone_actor = refind_best_drone_actor(carla_world, carla_drone_actor, expected_loc=expected)
                last_actor_refind = now
            if carla_drone_actor is not None:
                try:
                    tf = carla_drone_actor.get_transform()
                    # Reject obviously wrong actor binding
                    if forced_map is not None:
                        ex = expected[0]
                        ey = expected[1]
                        ez = expected[2]
                        da = math.sqrt((float(tf.location.x) - ex) ** 2 + (float(tf.location.y) - ey) ** 2 + (float(tf.location.z) - ez) ** 2)
                        if da > 80.0:
                            raise RuntimeError(f"actor mismatch distance too large: {da:.1f}m")
                    calibrator.update(airsim_ned_pose, tf)
                    # If we have a forced map, keep offset slowly synced to actor (reduces long-run drift)
                    if forced_map is not None:
                        axy = forced_map["A"] @ np.array([float(airsim_ned_pose["x"]), float(airsim_ned_pose["y"])], dtype=float)
                        pz = forced_map["z_sign"] * float(airsim_ned_pose["z"])
                        target_off = np.array([float(tf.location.x) - axy[0], float(tf.location.y) - axy[1], float(tf.location.z) - pz], dtype=float)
                        forced_map["offset"] = 0.92 * forced_map["offset"] + 0.08 * target_off
                    if now - last_diag_print > 2.0:
                        b = calibrator.best()
                        if b is not None:
                            print(f"[CALIB] best={b['name']} err={b['median_err']:.2f}m samples={b['samples']}")
                        last_diag_print = now
                    return {
                        "x": float(tf.location.x),
                        "y": float(tf.location.y),
                        "z": float(tf.location.z),
                        "pitch": float(tf.rotation.pitch),
                        "yaw": float(tf.rotation.yaw),
                        "roll": float(tf.rotation.roll),
                    }, {"mode": "actor", "ok": True}
                except Exception:
                    pass
        mapped, info = calibrator.map_pose(airsim_like_pose, airsim_ned_pose)
        return mapped, info

    def record_point():
        st = ac.getMultirotorState()
        pose_carla_like, pose_ned = airsim_state_to_pose(st)
        mapped, map_info = map_to_carla_pose(pose_carla_like, pose_ned)
        idx = len(recorded)
        point = {
            "index": idx,
            "sim_step": idx,
            "timestamp": time.time(),
            "drone_pose": {
                "x": round(mapped["x"], 4),
                "y": round(mapped["y"], 4),
                "z": round(mapped["z"], 4),
                "pitch": round(mapped["pitch"], 4),
                "yaw": round(mapped["yaw"], 4),
                "roll": round(mapped["roll"], 4),
            },
            "airsim_pose_ned": {
                "x": round(pose_ned["x"], 4),
                "y": round(pose_ned["y"], 4),
                "z": round(pose_ned["z"], 4),
                "pitch": round(pose_ned["pitch"], 4),
                "yaw": round(pose_ned["yaw"], 4),
                "roll": round(pose_ned["roll"], 4),
            },
            "camera_meta": {
                "fov": args.fov,
                "image_size": {"width": args.width, "height": args.height},
                "intrinsic": [],
                "world_to_camera": [],
            },
            "nav_waypoint": {"t1_world": None, "t1_image_uv": None, "t1_depth": None},
            "target": {"enabled": False},
            "mapping_info": map_info,
        }
        recorded.append(point)
        if not map_info.get("ok", False):
            print("[WARN] mapping not confident yet. Continue moving for calibration or use manual K/M workflow.")
        print(f"[REC] #{idx} map={map_info} carla={point['drone_pose']} airsim_ned={point['airsim_pose_ned']}")

    def save_trajectory():
        if not recorded:
            print("No points, skip save.")
            return None
        for i in range(len(recorded)):
            nav = {"t1_world": None, "t1_image_uv": None, "t1_depth": None}
            if i + 1 < len(recorded):
                t1 = recorded[i + 1]["drone_pose"]
                nav["t1_world"] = {"x": t1["x"], "y": t1["y"], "z": t1["z"]}
            recorded[i]["nav_waypoint"] = nav
        out = {
            "schema": "drone_nav_traj_v2",
            "source": "drone_traj_col_airsim_fps",
            "camera": {
                "fov": args.fov,
                "width": args.width,
                "height": args.height,
            },
            "trace_dir": os.path.basename(trace_dir),
            "frames_dir": "frames",
            "points": recorded,
        }
        p = os.path.join(trace_dir, "trajectory.json")
        with open(p, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print(f"[SAVE] {p}, points={len(recorded)}")
        return p

    print("=" * 60)
    print("AirSim Recorder (FPS)")
    print("WASD move by current look yaw; Mouse X controls yaw; E/Q up/down")
    print("R record + show trajectory in CARLA (if connected)")
    print("ENTER save trajectory.json; ESC quit")
    print("[/]=yaw sensitivity -/+ ,  ,/.=max yaw rate -/+ ,  -/=speed -/+")
    print("B toggle diagnostic blink point, K capture manual calib pair, M solve/apply manual calib")
    print("=" * 60)

    # Startup auto-calibration from current AirSim/CARLA drone position
    if args.auto_calibrate and carla_world is not None:
        try:
            st0 = ac.getMultirotorState()
            pose0_like, pose0_ned = airsim_state_to_pose(st0)
            carla_drone_actor = refind_best_drone_actor(
                carla_world,
                carla_drone_actor,
                expected_loc=(pose0_like["x"], pose0_like["y"], pose0_like["z"]),
            )
            if carla_drone_actor is not None:
                tf0 = carla_drone_actor.get_transform()
                forced_map = make_forced_map_from_single_pair(pose0_ned, tf0)
                save_map_to_file(args.mapping_file, forced_map)
                print("[AUTO_CALIB] startup pair:")
                print(f"  AirSim NED spawn: ({pose0_ned['x']:.3f}, {pose0_ned['y']:.3f}, {pose0_ned['z']:.3f})")
                print(f"  CARLA drone pos : ({tf0.location.x:.3f}, {tf0.location.y:.3f}, {tf0.location.z:.3f})")
                print(f"  Initial offset   : ({forced_map['offset'][0]:.3f}, {forced_map['offset'][1]:.3f}, {forced_map['offset'][2]:.3f})")
                print(f"  Saved mapping    : {args.mapping_file}")
            else:
                print("[AUTO_CALIB] no CARLA drone actor found at startup. using forced fallback mapping.")
        except Exception as e:
            print(f"[AUTO_CALIB] failed: {e}. using forced fallback mapping.")

    try:
        while running[0]:
            dt = max(0.01, clock.tick(30) / 1000.0)
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running[0] = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running[0] = False
                    elif ev.key == pygame.K_r:
                        record_point()
                        show_traj = True
                    elif ev.key == pygame.K_RETURN:
                        save_trajectory()
                    elif ev.key == pygame.K_t:
                        show_traj = not show_traj
                        print(f"[VIZ] show_traj={show_traj}")
                    elif ev.key == pygame.K_b:
                        diag_on = not diag_on
                        print(f"[DIAG] blink point {'ON' if diag_on else 'OFF'}")
                    elif ev.key == pygame.K_k:
                        if carla_world is None or diag_loc is None:
                            print("[DIAG] need --diag-point and CARLA connection.")
                        else:
                            st = ac.getMultirotorState()
                            _, ned = airsim_state_to_pose(st)
                            manual_pairs.append({
                                "airsim_ned": (float(ned["x"]), float(ned["y"]), float(ned["z"])),
                                "carla": (float(diag_loc.x), float(diag_loc.y), float(diag_loc.z)),
                            })
                            print(f"[DIAG] pair captured #{len(manual_pairs)} airsim={manual_pairs[-1]['airsim_ned']} carla={manual_pairs[-1]['carla']}")
                    elif ev.key == pygame.K_m:
                        if len(manual_pairs) < 2:
                            print("[DIAG] need >=2 pairs (press K at known point).")
                        else:
                            sol = calibrator.solve_from_pairs(manual_pairs)
                            if sol is None:
                                print("[DIAG] solve failed.")
                            else:
                                forced_map = sol
                                if save_map_to_file(args.mapping_file, forced_map):
                                    print(f"[DIAG] mapping saved -> {args.mapping_file}")
                                print(f"[DIAG] applied manual mapping: {sol['name']} err={sol['median_err']:.2f}m samples={sol['samples']}")
                    elif ev.key == pygame.K_LEFTBRACKET:
                        yaw_sensitivity = max(0.05, yaw_sensitivity - 0.05)
                        print(f"[CTRL] yaw_sensitivity={yaw_sensitivity:.2f}")
                    elif ev.key == pygame.K_RIGHTBRACKET:
                        yaw_sensitivity = min(2.5, yaw_sensitivity + 0.05)
                        print(f"[CTRL] yaw_sensitivity={yaw_sensitivity:.2f}")
                    elif ev.key == pygame.K_COMMA:
                        max_yaw_rate = max(30.0, max_yaw_rate - 20.0)
                        print(f"[CTRL] max_yaw_rate={max_yaw_rate:.0f}")
                    elif ev.key == pygame.K_PERIOD:
                        max_yaw_rate = min(720.0, max_yaw_rate + 20.0)
                        print(f"[CTRL] max_yaw_rate={max_yaw_rate:.0f}")
                    elif ev.key == pygame.K_MINUS:
                        move_speed = max(1.0, move_speed - 0.5)
                        print(f"[CTRL] move_speed={move_speed:.1f}")
                    elif ev.key == pygame.K_EQUALS:
                        move_speed = min(25.0, move_speed + 0.5)
                        print(f"[CTRL] move_speed={move_speed:.1f}")
                elif ev.type == pygame.MOUSEMOTION:
                    mouse_dx_acc += float(ev.rel[0])
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    if ev.button == 4:
                        move_speed = min(move_speed + 1.0, 20.0)
                        print(f"[SPEED] {move_speed:.1f} m/s")
                    elif ev.button == 5:
                        move_speed = max(move_speed - 1.0, 1.0)
                        print(f"[SPEED] {move_speed:.1f} m/s")

            yaw_rate = 0.0
            if abs(mouse_dx_acc) > 0.001:
                yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, mouse_dx_acc * yaw_sensitivity / dt))
            mouse_dx_acc = 0.0

            keys = pygame.key.get_pressed()
            vx = (keys[pygame.K_w] - keys[pygame.K_s]) * move_speed
            vy = (keys[pygame.K_d] - keys[pygame.K_a]) * move_speed
            up = (keys[pygame.K_e] or keys[pygame.K_SPACE])
            down = (keys[pygame.K_q] or keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT])
            vz = 0.0
            if up and not down:
                vz = -move_speed  # NED up
            elif down and not up:
                vz = move_speed

            ac.moveByVelocityBodyFrameAsync(
                vx, vy, vz, cmd_duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(True, yaw_rate),
            )

            if carla_world is not None and show_traj and len(recorded) > 0:
                viz_traj(carla_world, recorded, life_time=0.5)
            if carla_world is not None and diag_on and diag_loc is not None:
                blink = int(time.time() * 4.0) % 2
                c = carla.Color(255, 255, 0) if blink == 0 else carla.Color(255, 80, 80)
                carla_world.debug.draw_point(diag_loc, size=float(args.diag_point_size), color=c, life_time=0.3)
                carla_world.debug.draw_string(diag_loc + carla.Location(z=0.6), "DIAG_POINT", color=c, life_time=0.3, draw_shadow=False)

            with img_lock:
                di = drone_img[0]
            if di is not None:
                try:
                    s = pygame.image.frombuffer(di.tobytes(), (di.shape[1], di.shape[0]), "RGB")
                    s = pygame.transform.scale(s, (win_w, win_h))
                    screen.blit(s, (0, 0))
                except Exception:
                    screen.fill((18, 18, 28))
            else:
                screen.fill((18, 18, 28))
            status = f"Pts:{len(recorded)} Spd:{move_speed:.1f} YawRate:{yaw_rate:+.1f} sens:{yaw_sensitivity:.2f}"
            if carla_world is not None:
                status += f" Viz:{'ON' if show_traj else 'OFF'} Diag:{'ON' if diag_on else 'OFF'} pairs:{len(manual_pairs)}"
            txt = font.render(status, True, (0, 255, 200))
            screen.blit(txt, (8, 8))
            pygame.display.flip()
    except KeyboardInterrupt:
        pass
    finally:
        running[0] = False
        time.sleep(0.25)
        try:
            pygame.event.set_grab(False)
        except Exception:
            pass
        pygame.mouse.set_visible(True)
        try:
            ac.moveToZAsync(-3, 3).join()
            ac.landAsync().join()
            ac.armDisarm(False)
            ac.enableApiControl(False)
        except Exception:
            pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
