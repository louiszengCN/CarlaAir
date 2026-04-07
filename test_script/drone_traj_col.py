
import argparse
import json
import math
import os
import random
import threading
import time
from queue import Queue

import numpy as np
import pygame

try:
    import cv2
except ImportError:
    cv2 = None

from PIL import Image

import carla


def parse_color(s: str, default):
    try:
        parts = [int(x) for x in s.split(",")]
        if len(parts) == 3 and all(0 <= c <= 255 for c in parts):
            return parts
    except Exception:
        return default
    return default


class HumanTrajectoryRunner:
    """
    在地面上按预定义轨迹驱动一个行人：
    - 轨迹文件格式：[{x,y,z,jump?,speed?}, ...]（与 trajectory_collector.py 输出兼容）
    - 运动方式：按相邻点线性插值，速度优先使用点内 speed，否则使用默认速度
    - 可选：循环播放；可选：在地图上绘制轨迹点/虚线
    """
    def __init__(self, world: carla.World, bp_lib: carla.BlueprintLibrary, trajectory_path: str,
                 loop: bool = True, default_speed: float = 1.5, speed_scale: float = 1.0,
                 show_traj: bool = False, viz_life: float = 0.0,
                 use_ai: bool = False, ai_project_to_nav: bool = True, ai_arrival_dist: float = 0.6):
        self.world = world
        self.bp_lib = bp_lib
        self.map = world.get_map()
        self.loop = loop
        self.default_speed = default_speed
        self.speed_scale = speed_scale
        self.show_traj = show_traj
        self.viz_life = viz_life
        self.use_ai = use_ai
        self.ai_project_to_nav = ai_project_to_nav
        self.ai_arrival_dist = float(ai_arrival_dist)

        with open(trajectory_path, encoding="utf-8") as f:
            self.traj = json.load(f)
        if not self.traj:
            raise RuntimeError("human trajectory is empty")

        self.walker = None
        self.controller = None
        self._seg_idx = 0
        self._seg_progress = 0.0
        # runtime tuning
        self.min_speed_scale = 0.0
        self.max_speed_scale = 5.0
        self.paused = False
        self._speed_zero = False
        self._saved_speed_scale = float(speed_scale)
        self._ai_target_idx = 1

    def set_speed_scale(self, scale: float):
        self.speed_scale = float(max(self.min_speed_scale, min(self.max_speed_scale, scale)))

    def adjust_speed_scale(self, delta: float):
        self.set_speed_scale(self.speed_scale + delta)
        if not self._speed_zero:
            self._saved_speed_scale = float(self.speed_scale)

    def toggle_pause(self):
        self.paused = not self.paused
        return self.paused

    def reset_to_start(self):
        if self.walker is None:
            return
        self._seg_idx = 0
        self._seg_progress = 0.0
        self._ai_target_idx = 1
        p0 = self.traj[0]
        loc = carla.Location(x=float(p0["x"]), y=float(p0["y"]), z=float(p0.get("z", 1.0)) + 0.2)
        self.walker.set_transform(carla.Transform(loc, carla.Rotation(yaw=0.0)))
        if self.use_ai and self.controller is not None:
            # restart current destination
            self._ai_go_to_current_target()

    def toggle_speed_zero(self):
        if not self._speed_zero:
            self._saved_speed_scale = float(self.speed_scale)
            self.speed_scale = 0.0
            self._speed_zero = True
        else:
            self.speed_scale = float(self._saved_speed_scale)
            self._speed_zero = False
        return self.speed_scale

    def _project_to_nav(self, loc: carla.Location) -> carla.Location:
        if not self.ai_project_to_nav:
            return loc
        try:
            wp = self.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Any)
            if wp is not None:
                return wp.transform.location
        except Exception:
            pass
        return loc

    def _ai_go_to_current_target(self):
        if self.controller is None or self.walker is None:
            return
        if len(self.traj) < 2:
            return
        idx = self._ai_target_idx
        if idx >= len(self.traj):
            if self.loop:
                idx = 1
                self._ai_target_idx = 1
            else:
                return
        p = self.traj[idx]
        tgt = carla.Location(x=float(p["x"]), y=float(p["y"]), z=float(p.get("z", 1.0)))
        tgt = self._project_to_nav(tgt)
        try:
            self.controller.go_to_location(tgt)
        except Exception:
            pass

    def spawn(self):
        p0 = self.traj[0]
        loc = carla.Location(x=float(p0["x"]), y=float(p0["y"]), z=float(p0.get("z", 1.0)) + 0.2)
        rot = carla.Rotation(yaw=0.0)
        tf = carla.Transform(loc, rot)
        walker_bp = random.choice(self.bp_lib.filter("walker.pedestrian.*"))
        self.walker = self.world.try_spawn_actor(walker_bp, tf)
        if self.walker is None:
            # 回退：强制 spawn_actor（可能抛异常）
            self.walker = self.world.spawn_actor(walker_bp, tf)

        if self.use_ai:
            try:
                ctrl_bp = self.bp_lib.find("controller.ai.walker")
                self.controller = self.world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=self.walker)
                self.controller.start()
                # 初始速度
                self.controller.set_max_speed(float(self.default_speed) * float(self.speed_scale))
                self._ai_target_idx = 1
                self._ai_go_to_current_target()
                print("[HUMAN] AI controller enabled.")
            except Exception as e:
                print(f"[HUMAN][WARN] enable AI controller failed, fallback to manual: {e}")
                self.use_ai = False
                try:
                    if self.controller:
                        self.controller.destroy()
                except Exception:
                    pass
                self.controller = None

        if self.show_traj:
            self._visualize_full_trajectory()
        return self.walker

    def _draw_dashed_line(self, start: carla.Location, end: carla.Location, dash_length=0.6, gap_length=0.5,
                          color=None, thickness=0.05, life_time=0.0):
        if color is None:
            color = carla.Color(50, 50, 0)
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
            self.world.debug.draw_line(p1, p2, thickness=thickness, color=color, life_time=life_time)

    def _visualize_full_trajectory(self):
        prev = None
        for i, p in enumerate(self.traj):
            loc = carla.Location(x=float(p["x"]), y=float(p["y"]), z=float(p.get("z", 1.0)))
            self.world.debug.draw_point(loc + carla.Location(z=0.1), size=0.18, color=carla.Color(255, 0, 0), life_time=self.viz_life)
            if prev is not None:
                self._draw_dashed_line(prev + carla.Location(z=0.1), loc + carla.Location(z=0.1), life_time=self.viz_life)
            prev = loc

    def tick(self, dt: float):
        if self.walker is None:
            return
        if len(self.traj) < 2:
            return
        if self.paused or self.speed_scale <= 0.0:
            # AI 模式下将速度压到 0，而不是完全 return（避免 controller 状态漂移）
            if self.use_ai and self.controller is not None:
                try:
                    self.controller.set_max_speed(0.0)
                except Exception:
                    pass
            return

        # --- AI 模式：controller.ai.walker ---
        if self.use_ai and self.controller is not None:
            # 更新速度
            curr_speed = float(self.default_speed) * float(self.speed_scale)
            try:
                self.controller.set_max_speed(curr_speed)
            except Exception:
                pass

            # 检查是否到达当前目标点
            idx = self._ai_target_idx
            if idx >= len(self.traj):
                if self.loop:
                    self._ai_target_idx = 1
                    idx = 1
                else:
                    return
            p = self.traj[idx]
            tgt = carla.Location(x=float(p["x"]), y=float(p["y"]), z=float(p.get("z", 1.0)))
            tgt = self._project_to_nav(tgt)
            cur = self.walker.get_location()
            dist2d = math.sqrt((cur.x - tgt.x) ** 2 + (cur.y - tgt.y) ** 2)
            if dist2d < self.ai_arrival_dist:
                self._ai_target_idx += 1
                if self._ai_target_idx >= len(self.traj) and self.loop:
                    self._ai_target_idx = 1
                self._ai_go_to_current_target()
            return

        i0 = self._seg_idx
        i1 = i0 + 1
        if i1 >= len(self.traj):
            if self.loop:
                i0 = 0
                i1 = 1
                self._seg_idx = 0
                self._seg_progress = 0.0
            else:
                return

        p0 = self.traj[i0]
        p1 = self.traj[i1]
        a = carla.Location(x=float(p0["x"]), y=float(p0["y"]), z=float(p0.get("z", 1.0)))
        b = carla.Location(x=float(p1["x"]), y=float(p1["y"]), z=float(p1.get("z", 1.0)))
        seg = b - a
        seg_len = max(1e-6, math.sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z))

        spd = float(p1.get("speed", self.default_speed)) * float(self.speed_scale)
        step = spd * dt
        self._seg_progress += step / seg_len

        if self._seg_progress >= 1.0:
            self._seg_idx += 1
            self._seg_progress = 0.0
            return

        t = self._seg_progress
        loc = carla.Location(
            x=a.x + (b.x - a.x) * t,
            y=a.y + (b.y - a.y) * t,
            z=a.z + (b.z - a.z) * t,
        )
        yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
        self.walker.set_transform(carla.Transform(loc, carla.Rotation(yaw=yaw)))


def get_camera_intrinsic(width: int, height: int, fov: float) -> np.ndarray:
    f = width / (2.0 * math.tan(fov * math.pi / 360.0))
    k = np.identity(3)
    k[0, 0] = f
    k[1, 1] = f
    k[0, 2] = width / 2.0
    k[1, 2] = height / 2.0
    return k


def get_matrix(transform: carla.Transform) -> np.ndarray:
    r = transform.rotation
    t = transform.location
    cy, sy = math.cos(math.radians(r.yaw)), math.sin(math.radians(r.yaw))
    cr, sr = math.cos(math.radians(r.roll)), math.sin(math.radians(r.roll))
    cp, sp = math.cos(math.radians(r.pitch)), math.sin(math.radians(r.pitch))
    m = np.identity(4)
    m[0, 3], m[1, 3], m[2, 3] = t.x, t.y, t.z
    m[0, 0], m[0, 1], m[0, 2] = cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr
    m[1, 0], m[1, 1], m[1, 2] = sy * cp, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr
    m[2, 0], m[2, 1], m[2, 2] = sp, -cp * sr, cp * cr
    return m


def rotation_to_axes(rot: carla.Rotation):
    """Return forward/right/up unit vectors for a CARLA rotation (world frame)."""
    cy, sy = math.cos(math.radians(rot.yaw)), math.sin(math.radians(rot.yaw))
    cr, sr = math.cos(math.radians(rot.roll)), math.sin(math.radians(rot.roll))
    cp, sp = math.cos(math.radians(rot.pitch)), math.sin(math.radians(rot.pitch))
    forward = carla.Vector3D(x=cp * cy, y=cp * sy, z=sp)
    right = carla.Vector3D(x=cy * sp * sr - sy * cr, y=sy * sp * sr + cy * cr, z=-cp * sr)
    up = carla.Vector3D(x=-cy * sp * cr - sy * sr, y=-sy * sp * cr + cy * sr, z=cp * cr)
    return forward, right, up


def transform_to_dict(tf: carla.Transform):
    return {
        "x": round(tf.location.x, 4),
        "y": round(tf.location.y, 4),
        "z": round(tf.location.z, 4),
        "pitch": round(tf.rotation.pitch, 4),
        "yaw": round(tf.rotation.yaw, 4),
        "roll": round(tf.rotation.roll, 4),
    }


def mat_to_list(mat: np.ndarray, precision: int = 6):
    return [[round(float(x), precision) for x in row] for row in mat]

def save_carla_image_png(image: carla.Image, path: str):
    """Save CARLA image to PNG quickly (no annotation)."""
    if image is None:
        return
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))[:, :, :3]
    # image is BGRA; keep BGR for cv2, but we save via PIL as RGB for consistency with other tools
    rgb = arr[:, :, ::-1]
    Image.fromarray(rgb).save(path)


def draw_frustum(world, tf: carla.Transform, fov_deg: float, img_w: int, img_h: int,
                 life_time=15.0, color=None, near: float = 0.4, far: float = 8.0, thickness: float = 0.02):
    """Draw a short, dim frustum in CARLA debug view."""
    if color is None:
        color = carla.Color(0, 90, 140)
    if far <= near + 1e-3 or far <= 0.05:
        return
    aspect = img_w / img_h
    half_fov = math.radians(fov_deg * 0.5)
    h_far = math.tan(half_fov) * far
    w_far = h_far * aspect
    h_near = math.tan(half_fov) * near
    w_near = h_near * aspect

    origin = tf.location
    fwd, right, up = rotation_to_axes(tf.rotation)

    def to_np(vec):
        return np.array([vec.x, vec.y, vec.z], dtype=float)

    o = to_np(origin)
    f = to_np(fwd)
    r = to_np(right)
    u = to_np(up)

    fc = o + f * far
    nc = o + f * near

    far_pts = [
        fc + r * w_far + u * h_far,
        fc - r * w_far + u * h_far,
        fc - r * w_far - u * h_far,
        fc + r * w_far - u * h_far,
    ]
    near_pts = [
        nc + r * w_near + u * h_near,
        nc - r * w_near + u * h_near,
        nc - r * w_near - u * h_near,
        nc + r * w_near - u * h_near,
    ]

    def loc(arr):
        return carla.Location(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    for p in far_pts:
        world.debug.draw_line(origin, loc(p), thickness=thickness, color=color, life_time=life_time)
    for quad in (near_pts,):
        for i in range(4):
            a = loc(quad[i])
            b = loc(quad[(i + 1) % 4])
            world.debug.draw_line(a, b, thickness=thickness * 1.4, color=color, life_time=life_time)


def async_save_worker(q: Queue):
    while True:
        task = q.get()
        if task is None:
            q.task_done()
            break
        try:
            kind = task["kind"]
            if kind == "frame":
                # write rgb.png + meta.json
                frame_dir = task["frame_dir"]
                os.makedirs(frame_dir, exist_ok=True)
                if task.get("image") is not None:
                    save_carla_image_png(task["image"], os.path.join(frame_dir, "rgb.png"))
                with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
                    json.dump(task["meta"], f, indent=4, ensure_ascii=False)
            elif kind == "trajectory":
                with open(task["path"], "w", encoding="utf-8") as f:
                    json.dump(task["data"], f, indent=4, ensure_ascii=False)
        finally:
            q.task_done()


def project_world_point_to_uv_depth(world_point_xyz, intrinsic, world_to_camera):
    """
    将世界坐标点投影到像素坐标并返回深度（Z-forward）。
    返回: (uv, depth) 或 (None, None)
    """
    pt = np.array([[world_point_xyz[0], world_point_xyz[1], world_point_xyz[2]]], dtype=float)
    pts_h = np.concatenate([pt, np.ones((1, 1), dtype=float)], axis=1)
    pts_cam = (world_to_camera @ pts_h.T).T
    pts_std = np.zeros((1, 3), dtype=float)
    pts_std[:, 0] = pts_cam[:, 1]
    pts_std[:, 1] = -pts_cam[:, 2]
    pts_std[:, 2] = pts_cam[:, 0]
    depth = float(pts_std[0, 2])
    if depth <= 0.1:
        return None, None
    proj = pts_std @ intrinsic.T
    u = float(proj[0, 0] / proj[0, 2])
    v = float(proj[0, 1] / proj[0, 2])
    return [u, v], depth


def get_actor_bbox_2d(actor: carla.Actor, intrinsic: np.ndarray, world_to_camera: np.ndarray, img_w: int, img_h: int):
    """Project actor bounding box to 2D bbox. Returns [x1,y1,x2,y2] or None."""
    bb = actor.bounding_box
    ext = bb.extent
    vertices = np.array([
        [ext.x, ext.y, ext.z], [ext.x, ext.y, -ext.z], [ext.x, -ext.y, ext.z], [ext.x, -ext.y, -ext.z],
        [-ext.x, ext.y, ext.z], [-ext.x, ext.y, -ext.z], [-ext.x, -ext.y, ext.z], [-ext.x, -ext.y, -ext.z],
    ], dtype=float)
    vertices += [bb.location.x, bb.location.y, bb.location.z]
    tf = actor.get_transform()
    world_vertices = [tf.transform(carla.Location(x=float(v[0]), y=float(v[1]), z=float(v[2]))) for v in vertices]
    pts_world = np.array([[p.x, p.y, p.z] for p in world_vertices], dtype=float)

    pts_h = np.ones((len(pts_world), 4), dtype=float)
    pts_h[:, :3] = pts_world
    pts_cam = (world_to_camera @ pts_h.T).T
    pts_std = np.zeros((len(pts_world), 3), dtype=float)
    pts_std[:, 0] = pts_cam[:, 1]
    pts_std[:, 1] = -pts_cam[:, 2]
    pts_std[:, 2] = pts_cam[:, 0]
    in_front = pts_std[:, 2] > 0.1
    if not np.any(in_front):
        return None
    proj = pts_std[in_front] @ intrinsic.T
    u = proj[:, 0] / proj[:, 2]
    v = proj[:, 1] / proj[:, 2]
    x1, y1, x2, y2 = float(np.min(u)), float(np.min(v)), float(np.max(u)), float(np.max(v))
    if x2 <= 0 or x1 >= img_w or y2 <= 0 or y1 >= img_h:
        return None
    x1 = int(max(0, min(img_w, x1)))
    y1 = int(max(0, min(img_h, y1)))
    x2 = int(max(0, min(img_w, x2)))
    y2 = int(max(0, min(img_h, y2)))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    return [x1, y1, x2, y2]


def postprocess_add_nav_uvd(points, img_w, img_h, fov):
    """
    对记录的无人机轨迹点，补全每个点的 nav_waypoint：
    把 ego(t+1) 的世界位置投影到 ego(t) 图像，得到 (u,v,depth)。
    """
    intrinsic = get_camera_intrinsic(img_w, img_h, fov)
    for i in range(len(points)):
        nav = {"t1_world": None, "t1_image_uv": None, "t1_depth": None}
        if i + 1 < len(points):
            t_tf = points[i]["drone_pose_carla_tf"]
            t1 = points[i + 1]["drone_pose"]
            world_to_camera = np.linalg.inv(get_matrix(t_tf))
            uv, depth = project_world_point_to_uv_depth([t1["x"], t1["y"], t1["z"]], intrinsic, world_to_camera)
            nav["t1_world"] = {"x": t1["x"], "y": t1["y"], "z": t1["z"]}
            nav["t1_image_uv"] = uv
            nav["t1_depth"] = depth
        points[i]["nav_waypoint"] = nav
    return points


class DroneTrajectoryCollector:
    """
    无人机轨迹采集器：
    - 键盘控制无人机（或纯相机）在空中运动
    - R 记录当前 pose 点
    - Q 保存轨迹（自动补全 nav_waypoint: ego(t+1) 投影到 ego(t) 的 (u,v,depth)）
    """
    def __init__(self, host="localhost", port=2000, output_dir=None, img_w=1280, img_h=720, fov=120.0,
                 pitch_deg=-30.0, yaw_deg=0.0, roll_deg=0.0, lock_attitude=True,
                 viz_frustum_color=(0, 90, 140), viz_frustum_near=0.4, viz_frustum_far=0.5,
                 viz_frustum_thickness=0.02, viz_frustum_life=15.0):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        self.img_w = img_w
        self.img_h = img_h
        self.fov = fov

        self.base_output_dir = os.path.abspath(output_dir or os.path.join(os.path.dirname(__file__), "drone_traj_output"))
        os.makedirs(self.base_output_dir, exist_ok=True)
        self.trace_dir = None
        self.frames_dir = None
        self._start_new_trace_dir()

        self.drone_actor = None
        self.camera = None
        self.actor_list = []
        self.human_runner = None

        self.recorded = []  # each item: point meta for trajectory.json
        self.last_image = None
        self.save_queue = Queue()
        self.save_thread = threading.Thread(target=async_save_worker, args=(self.save_queue,), daemon=True)

        self.move_speed = 4.0
        self.mouse_sensitivity = 0.12
        self.lock_attitude = lock_attitude
        self.look_yaw = float(yaw_deg)
        self.look_pitch = float(pitch_deg)
        self.look_roll = float(roll_deg)
        # “定高平移”基准高度：初始化为起飞高度，后续若通过升降改变高度则同步更新
        self.fixed_z = 25.0
        self.viz_frustum_color = tuple(viz_frustum_color)
        self.viz_frustum_near = float(viz_frustum_near)
        self.viz_frustum_far = float(viz_frustum_far)
        self.viz_frustum_thickness = float(viz_frustum_thickness)
        self.viz_frustum_life = float(viz_frustum_life)

        self.was_sync = False
        self._last_record_loc = None
        self._tick_count = 0

    def _start_new_trace_dir(self):
        """Start a new trace folder so consecutive runs don't connect/overwrite."""
        ts = time.strftime("%Y%m%d_%H%M%S")
        # 如果同秒内多次保存，追加一个计数避免冲突
        suffix = ""
        k = 0
        while True:
            name = f"trace_drone_{ts}{suffix}"
            trace_dir = os.path.join(self.base_output_dir, name)
            if not os.path.exists(trace_dir):
                break
            k += 1
            suffix = f"_{k}"
        self.trace_dir = trace_dir
        self.frames_dir = os.path.join(self.trace_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)
        print(f"[TRACE] new trace dir: {self.trace_dir}")

    def _reset_for_next_trace(self):
        """Clear internal state so next R starts a brand-new polyline."""
        self.recorded = []
        self._last_record_loc = None
        # 开启新目录，避免把下一段写进旧目录
        self._start_new_trace_dir()

    def _ensure_sync_mode(self):
        settings = self.world.get_settings()
        self.was_sync = settings.synchronous_mode
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)
            tm = self.client.get_trafficmanager(8000)
            tm.set_synchronous_mode(True)

    def _restore_settings(self):
        try:
            if not self.was_sync:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)
                tm = self.client.get_trafficmanager(8000)
                tm.set_synchronous_mode(False)
        except Exception:
            pass

    def _spawn(self):
        drone_bp = None
        for bp_id in ["static.prop.drone", "static.prop.dji_inspire"]:
            try:
                drone_bp = self.bp_lib.find(bp_id)
                break
            except Exception:
                continue

        cam_bp = self.bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(self.img_w))
        cam_bp.set_attribute("image_size_y", str(self.img_h))
        cam_bp.set_attribute("fov", str(self.fov))

        # 起点：地图原点上空一点，避免穿模
        start_loc = carla.Location(x=0.0, y=0.0, z=25.0)
        start_rot = carla.Rotation(pitch=self.look_pitch, yaw=self.look_yaw, roll=self.look_roll)
        start_tf = carla.Transform(start_loc, start_rot)
        self.fixed_z = start_loc.z

        if drone_bp:
            drone_bp.set_attribute("role_name", "drone_collector")
            self.drone_actor = self.world.spawn_actor(drone_bp, start_tf)
            self.drone_actor.set_simulate_physics(False)
            self.actor_list.append(self.drone_actor)

        self.camera = self.world.spawn_actor(cam_bp, start_tf)
        self.actor_list.append(self.camera)

    def _maybe_spawn_human(self):
        if self.human_runner is None:
            return
        walker = self.human_runner.spawn()
        self.actor_list.append(walker)
        print(f"[HUMAN] spawned walker id={walker.id}")

    def _set_pose(self, loc: carla.Location, rot: carla.Rotation):
        tf = carla.Transform(loc, rot)
        if self.drone_actor:
            self.drone_actor.set_transform(tf)
        self.camera.set_transform(tf)

    def _draw_dashed_line(self, start: carla.Location, end: carla.Location, dash_length=0.8, gap_length=0.5,
                          color=None, thickness=0.05, life_time=0.0):
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
            self.world.debug.draw_line(p1, p2, thickness=thickness, color=color, life_time=life_time)

    def record_point(self, tick_count: int):
        tf = self.camera.get_transform()
        world_to_camera = np.linalg.inv(get_matrix(tf))
        intrinsic = get_camera_intrinsic(self.img_w, self.img_h, self.fov)
        # target (human) info if enabled and visible
        target = {
            "enabled": False,
            "actor_id": None,
            "world_location": None,
            "bbox_2d": None,
            "image_uv": None,
            "depth": None,
            "visible": False,
        }
        if self.human_runner is not None and getattr(self.human_runner, "walker", None) is not None:
            w = self.human_runner.walker
            loc = w.get_location()
            target["enabled"] = True
            target["actor_id"] = int(w.id)
            target["world_location"] = {"x": float(loc.x), "y": float(loc.y), "z": float(loc.z)}
            bbox = None
            try:
                bbox = get_actor_bbox_2d(w, intrinsic, world_to_camera, self.img_w, self.img_h)
            except Exception:
                bbox = None
            uv, depth = project_world_point_to_uv_depth([loc.x, loc.y, loc.z], intrinsic, world_to_camera)
            target["bbox_2d"] = bbox
            target["image_uv"] = uv
            target["depth"] = depth
            if bbox is not None:
                target["visible"] = True
            elif uv is not None and depth is not None:
                target["visible"] = (0 <= uv[0] < self.img_w) and (0 <= uv[1] < self.img_h) and (depth > 0.1)
        point_meta = {
            "index": len(self.recorded),
            "sim_step": tick_count,
            "timestamp": time.time(),
            "drone_pose": {
                "x": tf.location.x,
                "y": tf.location.y,
                "z": tf.location.z,
                "pitch": tf.rotation.pitch,
                "yaw": tf.rotation.yaw,
                "roll": tf.rotation.roll,
            },
            "camera_meta": {
                "fov": self.fov,
                "image_size": {"width": self.img_w, "height": self.img_h},
                "intrinsic": mat_to_list(intrinsic),
                "world_to_camera": mat_to_list(world_to_camera),
            },
            # nav placeholder (postprocess on save)
            "nav_waypoint": {"t1_world": None, "t1_image_uv": None, "t1_depth": None},
            "target": target,
        }
        self.recorded.append(point_meta)

        # 可视化：点+编号+与上一个点的虚线连接（默认永久显示，方便飞行观察）
        idx = len(self.recorded) - 1
        self.world.debug.draw_point(tf.location, size=0.18, color=carla.Color(0, 255, 255), life_time=0.0)
        self.world.debug.draw_string(tf.location + carla.Location(z=0.6), f"P{idx}", color=carla.Color(0, 255, 255), life_time=0.0, draw_shadow=False)
        if self._last_record_loc is not None:
            self._draw_dashed_line(self._last_record_loc, tf.location, life_time=0.0)
        self._last_record_loc = tf.location

        # 视椎体（短且暗）
        draw_frustum(
            self.world,
            tf,
            fov_deg=self.fov,
            img_w=self.img_w,
            img_h=self.img_h,
            life_time=self.viz_frustum_life,
            color=carla.Color(*self.viz_frustum_color),
            near=self.viz_frustum_near,
            far=self.viz_frustum_far,
            thickness=self.viz_frustum_thickness
        )

        # 异步保存当前帧 rgb + meta（与编号对应）
        frame_dir = os.path.join(self.frames_dir, f"frame_{idx:05d}")
        self.save_queue.put({
            "kind": "frame",
            "frame_dir": frame_dir,
            "image": self.last_image,
            "meta": point_meta
        })

        print(f"[REC] point={len(self.recorded)-1} pose={transform_to_dict(tf)}")

    def save(self):
        if not self.recorded:
            print("没有记录任何点，跳过保存。")
            return None
        # 等待当前保存队列刷完，避免 trajectory.json 和 frames 不一致
        print("[SAVE] flushing async queue...")
        self.save_queue.join()

        # postprocess nav uv/depth (ego(t+1) projected to ego(t))
        intrinsic = get_camera_intrinsic(self.img_w, self.img_h, self.fov)
        for i in range(len(self.recorded)):
            nav = {"t1_world": None, "t1_image_uv": None, "t1_depth": None}
            if i + 1 < len(self.recorded):
                t_meta = self.recorded[i]
                t1_pose = self.recorded[i + 1]["drone_pose"]
                world_to_camera = np.array(t_meta["camera_meta"]["world_to_camera"], dtype=float)
                uv, depth = project_world_point_to_uv_depth([t1_pose["x"], t1_pose["y"], t1_pose["z"]], intrinsic, world_to_camera)
                nav["t1_world"] = {"x": t1_pose["x"], "y": t1_pose["y"], "z": t1_pose["z"]}
                nav["t1_image_uv"] = uv
                nav["t1_depth"] = depth
            self.recorded[i]["nav_waypoint"] = nav
            # update per-frame meta.json too
            frame_dir = os.path.join(self.frames_dir, f"frame_{i:05d}")
            meta_path = os.path.join(frame_dir, "meta.json")
            if os.path.exists(meta_path):
                with open(meta_path, encoding="utf-8") as f:
                    m = json.load(f)
                m["nav_waypoint"] = nav
                with open(meta_path, "w", encoding="utf-8") as f:
                    json.dump(m, f, indent=4, ensure_ascii=False)

        out = {
            "schema": "drone_nav_traj_v1",
            "camera": {
                "fov": self.fov,
                "width": self.img_w,
                "height": self.img_h,
                "default_attitude_deg": {"pitch": self.look_pitch, "yaw": self.look_yaw, "roll": self.look_roll},
            },
            "trace_dir": os.path.basename(self.trace_dir),
            "frames_dir": "frames",
            "points": self.recorded,
        }
        filename = os.path.join(self.trace_dir, "trajectory.json")
        # trajectory file async write too (fast, but keep consistent)
        self.save_queue.put({"kind": "trajectory", "path": filename, "data": out})
        self.save_queue.join()
        print(f"[SAVE] {filename} points={len(self.recorded)}")
        return filename

    def run(self):
        self._ensure_sync_mode()
        self.save_thread.start()
        pygame.init()
        display = pygame.display.set_mode((self.img_w, self.img_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Drone Trajectory Collector")
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)

        self._spawn()
        self._maybe_spawn_human()

        def on_img(image: carla.Image):
            self.last_image = image
            array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))[:, :, :3]
            array = array[:, :, ::-1]   # 新增，解决RGB顺序问题
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        self.camera.listen(on_img)

        clock = pygame.time.Clock()
        tick_count = 0
        running = True
        try:
            print("Drone collector controls:")
            print("  Arrow Keys: move forward/back/left/right on fixed altitude (Z locked)")
            print("  WASD: forward/back/left/right (camera frame, may change Z if pitch != 0)")
            print("  E/Q: up/down (updates fixed altitude for Arrow Keys)")
            print("  Mouse: yaw/pitch (when view unlocked)")
            print("  L: toggle view lock (lock/unlock yaw&pitch updates)")
            print("  MouseWheel: speed +/-")
            print("  R: record point")
            print("  Q (hold) is down; use SHIFT+Q? -> use ESC to quit, ENTER to save")
            print("  ENTER: save trajectory json (with nav_waypoint uvd)")
            print("  ESC: quit")
            if self.human_runner is not None:
                print("  1/2: slow down / speed up human (human_speed_scale)")
                print("  3: pause/resume human")
                print("  4: reset human to start")
                print("  0: toggle human speed_scale to 0 / restore")

            while running:
                clock.tick(60)
                pygame.display.flip()

                # tick world
                self.world.tick()
                tick_count += 1
                self._tick_count = tick_count
                dt = self.world.get_settings().fixed_delta_seconds or 0.05

                # 更新地面行人轨迹
                if self.human_runner is not None:
                    self.human_runner.tick(dt)

                # mouse (可选)：允许动态调姿态；锁定姿态时保持初始 pitch/yaw/roll
                if not self.lock_attitude:
                    dx, dy = pygame.mouse.get_rel()
                    self.look_yaw += dx * self.mouse_sensitivity
                    self.look_pitch = float(np.clip(self.look_pitch - dy * self.mouse_sensitivity, -89.0, 89.0))

                # keyboard
                keys = pygame.key.get_pressed()
                # 1) 自由 3D 移动（WASD + E/Q）
                move = carla.Vector3D(0, 0, 0)
                if keys[pygame.K_w]:
                    move.x += 1
                if keys[pygame.K_s]:
                    move.x -= 1
                if keys[pygame.K_a]:
                    move.y -= 1
                if keys[pygame.K_d]:
                    move.y += 1
                if keys[pygame.K_e]:
                    move.z += 1
                if keys[pygame.K_q]:
                    move.z -= 1

                tf = self.camera.get_transform()
                rot = carla.Rotation(pitch=self.look_pitch, yaw=self.look_yaw, roll=self.look_roll)
                fwd, right, up = rotation_to_axes(rot)
                # normalize 2D
                v = carla.Vector3D(
                    x=fwd.x * move.x + right.x * move.y + up.x * move.z,
                    y=fwd.y * move.x + right.y * move.y + up.y * move.z,
                    z=fwd.z * move.x + right.z * move.y + up.z * move.z,
                )
                vlen = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                if vlen > 1e-6:
                    v = v / vlen
                    step = self.move_speed * dt
                    new_loc = carla.Location(
                        x=tf.location.x + v.x * step,
                        y=tf.location.y + v.y * step,
                        z=tf.location.z + v.z * step,
                    )
                else:
                    new_loc = tf.location

                # 2) 定高 XY 平移（方向键）：只改 x/y，z 固定为 fixed_z
                planar = carla.Vector3D(0, 0, 0)
                if keys[pygame.K_UP]:
                    planar.x += 1
                if keys[pygame.K_DOWN]:
                    planar.x -= 1
                if keys[pygame.K_LEFT]:
                    planar.y -= 1
                if keys[pygame.K_RIGHT]:
                    planar.y += 1

                planar_len = math.sqrt(planar.x * planar.x + planar.y * planar.y)
                if planar_len > 1e-6:
                    # 避免“跳高度”：
                    # 如果用户只是开始用方向键做定高平移，而没有按 E/Q 主动升降，
                    # 则先把 fixed_z 同步到当前高度，防止 fixed_z 仍是旧值导致瞬间跳回旧高度。
                    if not (keys[pygame.K_e] or keys[pygame.K_q]):
                        self.fixed_z = tf.location.z
                    # 使用 yaw 的水平前/右方向（忽略 pitch/roll），确保“定高平移”
                    yaw_rad = math.radians(self.look_yaw)
                    fwd_xy = carla.Vector3D(x=math.cos(yaw_rad), y=math.sin(yaw_rad), z=0.0)
                    right_xy = carla.Vector3D(x=-math.sin(yaw_rad), y=math.cos(yaw_rad), z=0.0)
                    planar = planar / planar_len
                    step = self.move_speed * dt
                    dx = fwd_xy.x * planar.x + right_xy.x * planar.y
                    dy = fwd_xy.y * planar.x + right_xy.y * planar.y
                    new_loc = carla.Location(
                        x=tf.location.x + dx * step,
                        y=tf.location.y + dy * step,
                        z=self.fixed_z
                    )

                # 如果发生了垂直移动（E/Q），更新 fixed_z
                if abs(new_loc.z - self.fixed_z) > 1e-4 and (keys[pygame.K_e] or keys[pygame.K_q]):
                    self.fixed_z = new_loc.z

                self._set_pose(new_loc, rot)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False
                        elif event.key == pygame.K_l:
                            self.lock_attitude = not self.lock_attitude
                            # 清一次相对鼠标移动，避免切换瞬间跳变
                            try:
                                pygame.mouse.get_rel()
                            except Exception:
                                pass
                            print(f"[VIEW] lock_attitude={'ON' if self.lock_attitude else 'OFF'} (L toggle)")
                        elif event.key == pygame.K_1 and self.human_runner is not None:
                            self.human_runner.adjust_speed_scale(-self.human_speed_step)
                            print(f"[HUMAN] speed_scale={self.human_runner.speed_scale:.2f}")
                        elif event.key == pygame.K_2 and self.human_runner is not None:
                            self.human_runner.adjust_speed_scale(+self.human_speed_step)
                            print(f"[HUMAN] speed_scale={self.human_runner.speed_scale:.2f}")
                        elif event.key == pygame.K_3 and self.human_runner is not None:
                            paused = self.human_runner.toggle_pause()
                            print(f"[HUMAN] paused={'ON' if paused else 'OFF'}")
                        elif event.key == pygame.K_4 and self.human_runner is not None:
                            self.human_runner.reset_to_start()
                            print("[HUMAN] reset to start")
                        elif event.key == pygame.K_0 and self.human_runner is not None:
                            scale = self.human_runner.toggle_speed_zero()
                            print(f"[HUMAN] speed_scale={scale:.2f} (0 toggle)")
                        elif event.key == pygame.K_r:
                            self.record_point(tick_count)
                        elif event.key == pygame.K_RETURN:
                            saved_path = self.save()
                            # 保存后自动开启新轨迹段，避免下一次 R 仍然连到旧点
                            if saved_path:
                                print("[TRACE] saved. ready for next trace (state reset).")
                                self._reset_for_next_trace()
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 4:  # wheel up
                            self.move_speed = min(self.move_speed + 1.0, 20.0)
                            print(f"[SPEED] {self.move_speed:.1f} m/s")
                        elif event.button == 5:  # wheel down
                            self.move_speed = max(self.move_speed - 1.0, 1.0)
                            print(f"[SPEED] {self.move_speed:.1f} m/s")

        finally:
            print("Cleaning up...")
            try:
                if self.camera:
                    self.camera.stop()
            except Exception:
                pass
            try:
                # stop async thread
                self.save_queue.put(None)
                self.save_thread.join(timeout=3)
            except Exception:
                pass
            for a in self.actor_list:
                try:
                    a.destroy()
                except Exception:
                    pass
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            pygame.quit()
            self._restore_settings()


def parse_args():
    p = argparse.ArgumentParser(description="Drone trajectory collector (records 3D poses + derived nav uvd)")
    p.add_argument("--host", type=str, default="localhost")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--output-dir", type=str, default=os.path.join(os.path.dirname(__file__), "drone_traj_output"))
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fov", type=float, default=120.0)
    p.add_argument("--pitch-deg", type=float, default=-30.0, help="默认姿态 pitch（度），负值向下")
    p.add_argument("--yaw-deg", type=float, default=0.0, help="默认姿态 yaw（度）")
    p.add_argument("--roll-deg", type=float, default=0.0, help="默认姿态 roll（度）")
    p.add_argument("--free-look", action="store_true", help="允许鼠标改变 yaw/pitch（默认锁定姿态）")
    p.add_argument("--viz-frustum-color", type=str, default="0,90,140", help="视椎体颜色 r,g,b (0-255)")
    p.add_argument("--viz-frustum-near", type=float, default=0.4, help="视椎体 near（米）")
    p.add_argument("--viz-frustum-far", type=float, default=0.5, help="视椎体 far（米，线长度），默认 0.5 避免长蓝线")
    p.add_argument("--viz-frustum-thickness", type=float, default=0.02, help="视椎体线宽")
    p.add_argument("--viz-frustum-life", type=float, default=15.0, help="视椎体留存时间（秒）")
    # human trajectory mode
    p.add_argument("--human-trajectory", type=str, default=None, help="可选：地面行人轨迹 json 路径（trajectory_collector 输出）")
    p.add_argument("--human-loop", action="store_true", help="地面行人循环走轨迹（默认不开启则走到终点停）")
    p.add_argument("--human-speed-scale", type=float, default=1.0, help="地面行人速度倍率（乘到轨迹 speed 上）")
    p.add_argument("--human-speed-step", type=float, default=0.1, help="按键 1/2 调速步长（作用于 speed_scale）")
    p.add_argument("--human-show-traj", action="store_true", help="在地图上显示地面行人轨迹点/虚线")
    p.add_argument("--human-traj-life", type=float, default=0.0, help="地面行人轨迹可视化留存时间（秒），0 表示永久")
    p.add_argument("--human-ai", action="store_true", help="启用行人 AI 控制器模式（更自然的行走），按轨迹点依次 go_to_location")
    p.add_argument("--human-ai-arrival", type=float, default=0.6, help="AI 模式到达阈值（米，2D）")
    p.add_argument("--human-ai-no-project-nav", action="store_true", help="AI 模式不将目标点投影到导航网格/道路")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    collector = DroneTrajectoryCollector(
        host=args.host,
        port=args.port,
        output_dir=args.output_dir,
        img_w=args.width,
        img_h=args.height,
        fov=args.fov,
        pitch_deg=args.pitch_deg,
        yaw_deg=args.yaw_deg,
        roll_deg=args.roll_deg,
        lock_attitude=not args.free_look,
        viz_frustum_color=parse_color(args.viz_frustum_color, (0, 90, 140)),
        viz_frustum_near=args.viz_frustum_near,
        viz_frustum_far=args.viz_frustum_far,
        viz_frustum_thickness=args.viz_frustum_thickness,
        viz_frustum_life=args.viz_frustum_life,
    )
    if args.human_trajectory:
        collector.human_runner = HumanTrajectoryRunner(
            world=collector.world,
            bp_lib=collector.bp_lib,
            trajectory_path=args.human_trajectory,
            loop=args.human_loop,
            default_speed=1.5,
            speed_scale=args.human_speed_scale,
            show_traj=args.human_show_traj,
            viz_life=args.human_traj_life,
            use_ai=args.human_ai,
            ai_project_to_nav=not args.human_ai_no_project_nav,
            ai_arrival_dist=args.human_ai_arrival,
        )
        collector.human_speed_step = float(args.human_speed_step)
        print(f"[HUMAN] initial speed_scale={collector.human_runner.speed_scale:.2f} (press 1/2 to adjust)")
    collector.run()



