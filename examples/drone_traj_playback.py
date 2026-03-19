      
import argparse
import json
import math
import os
import time

import numpy as np
import pygame

try:
    import cv2
except ImportError:
    cv2 = None

import carla


def parse_color(s: str, default):
    try:
        parts = [int(x) for x in s.split(",")]
        if len(parts) == 3 and all(0 <= c <= 255 for c in parts):
            return parts
    except Exception:
        return default
    return default


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


def project_world_point_to_uv_depth(world_point_xyz, intrinsic, world_to_camera):
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


def draw_cross(img, u, v, color=(0, 255, 255), size=8, thickness=2):
    if img is None:
        return img
    # np.frombuffer produces a read-only view; OpenCV drawing requires writable array
    if not img.flags.writeable:
        img = np.ascontiguousarray(img.copy())
    elif not img.flags["C_CONTIGUOUS"]:
        img = np.ascontiguousarray(img)
    h, w = img.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return img
    u, v = int(round(u)), int(round(v))
    cv2.line(img, (u - size, v), (u + size, v), color, thickness)
    cv2.line(img, (u, v - size), (u, v + size), color, thickness)
    return img


def _save_array_image(arr_bgr: np.ndarray, out_path: str, fmt: str, quality: int = 80, lossless: bool = False):
    fmt = (fmt or "png").lower().lstrip(".")
    quality = int(max(1, min(100, quality)))
    if cv2 is not None:
        params = []
        if fmt == "webp":
            if lossless and hasattr(cv2, "IMWRITE_WEBP_LOSSLESS"):
                params = [int(getattr(cv2, "IMWRITE_WEBP_LOSSLESS")), 1]
            else:
                params = [int(getattr(cv2, "IMWRITE_WEBP_QUALITY", 64)), quality]
        elif fmt in ("jpg", "jpeg"):
            params = [int(getattr(cv2, "IMWRITE_JPEG_QUALITY", 1)), quality]
        cv2.imwrite(out_path, arr_bgr, params)
        return
    raise RuntimeError("OpenCV (cv2) is required to save images in this script.")


def rotation_to_axes(rot: carla.Rotation):
    cy, sy = math.cos(math.radians(rot.yaw)), math.sin(math.radians(rot.yaw))
    cr, sr = math.cos(math.radians(rot.roll)), math.sin(math.radians(rot.roll))
    cp, sp = math.cos(math.radians(rot.pitch)), math.sin(math.radians(rot.pitch))
    forward = carla.Vector3D(x=cp * cy, y=cp * sy, z=sp)
    right = carla.Vector3D(x=cy * sp * sr - sy * cr, y=sy * sp * sr + cy * cr, z=-cp * sr)
    up = carla.Vector3D(x=-cy * sp * cr - sy * sr, y=-sy * sp * cr + cy * sr, z=cp * cr)
    return forward, right, up


def draw_frustum(world, tf: carla.Transform, fov_deg: float, img_w: int, img_h: int,
                 life_time=15.0, color=None, near: float = 0.4, far: float = 8.0, thickness: float = 0.02):
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


class DroneTrajectoryPlayback:
    def __init__(self, traj_path: str, output_dir: str = None, save_frames: bool = False,
                 record_video: bool = False, record_path: str = None, host="localhost", port=2000):
        self.traj_path = traj_path
        self.data = self._load(traj_path)
        self.points = self.data["points"]
        self.img_w = int(self.data["camera"]["width"])
        self.img_h = int(self.data["camera"]["height"])
        self.fov = float(self.data["camera"]["fov"])

        # 默认输出到轨迹文件同目录，方便管理
        self.output_dir = os.path.abspath(output_dir or os.path.dirname(traj_path))
        self.save_frames = save_frames
        self.record_video = record_video and cv2 is not None
        # 默认视频名：与 json 同名，放在同目录
        if record_path:
            self.record_path = record_path
        else:
            base = os.path.splitext(os.path.basename(traj_path))[0]
            self.record_path = os.path.join(self.output_dir, f"{base}.mp4")
        os.makedirs(self.output_dir, exist_ok=True)

        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        self.camera = None
        self.drone_actor = None
        self.actor_list = []
        self.last_image = None
        self.video_writer = None
        self.was_sync = False
        self.viz_frustum = True
        self.viz_frustum_life = 8.0
        self.viz_frustum_near = 0.4
        # 默认缩短视椎体长度，避免画面中出现很长的“蓝线”
        self.viz_frustum_far = 0.5
        self.viz_frustum_thickness = 0.02
        self.viz_frustum_color = (0, 90, 140)
        # 回放节奏策略
        self.use_sim_step = True
        self.min_seg_len = 1
        self.max_seg_len = 200
        self._last_saved_point_idx = -1
        self.capture_format = "webp"
        self.capture_quality = 80
        self.capture_lossless = False
        # 渲染后处理开关（减少路面“黑点噪声”/grain）
        self.disable_postprocess = False

    @staticmethod
    def _load(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

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
        if self.disable_postprocess:
            def set_if_exists(k: str, v: str):
                try:
                    if cam_bp.has_attribute(k):
                        cam_bp.set_attribute(k, v)
                except Exception:
                    pass

            set_if_exists("enable_postprocess_effects", "False")
            set_if_exists("grain_intensity", "0.0")
            set_if_exists("motion_blur_intensity", "0.0")
            set_if_exists("motion_blur_max_distortion", "0.0")
            set_if_exists("motion_blur_min_object_screen_size", "0.0")
            set_if_exists("bloom_intensity", "0.0")
            set_if_exists("lens_flare_intensity", "0.0")
            set_if_exists("chromatic_aberration_intensity", "0.0")

        p0 = self.points[0]["drone_pose"]
        tf0 = carla.Transform(
            carla.Location(x=p0["x"], y=p0["y"], z=p0["z"]),
            carla.Rotation(pitch=p0.get("pitch", 0.0), yaw=p0.get("yaw", 0.0), roll=p0.get("roll", 0.0)),
        )

        if drone_bp:
            drone_bp.set_attribute("role_name", "drone_playback")
            self.drone_actor = self.world.spawn_actor(drone_bp, tf0)
            self.drone_actor.set_simulate_physics(False)
            self.actor_list.append(self.drone_actor)

        self.camera = self.world.spawn_actor(cam_bp, tf0)
        self.actor_list.append(self.camera)

        # 预绘制轨迹（点 + 虚线），方便观察回放路径
        # 默认持久化显示轨迹（life_time=0），避免“很快消失”
        self._viz_full_traj(life_time=0.0)

    def _draw_dashed_line(self, start: carla.Location, end: carla.Location, dash_length=0.8, gap_length=0.5,
                          color=None, thickness=0.05, life_time=15.0):
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

    def _viz_full_traj(self, life_time=0.0):
        prev = None
        for i, p in enumerate(self.points):
            pose = p["drone_pose"]
            loc = carla.Location(x=pose["x"], y=pose["y"], z=pose["z"])
            self.world.debug.draw_point(loc, size=0.14, color=carla.Color(0, 255, 255), life_time=life_time)
            if prev is not None:
                self._draw_dashed_line(prev, loc, life_time=life_time)
            prev = loc

    def _set_tf(self, tf: carla.Transform):
        if self.drone_actor:
            self.drone_actor.set_transform(tf)
        self.camera.set_transform(tf)

    def run(self):
        self._ensure_sync_mode()
        pygame.init()
        display = pygame.display.set_mode((self.img_w, self.img_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Drone Trajectory Playback")

        self._spawn()
        intrinsic = get_camera_intrinsic(self.img_w, self.img_h, self.fov)

        settings = self.world.get_settings()
        dt = settings.fixed_delta_seconds or 0.05

        # --- Playback pacing ---
        # 如果 trajectory.json 的相邻点 sim_step 差距很大，默认会“每 tick 跳一个点”导致飞行过快。
        # 这里按 sim_step 差值进行插值回放，并默认按 dt sleep 逼近实时速度。
        playback_speed = getattr(self, "playback_speed", 1.0)  # >1 faster, <1 slower
        realtime = getattr(self, "realtime", True)              # sleep wall-clock to match dt
        default_ticks_per_point = getattr(self, "ticks_per_point", 10)
        use_sim_step = getattr(self, "use_sim_step", True)
        min_seg_len = int(getattr(self, "min_seg_len", 1))
        max_seg_len = int(getattr(self, "max_seg_len", 200))
        if max_seg_len < min_seg_len:
            max_seg_len = min_seg_len

        if self.record_video:
            for cc in ["mp4v", "avc1"]:
                vw = cv2.VideoWriter(self.record_path, cv2.VideoWriter_fourcc(*cc), round(1.0 / dt), (self.img_w, self.img_h))
                if vw.isOpened():
                    self.video_writer = vw
                    print(f"[INFO] video recording -> {self.record_path} codec={cc}")
                    break
            if self.video_writer is None:
                print("[WARN] cannot open video writer; record disabled.")
                self.record_video = False

        def on_img(image: carla.Image):
            self.last_image = image

        self.camera.listen(on_img)

        clock = pygame.time.Clock()
        running = True
        point_idx = 0
        seg_tick = 0
        try:
            while running and point_idx < len(self.points):
                clock.tick(60)
                self.world.tick()

                p0 = self.points[point_idx]
                pose0 = p0["drone_pose"]
                tf0 = carla.Transform(
                    carla.Location(x=pose0["x"], y=pose0["y"], z=pose0["z"]),
                    carla.Rotation(pitch=pose0.get("pitch", 0.0), yaw=pose0.get("yaw", 0.0), roll=pose0.get("roll", 0.0)),
                )

                if point_idx + 1 < len(self.points):
                    p1 = self.points[point_idx + 1]
                    pose1 = p1["drone_pose"]
                    tf1 = carla.Transform(
                        carla.Location(x=pose1["x"], y=pose1["y"], z=pose1["z"]),
                        carla.Rotation(pitch=pose1.get("pitch", 0.0), yaw=pose1.get("yaw", 0.0), roll=pose1.get("roll", 0.0)),
                    )

                    # segment length in ticks: prefer sim_step delta, fallback to default
                    s0 = p0.get("sim_step")
                    s1 = p1.get("sim_step")
                    if use_sim_step and isinstance(s0, (int, float)) and isinstance(s1, (int, float)) and s1 > s0:
                        seg_len = int(max(1, round((s1 - s0) / max(1e-6, playback_speed))))
                    else:
                        seg_len = int(max(1, round(default_ticks_per_point / max(1e-6, playback_speed))))
                    seg_len = max(min_seg_len, min(max_seg_len, seg_len))

                    alpha = min(1.0, seg_tick / max(1, seg_len))

                    # lerp location
                    loc = carla.Location(
                        x=tf0.location.x + (tf1.location.x - tf0.location.x) * alpha,
                        y=tf0.location.y + (tf1.location.y - tf0.location.y) * alpha,
                        z=tf0.location.z + (tf1.location.z - tf0.location.z) * alpha,
                    )

                    def lerp_angle(a0, a1, t):
                        # shortest signed delta
                        d = (a1 - a0 + 180.0) % 360.0 - 180.0
                        return a0 + d * t

                    rot = carla.Rotation(
                        pitch=lerp_angle(tf0.rotation.pitch, tf1.rotation.pitch, alpha),
                        yaw=lerp_angle(tf0.rotation.yaw, tf1.rotation.yaw, alpha),
                        roll=lerp_angle(tf0.rotation.roll, tf1.rotation.roll, alpha),
                    )

                    tf = carla.Transform(loc, rot)
                    self._set_tf(tf)

                    seg_tick += 1
                    if seg_tick >= seg_len:
                        seg_tick = 0
                        point_idx += 1
                else:
                    # last point
                    self._set_tf(tf0)
                    point_idx += 1
                    seg_tick = 0
                    tf = tf0

                # 回放时也绘制视椎体（短生命周期，避免堆满）
                if self.viz_frustum:
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

                # fetch latest image
                image = self.last_image
                if image is not None:
                    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))[:, :, :3]
                    # visualize nav target: ego(t+1) projected to t
                    nav = p0.get("nav_waypoint") or {}
                    uv = nav.get("t1_image_uv")
                    if cv2 is not None and uv is not None:
                        arr = draw_cross(arr, uv[0], uv[1])

                    surface = pygame.surfarray.make_surface(arr.swapaxes(0, 1))
                    display.blit(surface, (0, 0))
                    pygame.display.flip()

                    if self.record_video and self.video_writer is not None and cv2 is not None:
                        self.video_writer.write(arr)

                    # 抓拍：按“轨迹点 index”只保存一次，避免插值阶段反复覆盖同一帧
                    if self.save_frames and point_idx != self._last_saved_point_idx:
                        self._last_saved_point_idx = point_idx
                        out_folder = os.path.join(self.output_dir, f"frame_{point_idx:05d}")
                        os.makedirs(out_folder, exist_ok=True)
                        fmt = (self.capture_format or "webp").lower().lstrip(".")
                        rgb_name = f"rgb.{fmt}"
                        ann_name = f"annotated_rgb.{fmt}"
                        _save_array_image(arr, os.path.join(out_folder, rgb_name), fmt, quality=int(self.capture_quality), lossless=bool(self.capture_lossless))
                        # annotated: draw nav cross
                        ann = arr
                        nav = p0.get("nav_waypoint") or {}
                        uv = nav.get("t1_image_uv")
                        if uv is not None and cv2 is not None:
                            ann = draw_cross(ann, uv[0], uv[1])
                        _save_array_image(ann, os.path.join(out_folder, ann_name), fmt, quality=int(self.capture_quality), lossless=bool(self.capture_lossless))
                        meta = {
                            "frame_id": point_idx,
                            "rgb_file": rgb_name,
                            "annotated_rgb_file": ann_name,
                            "drone_pose": {"x": tf.location.x, "y": tf.location.y, "z": tf.location.z,
                                           "pitch": tf.rotation.pitch, "yaw": tf.rotation.yaw, "roll": tf.rotation.roll},
                            "camera_meta": {
                                "fov": self.fov,
                                "image_size": {"width": self.img_w, "height": self.img_h},
                                "intrinsic": intrinsic.tolist(),
                                "world_to_camera": np.linalg.inv(get_matrix(tf)).tolist(),
                            },
                            "nav_waypoint": nav,
                        }
                        with open(os.path.join(out_folder, "meta.json"), "w", encoding="utf-8") as f:
                            json.dump(meta, f, indent=4, ensure_ascii=False)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        running = False

                if realtime:
                    # 逼近真实时间播放；若 playback_speed < 1，可进一步放慢
                    time.sleep(max(0.0, dt / max(1e-6, playback_speed)))

        finally:
            try:
                if self.camera:
                    self.camera.stop()
            except Exception:
                pass
            if self.video_writer is not None:
                try:
                    self.video_writer.release()
                except Exception:
                    pass
            for a in self.actor_list:
                try:
                    a.destroy()
                except Exception:
                    pass
            pygame.quit()
            self._restore_settings()


def parse_args():
    p = argparse.ArgumentParser(description="Drone trajectory playback")
    p.add_argument("traj", type=str, help="trajectory json produced by drone_trajectory_collector.py")
    p.add_argument("--host", type=str, default="localhost")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--output-dir", type=str, default=None)
    p.add_argument("--save-frames", action="store_true", help="save rgb/meta for each point")
    p.add_argument("--capture", action="store_true", help="alias of --save-frames")
    p.add_argument("--capture-format", type=str, default="webp", choices=["webp", "png", "jpg"], help="image format for capture")
    p.add_argument("--capture-quality", type=int, default=80, help="webp/jpg quality (1-100)")
    p.add_argument("--capture-lossless", action="store_true", help="use lossless for webp if supported (bigger but no artifacts)")
    p.add_argument("--disable-postprocess", action="store_true", help="disable UE postprocess on RGB camera to reduce grain/noise")
    p.add_argument("--record-video", action="store_true", help="record mp4 from the camera view")
    p.add_argument("--record-path", type=str, default=None)
    p.add_argument("--no-frustum", action="store_true", help="disable frustum viz during playback")
    p.add_argument("--viz-frustum-life", type=float, default=8.0, help="frustum life_time in seconds")
    p.add_argument("--viz-frustum-near", type=float, default=0.4)
    p.add_argument("--viz-frustum-far", type=float, default=0.5)
    p.add_argument("--viz-frustum-thickness", type=float, default=0.02)
    p.add_argument("--viz-frustum-color", type=str, default="0,90,140", help="r,g,b")
    p.add_argument("--playback-speed", type=float, default=1.0, help=">1 faster, <1 slower (affects both timing and realtime sleep)")
    p.add_argument("--no-realtime", action="store_true", help="disable dt sleep; run as fast as possible")
    p.add_argument("--ticks-per-point", type=int, default=10, help="fallback interpolation ticks per point if sim_step delta missing")
    p.add_argument("--uniform-speed", action="store_true", help="ignore sim_step gaps; use a fixed ticks-per-point for all segments")
    p.add_argument("--min-seg-len", type=int, default=1, help="min interpolation ticks per segment (clamp)")
    p.add_argument("--max-seg-len", type=int, default=200, help="max interpolation ticks per segment (clamp)")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    pb = DroneTrajectoryPlayback(
        traj_path=args.traj,
        output_dir=args.output_dir,
        save_frames=(args.save_frames or args.capture),
        record_video=args.record_video,
        record_path=args.record_path,
        host=args.host,
        port=args.port,
    )
    pb.viz_frustum = not args.no_frustum
    pb.viz_frustum_life = args.viz_frustum_life
    pb.viz_frustum_near = args.viz_frustum_near
    pb.viz_frustum_far = args.viz_frustum_far
    pb.viz_frustum_thickness = args.viz_frustum_thickness
    pb.viz_frustum_color = tuple(parse_color(args.viz_frustum_color, (0, 90, 140)))
    pb.capture_format = args.capture_format
    pb.capture_quality = args.capture_quality
    pb.capture_lossless = bool(args.capture_lossless)
    pb.disable_postprocess = bool(args.disable_postprocess)
    pb.playback_speed = args.playback_speed
    pb.realtime = not args.no_realtime
    pb.ticks_per_point = args.ticks_per_point
    pb.use_sim_step = not args.uniform_speed
    pb.min_seg_len = args.min_seg_len
    pb.max_seg_len = args.max_seg_len
    if pb.record_video:
        print(f"[INFO] video output -> {pb.record_path}")
    pb.run()



    