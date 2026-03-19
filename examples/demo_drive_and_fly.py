#!/usr/bin/env python3
"""
demo_drive_and_fly.py — 开车 + 无人机跟随 双视角演示

功能:
  1. CARLA 键盘驾驶 (WASD + 方向键)
  2. AirSim 无人机自动跟随车辆 (鸟瞰视角)
  3. Pygame 双视角: 左=车载相机  右=无人机航拍
  4. 底部 HUD 显示车辆和无人机状态

用法:
    ./SimWorld.sh Town10HD
    python3 demo_drive_and_fly.py

驾驶控制:
    W / ↑       油门 (加速)
    S / ↓       刹车 / 倒车
    A / ← D / → 左转 / 右转
    SPACE       手刹
    TAB         切换自动驾驶
    R           重生车辆
    1-4         切换车载视角 (前/后/左/右)

无人机控制:
    F           切换跟随模式 (跟随车辆 / 自由悬停)
    G           切换无人机高度 (30m / 50m / 80m)

通用:
    W_key(天气)  不冲突 — 用 Z 切换天气
    Z           切换天气
    C           截图
    ESC / Q     退出

可选参数:
    --host HOST          (默认 localhost)
    --port PORT          (默认 2000)
    --airsim-port PORT   (默认 41451)
    --vehicle BP         车辆蓝图 (默认 vehicle.tesla.model3)
    --no-drone           不启用无人机, 仅开车
    --no-traffic         不生成背景交通
"""

import argparse
import math
import sys
import threading
import time
import traceback

import numpy as np

try:
    import pygame
except ImportError:
    sys.exit("需要 pygame: pip install pygame")
try:
    import carla
except ImportError:
    sys.exit("需要 carla Python API")
try:
    import airsim
except ImportError:
    airsim = None  # drone features disabled


# ── constants ────────────────────────────────────────────────────────
WEATHERS = [
    ("ClearNoon",    carla.WeatherParameters.ClearNoon),
    ("CloudyNoon",   carla.WeatherParameters.CloudyNoon),
    ("WetNoon",      carla.WeatherParameters.WetNoon),
    ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
    ("ClearSunset",  carla.WeatherParameters.ClearSunset),
]

DRONE_ALTS = [30, 50, 80]  # meters


# ── CARLA ego vehicle ────────────────────────────────────────────────
class EgoVehicle:
    """CARLA 自驾车辆 + 传感器"""

    def __init__(self, world, bp_name="vehicle.tesla.model3"):
        self.world = world
        self.bp_lib = world.get_blueprint_library()
        self.vehicle = None
        self.camera = None
        self.image = None  # latest numpy (H,W,3)
        self._img_lock = threading.Lock()
        self.autopilot = False
        self.bp_name = bp_name

        # control state
        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0
        self.hand_brake = False
        self.reverse = False

    def spawn(self, sp_index=0):
        """生成车辆并挂载前置相机"""
        self.destroy()
        bp = self.bp_lib.find(self.bp_name)
        sps = self.world.get_map().get_spawn_points()
        if sp_index >= len(sps):
            sp_index = 0
        self.vehicle = self.world.spawn_actor(bp, sps[sp_index])

        # attach RGB camera
        cam_bp = self.bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "800")
        cam_bp.set_attribute("image_size_y", "450")
        cam_bp.set_attribute("fov", "100")
        # default: front view
        self._cam_transforms = [
            carla.Transform(carla.Location(x=1.8, z=1.3)),                      # front
            carla.Transform(carla.Location(x=-3.0, z=2.5),
                            carla.Rotation(yaw=180)),                            # rear
            carla.Transform(carla.Location(y=-2.0, z=1.5),
                            carla.Rotation(yaw=-90)),                            # left
            carla.Transform(carla.Location(y=2.0, z=1.5),
                            carla.Rotation(yaw=90)),                             # right
        ]
        self._cam_idx = 0
        self.camera = self.world.spawn_actor(
            cam_bp, self._cam_transforms[0], attach_to=self.vehicle)
        self.camera.listen(self._on_image)
        return self.vehicle

    def switch_camera(self, idx):
        """切换车载相机视角 (0-3)"""
        if self.camera is None or self.vehicle is None:
            return
        idx = idx % len(self._cam_transforms)
        if idx == self._cam_idx:
            return
        self._cam_idx = idx
        # destroy and recreate camera at new position
        self.camera.stop()
        self.camera.destroy()
        cam_bp = self.bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "800")
        cam_bp.set_attribute("image_size_y", "450")
        cam_bp.set_attribute("fov", "100")
        self.camera = self.world.spawn_actor(
            cam_bp, self._cam_transforms[idx], attach_to=self.vehicle)
        self.camera.listen(self._on_image)

    def _on_image(self, image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape(image.height, image.width, 4)[:, :, :3]  # BGRA→BGR
        arr = arr[:, :, ::-1]  # BGR→RGB
        with self._img_lock:
            self.image = np.ascontiguousarray(arr)

    def get_image(self):
        with self._img_lock:
            return self.image

    def toggle_autopilot(self):
        self.autopilot = not self.autopilot
        if self.vehicle:
            self.vehicle.set_autopilot(self.autopilot)

    def apply_control(self):
        if self.vehicle is None or self.autopilot:
            return
        ctrl = carla.VehicleControl()
        ctrl.throttle = self.throttle
        ctrl.steer = self.steer
        ctrl.brake = self.brake
        ctrl.hand_brake = self.hand_brake
        ctrl.reverse = self.reverse
        self.vehicle.apply_control(ctrl)

    def get_transform(self):
        if self.vehicle:
            return self.vehicle.get_transform()
        return None

    def get_velocity_kmh(self):
        if self.vehicle:
            v = self.vehicle.get_velocity()
            return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        return 0

    def get_gear(self):
        if self.vehicle:
            ctrl = self.vehicle.get_control()
            if ctrl.reverse:
                return "R"
            return str(ctrl.gear) if ctrl.gear > 0 else "N"
        return "?"

    def destroy(self):
        if self.camera:
            try:
                self.camera.stop()
                self.camera.destroy()
            except Exception:
                pass
            self.camera = None
        if self.vehicle:
            try:
                self.vehicle.destroy()
            except Exception:
                pass
            self.vehicle = None


# ── Drone follower ───────────────────────────────────────────────────
class DroneFollower:
    """AirSim 无人机 — 相对跟随 CARLA 车辆

    坐标映射: CARLA(x,y,z-up) → AirSim NED(x,y,z-down)
    使用相对偏移: 记录车辆初始位置, 追踪增量, 同步到无人机
    """

    def __init__(self, port=41451):
        if airsim is None:
            self.client = None
            return
        self.client = airsim.MultirotorClient(port=port)
        self.client.confirmConnection()
        self.following = True
        self.alt_idx = 0
        self.altitude = DRONE_ALTS[0]
        self.image = None
        self._lock = threading.Lock()
        # calibration: will be set in calibrate()
        self._car_origin = None   # CARLA (x, y) at calibration
        self._drone_origin = None # AirSim (x, y) at calibration

    @property
    def available(self):
        return self.client is not None

    def setup(self):
        if not self.available:
            return
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-self.altitude, 5).join()
        time.sleep(0.5)
        # record drone home position
        p = self.client.getMultirotorState().kinematics_estimated.position
        self._drone_origin = (p.x_val, p.y_val)

    def calibrate(self, car_transform):
        """记录车辆初始 CARLA 位置, 用于计算相对偏移"""
        if car_transform:
            loc = car_transform.location
            self._car_origin = (loc.x, loc.y)

    def toggle_follow(self):
        self.following = not self.following

    def cycle_altitude(self):
        self.alt_idx = (self.alt_idx + 1) % len(DRONE_ALTS)
        self.altitude = DRONE_ALTS[self.alt_idx]
        return self.altitude

    def update(self, car_transform):
        """更新: 相对跟随车辆 + 采集图像"""
        if not self.available:
            return

        # ── relative follow ──
        if (self.following and car_transform is not None
                and self._car_origin is not None
                and self._drone_origin is not None):
            loc = car_transform.location
            # delta in CARLA meters
            dx = loc.x - self._car_origin[0]
            dy = loc.y - self._car_origin[1]
            # apply to drone (same XY axes, keep altitude in NED)
            target_x = self._drone_origin[0] + dx
            target_y = self._drone_origin[1] + dy
            target_z = -self.altitude
            self.client.moveToPositionAsync(
                target_x, target_y, target_z, 15,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, 0))

        # ── capture aerial image ──
        try:
            resps = self.client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
            ])
            if resps and resps[0].width > 0:
                buf = np.frombuffer(resps[0].image_data_uint8, dtype=np.uint8)
                n = resps[0].height * resps[0].width
                ch = len(buf) // n
                img = buf.reshape(resps[0].height, resps[0].width, ch)
                if ch >= 3:
                    img = img[:, :, [2, 1, 0]]  # BGR(A) → RGB
                with self._lock:
                    self.image = np.ascontiguousarray(img[:, :, :3])
        except Exception:
            pass

    def get_image(self):
        with self._lock:
            return self.image

    def get_position(self):
        if not self.available:
            return None
        try:
            st = self.client.getMultirotorState()
            p = st.kinematics_estimated.position
            return (p.x_val, p.y_val, -p.z_val)
        except Exception:
            return None

    def land(self):
        if not self.available:
            return
        try:
            self.client.moveToZAsync(-3, 3).join()
            self.client.landAsync().join()
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
        except Exception:
            pass


# ── Background traffic ───────────────────────────────────────────────
def spawn_traffic(world, n_vehicles=15, n_walkers=8):
    """生成背景交通。
    注意: walker AI controllers 与 AirSim 插件冲突 (segfault),
    因此行人只作为静态装饰存在, 不带 AI 导航。
    """
    import random
    bp_lib = world.get_blueprint_library()
    sps = world.get_map().get_spawn_points()
    actors = []

    # vehicles (with autopilot — works fine)
    vbps = list(bp_lib.filter("vehicle.*"))
    pts = list(sps[5:])  # skip first few for ego
    random.shuffle(pts)
    for sp in pts[:n_vehicles]:
        bp = random.choice(vbps)
        if bp.has_attribute("color"):
            bp.set_attribute("color", random.choice(
                bp.get_attribute("color").recommended_values))
        try:
            v = world.spawn_actor(bp, sp)
            v.set_autopilot(True)
            actors.append(v)
        except Exception:
            pass

    # walkers (static — NO AI controllers to avoid AirSim crash)
    wbps = list(bp_lib.filter("walker.pedestrian.*"))
    for _ in range(n_walkers * 3):
        if len(actors) >= n_vehicles + n_walkers:
            break
        bp = random.choice(wbps)
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        try:
            w = world.spawn_actor(bp, carla.Transform(loc))
            actors.append(w)
        except Exception:
            pass

    return actors


# ── Drone update thread ──────────────────────────────────────────────
def drone_thread(drone, ego, running_flag):
    while running_flag[0]:
        try:
            t = ego.get_transform()
            drone.update(t)
        except Exception:
            pass
        time.sleep(0.15)


# ── input handling ───────────────────────────────────────────────────
def process_input(ego):
    """读取 pygame 按键状态 → 更新车辆控制"""
    keys = pygame.key.get_pressed()

    # throttle / brake
    if keys[pygame.K_w] or keys[pygame.K_UP]:
        ego.throttle = min(ego.throttle + 0.05, 1.0)
        ego.reverse = False
    else:
        ego.throttle = max(ego.throttle - 0.05, 0.0)

    if keys[pygame.K_s] or keys[pygame.K_DOWN]:
        ego.brake = min(ego.brake + 0.1, 1.0)
        # if nearly stopped, switch to reverse
        if ego.get_velocity_kmh() < 2.0:
            ego.reverse = True
            ego.brake = 0.0
            ego.throttle = 0.4
    else:
        ego.brake = max(ego.brake - 0.1, 0.0)
        if not (keys[pygame.K_w] or keys[pygame.K_UP]):
            ego.reverse = False

    # steering
    if keys[pygame.K_a] or keys[pygame.K_LEFT]:
        ego.steer = max(ego.steer - 0.04, -1.0)
    elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
        ego.steer = min(ego.steer + 0.04, 1.0)
    else:
        # return to center
        if ego.steer > 0.02:
            ego.steer -= 0.06
        elif ego.steer < -0.02:
            ego.steer += 0.06
        else:
            ego.steer = 0.0

    ego.hand_brake = keys[pygame.K_SPACE]
    ego.apply_control()


# ── numpy → pygame ───────────────────────────────────────────────────
def np_to_surf(img, tw, th):
    if img is None:
        return None
    h, w = img.shape[:2]
    s = pygame.image.frombuffer(img.tobytes(), (w, h), "RGB")
    if w != tw or h != th:
        s = pygame.transform.scale(s, (tw, th))
    return s


# ── main pygame loop ─────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host",        default="localhost")
    ap.add_argument("--port",        type=int, default=2000)
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--vehicle",     default="vehicle.tesla.model3")
    ap.add_argument("--no-drone",    action="store_true")
    ap.add_argument("--no-traffic",  action="store_true")
    a = ap.parse_args()

    print("=" * 56)
    print("  SimWorld — 开车 + 无人机跟随")
    print("=" * 56)

    # ── connect ──
    print("[1] CARLA ...", end=" ", flush=True)
    cc = carla.Client(a.host, a.port)
    cc.set_timeout(10)
    world = cc.get_world()
    print(world.get_map().name)

    use_drone = not a.no_drone and airsim is not None
    drone = None
    d_thread = None
    running = [True]

    # ── ego vehicle ──
    print(f"[2] Spawning ego: {a.vehicle} ...", end=" ", flush=True)
    ego = EgoVehicle(world, a.vehicle)
    ego.spawn(sp_index=0)
    print("OK")

    # ── traffic (BEFORE AirSim — avoids client conflicts) ──
    traffic_actors = []
    if not a.no_traffic:
        print("[3] Spawning traffic ...", end=" ", flush=True)
        traffic_actors = spawn_traffic(world, 15, 10)
        print(f"{len(traffic_actors)} actors")
    else:
        print("[3] Skipping traffic (--no-traffic)")

    # ── drone (AFTER all CARLA spawning) ──
    if use_drone:
        print("[4] AirSim drone ...", end=" ", flush=True)
        try:
            time.sleep(1)  # let physics settle after spawns
            drone = DroneFollower(port=a.airsim_port)
            drone.setup()
            drone.calibrate(ego.get_transform())
            print(f"OK (following at {drone.altitude}m)")
            d_thread = threading.Thread(
                target=drone_thread, args=(drone, ego, running), daemon=True)
            d_thread.start()
        except Exception as e:
            print(f"FAILED ({e}), continuing without drone")
            drone = None
            use_drone = False
    else:
        print("[4] Drone disabled")

    # ── weather ──
    w_idx = 0
    world.set_weather(WEATHERS[w_idx][1])
    weather_name = WEATHERS[w_idx][0]

    # ── pygame ──
    pygame.init()

    if use_drone:
        PW, PH = 640, 360  # each panel
        WIN_W = PW * 2
    else:
        PW, PH = 960, 540
        WIN_W = PW

    HUD_H = 70
    WIN_H = PH + HUD_H

    scr = pygame.display.set_mode((WIN_W, WIN_H))
    title = "SimWorld — 开车"
    if use_drone:
        title += " + 无人机跟随"
    title += "  |  WASD=驾驶  TAB=自动  ESC=退出"
    pygame.display.set_caption(title)
    clk = pygame.time.Clock()

    font = pygame.font.SysFont("monospace", 14)
    font_b = pygame.font.SysFont("monospace", 15, bold=True)

    # placeholder
    ph = pygame.Surface((PW, PH))
    ph.fill((25, 25, 30))
    t = font_b.render("waiting for camera...", True, (60, 60, 60))
    ph.blit(t, (PW // 2 - t.get_width() // 2, PH // 2 - 8))

    cam_names = ["Front", "Rear", "Left", "Right"]
    cam_idx = 0
    shot = 0

    print("\n[Ready] 用 WASD / 方向键 驾驶，TAB 切换自动驾驶\n")

    try:
        while running[0]:
            # ── events ──
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running[0] = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                        running[0] = False
                    elif ev.key == pygame.K_TAB:
                        ego.toggle_autopilot()
                    elif ev.key == pygame.K_z:
                        w_idx = (w_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[w_idx][1])
                        weather_name = WEATHERS[w_idx][0]
                    elif ev.key == pygame.K_r:
                        ego.spawn(sp_index=np.random.randint(0, 50))
                        cam_idx = 0
                    elif ev.key == pygame.K_c:
                        p = f"/tmp/simworld_drive_{shot}.png"
                        pygame.image.save(scr, p)
                        shot += 1
                    elif ev.key == pygame.K_f and drone:
                        drone.toggle_follow()
                    elif ev.key == pygame.K_g and drone:
                        alt = drone.cycle_altitude()
                    elif ev.key == pygame.K_1:
                        cam_idx = 0; ego.switch_camera(0)
                    elif ev.key == pygame.K_2:
                        cam_idx = 1; ego.switch_camera(1)
                    elif ev.key == pygame.K_3:
                        cam_idx = 2; ego.switch_camera(2)
                    elif ev.key == pygame.K_4:
                        cam_idx = 3; ego.switch_camera(3)

            # ── input ──
            process_input(ego)

            # ── render car camera ──
            car_img = ego.get_image()
            if car_img is not None:
                scr.blit(np_to_surf(car_img, PW, PH), (0, 0))
            else:
                scr.blit(ph, (0, 0))

            # label
            lb = font_b.render(
                f"Car: {cam_names[cam_idx]}"
                + (" [AUTOPILOT]" if ego.autopilot else ""),
                True, (255, 255, 0))
            bg = pygame.Surface((lb.get_width() + 8, lb.get_height() + 4),
                                pygame.SRCALPHA)
            bg.fill((0, 0, 0, 170))
            scr.blit(bg, (4, 4))
            scr.blit(lb, (8, 6))

            # ── render drone camera ──
            if use_drone:
                drone_img = drone.get_image() if drone else None
                if drone_img is not None:
                    scr.blit(np_to_surf(drone_img, PW, PH), (PW, 0))
                else:
                    scr.blit(ph, (PW, 0))

                mode = "FOLLOW" if drone.following else "HOVER"
                dl = font_b.render(
                    f"Drone: {drone.altitude}m [{mode}]", True, (0, 255, 200))
                dbg = pygame.Surface((dl.get_width() + 8, dl.get_height() + 4),
                                     pygame.SRCALPHA)
                dbg.fill((0, 0, 0, 170))
                scr.blit(dbg, (PW + 4, 4))
                scr.blit(dl, (PW + 8, 6))

            # ── HUD ──
            pygame.draw.rect(scr, (18, 18, 28), (0, PH, WIN_W, HUD_H))
            pygame.draw.line(scr, (50, 50, 70), (0, PH), (WIN_W, PH), 2)

            G = (0, 255, 140)
            Y = (255, 210, 60)
            R = (255, 80, 80)
            GR = (160, 160, 180)

            y1, y2, y3 = PH + 6, PH + 24, PH + 44

            def txt(s, c, x, y):
                scr.blit(font.render(s, True, c), (x, y))

            spd = ego.get_velocity_kmh()
            gear = ego.get_gear()
            t_val = ego.get_transform()
            loc_str = ""
            if t_val:
                loc_str = f"({t_val.location.x:.0f},{t_val.location.y:.0f})"

            # row 1: car info
            txt(f"Speed: {spd:5.1f} km/h", G, 15, y1)
            txt(f"Gear: {gear}", G, 200, y1)
            txt(f"Steer: {ego.steer:+.2f}", GR, 290, y1)
            txt(f"Pos: {loc_str}", GR, 430, y1)
            mode_str = "AUTOPILOT" if ego.autopilot else "MANUAL"
            mc = Y if ego.autopilot else G
            txt(f"[{mode_str}]", mc, 620, y1)

            # throttle/brake bar
            bar_x, bar_y, bar_w, bar_h = 770, y1, 80, 12
            pygame.draw.rect(scr, (40, 40, 40), (bar_x, bar_y, bar_w, bar_h))
            tw = int(ego.throttle * bar_w)
            bw = int(ego.brake * bar_w)
            if tw > 0:
                pygame.draw.rect(scr, (0, 200, 0), (bar_x, bar_y, tw, bar_h))
            if bw > 0:
                pygame.draw.rect(scr, (200, 0, 0),
                                 (bar_x + bar_w - bw, bar_y, bw, bar_h))

            # row 2: drone + weather
            txt(f"Weather: {weather_name}", Y, 15, y2)
            if drone:
                dp = drone.get_position()
                ds = f"Drone: alt={drone.altitude}m"
                if dp:
                    ds += f" ({dp[0]:.0f},{dp[1]:.0f})"
                ds += f" [{'FOLLOW' if drone.following else 'HOVER'}]"
                txt(ds, (0, 200, 200), 250, y2)

            # row 3: controls
            txt("WASD=Drive  TAB=AutoPilot  1-4=Camera  Z=Weather  "
                "F=Follow  G=Altitude  R=Respawn  C=Shot  ESC=Quit",
                (55, 55, 75), 15, y3)

            pygame.display.flip()
            clk.tick(30)

    except Exception:
        traceback.print_exc()
    finally:
        running[0] = False
        print("\n[cleanup] ...")
        ego.destroy()
        if drone:
            drone.land()
        for a_actor in traffic_actors:
            try:
                a_actor.destroy()
            except Exception:
                pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
