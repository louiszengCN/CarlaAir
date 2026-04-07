#!/usr/bin/env python3
"""
demo_flight_city.py — 无人机飞越城市 + CARLA 实时交通 + 传感器可视化

功能:
  1. 无人机沿圆形航线自动飞行
  2. 地面同步运行 CARLA 车辆(自动驾驶) 和行人(AI步行)
  3. Pygame 窗口实时显示 RGB / 深度 / 语义分割 三路传感器
  4. 底部 HUD 显示飞行遥测和状态

用法:
    ./SimWorld.sh Town10HD          # 先启动模拟器
    python3 demo_flight_city.py     # 再运行本脚本

按键:
    ESC/Q   退出        P   暂停/恢复飞行
    W       切换天气    C   截图
    +/-     加速/减速   R   重置无人机位置
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
    sys.exit("需要 airsim: pip install airsim")


# ── shared state ─────────────────────────────────────────────────────
g_lock = threading.Lock()
g_rgb = None          # numpy (H,W,3) RGB
g_depth = None        # numpy (H,W,3) pseudo-color
g_seg = None          # numpy (H,W,3) segmentation
g_telem = {}          # telemetry dict
g_running = True
g_paused = False
g_speed = 8.0         # m/s


# ── CARLA traffic ────────────────────────────────────────────────────
WEATHERS = [
    ("ClearNoon",    carla.WeatherParameters.ClearNoon),
    ("CloudyNoon",   carla.WeatherParameters.CloudyNoon),
    ("HardRainNoon", carla.WeatherParameters.HardRainNoon),
    ("ClearSunset",  carla.WeatherParameters.ClearSunset),
]


class TrafficManager:
    def __init__(self, client):
        self.world = client.get_world()
        self.bp = self.world.get_blueprint_library()
        self.spawns = self.world.get_map().get_spawn_points()
        self.vehicles = []
        self.walkers = []
        self.w_ctrls = []
        self.w_idx = 0

    # ─── weather ───
    def set_weather(self, idx):
        self.w_idx = idx % len(WEATHERS)
        self.world.set_weather(WEATHERS[self.w_idx][1])
        return WEATHERS[self.w_idx][0]

    def next_weather(self):
        return self.set_weather(self.w_idx + 1)

    # ─── vehicles ───
    def spawn_vehicles(self, n):
        import random
        bps = list(self.bp.filter("vehicle.*"))
        pts = list(self.spawns); random.shuffle(pts)
        ok = 0
        for sp in pts:
            if ok >= n:
                break
            bp = random.choice(bps)
            if bp.has_attribute("color"):
                bp.set_attribute("color", random.choice(
                    bp.get_attribute("color").recommended_values))
            try:
                v = self.world.spawn_actor(bp, sp)
                v.set_autopilot(True)
                self.vehicles.append(v)
                ok += 1
            except Exception:
                pass
        return ok

    # ─── walkers ───
    def spawn_walkers(self, n):
        import random
        bps = list(self.bp.filter("walker.pedestrian.*"))
        ctrl_bp = self.bp.find("controller.ai.walker")
        ok = 0
        for _ in range(n * 4):
            if ok >= n:
                break
            bp = random.choice(bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            loc = self.world.get_random_location_from_navigation()
            if loc is None:
                continue
            try:
                w = self.world.spawn_actor(bp, carla.Transform(loc))
                self.walkers.append(w)
                ok += 1
            except Exception:
                pass
        time.sleep(0.3)
        for w in self.walkers:
            try:
                c = self.world.spawn_actor(ctrl_bp, carla.Transform(), w)
                self.w_ctrls.append(c)
            except Exception:
                self.w_ctrls.append(None)
        time.sleep(0.2)
        for c in self.w_ctrls:
            if c is None:
                continue
            try:
                c.start()
                dest = self.world.get_random_location_from_navigation()
                if dest:
                    c.go_to_location(dest)
                c.set_max_speed(1.0 + np.random.random())
            except Exception:
                pass
        return ok

    # ─── cleanup ───
    def destroy_all(self):
        for c in self.w_ctrls:
            try:
                if c: c.stop(); c.destroy()
            except Exception:
                pass
        for a in self.walkers + self.vehicles:
            try:
                a.destroy()
            except Exception:
                pass
        self.vehicles.clear()
        self.walkers.clear()
        self.w_ctrls.clear()


# ── image helpers ────────────────────────────────────────────────────
def decode_rgb(resp):
    """AirSim Scene → numpy (H,W,3) RGB"""
    if resp.width < 1:
        return None
    buf = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
    n_pixels = resp.height * resp.width
    ch = len(buf) // n_pixels          # 3 (RGB) or 4 (BGRA)
    img = buf.reshape(resp.height, resp.width, ch)
    if ch == 4:                        # BGRA → RGB
        img = img[:, :, [2, 1, 0]]
    elif ch == 3:                      # BGR → RGB  (some builds)
        img = img[:, :, [2, 1, 0]]
    return np.ascontiguousarray(img)


def decode_depth(resp):
    """AirSim DepthPerspective → pseudo-color numpy (H,W,3)"""
    if resp.width < 1:
        return None
    d = airsim.list_to_2d_float_array(
        resp.image_data_float, resp.width, resp.height)
    d = np.clip(d, 0, 100)
    n = (d / 100.0 * 255).astype(np.uint8)
    out = np.zeros((resp.height, resp.width, 3), dtype=np.uint8)
    out[:, :, 0] = n                                       # R = far
    out[:, :, 1] = np.clip(128 - np.abs(n.astype(int) - 128),
                            0, 255).astype(np.uint8)
    out[:, :, 2] = 255 - n                                 # B = near
    return out


def decode_seg(resp):
    """AirSim Segmentation → numpy (H,W,3) RGB"""
    if resp.width < 1:
        return None
    buf = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
    n_pixels = resp.height * resp.width
    ch = len(buf) // n_pixels
    img = buf.reshape(resp.height, resp.width, ch)
    if ch == 4:
        img = img[:, :, :3]
    return np.ascontiguousarray(img)


def np_to_surface(img, tw, th):
    """numpy (H,W,3) → pygame Surface scaled to (tw, th)"""
    h, w = img.shape[:2]
    surf = pygame.image.frombuffer(img.tobytes(), (w, h), "RGB")
    if w != tw or h != th:
        surf = pygame.transform.scale(surf, (tw, th))
    return surf


# ── flight thread ────────────────────────────────────────────────────
def flight_loop(ac, altitude, radius, n_wp=36):
    global g_rgb, g_depth, g_seg, g_telem, g_running, g_paused, g_speed

    wps = []
    for i in range(n_wp):
        a = 2 * math.pi * i / n_wp
        wps.append((radius * math.cos(a), radius * math.sin(a), -altitude))
    idx = 0

    while g_running:
        try:
            # ── move ──
            if not g_paused:
                x, y, z = wps[idx]
                ac.moveToPositionAsync(x, y, z, g_speed)
                idx = (idx + 1) % n_wp

            # ── images ──
            resps = ac.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene,      False, False),
                airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
                airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
            ])
            rgb   = decode_rgb(resps[0])   if resps and len(resps) > 0 else None
            depth = decode_depth(resps[1]) if resps and len(resps) > 1 else None
            seg   = decode_seg(resps[2])   if resps and len(resps) > 2 else None

            # ── telemetry ──
            st = ac.getMultirotorState()
            p = st.kinematics_estimated.position
            v = st.kinematics_estimated.linear_velocity
            o = st.kinematics_estimated.orientation
            spd = math.sqrt(v.x_val**2 + v.y_val**2 + v.z_val**2)
            yaw = math.degrees(math.atan2(
                2*(o.w_val*o.z_val + o.x_val*o.y_val),
                1 - 2*(o.y_val**2 + o.z_val**2)))

            with g_lock:
                g_rgb, g_depth, g_seg = rgb, depth, seg
                g_telem = {"x": p.x_val, "y": p.y_val, "z": p.z_val,
                               "alt": -p.z_val, "spd": spd, "yaw": yaw,
                               "wp": idx, "wp_n": n_wp}
        except Exception:
            pass

        time.sleep(0.2)   # ~5 Hz


# ── pygame main ──────────────────────────────────────────────────────
def run_display(tm):
    global g_running, g_paused, g_speed

    PW, PH = 480, 270        # panel size
    HUD = 80
    W, H = PW * 3, PH + HUD

    pygame.init()
    scr = pygame.display.set_mode((W, H))
    pygame.display.set_caption(
        "SimWorld — 无人机城市巡航  ESC退出 P暂停 W天气 +/-速度 C截图")
    clk = pygame.time.Clock()

    font   = pygame.font.SysFont("monospace", 15)
    font_b = pygame.font.SysFont("monospace", 16, bold=True)

    # placeholder
    ph = pygame.Surface((PW, PH)); ph.fill((25, 25, 30))
    t = font_b.render("waiting...", True, (60, 60, 60))
    ph.blit(t, (PW//2 - t.get_width()//2, PH//2 - 8))

    weather_name = WEATHERS[tm.w_idx][0]
    shot = 0

    while g_running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                g_running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    g_running = False
                elif ev.key == pygame.K_p:
                    g_paused = not g_paused
                elif ev.key == pygame.K_w:
                    weather_name = tm.next_weather()
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    g_speed = min(g_speed + 2, 25)
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    g_speed = max(g_speed - 2, 2)
                elif ev.key == pygame.K_c:
                    p = f"/tmp/simworld_shot_{shot}.png"
                    pygame.image.save(scr, p); shot += 1
                    print(f"[截图] {p}")
                elif ev.key == pygame.K_r:
                    g_paused = True  # will be unpaused after reset

        with g_lock:
            rgb, depth, seg = g_rgb, g_depth, g_seg
            tel = dict(g_telem)

        # ── 3 panels ──
        labels = ["RGB Camera", "Depth (pseudo-color)", "Segmentation"]
        imgs   = [rgb, depth, seg]
        for i in range(3):
            xo = i * PW
            if imgs[i] is not None:
                scr.blit(np_to_surface(imgs[i], PW, PH), (xo, 0))
            else:
                scr.blit(ph, (xo, 0))
            # label
            lb = font_b.render(labels[i], True, (255, 255, 0))
            bg = pygame.Surface((lb.get_width()+8, lb.get_height()+4), pygame.SRCALPHA)
            bg.fill((0, 0, 0, 170))
            scr.blit(bg, (xo+4, 4))
            scr.blit(lb, (xo+8, 6))

        # ── HUD ──
        pygame.draw.rect(scr, (18, 18, 28), (0, PH, W, HUD))
        pygame.draw.line(scr, (50, 50, 70), (0, PH), (W, PH), 2)

        G = (0, 255, 140)
        Y = (255, 210, 60)
        GR = (160, 160, 180)

        if tel:
            def txt(s, c, x, y):
                scr.blit(font.render(s, True, c), (x, y))

            y1, y2, y3 = PH+8, PH+30, PH+52

            txt(f"ALT {tel.get('alt',0):5.1f}m",     G,  15, y1)
            txt(f"SPD {tel.get('spd',0):4.1f}m/s",   G, 175, y1)
            txt(f"HDG {tel.get('yaw',0):+6.0f}\u00b0",G, 340, y1)
            txt(f"POS ({tel.get('x',0):+.0f},{tel.get('y',0):+.0f})", GR, 500, y1)
            txt(f"WP {tel.get('wp',0)}/{tel.get('wp_n',0)}", GR, 720, y1)

            status = "\u25a0 PAUSED" if g_paused else "\u25b6 FLYING"
            sc = (255,80,80) if g_paused else (0,255,100)
            txt(status,                             sc, 15,  y2)
            txt(f"Weather: {weather_name}",         Y,  175, y2)
            txt(f"Set SPD: {g_speed:.0f}m/s",       GR, 420, y2)
            txt(f"Vehicles: {len(tm.vehicles)}",    GR, 600, y2)
            txt(f"Walkers: {len(tm.walkers)}",      GR, 760, y2)

            txt("ESC\u2006Quit  P\u2006Pause  W\u2006Weather  +/-\u2006Speed  C\u2006Screenshot",
                (60,60,80), 15, y3)

        pygame.display.flip()
        clk.tick(30)


# ── entry ────────────────────────────────────────────────────────────
def main():
    global g_running, g_speed

    ap = argparse.ArgumentParser()
    ap.add_argument("--host",         default="localhost")
    ap.add_argument("--port",         type=int,   default=2000)
    ap.add_argument("--airsim-port",  type=int,   default=41451)
    ap.add_argument("--altitude",     type=float, default=40)
    ap.add_argument("--radius",       type=float, default=80)
    ap.add_argument("--speed",        type=float, default=8)
    ap.add_argument("--vehicles",     type=int,   default=20)
    ap.add_argument("--walkers",      type=int,   default=15)
    ap.add_argument("--no-spawn",     action="store_true")
    a = ap.parse_args()
    g_speed = a.speed

    print("=" * 56)
    print("  SimWorld — 无人机城市巡航")
    print("=" * 56)

    # connect
    print("[1] CARLA ...", end=" ", flush=True)
    cc = carla.Client(a.host, a.port); cc.set_timeout(10)
    print(cc.get_world().get_map().name)

    print("[2] AirSim ...", end=" ", flush=True)
    ac = airsim.MultirotorClient(port=a.airsim_port)
    ac.confirmConnection(); print("OK")

    # traffic
    tm = TrafficManager(cc)
    tm.set_weather(0)
    if not a.no_spawn:
        print(f"[3] Spawning {a.vehicles} vehicles + {a.walkers} walkers ...")
        nv = tm.spawn_vehicles(a.vehicles)
        nw = tm.spawn_walkers(a.walkers)
        print(f"    vehicles={nv}  walkers={nw}")
    else:
        print("[3] --no-spawn, skipping traffic")

    # drone
    print(f"[4] Takeoff → {a.altitude}m ...", end=" ", flush=True)
    ac.enableApiControl(True); ac.armDisarm(True)
    ac.takeoffAsync().join()
    ac.moveToZAsync(-a.altitude, 5).join()
    print("OK")

    # start flight thread
    print(f"[5] Cruising  radius={a.radius}m  speed={a.speed}m/s")
    ft = threading.Thread(target=flight_loop,
                          args=(ac, a.altitude, a.radius), daemon=True)
    ft.start()

    # pygame (main thread)
    print("[6] Opening sensor window ...\n")
    try:
        run_display(tm)
    except Exception:
        traceback.print_exc()
    finally:
        g_running = False
        print("\n[cleanup] landing ...")
        try:
            ac.landAsync().join()
            ac.armDisarm(False)
            ac.enableApiControl(False)
        except Exception:
            pass
        print("[cleanup] destroying actors ...")
        tm.destroy_all()
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
