#!/usr/bin/env python3
"""
switch_maps_v2.py — 简洁可靠的 CarlaAir 地图切换工具
=====================================================
纯同步，无 asyncio/threading，不开额外窗口。

用法:
    # 列出所有可用地图
    python3 examples/switch_maps_v2.py --list

    # 切换到指定地图
    python3 examples/switch_maps_v2.py Town01
    python3 examples/switch_maps_v2.py Town03

    # 遍历所有地图（每张停留 10 秒）
    python3 examples/switch_maps_v2.py --all --stay 10

    # 切换后移动 spectator 到鸟瞰位置
    python3 examples/switch_maps_v2.py Town10HD --orbit
"""

import carla
import argparse
import math
import sys
import time


def cleanup_world(world):
    """切换前清理所有传感器和生成的车辆，避免残留导致卡死。"""
    count = 0
    for a in world.get_actors().filter("sensor.*"):
        try:
            a.stop()
            a.destroy()
            count += 1
        except:
            pass
    for a in world.get_actors().filter("vehicle.*"):
        try:
            a.destroy()
            count += 1
        except:
            pass
    for a in world.get_actors().filter("walker.*"):
        try:
            a.destroy()
            count += 1
        except:
            pass
    if count:
        print(f"  Cleaned up {count} actors", flush=True)


def ensure_async_mode(world):
    """确保世界在异步模式（同步模式下 load_world 会卡死）。"""
    settings = world.get_settings()
    if settings.synchronous_mode:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("  Reset to async mode", flush=True)
        time.sleep(0.5)


def wait_world_ready(client, timeout=120.0):
    """等待新地图加载完毕，能拿到 spawn_points 说明就绪。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            world = client.get_world()
            sps = world.get_map().get_spawn_points()
            if sps:
                return world, sps
        except Exception:
            pass
        time.sleep(0.5)
    raise TimeoutError("World not ready after load")


def switch_map(client, map_name, timeout=300.0):
    """
    安全地切换地图：
    1. 清理当前世界的 actors
    2. 确保异步模式
    3. 用足够大的 timeout 调用 load_world
    4. 等待新世界就绪
    """
    # 准备
    try:
        world = client.get_world()
        current = world.get_map().name.split("/")[-1]
        if current == map_name:
            print(f"  Already on {map_name}, reloading...", flush=True)
        cleanup_world(world)
        ensure_async_mode(world)
    except Exception as e:
        print(f"  Warning during prep: {e}", flush=True)

    # 设置超长 timeout（Shipping 包 load_world 很慢）
    old_timeout = timeout
    client.set_timeout(timeout)

    # 切换
    print(f"  Loading {map_name} (this may take a few minutes)...", flush=True)
    t0 = time.time()

    try:
        client.load_world(map_name)
    except RuntimeError as e:
        # CARLA 有时在 load_world 成功后仍抛超时异常，检查是否实际已切换
        print(f"  load_world raised: {e}", flush=True)
        print("  Checking if map loaded anyway...", flush=True)
        time.sleep(3.0)

    # 等就绪
    try:
        world, spawn_points = wait_world_ready(client, timeout=60.0)
    except TimeoutError:
        print("  ERROR: World not ready after loading!", flush=True)
        return None

    loaded = world.get_map().name.split("/")[-1]
    elapsed = time.time() - t0
    print(f"  Loaded {loaded} in {elapsed:.1f}s ({len(spawn_points)} spawn points)", flush=True)

    # 让新地图稳定几秒（AirSim 插件需要时间重新初始化）
    time.sleep(2.0)

    # 恢复正常 timeout
    client.set_timeout(10.0)
    return world


def orbit_spectator(world, center, duration=8.0):
    """在 spawn point 周围旋转 spectator 摄像机。"""
    spectator = world.get_spectator()
    t0 = time.time()
    angle = 0.0
    radius = 60.0
    height = 40.0

    while time.time() - t0 < duration:
        angle += 0.5
        rad = math.radians(angle)
        cx = center.x + radius * math.cos(rad)
        cy = center.y + radius * math.sin(rad)
        cz = center.z + height

        look_yaw = math.degrees(math.atan2(center.y - cy, center.x - cx))
        tf = carla.Transform(
            carla.Location(x=cx, y=cy, z=cz),
            carla.Rotation(pitch=-30.0, yaw=look_yaw),
        )
        spectator.set_transform(tf)
        time.sleep(1.0 / 30.0)


def get_available_maps(client):
    """获取可用地图列表（过滤掉 _Opt 变体和 Flatland 等）。"""
    all_maps = client.get_available_maps()
    names = sorted(set(m.split("/")[-1] for m in all_maps))
    return [m for m in names if m.startswith("Town") and "_Opt" not in m]


def main():
    ap = argparse.ArgumentParser(
        description="Switch CarlaAir maps reliably. No extra windows.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 switch_maps_v2.py --list           # list available maps
  python3 switch_maps_v2.py Town01           # switch to Town01
  python3 switch_maps_v2.py Town03 --orbit   # switch + orbit camera
  python3 switch_maps_v2.py --all --stay 15  # tour all maps
""",
    )
    ap.add_argument("map", nargs="?", default=None, help="Target map name (e.g. Town01, Town10HD)")
    ap.add_argument("--list", action="store_true", help="List available maps and exit")
    ap.add_argument("--all", action="store_true", help="Tour all available maps")
    ap.add_argument("--orbit", action="store_true", help="Orbit spectator after switching")
    ap.add_argument("--stay", type=float, default=8.0, help="Seconds to orbit per map (default: 8)")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    # Connect
    print("Connecting to CarlaAir...", flush=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        current = world.get_map().name.split("/")[-1]
        print(f"  Current map: {current}", flush=True)
    except Exception as e:
        print(f"  ERROR: Cannot connect to CarlaAir: {e}", flush=True)
        sys.exit(1)

    maps = get_available_maps(client)

    # --list
    if args.list:
        print(f"\nAvailable maps ({len(maps)}):", flush=True)
        for m in maps:
            marker = " <-- current" if m == current else ""
            print(f"  {m}{marker}", flush=True)
        return

    # Determine targets
    if args.all:
        targets = maps
    elif args.map:
        # Fuzzy match: "town01" -> "Town01", "10hd" -> "Town10HD"
        query = args.map.lower()
        matched = [m for m in maps if query in m.lower()]
        if not matched:
            print(f"  ERROR: No map matching '{args.map}'", flush=True)
            print(f"  Available: {', '.join(maps)}", flush=True)
            sys.exit(1)
        targets = matched[:1]
    else:
        # 不带参数：随机切到一张不同的地图
        import random
        others = [m for m in maps if m != current]
        if not others:
            print("  Only one map available, nothing to switch to.", flush=True)
            return
        pick = random.choice(others)
        print(f"  No map specified, randomly picking: {pick}", flush=True)
        targets = [pick]

    # Switch
    try:
        for i, map_name in enumerate(targets):
            print(f"\n[{i+1}/{len(targets)}] Switching to {map_name}...", flush=True)
            world = switch_map(client, map_name)

            if world is None:
                print(f"  FAILED to load {map_name}, skipping.", flush=True)
                continue

            if args.orbit or args.all:
                sps = world.get_map().get_spawn_points()
                center = sps[len(sps) // 2].location if sps else carla.Location(0, 0, 30)
                print(f"  Orbiting for {args.stay}s (watch in CarlaAir window)...", flush=True)
                orbit_spectator(world, center, args.stay)

        print("\nDone!", flush=True)

    except KeyboardInterrupt:
        print("\nInterrupted.", flush=True)


if __name__ == "__main__":
    main()
