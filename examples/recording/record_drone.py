#!/usr/bin/env python3
"""
record_drone.py — 终端录制无人机轨迹（零侵入）
================================================
从 CARLA 侧读取无人机 actor 位姿，完全不连 AirSim API，
不影响 CarlaAir 窗口的飞行控制。

流程:
  1. 启动脚本 → 连接 CARLA，找到无人机 actor
  2. 在 CarlaAir 窗口里飞到起始位置
  3. 终端按 Enter → 开始录制
  4. 飞完按 Enter → 保存，可继续录下一段
  5. 输入 q → 退出

Options:
  --loop-vehicle PATH   循环回放一条车辆轨迹（幽灵车）

Output: ../trajectories/drone_<timestamp>_<nn>.json
  (含 transform=CARLA坐标，demo_director 回放时自动校准到 AirSim)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

from trajectory_helpers import (
    load_trajectory_json,
    spawn_ghost_vehicle,
    vehicle_apply_frame_transform,
)


def main():
    ap = argparse.ArgumentParser(description="Record drone trajectory (terminal, zero AirSim)")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--hz", type=float, default=20.0, help="Sample rate")
    ap.add_argument("--output-dir", default=None)
    ap.add_argument(
        "--loop-vehicle", default=None, metavar="PATH",
        help="Loop vehicle trajectory JSON (ghost car in CARLA)",
    )
    ap.add_argument("--verbose", action="store_true", help="Print pose every ~2s")
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.output_dir or os.path.join(os.path.dirname(script_dir), "trajectories")
    os.makedirs(out_dir, exist_ok=True)
    dt = 1.0 / max(1.0, args.hz)

    # ── CARLA ──
    import carla
    print("Connecting to CARLA...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    map_name = world.get_map().name.split("/")[-1]

    # Find drone actor
    drone_actor = None
    for a in world.get_actors():
        if "drone" in a.type_id.lower() or "airsim" in a.type_id.lower():
            drone_actor = a
            break
    if drone_actor is None:
        raise SystemExit("Drone actor not found in CARLA. Is CarlaAir running?")
    print(f"Drone: id={drone_actor.id} type={drone_actor.type_id}")

    # ── Ghost vehicle ──
    ghost_vehicle = None
    veh_frames = []
    veh_dt = 0.05

    if args.loop_vehicle:
        bp_lib = world.get_blueprint_library()
        vtraj = load_trajectory_json(args.loop_vehicle)
        if vtraj.get("type") != "vehicle":
            raise SystemExit("--loop-vehicle JSON must have type 'vehicle'")
        veh_frames = vtraj["frames"]
        veh_dt = float(vtraj.get("delta_time", 0.05))
        ghost_vehicle = spawn_ghost_vehicle(world, bp_lib, vtraj, carla)
        print(f"Ghost vehicle: {len(veh_frames)} frames, {veh_dt}s dt")

    # ── State ──
    saved_count = 0
    frames = []
    recording = False
    quit_flag = threading.Event()

    ghost_idx = [0]
    ghost_speed = [1.0]
    ghost_paused = [False]
    ghost_jump_to = [None]

    def save_segment():
        nonlocal frames, saved_count
        if not frames:
            print("  (no frames)")
            return
        saved_count += 1
        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(out_dir, f"drone_{ts}_{saved_count:02d}.json")
        data = {
            "type": "drone",
            "map": map_name,
            "delta_time": dt,
            "total_frames": len(frames),
            "coordinate_system": "carla",
            "frames": frames,
        }
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        dur = len(frames) * dt
        print(f"\n  Saved #{saved_count}: {filename}")
        print(f"  {len(frames)} frames, {dur:.1f}s")
        frames = []

    # ── Ghost vehicle thread ──
    def ghost_vehicle_loop():
        while not quit_flag.is_set():
            if ghost_vehicle is not None and veh_frames:
                if ghost_jump_to[0] is not None:
                    ghost_idx[0] = ghost_jump_to[0]
                    ghost_jump_to[0] = None
                if not ghost_paused[0]:
                    try:
                        vehicle_apply_frame_transform(
                            ghost_vehicle, veh_frames[ghost_idx[0]], carla
                        )
                    except Exception:
                        pass
                    step = max(1, int(ghost_speed[0]))
                    ghost_idx[0] = (ghost_idx[0] + step) % len(veh_frames)
            time.sleep(veh_dt / max(0.25, ghost_speed[0]))

    if ghost_vehicle is not None:
        threading.Thread(target=ghost_vehicle_loop, daemon=True).start()

    # ── Input thread ──
    def input_loop():
        nonlocal recording
        while not quit_flag.is_set():
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                quit_flag.set()
                break
            cmd = line.strip().lower()
            if cmd == "q":
                if recording:
                    recording = False
                    save_segment()
                quit_flag.set()
                break
            elif cmd in ("home", "h"):
                ghost_jump_to[0] = 0
                print("  Vehicle → frame 0")
            elif cmd == "[":
                ghost_speed[0] = max(0.25, ghost_speed[0] / 2.0)
                print(f"  Vehicle speed: {ghost_speed[0]:.2f}x")
            elif cmd == "]":
                ghost_speed[0] = min(4.0, ghost_speed[0] * 2.0)
                print(f"  Vehicle speed: {ghost_speed[0]:.2f}x")
            elif cmd == "p":
                ghost_paused[0] = not ghost_paused[0]
                print(f"  Vehicle: {'PAUSED' if ghost_paused[0] else 'PLAYING'}")
            else:
                if not recording:
                    frames.clear()
                    recording = True
                    print(f"\n  >>> Recording segment #{saved_count + 1} ({1/dt:.0f} Hz) ...")
                else:
                    recording = False
                    save_segment()
                    print("  Enter=Record  h=Home  [/]=Speed  p=Pause  q=Quit")

    has_ghost = ghost_vehicle is not None
    print(f"\n{'='*50}")
    print(f"  Drone Recorder (zero-intrusion mode)")
    print(f"  Map: {map_name} | {1/dt:.0f} Hz")
    print(f"  Reading from CARLA — no AirSim API — fly freely!")
    print(f"")
    print(f"  Enter       Start/Stop recording")
    if has_ghost:
        print(f"  h / home    Vehicle jump to start")
        print(f"  [ / ]       Vehicle speed (0.25x ~ 4x)")
        print(f"  p           Vehicle pause/resume")
        print(f"  Vehicle: {len(veh_frames)} frames, {len(veh_frames)*veh_dt:.1f}s")
    print(f"  q           Quit")
    print(f"{'='*50}")
    print(f"\n  Ready. Fly to start position, then press Enter.\n")

    threading.Thread(target=input_loop, daemon=True).start()

    last_print = time.perf_counter()
    last_sample = time.perf_counter()

    try:
        while not quit_flag.is_set():
            now = time.perf_counter()

            if recording and (now - last_sample) >= dt:
                last_sample = now
                try:
                    tf = drone_actor.get_transform()
                    vel = drone_actor.get_velocity()
                    frames.append({
                        "frame": len(frames),
                        "transform": {
                            "x": round(tf.location.x, 4),
                            "y": round(tf.location.y, 4),
                            "z": round(tf.location.z, 4),
                            "pitch": round(tf.rotation.pitch, 4),
                            "yaw": round(tf.rotation.yaw, 4),
                            "roll": round(tf.rotation.roll, 4),
                        },
                        "velocity": {
                            "x": round(vel.x, 4),
                            "y": round(vel.y, 4),
                            "z": round(vel.z, 4),
                        },
                    })
                except Exception:
                    pass

                if args.verbose and (now - last_print) >= 2.0 and frames:
                    last_print = now
                    p = frames[-1]["transform"]
                    print(
                        f"  #{len(frames)}  "
                        f"x={p['x']:.1f} y={p['y']:.1f} z={p['z']:.1f}  "
                        f"yaw={p['yaw']:.0f}"
                    )

            time.sleep(0.002 if recording else 0.05)

    except KeyboardInterrupt:
        quit_flag.set()

    if recording and frames:
        save_segment()

    if ghost_vehicle is not None:
        try:
            ghost_vehicle.destroy()
        except Exception:
            pass

    if saved_count:
        print(f"\n  Done. {saved_count} file(s) saved to {out_dir}")
    else:
        print("\n  No trajectories saved.")


if __name__ == "__main__":
    main()
