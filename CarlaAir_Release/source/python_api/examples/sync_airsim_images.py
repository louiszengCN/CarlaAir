#!/usr/bin/env python3
"""Capture AirSim images without blocking a CARLA synchronous world."""

import argparse
from pathlib import Path
import sys

try:
    import airsim
except ImportError:
    sys.exit("Need airsim. Run SetupEnv.bat first.")

try:
    import carla
except ImportError:
    sys.exit("Need carla. Run SetupEnv.bat first.")

from airsim_image_fetcher import AirSimImageFetcher, AirSimImageTimeoutError


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def positive_float(value):
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture AirSim images while ticking CARLA sync mode"
    )
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port")
    parser.add_argument(
        "--airsim-host", default="127.0.0.1", help="AirSim host"
    )
    parser.add_argument(
        "--airsim-port", type=int, default=41451, help="AirSim port"
    )
    parser.add_argument("--camera", default="0", help="AirSim camera name")
    parser.add_argument("--frames", type=positive_int, default=3)
    parser.add_argument(
        "--fixed-delta-seconds", type=positive_float, default=1.0 / 30.0
    )
    parser.add_argument("--timeout", type=positive_float, default=10.0)
    parser.add_argument(
        "--output", type=Path, default=Path("recordings") / "sync_airsim"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    original_settings = world.get_settings()
    sync_settings = world.get_settings()
    sync_settings.synchronous_mode = True
    sync_settings.fixed_delta_seconds = args.fixed_delta_seconds

    print(f"CARLA map: {world.get_map().name}")
    print(f"Output: {args.output.resolve()}")

    sync_enabled = False
    try:
        world.apply_settings(sync_settings)
        sync_enabled = True

        with AirSimImageFetcher.for_multirotor(
            host=args.airsim_host,
            port=args.airsim_port,
            rpc_timeout=args.timeout,
        ) as fetcher:
            for index in range(args.frames):
                requests = [
                    airsim.ImageRequest(
                        args.camera,
                        airsim.ImageType.Scene,
                        False,
                        True,
                    )
                ]
                responses = fetcher.get_images_while_ticking(
                    world,
                    requests,
                    timeout=args.timeout,
                )
                if not responses or responses[0].width <= 0:
                    raise RuntimeError("AirSim returned an empty image")

                output_path = args.output / f"scene_{index:04d}.png"
                airsim.write_file(
                    str(output_path),
                    responses[0].image_data_uint8,
                )
                print(
                    f"[{index + 1}/{args.frames}] {output_path} "
                    f"({responses[0].width}x{responses[0].height})"
                )
    except AirSimImageTimeoutError as exc:
        print(f"Capture failed: {exc}", file=sys.stderr)
        return 2
    finally:
        if sync_enabled:
            try:
                world.apply_settings(original_settings)
                print("CARLA world settings restored")
            except RuntimeError as exc:
                print(f"Failed to restore CARLA settings: {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
