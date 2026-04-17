#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
GBuffer tutorial for CARLA

Spawn a vehicle, attach an RGB camera with gbuffer output enabled,
and save all gbuffer textures to disk.
"""

from __future__ import annotations

import random
import time

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_CARLA_HOST: str = "127.0.0.1"
_CARLA_PORT: int = 2000
_CARLA_TIMEOUT: float = 2.0

# Camera
_CAMERA_WIDTH: str = "1920"
_CAMERA_HEIGHT: str = "1080"
_CAMERA_X: float = 1.5
_CAMERA_Z: float = 2.4
_OUTPUT_PATTERN: str = "_out/FinalColor-%06d.png"

# GBuffer output pattern
_GBUFFER_OUTPUT_DIR: str = "_out"
_GBUFFER_PATTERN: str = "_out/GBuffer-%s-%06d.png"

# Duration
_SIMULATION_DURATION: float = 10.0

# Filters
_VEHICLE_FILTER: str = "vehicle"
_CAMERA_BLUEPRINT: str = "sensor.camera.rgb"


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Run the GBuffer tutorial."""
    actor_list: list[carla.Actor] = []

    try:
        client = carla.Client(_CARLA_HOST, _CARLA_PORT)
        client.set_timeout(_CARLA_TIMEOUT)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Spawn random vehicle
        bp = random.choice(blueprint_library.filter(_VEHICLE_FILTER))
        if bp.has_attribute("color"):
            color = random.choice(
                bp.get_attribute("color").recommended_values,
            )
            bp.set_attribute("color", color)

        transform = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)

        # Enable autopilot
        vehicle.set_autopilot(True)

        # Spawn RGB camera
        camera_bp = blueprint_library.find(_CAMERA_BLUEPRINT)
        camera_bp.set_attribute("image_size_x", _CAMERA_WIDTH)
        camera_bp.set_attribute("image_size_y", _CAMERA_HEIGHT)
        camera_transform = carla.Transform(
            carla.Location(x=_CAMERA_X, z=_CAMERA_Z),
        )
        camera = world.spawn_actor(
            camera_bp, camera_transform, attach_to=vehicle,
        )
        actor_list.append(camera)

        # Register callback for final color
        camera.listen(
            lambda image: image.save_to_disk(
                _OUTPUT_PATTERN % image.frame,
            ),
        )

        # Enable gbuffer textures
        camera.enable_gbuffers(True)

        # Register gbuffer callbacks
        gbuffer_ids = [
            (carla.GBufferTextureID.SceneColor, "SceneColor"),
            (carla.GBufferTextureID.SceneDepth, "SceneDepth"),
            (carla.GBufferTextureID.SceneStencil, "SceneStencil"),
            (carla.GBufferTextureID.GBufferA, "A"),
            (carla.GBufferTextureID.GBufferB, "B"),
            (carla.GBufferTextureID.GBufferC, "C"),
            (carla.GBufferTextureID.GBufferD, "D"),
            (carla.GBufferTextureID.GBufferE, "E"),
            (carla.GBufferTextureID.GBufferF, "F"),
            (carla.GBufferTextureID.Velocity, "Velocity"),
            (carla.GBufferTextureID.SSAO, "SSAO"),
            (carla.GBufferTextureID.CustomDepth, "CustomDepth"),
            (carla.GBufferTextureID.CustomStencil, "CustomStencil"),
        ]

        for gbuffer_id, name in gbuffer_ids:
            camera.listen_to_gbuffer(
                gbuffer_id,
                lambda image, n=name: image.save_to_disk(
                    _GBUFFER_PATTERN % (n, image.frame),
                ),
            )

        time.sleep(_SIMULATION_DURATION)

    finally:
        camera.destroy()
        client.apply_batch(
            [carla.command.DestroyActor(x) for x in actor_list],
        )


if __name__ == "__main__":
    main()
