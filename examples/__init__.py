"""
CarlaAir example scripts.

Each module is a standalone runnable demo that requires a running CarlaAir
server (``./carlaAir.sh``).

Available examples
------------------
air_ground_sync      -- split-screen car + drone in the same world
drive_vehicle        -- keyboard-controlled ground vehicle
quick_start_showcase -- 4-panel sensor showcase (RGB/Depth/Semantic/LiDAR)
sensor_gallery       -- cycle through all sensor types
switch_maps          -- hot-swap maps while the server stays running
walk_pedestrian      -- control a pedestrian on foot

Usage::

    conda activate carlaAir
    python3 examples/quick_start_showcase.py
"""

from __future__ import annotations

__all__ = [
    "air_ground_sync",
    "drive_vehicle",
    "quick_start_showcase",
    "sensor_gallery",
    "switch_maps",
    "walk_pedestrian",
]
