# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Smoke test base classes."""

from __future__ import annotations

import os
import time
import unittest
from typing import TYPE_CHECKING

import carla

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# Connection
_DEFAULT_HOST: str = os.environ.get("CARLA_HOST", "localhost")
_DEFAULT_PORT: int = int(os.environ.get("CARLA_PORT", "2000"))
_CONNECTION_TIMEOUT: float = 120.0
_SYNC_DELTA: float = 0.05
_WAIT_FOR_TICK_TIMEOUT: float = 30.0


class SmokeTest(unittest.TestCase):
    """Base class for CARLA smoke tests."""

    client: carla.Client
    world: carla.World
    settings: carla.WorldSettings

    @classmethod
    def setUpClass(cls) -> None:
        """Set up CARLA client connection."""
        host = _DEFAULT_HOST
        port = _DEFAULT_PORT
        print(f"Connecting to {host}:{port}...", flush=True)
        cls.client = carla.Client(host, port)
        cls.client.set_timeout(_CONNECTION_TIMEOUT)
        cls.world = cls.client.get_world()
        cls.settings = cls.world.get_settings()

    def tearDown(self) -> None:
        """Restore original world settings after each test."""
        self.world.apply_settings(self.settings)


class SyncSmokeTest(SmokeTest):
    """Smoke test with synchronous mode enabled."""

    @classmethod
    def setUpClass(cls) -> None:
        """Set up CARLA client with synchronous mode."""
        super().setUpClass()
        settings = cls.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = _SYNC_DELTA
        cls.world.apply_settings(settings)

    def tick(self) -> int:
        """Advance simulation by one tick.

        Returns:
            frame number
        """
        return self.world.tick()

    def wait_for_tick(self) -> carla.WorldSnapshot:
        """Wait for next tick and return snapshot.

        Returns:
            world snapshot
        """
        return self.world.wait_for_tick(seconds=_WAIT_FOR_TICK_TIMEOUT)
