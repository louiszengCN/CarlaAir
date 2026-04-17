#!/usr/bin/env python3
"""
Test script: Fly AirSim drone in CARLA Town01 environment.

Both CARLA (port 2000) and AirSim (port 41451) must be running.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from PIL import Image

if TYPE_CHECKING:
    import carla


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_CARLA_DEFAULT_PORT: int = 2000
_AIRSIM_DEFAULT_PORT: int = 41451
_CARLA_TIMEOUT: float = 30.0
_AIRSIM_ALTITUDE: float = -20.0
_AIRSIM_FLIGHT_DURATION: float = 3.0
_WAYPOINT_SPEED: float = 3.0
_WAYPOINT_PAUSE: float = 1.0


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class Waypoint:
    """A 3D waypoint with speed."""

    x: float
    y: float
    z: float
    speed: float = _WAYPOINT_SPEED


@dataclass(frozen=True, slots=True)
class DroneState:
    """Drone position state."""

    x: float
    y: float
    z: float

    @classmethod
    def from_airsim(cls, position: Any) -> DroneState:  # airsim.Vector3r
        """Create from AirSim position vector."""
        return cls(
            x=position.x_val,
            y=position.y_val,
            z=position.z_val,
        )


# ──────────────────────────────────────────────────────────────────────────────
# Test Functions
# ──────────────────────────────────────────────────────────────────────────────


def test_carla_features(carla_port: int = _CARLA_DEFAULT_PORT) -> carla.World:
    """Test CARLA features are working."""
    import carla

    client = carla.Client("localhost", carla_port)
    client.set_timeout(_CARLA_TIMEOUT)

    world = client.get_world()

    try:
        carla_map = world.get_map()
        carla_map.get_spawn_points()
    except Exception:
        pass

    actors = world.get_actors()

    bp_lib = world.get_blueprint_library()

    bp_lib.filter("vehicle.*")

    try:
        world.get_weather()
        world.set_weather(carla.WeatherParameters.ClearNoon)
    except Exception:
        pass

    actors.filter("traffic.traffic_light*")

    return world


def test_airsim_drone(
    airsim_port: int = _AIRSIM_DEFAULT_PORT,
) -> Any:  # airsim.MultirotorClient
    """Test AirSim drone control."""
    import airsim

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    DroneState.from_airsim(pos)

    return client


def _capture_drone_image(airsim_client: Any, label: str) -> None:  # airsim.MultirotorClient
    """Capture and display drone camera image."""
    import airsim
    import numpy as np

    responses = airsim_client.simGetImages(
        [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)],
    )
    if responses and responses[0].width > 0:
        img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        return img.reshape(responses[0].height, responses[0].width, 3)
    return None


def _get_current_drone_state(airsim_client: Any) -> DroneState:  # airsim.MultirotorClient
    """Get current drone state."""
    state = airsim_client.getMultirotorState()
    return DroneState.from_airsim(state.kinematics_estimated.position)


def fly_drone_in_carla(
    airsim_client: Any,  # airsim.MultirotorClient
    carla_world: carla.World,
    waypoints: list[Waypoint] | None = None,
) -> list[DroneState]:
    """Fly the drone around the CARLA environment."""


    # Take off
    airsim_client.takeoffAsync().join()
    time.sleep(_WAYPOINT_PAUSE)

    # Fly to altitude
    airsim_client.moveToZAsync(_AIRSIM_ALTITUDE, _AIRSIM_FLIGHT_DURATION).join()
    time.sleep(_WAYPOINT_PAUSE)

    # Capture initial camera image
    _capture_drone_image(airsim_client, "Initial drone camera")

    # Use default waypoints or custom
    if waypoints is None:
        waypoints = [
            Waypoint(10, 10, _AIRSIM_ALTITUDE),  # Forward-right
            Waypoint(10, -10, _AIRSIM_ALTITUDE),  # Forward-left
            Waypoint(-10, -10, _AIRSIM_ALTITUDE),  # Back-left
            Waypoint(-10, 10, _AIRSIM_ALTITUDE),  # Back-right
            Waypoint(0, 0, _AIRSIM_ALTITUDE),  # Return to center
        ]

    flight_path: list[DroneState] = []

    for _i, wp in enumerate(waypoints):
        airsim_client.moveToPositionAsync(wp.x, wp.y, wp.z, wp.speed).join()
        time.sleep(_WAYPOINT_PAUSE)

        # Get drone position
        state = _get_current_drone_state(airsim_client)
        flight_path.append(state)

    # Final camera capture
    img = _capture_drone_image(airsim_client, "Final drone camera")
    if img is not None:
        pil_img = Image.fromarray(img)
        pil_img.save("/tmp/drone_in_carla.png")

    # Land
    airsim_client.landAsync().join()

    return flight_path


def main() -> int:
    """Main entry point."""
    carla_port = int(sys.argv[1]) if len(sys.argv) > 1 else _CARLA_DEFAULT_PORT


    try:
        carla_world = test_carla_features(carla_port)
        airsim_client = test_airsim_drone()
        flight_path = fly_drone_in_carla(airsim_client, carla_world)

        for _i, _state in enumerate(flight_path):
            pass

        return 0
    except Exception:
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
