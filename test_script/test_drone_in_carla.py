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

    print(f"--- Testing CARLA API (port {carla_port}) ---")
    client = carla.Client("localhost", carla_port)
    client.set_timeout(_CARLA_TIMEOUT)

    print(f"Server version: {client.get_server_version()}")
    world = client.get_world()

    try:
        carla_map = world.get_map()
        print(f"Map: {carla_map.name}")
        spawn_points = carla_map.get_spawn_points()
        print(f"Spawn points: {len(spawn_points)}")
    except Exception as e:  # noqa: BLE001
        print(f"Map info: {e}")

    actors = world.get_actors()
    print(f"Total actors: {len(actors)}")

    bp_lib = world.get_blueprint_library()
    print(f"Blueprint library: {len(bp_lib)} blueprints")

    vehicle_bps = bp_lib.filter("vehicle.*")
    print(f"Vehicle types: {len(vehicle_bps)}")

    try:
        weather = world.get_weather()
        print(f"Weather: sun_alt={weather.sun_altitude_angle:.1f}")
        world.set_weather(carla.WeatherParameters.ClearNoon)
        print("Weather set to ClearNoon")
    except Exception:  # noqa: BLE001
        print("Weather: N/A")

    traffic_lights = actors.filter("traffic.traffic_light*")
    print(f"Traffic lights: {len(traffic_lights)}")

    return world


def test_airsim_drone(
    airsim_port: int = _AIRSIM_DEFAULT_PORT,
) -> Any:  # airsim.MultirotorClient
    """Test AirSim drone control."""
    import airsim

    print(f"\n--- Testing AirSim Drone (port {airsim_port}) ---")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    drone_state = DroneState.from_airsim(pos)
    print(f"Drone position: x={drone_state.x:.1f}, y={drone_state.y:.1f}, z={drone_state.z:.1f}")

    return client


def _capture_drone_image(airsim_client: Any, label: str) -> None:  # airsim.MultirotorClient
    """Capture and display drone camera image."""
    import airsim
    import numpy as np

    responses = airsim_client.simGetImages(
        [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
    )
    if responses and responses[0].width > 0:
        img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img = img.reshape(responses[0].height, responses[0].width, 3)
        print(f"{label}: {img.shape[1]}x{img.shape[0]}, mean={img.mean():.1f}")
        return img
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

    print("\n--- Flying Drone in CARLA ---")

    # Take off
    print("Taking off...")
    airsim_client.takeoffAsync().join()
    time.sleep(_WAYPOINT_PAUSE)

    # Fly to altitude
    print(f"Ascending to {abs(_AIRSIM_ALTITUDE):.0f}m...")
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

    for i, wp in enumerate(waypoints):
        print(f"Flying to waypoint {i + 1}: ({wp.x}, {wp.y}, {wp.z})")
        airsim_client.moveToPositionAsync(wp.x, wp.y, wp.z, wp.speed).join()
        time.sleep(_WAYPOINT_PAUSE)

        # Get drone position
        state = _get_current_drone_state(airsim_client)
        flight_path.append(state)
        print(f"  Position: ({state.x:.1f}, {state.y:.1f}, {state.z:.1f})")

    # Final camera capture
    img = _capture_drone_image(airsim_client, "Final drone camera")
    if img is not None:
        pil_img = Image.fromarray(img)
        pil_img.save("/tmp/drone_in_carla.png")
        print("Image saved to /tmp/drone_in_carla.png")

    # Land
    print("\nLanding...")
    airsim_client.landAsync().join()

    print("\n=== Drone flight in CARLA complete! ===")
    return flight_path


def main() -> int:
    """Main entry point."""
    carla_port = int(sys.argv[1]) if len(sys.argv) > 1 else _CARLA_DEFAULT_PORT

    print("=== AirSim Drone in CARLA Environment ===\n")

    try:
        carla_world = test_carla_features(carla_port)
        airsim_client = test_airsim_drone()
        flight_path = fly_drone_in_carla(airsim_client, carla_world)

        print(f"\nFlight path recorded: {len(flight_path)} states")
        for i, state in enumerate(flight_path):
            print(f"  {i + 1}: ({state.x:.1f}, {state.y:.1f}, {state.z:.1f})")

        return 0
    except Exception as e:  # noqa: BLE001
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
