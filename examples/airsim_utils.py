"""
AirSim utility functions for CarlaAir.

In CARLA maps, PlayerStart is typically 20-30m above ground level. The drone
spawns at the NED origin (PlayerStart) and freefalls to the ground.

Use safe_takeoff() instead of client.takeoffAsync() for reliable drone launch.
"""

import airsim
import time


def safe_takeoff(client, z=-5, speed=3, vehicle_name=""):
    """
    Reliably launch the drone to a target altitude.

    In CARLA maps, the standard takeoffAsync() may not fully lift the drone
    due to ground lock at the landing position. This function uses moveToZAsync
    which reliably overcomes ground lock and flies to the target altitude.

    Args:
        client: airsim.MultirotorClient instance (already connected)
        z: Target altitude in NED (negative = above origin). Default -5.
        speed: Ascent/flight speed in m/s. Default 3.
        vehicle_name: Vehicle name for multi-vehicle setups. Default "".
    """
    client.enableApiControl(True, vehicle_name)
    client.armDisarm(True, vehicle_name)
    client.moveToZAsync(z, speed, vehicle_name=vehicle_name).join()


def safe_land(client, vehicle_name=""):
    """
    Land the drone and disarm.

    Args:
        client: airsim.MultirotorClient instance
        vehicle_name: Vehicle name for multi-vehicle setups. Default "".
    """
    client.moveToZAsync(0, 2, vehicle_name=vehicle_name).join()
    client.armDisarm(False, vehicle_name)
    client.enableApiControl(False, vehicle_name)
