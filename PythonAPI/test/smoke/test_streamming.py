# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test multi-client streaming functionality."""

import threading
import time

import carla

from . import SmokeTest

# Constants
_CLIENT_TIMEOUT: float = 60.0
_GNSS_SENSOR_TICK: float = 1.0
_THREAD_SLEEP: float = 5.0
_THREAD_COUNT: int = 5
_LAT_LON_PLACES: int = 4

# Filters
_GNSS_SENSOR: str = "sensor.other.gnss"


class TestStreamming(SmokeTest):
    """Test multi-client GNSS streaming."""

    lat: float = 0.0
    lon: float = 0.0

    def on_gnss_set(self, event: carla.GnssMeasurement) -> None:
        """Store GNSS coordinates.

        Args:
            event: GNSS measurement data
        """
        self.lat = event.latitude
        self.lon = event.longitude

    def on_gnss_check(self, event: carla.GnssMeasurement) -> None:
        """Verify GNSS coordinates match stored values.

        Args:
            event: GNSS measurement data
        """
        self.assertAlmostEqual(event.latitude, self.lat, places=_LAT_LON_PLACES)
        self.assertAlmostEqual(event.longitude, self.lon, places=_LAT_LON_PLACES)

    def _create_client(self) -> None:
        """Create a secondary client and verify GNSS streaming."""
        client = carla.Client(*self.testing_address)
        client.set_timeout(_CLIENT_TIMEOUT)
        world = client.get_world()
        actors = world.get_actors()
        for actor in actors:
            if actor.type_id == _GNSS_SENSOR:
                actor.listen(self.on_gnss_check)
        time.sleep(_THREAD_SLEEP)
        # Stop
        for actor in actors:
            if actor.type_id == _GNSS_SENSOR:
                actor.stop()

    def test_multistream(self) -> None:
        """Test multiple clients streaming GNSS data concurrently."""
        # Create the sensor
        world = self.client.get_world()
        bp = world.get_blueprint_library().find(_GNSS_SENSOR)
        bp.set_attribute("sensor_tick", str(_GNSS_SENSOR_TICK))
        gnss_sensor = world.spawn_actor(bp, carla.Transform())
        gnss_sensor.listen(self.on_gnss_set)
        world.wait_for_tick()

        # Create multiple clients
        threads: list[threading.Thread] = []
        for _ in range(_THREAD_COUNT):
            thread = threading.Thread(target=self._create_client)
            thread.setDaemon(True)
            thread.start()
            threads.append(thread)

        # Wait for all clients to finish
        for thread in threads:
            thread.join()

        gnss_sensor.destroy()
