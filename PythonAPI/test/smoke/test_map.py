# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test map loading and validation."""

import random
import time

import carla

from . import SmokeTest

# Constants
_RELOAD_DELAY: float = 5.0
_WAYPOINT_DISTANCE: float = 2.0
_WAYPOINT_LIMIT: int = 200
_NEXT_WAYPOINT_ITERATIONS: int = 20
_NEXT_WAYPOINT_DISTANCE: float = 4.0
_LANE_WIDTH_MIN: float = 0.0

# Maps without crosswalks
_MAPS_WITHOUT_CROSSWALKS: list[str] = [
    "Town01",
    "Town01_Opt",
    "Town02",
    "Town02_Opt",
]

# Standard map list
_MAP_NAMES: list[str] = [
    "Town01",
    "Town01_Opt",
    "Town02",
    "Town02_Opt",
    "Town03",
    "Town03_Opt",
    "Town04",
    "Town04_Opt",
    "Town05",
    "Town05_Opt",
    "Town10HD",
    "Town10HD_Opt",
]


class TestMap(SmokeTest):
    """Test map loading and topology validation."""

    def test_reload_world(self) -> None:
        """Verify world can be reloaded and returns same map."""
        map_name = self.client.get_world().get_map().name
        world = self.client.reload_world()
        assert map_name == world.get_map().name

    def test_load_all_maps(self) -> None:
        """Verify all standard maps can be loaded and validated."""
        for map_name in _MAP_NAMES:
            world = self.client.load_world(map_name)
            # Workaround: give time to UE4 to clean memory after loading
            time.sleep(_RELOAD_DELAY)
            m = world.get_map()
            assert map_name.split("/")[-1] == m.name.split("/")[-1]
            self._check_map(m)

    def _check_map(self, m: carla.Map) -> None:
        """Validate map topology and spawn points.

        Args:
            m: CARLA map to validate
        """
        for spawn_point in m.get_spawn_points():
            waypoint = m.get_waypoint(
                spawn_point.location, project_to_road=False,
            )
            assert waypoint is not None

        topology = m.get_topology()
        assert len(topology) > 0

        waypoints = list(m.generate_waypoints(_WAYPOINT_DISTANCE))
        assert len(waypoints) > 0
        random.shuffle(waypoints)

        for initial_waypoint in waypoints[:_WAYPOINT_LIMIT]:
            current_waypoint = initial_waypoint
            for _ in range(_NEXT_WAYPOINT_ITERATIONS):
                assert current_waypoint.lane_width >= _LANE_WIDTH_MIN
                _ = current_waypoint.get_right_lane()
                _ = current_waypoint.get_left_lane()
                next_waypoints = current_waypoint.next(_NEXT_WAYPOINT_DISTANCE)
                if not next_waypoints:
                    break
                current_waypoint = random.choice(next_waypoints)

        _ = m.transform_to_geolocation(carla.Location())
        assert str(m.to_opendrive())

        if not any(
            map_name in m.name for map_name in _MAPS_WITHOUT_CROSSWALKS
        ):
            crosswalks = m.get_crosswalks()
            assert len(crosswalks) > 0, f"Map {m.name} has no crosswalks."
