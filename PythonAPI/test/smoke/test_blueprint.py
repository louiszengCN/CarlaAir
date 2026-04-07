# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test blueprint library ID format validation."""

import re

from . import SmokeTest

# Blueprint ID patterns
_FULL_ID_PATTERN: str = r"\S+\.\S+\.\S+"
_VEHICLE_ID_PATTERN: str = r"(vehicle)\.\S+\.\S+"

# Blueprint filters
_SENSOR_FILTER: str = "sensor.*"
_STATIC_FILTER: str = "static.*"
_VEHICLE_FILTER: str = "vehicle.*"
_WALKER_FILTER: str = "walker.*"


class TestBlueprintLibrary(SmokeTest):
    """Test CARLA blueprint library."""

    def test_blueprint_ids(self) -> None:
        """Verify blueprint IDs follow the expected format."""
        print("TestBlueprintLibrary.test_blueprint_ids")
        library = self.client.get_world().get_blueprint_library()

        # Verify filters return non-empty results
        self.assertTrue(list(library))
        self.assertTrue(list(library.filter(_SENSOR_FILTER)))
        self.assertTrue(list(library.filter(_STATIC_FILTER)))
        self.assertTrue(list(library.filter(_VEHICLE_FILTER)))
        self.assertTrue(list(library.filter(_WALKER_FILTER)))

        # Verify ID format: all blueprints should match X.Y.Z pattern
        full_id_regex = re.compile(_FULL_ID_PATTERN)
        for bp in library:
            self.assertTrue(full_id_regex.match(bp.id))

        # Verify vehicle IDs start with 'vehicle.'
        vehicle_id_regex = re.compile(_VEHICLE_ID_PATTERN)
        for bp in library.filter(_VEHICLE_FILTER):
            self.assertTrue(vehicle_id_regex.match(bp.id))
