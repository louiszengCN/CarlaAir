# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test props loading and spawning."""

import random

import carla

from . import SmokeTest

# Constants
_PROPS_FILTER: str = "static.prop.*"
_Z_INCREMENT: float = 100.0


class TestPropsLoading(SmokeTest):
    """Test props loading from blueprint library."""

    def test_spawn_loaded_props(self) -> None:
        """Verify props can be spawned at randomized locations."""
        print("TestPropsLoading.test_spawn_loaded_props")
        client = self.client
        world = client.get_world()

        props = world.get_blueprint_library().filter(_PROPS_FILTER)
        spawn_points = world.get_map().get_spawn_points()

        z_offset = 0.0
        batch: list[carla.command.BatchCommand] = []
        for prop in props:
            spawn_point = random.choice(spawn_points)
            spawn_point.location.z += z_offset
            batch.append(carla.command.SpawnActor(prop, spawn_point))
            z_offset += _Z_INCREMENT

        response = client.apply_batch_sync(batch)
        spawned_ids: list[carla.ActorId] = []
        for resp in response:
            self.assertFalse(resp.error)
            spawned_ids.append(resp.actor_id)

        self.assertEqual(len(spawned_ids), len(props))
