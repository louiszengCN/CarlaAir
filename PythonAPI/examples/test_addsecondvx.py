"""V2X sensor test script for CARLA."""  # noqa: INP001

from __future__ import annotations

import random
import weakref

import carla


def get_actor_blueprints(
    world: carla.World,
    pattern: str,
    generation: str,
) -> list[carla.ActorBlueprint]:
    """Get filtered actor blueprints by generation."""
    bps = world.get_blueprint_library().filter(pattern)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
    except ValueError:
        return []
    else:
        if int_generation in [1, 2]:
            return [x for x in bps if int(x.get_attribute("generation")) == int_generation]
        return []


class V2XSensor:
    """V2X sensor wrapper."""

    def __init__(self, parent_actor: carla.Actor) -> None:
        self.sensor: carla.Sensor | None = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find("sensor.other.v2x")
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent,
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: V2XSensor._v2x_callback(weak_self, sensor_data),
        )

    def destroy(self) -> None:
        """Stop and destroy the sensor."""
        self.sensor.stop()
        self.sensor.destroy()

    @staticmethod
    def _v2x_callback(weak_self: weakref.ref[V2XSensor], sensor_data: object) -> None:
        """Process V2X sensor data."""
        instance = weak_self()
        if not instance:
            return
        for data in sensor_data:
            data.get()


def _main() -> None:
    """Run V2X sensor test."""
    client = carla.Client("localhost", 2000)
    client.set_timeout(2000.0)

    world = client.get_world()
    smap = world.get_map()

    spawn_points = smap.get_spawn_points()
    spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
    blueprint = random.choice(get_actor_blueprints(world, "vehicle.*", "2"))
    blueprint.set_attribute("role_name", "test")
    player = world.try_spawn_actor(blueprint, spawn_point)
    v2x_sensor = V2XSensor(player)

    world.wait_for_tick()
    try:
        while True:
            world.wait_for_tick()
    finally:
        v2x_sensor.destroy()
        player.destroy()


if __name__ == "__main__":
    _main()
