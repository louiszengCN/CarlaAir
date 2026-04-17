#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation."""

from __future__ import annotations

import argparse
import contextlib
import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

from numpy import random
from pydantic import BaseModel, ConfigDict, Field, field_validator

import carla
from carla.command import DestroyActor, FutureActor, SetAutopilot, SpawnActor

if TYPE_CHECKING:
    from carla import ActorId


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_DEFAULT_HOST: str = "127.0.0.1"
_DEFAULT_PORT: int = 2000
_DEFAULT_TM_PORT: int = 8000
_DEFAULT_VEHICLE_COUNT: int = 30
_DEFAULT_WALKER_COUNT: int = 10
_DEFAULT_VEHICLE_FILTER: str = "vehicle.*"
_DEFAULT_WALKER_FILTER: str = "walker.pedestrian.*"
_DEFAULT_VEHICLE_GENERATION: str = "All"
_DEFAULT_WALKER_GENERATION: str = "2"
_CARLA_TIMEOUT: float = 10.0
_TM_GLOBAL_DISTANCE: float = 2.5
_TM_HYBRID_RADIUS: float = 70.0
_TM_SPEED_PERCENTAGE: float = 30.0
_SYNC_DELTA: float = 0.05
_WALKER_SPAWN_ATTEMPTS_MULTIPLIER: int = 3
_WALKER_WHEELCHAIR_PROBABILITY: int = 11
_WALKER_RECOMMENDED_SPEED_WALK: int = 1
_WALKER_RECOMMENDED_SPEED_RUN: int = 2
_WALKER_DEFAULT_SPEED: float = 0.0
_PEDESTRIAN_RUN_PROBABILITY: float = 0.0
_PEDESTRIAN_CROSS_PROBABILITY: float = 0.0
_WALKER_ACTOR_STEP: int = 2
_CLEANUP_DELAY: float = 0.5
_MAX_PROBABILITY: int = 100


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class ActorGeneration(Enum):
    """Actor generation filter options."""

    GEN_1 = 1
    GEN_2 = 2
    GEN_3 = 3
    ALL = "all"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class WalkerEntry:
    """Immutable record for a spawned walker and its controller."""

    walker_id: ActorId
    controller_id: ActorId | None = None


@dataclass
class SpawnState:
    """Mutable state for tracking spawned actors."""

    vehicles: list[ActorId] = field(default_factory=list)
    walkers: list[WalkerEntry] = field(default_factory=list)
    all_actor_ids: list[ActorId] = field(default_factory=list)


# ──────────────────────────────────────────────────────────────────────────────
# Pydantic Models
# ──────────────────────────────────────────────────────────────────────────────


class TrafficConfig(BaseModel):
    """Configuration for traffic generation."""

    model_config = ConfigDict(frozen=True)

    host: str = Field(default=_DEFAULT_HOST, min_length=1)
    port: int = Field(default=_DEFAULT_PORT, gt=0, le=65535)
    tm_port: int = Field(default=_DEFAULT_TM_PORT, gt=0, le=65535)
    vehicle_count: int = Field(default=_DEFAULT_VEHICLE_COUNT, ge=0)
    walker_count: int = Field(default=_DEFAULT_WALKER_COUNT, ge=0)
    vehicle_filter: str = Field(default=_DEFAULT_VEHICLE_FILTER, min_length=1)
    walker_filter: str = Field(default=_DEFAULT_WALKER_FILTER, min_length=1)
    vehicle_generation: str = Field(default=_DEFAULT_VEHICLE_GENERATION)
    walker_generation: str = Field(default=_DEFAULT_WALKER_GENERATION)
    safe_mode: bool = False
    hybrid_mode: bool = False
    car_lights_on: bool = False
    hero_vehicle: bool = False
    respawn_vehicles: bool = False
    no_rendering: bool = False
    asynchronous: bool = False
    seed: int | None = None
    walker_seed: int = 0

    @field_validator("port", "tm_port")
    @classmethod
    def _validate_port(cls, v: int) -> int:
        if not (1 <= v <= 65535):
            msg = f"Port must be in range 1-65535, got {v}"
            raise ValueError(msg)
        return v


# ──────────────────────────────────────────────────────────────────────────────
# Helper Functions
# ──────────────────────────────────────────────────────────────────────────────


def get_actor_blueprints(
    world: carla.World,
    filter_str: str,
    generation: str,
) -> list[carla.ActorBlueprint]:
    """Get filtered actor blueprints by generation.

    Args:
        world: CARLA world instance
        filter_str: blueprint filter pattern
        generation: generation filter ("1", "2", "3", or "All")

    Returns:
        list of matching actor blueprints
    """
    bps = world.get_blueprint_library().filter(filter_str)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            return [
                x
                for x in bps
                if int(x.get_attribute("generation").as_int()) == int_generation
            ]
        return []
    except ValueError:
        return []


def _cleanup(
    client: carla.Client,
    state: SpawnState,
    original_settings: carla.WorldSettings | None,
    world: carla.World,
    asynchronous: bool,
) -> None:
    """Clean up all spawned actors and restore world settings."""
    if not asynchronous:
        settings = (
            original_settings
            if original_settings
            else world.get_settings()
        )
        if not original_settings:
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
        world.apply_settings(settings)

    client.apply_batch([DestroyActor(x) for x in state.vehicles])

    # stop walker controllers (list is [controller, actor, controller, actor ...])
    for i in range(0, len(state.all_actor_ids), _WALKER_ACTOR_STEP):
        all_actors = world.get_actors(state.all_actor_ids)
        with contextlib.suppress(Exception):
            all_actors[i].stop()

    client.apply_batch([DestroyActor(x) for x in state.all_actor_ids])

    time.sleep(_CLEANUP_DELAY)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Main entry point for traffic generation."""
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default=_DEFAULT_HOST,
        help="IP of the host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=_DEFAULT_PORT,
        type=int,
        help=f"TCP port to listen to (default: {_DEFAULT_PORT})",
    )
    argparser.add_argument(
        "-n",
        "--number-of-vehicles",
        metavar="N",
        default=_DEFAULT_VEHICLE_COUNT,
        type=int,
        help=f"Number of vehicles (default: {_DEFAULT_VEHICLE_COUNT})",
    )
    argparser.add_argument(
        "-w",
        "--number-of-walkers",
        metavar="W",
        default=_DEFAULT_WALKER_COUNT,
        type=int,
        help=f"Number of walkers (default: {_DEFAULT_WALKER_COUNT})",
    )
    argparser.add_argument(
        "--safe",
        action="store_true",
        help="Avoid spawning vehicles prone to accidents",
    )
    argparser.add_argument(
        "--filterv",
        metavar="PATTERN",
        default=_DEFAULT_VEHICLE_FILTER,
        help=f'Filter vehicle model (default: "{_DEFAULT_VEHICLE_FILTER}")',
    )
    argparser.add_argument(
        "--generationv",
        metavar="G",
        default=_DEFAULT_VEHICLE_GENERATION,
        help=(
            f'restrict to certain vehicle generation '
            f'(values: "1","2","All" - default: "{_DEFAULT_VEHICLE_GENERATION}")'
        ),
    )
    argparser.add_argument(
        "--filterw",
        metavar="PATTERN",
        default=_DEFAULT_WALKER_FILTER,
        help=f'Filter pedestrian type (default: "{_DEFAULT_WALKER_FILTER}")',
    )
    argparser.add_argument(
        "--generationw",
        metavar="G",
        default=_DEFAULT_WALKER_GENERATION,
        help=(
            f'restrict to certain pedestrian generation '
            f'(values: "1","2","All" - default: "{_DEFAULT_WALKER_GENERATION}")'
        ),
    )
    argparser.add_argument(
        "--tm-port",
        metavar="P",
        default=_DEFAULT_TM_PORT,
        type=int,
        help=f"Port to communicate with TM (default: {_DEFAULT_TM_PORT})",
    )
    argparser.add_argument(
        "--asynch",
        action="store_true",
        help="Activate asynchronous mode execution",
    )
    argparser.add_argument(
        "--hybrid",
        action="store_true",
        help="Activate hybrid mode for Traffic Manager",
    )
    argparser.add_argument(
        "-s",
        "--seed",
        metavar="S",
        type=int,
        help="Set random device seed and deterministic mode for Traffic Manager",
    )
    argparser.add_argument(
        "--seedw",
        metavar="S",
        default=0,
        type=int,
        help="Set the seed for pedestrians module",
    )
    argparser.add_argument(
        "--car-lights-on",
        action="store_true",
        default=False,
        help="Enable automatic car light management",
    )
    argparser.add_argument(
        "--hero",
        action="store_true",
        default=False,
        help="Set one of the vehicles as hero",
    )
    argparser.add_argument(
        "--respawn",
        action="store_true",
        default=False,
        help="Automatically respawn dormant vehicles (only in large maps)",
    )
    argparser.add_argument(
        "--no-rendering",
        action="store_true",
        default=False,
        help="Activate no rendering mode",
    )

    args = argparser.parse_args()

    # Build config from args
    config = TrafficConfig(
        host=args.host,
        port=args.port,
        tm_port=args.tm_port,
        vehicle_count=args.number_of_vehicles,
        walker_count=args.number_of_walkers,
        vehicle_filter=args.filterv,
        walker_filter=args.filterw,
        vehicle_generation=args.generationv,
        walker_generation=args.generationw,
        safe_mode=args.safe,
        hybrid_mode=args.hybrid,
        car_lights_on=args.car_lights_on,
        hero_vehicle=args.hero,
        respawn_vehicles=args.respawn,
        no_rendering=args.no_rendering,
        asynchronous=args.asynch,
        seed=args.seed,
        walker_seed=args.seedw,
    )

    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    state = SpawnState()
    client = carla.Client(config.host, config.port)
    client.set_timeout(_CARLA_TIMEOUT)
    random.seed(config.seed if config.seed is not None else int(time.time()))

    original_world_settings: carla.WorldSettings | None = None
    world: carla.World | None = None

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(config.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(
            _TM_GLOBAL_DISTANCE,
        )
        if config.respawn_vehicles:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if config.hybrid_mode:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(_TM_HYBRID_RADIUS)
        if config.seed is not None:
            traffic_manager.set_random_device_seed(config.seed)

        original_world_settings = world.get_settings()
        settings = original_world_settings
        if not config.asynchronous:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = _SYNC_DELTA
        else:
            pass

        if config.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        blueprints = get_actor_blueprints(
            world, config.vehicle_filter, config.vehicle_generation,
        )
        if not blueprints:
            raise ValueError(
                "Couldn't find any vehicles with the specified filters",
            )
        blueprints_walkers = get_actor_blueprints(
            world, config.walker_filter, config.walker_generation,
        )
        if not blueprints_walkers:
            raise ValueError(
                "Couldn't find any walkers with the specified filters",
            )

        if config.safe_mode:
            blueprints = [
                x
                for x in blueprints
                if x.get_attribute("base_type") == "car"
            ]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if config.vehicle_count < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif config.vehicle_count > number_of_spawn_points:
            msg = (
                "requested %d vehicles, but could only find %d spawn points"
            )
            logging.warning(
                msg, config.vehicle_count, number_of_spawn_points,
            )

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = config.hero_vehicle
        for n, transform in enumerate(spawn_points):
            if n >= config.vehicle_count:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("color"):
                color = random.choice(
                    blueprint.get_attribute("color").recommended_values,
                )
                blueprint.set_attribute("color", color)
            if blueprint.has_attribute("driver_id"):
                driver_id = random.choice(
                    blueprint.get_attribute("driver_id").recommended_values,
                )
                blueprint.set_attribute("driver_id", driver_id)
            if hero:
                blueprint.set_attribute("role_name", "hero")
                hero = False
            else:
                blueprint.set_attribute("role_name", "autopilot")

            # spawn the cars and set their autopilot and light state all together
            batch.append(
                SpawnActor(blueprint, transform).then(
                    SetAutopilot(FutureActor, True, traffic_manager.get_port()),
                ),
            )

        for response in client.apply_batch_sync(batch, do_tick=True):
            if response.error:
                logging.error(response.error)
            else:
                state.vehicles.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if config.car_lights_on:
            all_vehicle_actors = world.get_actors(state.vehicles)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # -------------
        # Spawn Walkers
        # -------------
        if config.walker_seed:
            world.set_pedestrians_seed(config.walker_seed)
            random.seed(config.walker_seed)

        # 1. take all the random locations to spawn
        spawn_points: list[carla.Transform] = []
        for _ in range(config.walker_count):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        # 2. we spawn the walker object
        batch = []
        walker_speed: list[float] = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprints_walkers)
            # set as not invincible
            probability = random.randint(0, _MAX_PROBABILITY + 1)
            if walker_bp.has_attribute("is_invincible"):
                walker_bp.set_attribute("is_invincible", "false")
            if (
                walker_bp.has_attribute("can_use_wheelchair")
                and probability < _WALKER_WHEELCHAIR_PROBABILITY
            ):
                walker_bp.set_attribute("use_wheelchair", "true")
            # set the max speed
            if walker_bp.has_attribute("speed"):
                if random.random() > _PEDESTRIAN_RUN_PROBABILITY:
                    # walking
                    walker_speed.append(
                        walker_bp.get_attribute("speed").recommended_values[
                            _WALKER_RECOMMENDED_SPEED_WALK
                        ],
                    )
                else:
                    # running
                    walker_speed.append(
                        walker_bp.get_attribute("speed").recommended_values[
                            _WALKER_RECOMMENDED_SPEED_RUN
                        ],
                    )
            else:
                walker_speed.append(_WALKER_DEFAULT_SPEED)
            batch.append(SpawnActor(walker_bp, spawn_point))

        results = client.apply_batch_sync(batch, do_tick=True)
        walker_speed2: list[float] = []
        for i, result in enumerate(results):
            if result.error:
                logging.error(result.error)
            else:
                state.walkers.append(WalkerEntry(walker_id=result.actor_id))
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2

        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find(
            "controller.ai.walker",
        )
        for walker_entry in state.walkers:
            batch.append(
                SpawnActor(
                    walker_controller_bp,
                    carla.Transform(),
                    walker_entry.walker_id,
                ),
            )
        results = client.apply_batch_sync(batch, do_tick=True)
        for i, result in enumerate(results):
            if result.error:
                logging.error(result.error)
            else:
                state.walkers[i] = WalkerEntry(
                    walker_id=state.walkers[i].walker_id,
                    controller_id=result.actor_id,
                )

        # 4. we put together the walkers and controllers id to get the objects from their id
        for walker_entry in state.walkers:
            if walker_entry.controller_id is not None:
                state.all_actor_ids.append(walker_entry.controller_id)
            state.all_actor_ids.append(walker_entry.walker_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if config.asynchronous:
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to
        world.set_pedestrians_cross_factor(_PEDESTRIAN_CROSS_PROBABILITY)
        all_actors = world.get_actors(state.all_actor_ids)
        for i in range(0, len(state.all_actor_ids), _WALKER_ACTOR_STEP):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(
                world.get_random_location_from_navigation(),
            )
            # max speed
            all_actors[i].set_max_speed(
                float(walker_speed[int(i / 2)]),
            )


        # Example of how to use Traffic Manager parameters
        traffic_manager.global_percentage_speed_difference(
            _TM_SPEED_PERCENTAGE,
        )

        while True:
            if not config.asynchronous:
                world.tick()
            else:
                world.wait_for_tick()

    except KeyboardInterrupt:
        pass
    finally:
        if world is not None:
            _cleanup(
                client,
                state,
                original_world_settings,
                world,
                config.asynchronous,
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pass
