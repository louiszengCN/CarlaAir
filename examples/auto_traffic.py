#!/usr/bin/env python3
"""
auto_traffic.py — CarlaAir Automatic Traffic Generator (v0.1.7)

Generates vehicles and pedestrians in asynchronous mode, keeping them running
in the background with periodic health checks.

Frozen pedestrians are automatically restarted; stalled vehicles regain autopilot.
On Ctrl+C or SIGTERM, all spawned actors are cleanly destroyed.

Usage:
    python3 auto_traffic.py [--vehicles 30] [--walkers 50] [--port 2000]
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import random
import signal
import sys
import time
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from enum import Enum
from typing import TYPE_CHECKING

from pydantic import BaseModel, ConfigDict, Field, field_validator

import carla

if TYPE_CHECKING:
    from carla import ActorId


_LOGGER_FORMAT: str = "[Traffic] %(message)s"
_LOG: logging.Logger = logging.getLogger(__name__)


def _configure_logging(level: int = logging.INFO) -> None:
    """Configure the module-level logger with the given level."""
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(_LOGGER_FORMAT))
    _LOG.setLevel(level)
    _LOG.addHandler(handler)


_DEFAULT_VEHICLE_COUNT: int = 10
_DEFAULT_WALKER_COUNT: int = 10
_DEFAULT_CARLA_PORT: int = 2000
_DEFAULT_TM_PORT: int = 8000
_CONNECTION_TIMEOUT: float = 20.0
_CONNECTION_MAX_ATTEMPTS: int = 10
_CONNECTION_RETRY_DELAY: float = 5.0
_HEALTH_CHECK_INTERVAL: float = 10.0
_MAIN_LOOP_SLEEP: float = 0.5
_RECONNECT_DELAY: float = 5.0
_ERROR_LOG_THRESHOLD: int = 5
_MAX_PORT: int = 65535
_TM_GLOBAL_DISTANCE: float = 2.5
_TM_SPEED_PERCENTAGE: float = 10.0
_WALKER_SPAWN_MULTIPLIER: int = 3
_WALKER_RUN_PROBABILITY: float = 0.15
_WALKER_RECOMMENDED_SPEED_INDEX_RUN: int = 2
_WALKER_RECOMMENDED_SPEED_INDEX_WALK: int = 1
_WALKER_DEFAULT_SPEED: float = 1.4
_WALKER_CROSS_FACTOR: float = 0.05
_WALKER_ACTOR_STEP: int = 2
_VEHICLE_STALL_SPEED_THRESHOLD: float = 0.01
_ZERO_SPEED: float = 0.0
_NUM_WHEELS_CAR: int = 4


class ActorCategory(Enum):
    """Categories of spawned actors for health-check filtering."""
    VEHICLE = "vehicle"
    WALKER = "walker"
    CONTROLLER = "controller"


class WalkerSpeedMode(Enum):
    """Speed modes for pedestrian spawning."""
    WALK = "walk"
    RUN = "run"


class TrafficManagerConfig(BaseModel):
    """Configuration for the CARLA Traffic Manager."""
    model_config = ConfigDict(frozen=True)
    global_distance_to_leading_vehicle: float = Field(default=_TM_GLOBAL_DISTANCE, gt=0)
    speed_percentage_difference: float = Field(default=_TM_SPEED_PERCENTAGE, ge=0, le=100)


class WalkerSpawnConfig(BaseModel):
    """Configuration for walker spawning behaviour."""
    model_config = ConfigDict(frozen=True)
    spawn_multiplier: int = Field(default=_WALKER_SPAWN_MULTIPLIER, gt=0)
    run_probability: float = Field(default=_WALKER_RUN_PROBABILITY, ge=0, le=1)
    default_speed: float = Field(default=_WALKER_DEFAULT_SPEED, gt=0)
    cross_factor: float = Field(default=_WALKER_CROSS_FACTOR, ge=0, le=1)


class VehicleSpawnConfig(BaseModel):
    """Configuration for vehicle spawning behaviour."""
    model_config = ConfigDict(frozen=True)
    color_role: str = Field(default="autopilot", min_length=1)


class SimulationConfig(BaseModel):
    """Top-level simulation configuration."""
    model_config = ConfigDict(frozen=True)
    vehicle_count: int = Field(default=_DEFAULT_VEHICLE_COUNT, gt=0)
    walker_count: int = Field(default=_DEFAULT_WALKER_COUNT, gt=0)
    carla_port: int = Field(default=_DEFAULT_CARLA_PORT, gt=0)
    tm_port: int = Field(default=_DEFAULT_TM_PORT, gt=0)
    health_interval: float = Field(default=_HEALTH_CHECK_INTERVAL, gt=0)
    connection_timeout: float = Field(default=_CONNECTION_TIMEOUT, gt=0)
    connection_max_attempts: int = Field(default=_CONNECTION_MAX_ATTEMPTS, gt=0)
    connection_retry_delay: float = Field(default=_CONNECTION_RETRY_DELAY, gt=0)
    main_loop_sleep: float = Field(default=_MAIN_LOOP_SLEEP, gt=0)
    vehicle_stall_speed: float = Field(default=_VEHICLE_STALL_SPEED_THRESHOLD, ge=0)

    @field_validator("carla_port", "tm_port")
    @classmethod
    def _validate_port(cls, v: int) -> int:
        if not (1 <= v <= _MAX_PORT):
            msg = f"Port must be in range 1-{_MAX_PORT}, got {v}"
            raise ValueError(msg)
        return v


@dataclass(frozen=True, slots=True)
class WalkerEntry:
    """Immutable record for a spawned walker and its controller."""
    walker_id: ActorId
    controller_id: ActorId | None = None


@dataclass(frozen=True, slots=True)
class SpawnResult:
    """Result of a spawn operation."""
    spawned: int
    requested: int


@dataclass
class _GlobalState:
    """Mutable container for process-wide state."""
    vehicles: list[ActorId] = dataclass_field(default_factory=list)
    walkers: list[WalkerEntry] = dataclass_field(default_factory=list)
    all_actor_ids: list[ActorId] = dataclass_field(default_factory=list)
    client: carla.Client | None = None
    world: carla.World | None = None
    running: bool = True


_state = _GlobalState()


def _signal_handler(sig: int, _frame: object) -> None:
    """Handle SIGINT / SIGTERM."""
    _LOG.info("Received %s signal, shutting down", signal.Signals(sig).name)
    _state.running = False


def _spawn_vehicles(
    world: carla.World, client: carla.Client, config: VehicleSpawnConfig,
    tm_config: TrafficManagerConfig, count: int, tm_port: int,
) -> SpawnResult:
    """Spawn vehicles with autopilot via the Traffic Manager (async mode)."""
    bp_lib = world.get_blueprint_library()
    vehicle_bps = bp_lib.filter("vehicle.*")
    car_bps = [
        bp for bp in vehicle_bps
        if int(bp.get_attribute("number_of_wheels").as_int()) == _NUM_WHEELS_CAR
    ]
    if not car_bps:
        car_bps = list(vehicle_bps)
    spawn_points = world.get_map().get_spawn_points()
    spawn_points.sort(key=lambda _: random.random())
    tm = client.get_trafficmanager(tm_port)
    tm.set_global_distance_to_leading_vehicle(tm_config.global_distance_to_leading_vehicle)
    tm.global_percentage_speed_difference(tm_config.speed_percentage_difference)
    batch: list[carla.command.BatchCommand] = []
    for sp in spawn_points[:count]:
        bp = random.choice(car_bps)
        if bp.has_attribute("color"):
            bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))
        bp.set_attribute("role_name", config.color_role)
        batch.append(
            carla.command.SpawnActor(bp, sp).then(
                carla.command.SetAutopilot(
                    carla.command.FutureActor, enabled=True, tm_port=tm.get_port(),
                ),
            ),
        )
    results = client.apply_batch_sync(batch, do_tick=False)
    spawned = 0
    for r in results:
        if not r.error:
            _state.vehicles.append(r.actor_id)
            spawned += 1
    _LOG.info("Vehicles: %d/%d spawned", spawned, count)
    return SpawnResult(spawned=spawned, requested=count)


def _spawn_walkers(
    world: carla.World, client: carla.Client,
    walker_config: WalkerSpawnConfig, count: int,
) -> SpawnResult:
    """Spawn pedestrians with AI controllers (async mode)."""
    bp_lib = world.get_blueprint_library()
    walker_bps = bp_lib.filter("walker.pedestrian.*")
    controller_bp = bp_lib.find("controller.ai.walker")
    spawn_points: list[carla.Transform] = []
    for _ in range(count * walker_config.spawn_multiplier):
        loc = world.get_random_location_from_navigation()
        if loc:
            spawn_points.append(carla.Transform(loc))
        if len(spawn_points) >= count:
            break
    batch, speeds = _build_walker_batch(spawn_points[:count], walker_bps, walker_config)
    results = client.apply_batch_sync(batch, do_tick=False)
    valid_speeds: list[float] = []
    for i, r in enumerate(results):
        if not r.error:
            _state.walkers.append(WalkerEntry(walker_id=r.actor_id))
            valid_speeds.append(speeds[i])
    batch_ctrl: list[carla.command.BatchCommand] = [
        carla.command.SpawnActor(controller_bp, carla.Transform(), w.walker_id)
        for w in _state.walkers
    ]
    ctrl_results = client.apply_batch_sync(batch_ctrl, do_tick=False)
    for i, r in enumerate(ctrl_results):
        if not r.error:
            _state.walkers[i] = WalkerEntry(
                walker_id=_state.walkers[i].walker_id, controller_id=r.actor_id,
            )
    for w in _state.walkers:
        if w.controller_id is not None:
            _state.all_actor_ids.append(w.controller_id)
        _state.all_actor_ids.append(w.walker_id)
    world.wait_for_tick()
    world.set_pedestrians_cross_factor(walker_config.cross_factor)
    _start_walker_controllers(world, valid_speeds, walker_config.default_speed)
    _LOG.info("Walkers: %d/%d spawned with AI controllers", len(_state.walkers), count)
    return SpawnResult(spawned=len(_state.walkers), requested=count)


def _build_walker_batch(
    spawn_points: list[carla.Transform],
    walker_bps: list[carla.ActorBlueprint],
    walker_config: WalkerSpawnConfig,
) -> tuple[list[carla.command.BatchCommand], list[float]]:
    """Build spawn batch and speed list for walkers."""
    batch: list[carla.command.BatchCommand] = []
    speeds: list[float] = []
    for sp in spawn_points:
        bp = random.choice(walker_bps)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        speed_attr = bp.get_attribute("speed")
        if speed_attr and speed_attr.recommended_values:
            idx = (
                _WALKER_RECOMMENDED_SPEED_INDEX_RUN
                if random.random() < walker_config.run_probability
                else _WALKER_RECOMMENDED_SPEED_INDEX_WALK
            )
            speeds.append(float(speed_attr.recommended_values[idx]))
        else:
            speeds.append(walker_config.default_speed)
        batch.append(carla.command.SpawnActor(bp, sp))
    return batch, speeds


def _start_walker_controllers(
    world: carla.World, valid_speeds: list[float], default_speed: float,
) -> None:
    """Start AI controllers for all spawned walkers."""
    all_actors = world.get_actors(_state.all_actor_ids)
    for i in range(0, len(_state.all_actor_ids), _WALKER_ACTOR_STEP):
        with contextlib.suppress(RuntimeError, IndexError):
            ctrl = all_actors[i]
            ctrl.start()
            ctrl.go_to_location(world.get_random_location_from_navigation())
            idx = i // _WALKER_ACTOR_STEP
            ctrl.set_max_speed(valid_speeds[idx] if idx < len(valid_speeds) else default_speed)


def _health_check_walkers(world: carla.World) -> int:
    """Reassign destinations to all walkers; fixes frozen walkers."""
    if not _state.all_actor_ids:
        return 0
    try:
        actors = world.get_actors(_state.all_actor_ids)
        reassigned = 0
        for i in range(0, len(_state.all_actor_ids), _WALKER_ACTOR_STEP):
            with contextlib.suppress(RuntimeError, IndexError):
                ctrl = actors[i]
                loc = world.get_random_location_from_navigation()
                if loc and ctrl.is_alive:
                    ctrl.go_to_location(loc)
                    reassigned += 1
    except RuntimeError:
        return 0
    else:
        return reassigned


def _health_check_vehicles(
    world: carla.World, client: carla.Client, tm_port: int, stall_speed: float,
) -> int:
    """Re-enable autopilot on any vehicle that lost it."""
    if not _state.vehicles:
        return 0
    try:
        actors = world.get_actors(_state.vehicles)
        restarted = 0
        for a in actors:
            with contextlib.suppress(RuntimeError):
                vel = a.get_velocity()
                speed = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** _ZERO_SPEED
                if speed < stall_speed:
                    a.set_autopilot(enabled=True, tm_port=tm_port)
                    restarted += 1
    except RuntimeError:
        return 0
    else:
        return restarted


def _cleanup() -> None:
    """Destroy all spawned actors."""
    if not _state.client:
        return
    try:
        if _state.all_actor_ids and _state.world:
            all_actors = _state.world.get_actors(_state.all_actor_ids)
            for i in range(0, len(_state.all_actor_ids), _WALKER_ACTOR_STEP):
                with contextlib.suppress(RuntimeError):
                    all_actors[i].stop()
        if _state.vehicles:
            _state.client.apply_batch(
                [carla.command.DestroyActor(aid) for aid in _state.vehicles],
            )
            _LOG.info("Destroyed %d vehicles", len(_state.vehicles))
        if _state.all_actor_ids:
            _state.client.apply_batch(
                [carla.command.DestroyActor(aid) for aid in _state.all_actor_ids],
            )
            _LOG.info("Destroyed %d walkers/controllers", len(_state.all_actor_ids))
    except RuntimeError as exc:
        _LOG.warning("Cleanup warning: %s", exc)


def _parse_args() -> SimulationConfig:
    """Parse CLI arguments and return a validated SimulationConfig."""
    parser = argparse.ArgumentParser(description="CarlaAir Auto Traffic Generator")
    parser.add_argument("--vehicles", "-n", type=int, default=_DEFAULT_VEHICLE_COUNT)
    parser.add_argument("--walkers", "-w", type=int, default=_DEFAULT_WALKER_COUNT)
    parser.add_argument("--port", "-p", type=int, default=_DEFAULT_CARLA_PORT)
    parser.add_argument("--tm-port", type=int, default=_DEFAULT_TM_PORT)
    args = parser.parse_args()
    return SimulationConfig(
        vehicle_count=args.vehicles, walker_count=args.walkers,
        carla_port=args.port, tm_port=args.tm_port,
    )


def _connect_to_carla(
    port: int, timeout: float, max_attempts: int, retry_delay: float,
) -> tuple[carla.Client, carla.World]:
    """Connect to the CARLA simulator with retry logic."""
    client = carla.Client("localhost", port)
    client.set_timeout(timeout)

    def _try_connect(attempt: int) -> carla.World | None:
        _LOG.info("Connecting to CARLA (port %d) …", port)
        try:
            return client.get_world()
        except RuntimeError as exc:
            if attempt < max_attempts:
                _LOG.info("  Waiting … (attempt %d/%d, error: %s)", attempt, max_attempts, exc)
                time.sleep(retry_delay)
            else:
                _LOG.error("Failed to connect after %d attempts.", max_attempts)
                sys.exit(1)
            return None

    for attempt in range(1, max_attempts + 1):
        world = _try_connect(attempt)
        if world is not None:
            return client, world

    msg = "Connection loop exited unexpectedly"
    raise RuntimeError(msg)


def _ensure_async_mode(world: carla.World) -> None:
    """Switch world to async mode if synchronous."""
    settings = world.get_settings()
    if settings.synchronous_mode:
        _LOG.warning("World is in sync mode; switching to async for traffic.")
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = _ZERO_SPEED
        world.apply_settings(settings)


def _handle_runtime_error() -> None:
    """Handle lost CARLA connection in health loop."""
    _LOG.info("Connection interrupted; waiting to reconnect …")
    time.sleep(_RECONNECT_DELAY)
    if _state.client:
        with contextlib.suppress(RuntimeError):
            _state.world = _state.client.get_world()
            if _state.world:
                _LOG.info("Reconnected: %s", _state.world.get_map().name)


def _health_tick(config: SimulationConfig, last_health: float) -> float:
    """Run one health check tick; return updated last_health timestamp."""
    time.sleep(config.main_loop_sleep)
    now = time.time()
    if now - last_health >= config.health_interval and _state.world and _state.client:
        _health_check_walkers(_state.world)
        rv = _health_check_vehicles(
            _state.world, _state.client, config.tm_port, config.vehicle_stall_speed,
        )
        if rv > _ERROR_LOG_THRESHOLD:
            _LOG.info("Health: restarted %d stalled vehicles", rv)
        return now
    return last_health


def _run_health_loop(config: SimulationConfig) -> None:
    """Run the main health-check loop."""
    last_health = time.time()
    # Exception handling intentionally wraps the full loop body to catch
    # connection loss and OS errors without nested try-except.
    while _state.running:
        exc_rt: RuntimeError | None = None
        exc_os: OSError | None = None
        try:
            last_health = _health_tick(config, last_health)
        except RuntimeError as exc:
            exc_rt = exc
        except OSError as exc:
            exc_os = exc
        if exc_rt is not None:
            _handle_runtime_error()
        elif exc_os is not None and _state.running:
            _LOG.error("Error: %s", exc_os)
            time.sleep(_RECONNECT_DELAY)


def main() -> None:
    """Entry-point: spawn vehicles/walkers and run periodic health checks."""
    _configure_logging()
    config = _parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    _state.client, _state.world = _connect_to_carla(
        port=config.carla_port, timeout=config.connection_timeout,
        max_attempts=config.connection_max_attempts, retry_delay=config.connection_retry_delay,
    )
    map_name = _state.world.get_map().name.split("/")[-1]
    _LOG.info("Connected: %s", map_name)
    _ensure_async_mode(_state.world)
    nv = _spawn_vehicles(
        _state.world, _state.client, VehicleSpawnConfig(), TrafficManagerConfig(),
        count=config.vehicle_count, tm_port=config.tm_port,
    )
    nw = _spawn_walkers(_state.world, _state.client, WalkerSpawnConfig(), count=config.walker_count)
    _LOG.info("Traffic ready: %d vehicles + %d walkers on %s", nv.spawned, nw.spawned, map_name)
    _LOG.info("Running in background. Health checks every %.1fs.", config.health_interval)
    _run_health_loop(config)
    _cleanup()
    _LOG.info("Traffic generator stopped.")


if __name__ == "__main__":
    main()
