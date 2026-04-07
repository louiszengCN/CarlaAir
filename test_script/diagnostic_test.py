#!/usr/bin/env python3
"""
diagnostic_test.py — Non-interactive diagnostic for all CARLA+AirSim features

Tests each feature systematically and reports pass/fail.
"""

from __future__ import annotations

import sys
import time
import traceback
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import carla


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_CARLA_DEFAULT_PORT: int = 2000
_AIRSIM_DEFAULT_PORT: int = 41451
_CARLA_TIMEOUT: float = 10.0
_TEST_VEHICLE_COUNT: int = 3
_TEST_WALKER_COUNT: int = 3
_WALKER_SPAWN_OFFSET_Z: float = 1.0
_WALKER_CONTROL_SPEED: float = 1.5
_WALKER_CONTROL_DURATION: float = 0.5
_CAMERA_RESOLUTION_X: str = "640"
_CAMERA_RESOLUTION_Y: str = "480"
_CAMERA_FOV: str = "100"
_CAMERA_DURATION: float = 2.0
_CAMERA_Z_OFFSET: float = 5.0
_CAMERA_PITCH: float = -30.0
_WAYPOINT_DISTANCE: float = 5.0
_SYNC_DELTA: float = 0.05
_SYNC_TICKS: int = 5
_TM_PORT: int = 8000
_DEBUG_Z: float = 10.0
_DEBUG_LINE_END_X: float = 5.0
_DEBUG_STRING_Z: float = 12.0
_SPECTATOR_Z: float = 50.0
_SPECTATOR_PITCH: float = -45.0
_AIRSIM_ALTITUDE: float = -5.0
_AIRSIM_FLIGHT_DURATION: float = 3.0


# ──────────────────────────────────────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────────────────────────────────────


class TestCategory(Enum):
    """Test categories for grouping and reporting."""

    CARLA = "carla"
    AIRSIM = "airsim"
    DRONE = "drone"


class TestStatus(Enum):
    """Test execution status."""

    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"


# ──────────────────────────────────────────────────────────────────────────────
# Dataclasses
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, slots=True)
class TestResult:
    """Result of a single test execution."""

    name: str
    status: TestStatus
    category: TestCategory
    error: str | None = None
    data: Any = None


@dataclass(frozen=True, slots=True)
class TestSummary:
    """Summary of all test results."""

    results: list[TestResult]
    passed: int
    failed: int
    skipped: int
    total: int

    @property
    def all_passed(self) -> bool:
        """Check if all tests passed."""
        return self.failed == 0


# ──────────────────────────────────────────────────────────────────────────────
# Test Functions
# ──────────────────────────────────────────────────────────────────────────────


def test_carla_basic(
    carla_port: int = _CARLA_DEFAULT_PORT,
) -> tuple[carla.Client, carla.World, carla.BlueprintLibrary, list[carla.Transform]]:
    """Test basic CARLA connection and world access."""
    import carla

    client = carla.Client("localhost", carla_port)
    client.set_timeout(_CARLA_TIMEOUT)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    map_inst = world.get_map()
    spawn_points = map_inst.get_spawn_points()

    print(f"  Map: {map_inst.name}")
    print(f"  Spawn points: {len(spawn_points)}")
    print(f"  Blueprints: {len(bp_lib)}")
    return client, world, bp_lib, spawn_points


def test_weather(world: carla.World) -> None:
    """Test weather control."""
    import carla

    world.set_weather(carla.WeatherParameters.ClearNoon)
    weather = world.get_weather()
    assert weather.sun_altitude_angle > 0, "Sun should be up at ClearNoon"

    world.set_weather(carla.WeatherParameters.HardRainNoon)
    weather = world.get_weather()
    assert weather.precipitation > 0, "Should have rain"

    world.set_weather(carla.WeatherParameters.ClearNoon)
    print("  Weather control OK")


def test_vehicle_spawn(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    spawn_points: list[carla.Transform],
) -> list[carla.Vehicle]:
    """Test vehicle spawning and autopilot."""
    import random

    vehicle_bps = bp_lib.filter("vehicle.*")
    print(f"  Vehicle types: {len(vehicle_bps)}")

    vehicles: list[carla.Vehicle] = []
    for i in range(_TEST_VEHICLE_COUNT):
        bp = random.choice(vehicle_bps)
        try:
            v = world.spawn_actor(bp, spawn_points[i])
            v.set_autopilot(True)
            vehicles.append(v)
        except Exception as e:  # noqa: BLE001
            print(f"  WARN: spawn failed for {bp.id}: {e}")

    print(f"  Spawned {len(vehicles)} vehicles with autopilot")
    assert len(vehicles) > 0, "Should spawn at least 1 vehicle"
    return vehicles


def test_walker_spawn(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    spawn_points: list[carla.Transform],
) -> list[carla.Actor]:
    """Test walker spawning."""
    import random

    walker_bps = bp_lib.filter("walker.pedestrian.*")
    print(f"  Walker types: {len(walker_bps)}")

    walkers: list[carla.Actor] = []
    for i in range(_TEST_WALKER_COUNT):
        bp = random.choice(walker_bps)
        sp = spawn_points[min(20 + i, len(spawn_points) - 1)]
        sp.location.z += _WALKER_SPAWN_OFFSET_Z
        try:
            w = world.spawn_actor(bp, sp)
            walkers.append(w)
        except Exception as e:  # noqa: BLE001
            print(f"  WARN: spawn failed: {e}")

    print(f"  Spawned {len(walkers)} walkers")
    assert len(walkers) > 0, "Should spawn at least 1 walker"
    return walkers


def test_walker_control(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    spawn_points: list[carla.Transform],
) -> None:
    """Test WalkerControl (used by human_traj_col.py)."""
    import carla

    walker_bp = bp_lib.filter("walker.pedestrian.*")[0]
    sp = spawn_points[min(40, len(spawn_points) - 1)]
    sp.location.z += _WALKER_SPAWN_OFFSET_Z

    walker = world.spawn_actor(walker_bp, sp)
    control = carla.WalkerControl()
    control.direction = carla.Vector3D(x=1.0, y=0.0, z=0.0)
    control.speed = _WALKER_CONTROL_SPEED
    walker.apply_control(control)
    time.sleep(_WALKER_CONTROL_DURATION)

    loc = walker.get_location()
    print(f"  Walker at ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
    walker.destroy()
    print("  WalkerControl OK")


def test_camera_sensor(
    world: carla.World,
    bp_lib: carla.BlueprintLibrary,
    spawn_points: list[carla.Transform],
) -> None:
    """Test camera sensor attachment."""
    import carla

    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", _CAMERA_RESOLUTION_X)
    cam_bp.set_attribute("image_size_y", _CAMERA_RESOLUTION_Y)
    cam_bp.set_attribute("fov", _CAMERA_FOV)

    sp = spawn_points[0]
    cam = world.spawn_actor(
        cam_bp,
        carla.Transform(
            carla.Location(x=sp.location.x, y=sp.location.y, z=sp.location.z + _CAMERA_Z_OFFSET),
            carla.Rotation(pitch=_CAMERA_PITCH),
        ),
    )

    received = [False]
    width_height = [0, 0]

    def on_image(image: carla.Image) -> None:
        if not received[0]:
            width_height[0] = image.width
            width_height[1] = image.height
            received[0] = True

    cam.listen(on_image)
    time.sleep(_CAMERA_DURATION)
    cam.stop()
    cam.destroy()

    assert received[0], "Should receive camera image"
    print(f"  Camera: {width_height[0]}x{width_height[1]} OK")


def test_opendrive(world: carla.World) -> None:
    """Test OpenDRIVE map data."""
    map_inst = world.get_map()
    waypoints = map_inst.generate_waypoints(_WAYPOINT_DISTANCE)
    topology = map_inst.get_topology()
    print(f"  Waypoints: {len(waypoints)}, Topology: {len(topology)}")
    assert len(waypoints) > 0, "Should have waypoints"
    assert len(topology) > 0, "Should have topology"


def test_navigation_mesh(world: carla.World) -> bool:
    """Test navigation mesh (used by human_traj_col.py)."""
    loc = world.get_random_location_from_navigation()
    if loc is not None:
        print(f"  Nav mesh location: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
    else:
        print("  WARN: Navigation mesh returned None (may not be loaded)")
    return loc is not None


def test_debug_drawing(world: carla.World) -> None:
    """Test debug drawing (used by all test_scripts)."""
    import carla

    world.debug.draw_point(
        carla.Location(x=0, y=0, z=_DEBUG_Z),
        size=0.5,
        color=carla.Color(255, 0, 0),
        life_time=5.0,
    )
    world.debug.draw_line(
        carla.Location(x=0, y=0, z=_DEBUG_Z),
        carla.Location(x=_DEBUG_LINE_END_X, y=0, z=_DEBUG_Z),
        thickness=0.1,
        color=carla.Color(0, 255, 0),
        life_time=5.0,
    )
    world.debug.draw_string(
        carla.Location(x=0, y=0, z=_DEBUG_STRING_Z),
        "TEST",
        color=carla.Color(255, 255, 0),
        life_time=5.0,
    )
    print("  Debug drawing OK (point, line, string)")


def test_sync_mode(client: carla.Client, world: carla.World) -> None:
    """Test synchronous mode (used by drone_traj_col.py and playback scripts)."""

    settings = world.get_settings()
    was_sync = settings.synchronous_mode

    settings.synchronous_mode = True
    settings.fixed_delta_seconds = _SYNC_DELTA
    world.apply_settings(settings)

    tm = client.get_trafficmanager(_TM_PORT)
    tm.set_synchronous_mode(True)

    for _ in range(_SYNC_TICKS):
        world.tick()

    settings.synchronous_mode = was_sync
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    tm.set_synchronous_mode(False)
    print("  Synchronous mode OK")


def test_drone_prop_blueprint(bp_lib: carla.BlueprintLibrary) -> str | None:
    """Check if drone prop blueprints exist (used by drone_traj_col/playback)."""
    drone_bp_ids = ["static.prop.drone", "static.prop.dji_inspire"]
    found: str | None = None

    for bp_id in drone_bp_ids:
        try:
            bp_lib.find(bp_id)
            found = bp_id
            print(f"  Found: {bp_id}")
            break
        except Exception:  # noqa: BLE001
            print(f"  NOT found: {bp_id}")

    if found is None:
        print("  WARN: No drone prop blueprint found!")
        print("  Listing all static.prop.* blueprints:")
        props = bp_lib.filter("static.prop.*")
        for p in props:
            print(f"    - {p.id}")
    return found


def test_environment_objects(world: carla.World) -> bool:
    """Test get_environment_objects (used by human_traj_col.py for max building height)."""
    import carla

    buildings = world.get_environment_objects(carla.CityObjectLabel.Buildings)
    print(f"  Buildings: {len(buildings)}")
    if buildings:
        max_h = max(
            b.bounding_box.location.z + b.bounding_box.extent.z for b in buildings
        )
        print(f"  Max building height: {max_h:.1f}m")
    return len(buildings) > 0


def test_spectator(world: carla.World) -> None:
    """Test spectator control."""
    import carla

    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            carla.Location(x=0, y=0, z=_SPECTATOR_Z),
            carla.Rotation(pitch=_SPECTATOR_PITCH),
        )
    )
    print("  Spectator control OK")


def test_airsim_connection(
    airsim_port: int = _AIRSIM_DEFAULT_PORT,
) -> Any:  # airsim.MultirotorClient
    """Test AirSim connection."""
    import airsim

    client = airsim.MultirotorClient(port=airsim_port)
    client.confirmConnection()
    print("  AirSim connected")
    return client


def test_airsim_flight(client: Any) -> None:  # airsim.MultirotorClient
    """Test AirSim takeoff and basic movement."""
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    pos = client.getMultirotorState().kinematics_estimated.position
    print(
        f"  Takeoff position: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})"
    )

    client.moveToZAsync(_AIRSIM_ALTITUDE, _AIRSIM_FLIGHT_DURATION).join()
    pos = client.getMultirotorState().kinematics_estimated.position
    print(
        f"  At altitude: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})"
    )

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("  Flight test OK")


def test_airsim_camera(client: Any) -> None:  # airsim.MultirotorClient
    """Test AirSim camera."""
    import airsim

    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(_AIRSIM_ALTITUDE, _AIRSIM_FLIGHT_DURATION).join()

    responses = client.simGetImages(
        [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
    )

    w, h = responses[0].width, responses[0].height
    print(f"  Camera: {w}x{h}")
    assert w > 0 and h > 0, "Camera image should have valid dimensions"

    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("  AirSim camera OK")


# ──────────────────────────────────────────────────────────────────────────────
# Test Runner
# ──────────────────────────────────────────────────────────────────────────────


def run_test(
    name: str,
    func: Any,  # Callable
    category: TestCategory,
    *args: Any,
    **kwargs: Any,
) -> TestResult:
    """Run a test and catch exceptions."""
    print(f"\n--- {name} ---")
    try:
        result = func(*args, **kwargs)
        print(f"  [PASS] {name}")
        return TestResult(
            name=name,
            status=TestStatus.PASS,
            category=category,
            data=result,
        )
    except Exception as e:  # noqa: BLE001
        print(f"  [FAIL] {name}: {e}")
        traceback.print_exc()
        return TestResult(
            name=name,
            status=TestStatus.FAIL,
            category=category,
            error=str(e),
        )


def _print_summary(summary: TestSummary) -> None:
    """Print test summary."""
    print("\n" + "=" * 60)
    print("  SUMMARY")
    print("=" * 60)

    for result in summary.results:
        status = result.status.value
        print(f"  [{status}] {result.name}")

    print(f"\n  {summary.passed}/{summary.total} tests passed")

    if summary.all_passed:
        print("\n  ALL TESTS PASSED!")
    else:
        failed_names = [r.name for r in summary.results if r.status == TestStatus.FAIL]
        print(f"\n  FAILED: {', '.join(failed_names)}")


def main() -> int:
    """Run all diagnostic tests."""
    results: list[TestResult] = []

    print("=" * 60)
    print("  CARLA + AirSim Diagnostic Test")
    print("=" * 60)

    # CARLA tests
    result = run_test(
        "CARLA Connection", test_carla_basic, TestCategory.CARLA
    )
    results.append(result)
    if result.status != TestStatus.PASS:
        print("\nCRITICAL: Cannot connect to CARLA. Aborting.")
        _print_summary(TestSummary(results, 0, 1, 0, 1))
        return 1

    client, world, bp_lib, spawn_points = result.data  # type: ignore[misc]

    carla_tests = [
        ("Weather Control", test_weather, (world,)),
        ("Vehicle Spawn", test_vehicle_spawn, (world, bp_lib, spawn_points)),
        ("Walker Spawn", test_walker_spawn, (world, bp_lib, spawn_points)),
        ("Walker Control", test_walker_control, (world, bp_lib, spawn_points)),
        ("Camera Sensor", test_camera_sensor, (world, bp_lib, spawn_points)),
        ("OpenDRIVE", test_opendrive, (world,)),
        ("Navigation Mesh", test_navigation_mesh, (world,)),
        ("Debug Drawing", test_debug_drawing, (world,)),
        ("Sync Mode", test_sync_mode, (client, world)),
        ("Drone Prop Blueprint", test_drone_prop_blueprint, (bp_lib,)),
        ("Environment Objects", test_environment_objects, (world,)),
        ("Spectator", test_spectator, (world,)),
    ]

    for name, func, args in carla_tests:
        result = run_test(name, func, TestCategory.CARLA, *args)
        results.append(result)

    # Cleanup CARLA actors
    vehicles: list[carla.Vehicle] = []
    walkers: list[carla.Actor] = []
    for r in results:
        if r.name == "Vehicle Spawn" and r.data:
            vehicles = r.data
        elif r.name == "Walker Spawn" and r.data:
            walkers = r.data

    for v in vehicles:
        try:
            v.destroy()
        except Exception:  # noqa: BLE001
            pass
    for w in walkers:
        try:
            w.destroy()
        except Exception:  # noqa: BLE001
            pass

    # AirSim tests
    result = run_test(
        "AirSim Connection", test_airsim_connection, TestCategory.AIRSIM
    )
    results.append(result)

    if result.status == TestStatus.PASS:
        airsim_client = result.data
        airsim_tests = [
            ("AirSim Flight", test_airsim_flight, (airsim_client,)),
            ("AirSim Camera", test_airsim_camera, (airsim_client,)),
        ]
        for name, func, args in airsim_tests:
            result = run_test(name, func, TestCategory.AIRSIM, *args)
            results.append(result)

    # Summary
    passed = sum(1 for r in results if r.status == TestStatus.PASS)
    failed = sum(1 for r in results if r.status == TestStatus.FAIL)
    skipped = sum(1 for r in results if r.status == TestStatus.SKIP)
    total = len(results)

    _print_summary(TestSummary(results, passed, failed, skipped, total))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
