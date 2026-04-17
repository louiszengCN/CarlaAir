# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import annotations

import filecmp
import os
import shutil
import time

import numpy as np

import carla

from . import SmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_SNAPSHOT_COLUMNS: int = 11
_MAX_SNAPSHOT_ERROR: float = 0.2
_WORLD_RELOAD_DELAY: float = 5.0
_DEFAULT_WAIT_FRAMES: int = 100
_DEFAULT_SIM_TICS: int = 200
_FIXED_DELTA: float = 0.05
_MAX_SUBSTEPS: int = 16
_SPECTATOR_Z: float = 10.0
_DEFAULT_REPETITIONS: int = 5
_DEFAULT_TEST_TICS: int = 100

SpawnActor = carla.command.SpawnActor
FutureActor = carla.command.FutureActor
ApplyTargetVelocity = carla.command.ApplyTargetVelocity


class DeterminismError(Exception):
    pass


class Scenario:
    def __init__(
        self,
        client: object,
        world: object,
        *,
        save_snapshots_mode: bool = False,
    ) -> None:
        self.world = world
        self.client = client
        self.actor_list: list[tuple[str, object]] = []
        self.init_timestamp: dict[str, float] = {}
        self.active = False
        self.prefix = ""
        self.save_snapshots_mode = save_snapshots_mode
        self.snapshots: list[object] = []

    def init_scene(
        self,
        prefix: str,
        settings: object = None,
        spectator_tr: object = None,
    ) -> None:
        self.prefix = prefix
        self.actor_list = []
        self.active = True
        self.snapshots = []

        self.reload_world(settings, spectator_tr)

        # Init timestamp
        snapshot = self.world.get_snapshot()
        self.init_timestamp = {
            "frame0": snapshot.frame,
            "time0": snapshot.timestamp.elapsed_seconds,
        }

    def add_actor(self, actor: object, actor_name: str = "Actor") -> None:
        actor_idx = len(self.actor_list)
        name = f"{actor_idx}_{actor_name}"
        self.actor_list.append((name, actor))

        if self.save_snapshots_mode:
            self.snapshots.append(np.empty((0, _SNAPSHOT_COLUMNS), float))

    def wait(self, frames: int = _DEFAULT_WAIT_FRAMES) -> None:
        for _i in range(frames):
            self.world.tick()

    def clear_scene(self) -> None:
        for actor in self.actor_list:
            actor[1].destroy()
        self.active = False

    def reload_world(
        self,
        settings: object = None,
        spectator_tr: object = None,
    ) -> None:
        if settings is not None:
            self.world.apply_settings(settings)
        self.wait(5)

        self.client.reload_world(reset_settings=False)
        # workaround: give time to UE4 to clean memory after loading
        time.sleep(_WORLD_RELOAD_DELAY)

        self.wait(5)

    def reset_spectator(self, spectator_tr: object) -> None:
        spectator = self.world.get_spectator()
        spectator.set_transform(spectator_tr)

    def save_snapshot(self, actor: object) -> np.ndarray:
        snapshot = self.world.get_snapshot()
        return np.array([
            float(snapshot.frame - self.init_timestamp["frame0"]),
            snapshot.timestamp.elapsed_seconds - self.init_timestamp["time0"],
            actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z,
            actor.get_location().x, actor.get_location().y, actor.get_location().z,
            actor.get_angular_velocity().x,
            actor.get_angular_velocity().y,
            actor.get_angular_velocity().z,
        ])

    def save_snapshots(self) -> None:
        if not self.save_snapshots_mode:
            return
        for i in range(len(self.actor_list)):
            self.snapshots[i] = np.vstack((
                self.snapshots[i],
                self.save_snapshot(self.actor_list[i][1]),
            ))

    def save_snapshots_to_disk(self) -> None:
        if not self.save_snapshots_mode:
            return
        for i, actor in enumerate(self.actor_list):
            np.savetxt(self.get_filename(actor[0]), self.snapshots[i])

    def get_filename_with_prefix(
        self,
        prefix: str,
        actor_id: str | None = None,
        frame: int | None = None,
    ) -> str:
        add_id = "" if actor_id is None else f"_{actor_id}"
        add_frame = "" if frame is None else f"_{frame:04d}"
        return prefix + add_id + add_frame + ".out"

    def get_filename(
        self,
        actor_id: str | None = None,
        frame: int | None = None,
    ) -> str:
        return self.get_filename_with_prefix(self.prefix, actor_id, frame)

    def run_simulation(
        self,
        prefix: str,
        run_settings: object,
        spectator_tr: object,
        *,
        tics: int = _DEFAULT_SIM_TICS,
    ) -> None:
        original_settings = self.world.get_settings()
        self.init_scene(prefix, run_settings, spectator_tr)

        for _i in range(tics):
            self.world.tick()
            self.save_snapshots()

        self.world.apply_settings(original_settings)
        self.save_snapshots_to_disk()
        self.clear_scene()


class TwoCarsHighSpeedCollision(Scenario):
    def init_scene(
        self,
        prefix: str,
        settings: object = None,
        spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)

        blueprint_library = self.world.get_blueprint_library()

        vehicle00_bp = blueprint_library.filter("tt")[0]
        vehicle01_bp = blueprint_library.filter("mkz_2017")[0]

        vehicle00_tr = carla.Transform(
            carla.Location(140, -256, 0.015), carla.Rotation(yaw=180),
        )
        vehicle01_tr = carla.Transform(
            carla.Location(40, -255, 0.04), carla.Rotation(yaw=0),
        )

        batch = [
            SpawnActor(vehicle00_bp, vehicle00_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(-50, 0, 0))),
            SpawnActor(vehicle01_bp, vehicle01_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(+50, 0, 0))),
        ]

        responses = self.client.apply_batch_sync(batch)
        veh_ids = [x.actor_id for x in responses]
        veh_refs = [self.world.get_actor(x) for x in veh_ids]

        if (0 in veh_ids) or (None in veh_refs):
            self.fail("The test cars could not be correctly spawned")

        self.add_actor(veh_refs[0], "Car")
        self.add_actor(veh_refs[1], "Car")
        self.wait(1)


class ThreeCarsSlowSpeedCollision(Scenario):
    def init_scene(
        self,
        prefix: str,
        settings: object = None,
        spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)

        blueprint_library = self.world.get_blueprint_library()

        vehicle00_bp = blueprint_library.filter("prius")[0]
        vehicle01_bp = blueprint_library.filter("a2")[0]
        vehicle02_bp = blueprint_library.filter("lincoln")[0]

        vehicle00_tr = carla.Transform(
            carla.Location(110, -255, 0.05), carla.Rotation(yaw=180),
        )
        vehicle01_tr = carla.Transform(
            carla.Location(53, -257, 0.00), carla.Rotation(yaw=0),
        )
        vehicle02_tr = carla.Transform(
            carla.Location(85, -230, 0.04), carla.Rotation(yaw=-90),
        )

        batch = [
            SpawnActor(vehicle00_bp, vehicle00_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(-15, 0, 0))),
            SpawnActor(vehicle01_bp, vehicle01_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(+15, 0, 0))),
            SpawnActor(vehicle02_bp, vehicle02_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, -15, 0))),
        ]

        responses = self.client.apply_batch_sync(batch)
        veh_ids = [x.actor_id for x in responses]
        veh_refs = [self.world.get_actor(x) for x in veh_ids]

        self.add_actor(veh_refs[0], "Car")
        self.add_actor(veh_refs[1], "Car")
        self.add_actor(veh_refs[2], "Car")
        self.wait(1)


class CarBikeCollision(Scenario):
    def init_scene(
        self,
        prefix: str,
        settings: object = None,
        spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)

        blueprint_library = self.world.get_blueprint_library()

        car_bp = blueprint_library.filter("mkz_2017")[0]
        bike_bp = blueprint_library.filter("gazelle")[0]

        car_tr = carla.Transform(
            carla.Location(50, -255, 0.04), carla.Rotation(yaw=0),
        )
        bike_tr = carla.Transform(
            carla.Location(85, -245, 0.04), carla.Rotation(yaw=-90),
        )

        batch = [
            SpawnActor(car_bp, car_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(30, 0, 0))),
            SpawnActor(bike_bp, bike_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, -12, 0))),
        ]

        responses = self.client.apply_batch_sync(batch)
        veh_ids = [x.actor_id for x in responses]
        veh_refs = [self.world.get_actor(x) for x in veh_ids]

        if (0 in veh_ids) or (None in veh_refs):
            self.fail("The test cars could not be correctly spawned")

        self.add_actor(veh_refs[0], "Car")
        self.add_actor(veh_refs[1], "Bike")
        self.wait(1)


class CarWalkerCollision(Scenario):
    def init_scene(
        self,
        prefix: str,
        settings: object = None,
        spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)

        blueprint_library = self.world.get_blueprint_library()

        car_bp = blueprint_library.filter("mkz_2017")[0]
        walker_bp = blueprint_library.filter("walker.pedestrian.0007")[0]
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "false")

        car_tr = carla.Transform(
            carla.Location(50, -255, 0.04), carla.Rotation(yaw=0),
        )
        walker_tr = carla.Transform(
            carla.Location(85, -255, 1.00), carla.Rotation(yaw=-90),
        )

        batch = [
            SpawnActor(car_bp, car_tr)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(20, 0, 0))),
            SpawnActor(walker_bp, walker_tr),
        ]

        responses = self.client.apply_batch_sync(batch)
        veh_ids = [x.actor_id for x in responses]
        veh_refs = [self.world.get_actor(x) for x in veh_ids]

        if (0 in veh_ids) or (None in veh_refs):
            self.fail("The test cars could not be correctly spawned")

        self.wait(1)
        self.add_actor(veh_refs[0], "Car")
        self.add_actor(veh_refs[1], "Walker")
        self.wait(1)


class CollisionScenarioTester:
    def __init__(self, scene: Scenario, output_path: str) -> None:
        self.scene = scene
        self.world = self.scene.world
        self.client = self.scene.client
        self.scenario_name = self.scene.__class__.__name__
        self.output_path = output_path

    def compare_files(self, file_i: str, file_j: str) -> bool:
        check_ij = filecmp.cmp(file_i, file_j)
        if check_ij:
            return True

        data_i = np.loadtxt(file_i)
        data_j = np.loadtxt(file_j)
        max_error = np.amax(np.abs(data_i - data_j))
        return max_error < _MAX_SNAPSHOT_ERROR

    def check_simulations(
        self,
        rep_prefixes: list[str],
        gen_prefix: str,
    ) -> list[int]:
        repetitions = len(rep_prefixes)
        mat_check = np.zeros((repetitions, repetitions), int)

        for i in range(repetitions):
            mat_check[i][i] = 1
            for j in range(i):
                sim_check = True
                for actor in self.scene.actor_list:
                    actor_id = actor[0]
                    file_i = self.scene.get_filename_with_prefix(rep_prefixes[i], actor_id)
                    file_j = self.scene.get_filename_with_prefix(rep_prefixes[j], actor_id)
                    check_ij = self.compare_files(file_i, file_j)
                    sim_check = sim_check and check_ij
                mat_check[i][j] = int(sim_check)
                mat_check[j][i] = int(sim_check)

        determinism = np.sum(mat_check, axis=1)
        return sorted(set(determinism), reverse=True)

    def save_simulations(
        self,
        rep_prefixes: list[str],
        prefix: str,
        max_idx: int,
        min_idx: int,
    ) -> None:
        for actor in self.scene.actor_list:
            actor_id = actor[0]
            reference_id = "reference_" + actor_id
            file_repetition = self.scene.get_filename_with_prefix(rep_prefixes[max_idx], actor_id)
            file_reference = self.scene.get_filename_with_prefix(prefix, reference_id)
            shutil.copyfile(file_repetition, file_reference)

        if min_idx != max_idx:
            for actor in self.scene.actor_list:
                actor_id = actor[0]
                failed_id = "failed_" + actor_id
                file_repetition = self.scene.get_filename_with_prefix(
                    rep_prefixes[min_idx], actor_id,
                )
                file_failed = self.scene.get_filename_with_prefix(prefix, failed_id)
                shutil.copyfile(file_repetition, file_failed)

    def test_scenario(
        self,
        fps: int = 20,
        fps_phys: int = 100,
        repetitions: int = 1,
        sim_tics: int = 100,
    ) -> None:
        prefix = (
            self.output_path + self.scenario_name
            + "_" + str(fps) + "_" + str(fps_phys)
        )

        config_settings = self.world.get_settings()
        config_settings.synchronous_mode = True
        config_settings.fixed_delta_seconds = 1.0 / fps
        config_settings.substepping = True
        config_settings.max_substep_delta_time = 1.0 / fps_phys
        config_settings.max_substeps = _MAX_SUBSTEPS

        spectator_tr = carla.Transform(
            carla.Location(120, -256, _SPECTATOR_Z),
            carla.Rotation(yaw=180),
        )

        sim_prefixes = []
        for i in range(repetitions):
            prefix_rep = prefix + "_rep" + str(i)
            self.scene.run_simulation(
                prefix_rep, config_settings, spectator_tr, tics=sim_tics,
            )
            sim_prefixes.append(prefix_rep)

        determ_repet = self.check_simulations(sim_prefixes, prefix)

        if determ_repet[0] != repetitions:
            msg = (
                f"CollisionTransfError: Scenario {self.scenario_name} "
                f"is not deterministic: {determ_repet[0]} / {repetitions}"
            )
            raise DeterminismError(msg)


class TestCollisionDeterminism(SmokeTest):
    def setUp(self) -> None:
        super().setUp()
        self.world = self.client.get_world()
        self.settings = self.world.get_settings()
        settings = carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=_FIXED_DELTA,
        )
        self.world.apply_settings(settings)
        self.world.tick()

    def tearDown(self) -> None:
        self.settings.synchronous_mode = False
        self.world.apply_settings(self.settings)
        self.world.tick()
        self.settings = None
        self.world = None
        super().tearDown()

    def _run_collision_test(self, scenario_cls: type) -> None:
        output_path = os.path.dirname(os.path.realpath(__file__))
        output_path = os.path.join(output_path, "_collisions") + os.path.sep
        if not os.path.exists(output_path):
            os.mkdir(output_path)

        self.client.load_world("Town03")
        time.sleep(_WORLD_RELOAD_DELAY)

        try:
            test_collision = CollisionScenarioTester(
                scene=scenario_cls(
                    self.client, self.world, save_snapshots_mode=True,
                ),
                output_path=output_path,
            )
            test_collision.test_scenario(
                repetitions=_DEFAULT_REPETITIONS,
                sim_tics=_DEFAULT_TEST_TICS,
            )
        except DeterminismError as err:
            test_collision.scene.clear_scene()
            shutil.rmtree(output_path)
            self.fail(err)

        shutil.rmtree(output_path)

    def test_two_cars(self) -> None:
        self._run_collision_test(TwoCarsHighSpeedCollision)

    def test_three_cars(self) -> None:
        self._run_collision_test(ThreeCarsSlowSpeedCollision)

    def test_car_bike(self) -> None:
        self._run_collision_test(CarBikeCollision)

    def test_car_walker(self) -> None:
        self._run_collision_test(CarWalkerCollision)
