#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Test collisions example for CARLA
This script runs several scenarios involving collisions and check if they
are deterministic for different simulation parameters.
"""

from __future__ import annotations

import argparse
import contextlib
import filecmp
import os
import shutil
import time

import numpy as np

import carla

_SNAPSHOT_COLUMNS: int = 11
_MAX_ERROR: float = 0.01
_MAX_SUBSTEPS: int = 16
_DEFAULT_REPETITIONS: int = 10


class Scenario:
    def __init__(
        self, client: object, world: object, *, save_snapshots_mode: bool = False,
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
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        self.prefix = prefix
        self.actor_list = []
        self.active = True
        self.snapshots = []
        self.reload_world(settings, spectator_tr)
        world_snapshot = self.world.get_snapshot()
        self.init_timestamp = {
            "frame0": world_snapshot.frame,
            "time0": world_snapshot.timestamp.elapsed_seconds,
        }

    def add_actor(self, actor: object, actor_name: str = "Actor") -> None:
        actor_idx = len(self.actor_list)
        name = f"{actor_idx}_{actor_name}"
        self.actor_list.append((name, actor))
        if self.save_snapshots_mode:
            self.snapshots.append(np.empty((0, _SNAPSHOT_COLUMNS), float))

    def wait(self, frames: int = 100) -> None:
        for _i in range(frames):
            self.world.tick()

    def clear_scene(self) -> None:
        for actor in self.actor_list:
            actor[1].destroy()
        self.active = False

    def reload_world(
        self, settings: object = None, spectator_tr: object = None,
    ) -> None:
        self.client.reload_world()
        if settings is not None:
            self.world.apply_settings(settings)
        if spectator_tr is not None:
            self.reset_spectator(spectator_tr)

    def reset_spectator(self, spectator_tr: object) -> None:
        spectator = self.world.get_spectator()
        spectator.set_transform(spectator_tr)

    def save_snapshot(self, actor: object) -> np.ndarray:
        snapshot = self.world.get_snapshot()
        return np.array([
            float(snapshot.frame - self.init_timestamp["frame0"]),
            snapshot.timestamp.elapsed_seconds - self.init_timestamp["time0"],
            actor.get_location().x, actor.get_location().y, actor.get_location().z,
            actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z,
            actor.get_angular_velocity().x,
            actor.get_angular_velocity().y,
            actor.get_angular_velocity().z,
        ])

    def save_snapshots(self) -> None:
        if not self.save_snapshots_mode:
            return
        for i in range(len(self.actor_list)):
            self.snapshots[i] = np.vstack((
                self.snapshots[i], self.save_snapshot(self.actor_list[i][1]),
            ))

    def save_snapshots_to_disk(self) -> None:
        if not self.save_snapshots_mode:
            return
        for i, actor in enumerate(self.actor_list):
            np.savetxt(self.get_filename(actor[0]), self.snapshots[i])

    def get_filename_with_prefix(
        self, prefix: str, actor_id: str | None = None, frame: int | None = None,
    ) -> str:
        add_id = "" if actor_id is None else f"_{actor_id}"
        add_frame = "" if frame is None else f"_{frame:04d}"
        return prefix + add_id + add_frame + ".out"

    def get_filename(
        self, actor_id: str | None = None, frame: int | None = None,
    ) -> str:
        return self.get_filename_with_prefix(self.prefix, actor_id, frame)

    def run_simulation(
        self,
        prefix: str,
        run_settings: object,
        spectator_tr: object,
        *,
        tics: int = 200,
    ) -> float:
        original_settings = self.world.get_settings()
        self.init_scene(prefix, run_settings, spectator_tr)

        t_start = time.perf_counter()
        for _i in range(tics):
            self.world.tick()
            self.save_snapshots()
        t_end = time.perf_counter()

        self.world.apply_settings(original_settings)
        self.save_snapshots_to_disk()
        self.clear_scene()
        return t_end - t_start


class TwoSpawnedCars(Scenario):
    def init_scene(
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)
        bl = self.world.get_blueprint_library()
        v00 = self.world.spawn_actor(
            bl.filter("tt")[0],
            carla.Transform(carla.Location(100, -257, 0.02), carla.Rotation(yaw=181.5)),
        )
        v01 = self.world.spawn_actor(
            bl.filter("lincoln")[0],
            carla.Transform(carla.Location(110, -253, 0.04), carla.Rotation(yaw=181.5)),
        )
        self.wait(1)
        v00.set_target_velocity(carla.Vector3D(-25, 0, 0))
        v01.set_target_velocity(carla.Vector3D(-25, 0, 0))
        self.add_actor(v00, "Car")
        self.add_actor(v01, "Car")
        self.wait(1)


class TwoCarsSlowSpeedCollision(Scenario):
    def init_scene(
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)
        bl = self.world.get_blueprint_library()
        v00 = self.world.spawn_actor(
            bl.filter("tt")[0],
            carla.Transform(carla.Location(100, -256, 0.015), carla.Rotation(yaw=178)),
        )
        v01 = self.world.spawn_actor(
            bl.filter("lincoln")[0],
            carla.Transform(carla.Location(40, -255, 0.04), carla.Rotation(yaw=0)),
        )
        self.wait(1)
        v00.set_target_velocity(carla.Vector3D(-12, 0, 0))
        v01.set_target_velocity(carla.Vector3D(+12, 0, 0))
        self.add_actor(v00, "Car")
        self.add_actor(v01, "Car")
        self.wait(1)


class TwoCarsHighSpeedCollision(Scenario):
    def init_scene(
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)
        bl = self.world.get_blueprint_library()
        v00 = self.world.spawn_actor(
            bl.filter("tt")[0],
            carla.Transform(carla.Location(140, -256, 0.015), carla.Rotation(yaw=180)),
        )
        v01 = self.world.spawn_actor(
            bl.filter("lincoln")[0],
            carla.Transform(carla.Location(40, -255, 0.04), carla.Rotation(yaw=0)),
        )
        self.wait(1)
        v00.set_target_velocity(carla.Vector3D(-50, 0, 0))
        v01.set_target_velocity(carla.Vector3D(+50, 0, 0))
        self.add_actor(v00, "Car")
        self.add_actor(v01, "Car")
        self.wait(1)


class ThreeCarsSlowSpeedCollision(Scenario):
    def init_scene(
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)
        bl = self.world.get_blueprint_library()
        v00 = self.world.spawn_actor(bl.filter("prius")[0],
            carla.Transform(carla.Location(110, -255, 0.05), carla.Rotation(yaw=180)))
        v01 = self.world.spawn_actor(bl.filter("a2")[0],
            carla.Transform(carla.Location(53, -257, 0.00), carla.Rotation(yaw=0)))
        v02 = self.world.spawn_actor(bl.filter("lincoln")[0],
            carla.Transform(carla.Location(85, -230, 0.04), carla.Rotation(yaw=-90)))
        self.wait(1)
        v00.set_target_velocity(carla.Vector3D(-15, 0, 0))
        v01.set_target_velocity(carla.Vector3D(+15, 0, 0))
        v02.set_target_velocity(carla.Vector3D(0, -15, 0))
        self.add_actor(v00, "Car")
        self.add_actor(v01, "Car")
        self.add_actor(v02, "Car")
        self.wait(1)


class ThreeCarsHighSpeedCollision(Scenario):
    def init_scene(
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)
        bl = self.world.get_blueprint_library()
        v00 = self.world.spawn_actor(bl.filter("prius")[0],
            carla.Transform(carla.Location(110, -255, 0.05), carla.Rotation(yaw=180)))
        v01 = self.world.spawn_actor(bl.filter("a2")[0],
            carla.Transform(carla.Location(53, -257, 0.00), carla.Rotation(yaw=0)))
        v02 = self.world.spawn_actor(bl.filter("lincoln")[0],
            carla.Transform(carla.Location(85, -230, 0.04), carla.Rotation(yaw=-90)))
        self.wait(1)
        v00.set_target_velocity(carla.Vector3D(-30, 0, 0))
        v01.set_target_velocity(carla.Vector3D(+30, 0, 0))
        v02.set_target_velocity(carla.Vector3D(0, -30, 0))
        self.add_actor(v00, "Car")
        self.add_actor(v01, "Car")
        self.add_actor(v02, "Car")
        self.wait(1)


class CarBikeCollision(Scenario):
    def init_scene(
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)
        bl = self.world.get_blueprint_library()
        car = self.world.spawn_actor(bl.filter("*lincoln*")[0],
            carla.Transform(carla.Location(50, -255, 0.04), carla.Rotation(yaw=0)))
        bike = self.world.spawn_actor(bl.filter("*gazelle*")[0],
            carla.Transform(carla.Location(85, -245, 0.04), carla.Rotation(yaw=-90)))
        self.wait(1)
        car.set_target_velocity(carla.Vector3D(+30, 0, 0))
        bike.set_target_velocity(carla.Vector3D(0, -12, 0))
        self.add_actor(car, "Car")
        self.add_actor(bike, "Bike")
        self.wait(1)


class CarWalkerCollision(Scenario):
    def init_scene(
        self, prefix: str, settings: object = None, spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)
        bl = self.world.get_blueprint_library()
        car = self.world.spawn_actor(bl.filter("*lincoln*")[0],
            carla.Transform(carla.Location(50, -255, 0.04), carla.Rotation(yaw=0)))
        walker_bp = bl.filter("walker.pedestrian.0007")[0]
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "false")
        walker = self.world.spawn_actor(walker_bp,
            carla.Transform(carla.Location(85, -255, 1.00), carla.Rotation(yaw=-90)))
        self.wait(1)
        car.set_target_velocity(carla.Vector3D(+20, 0, 0))
        walker.set_simulate_physics(enabled=True)
        self.add_actor(car, "Car")
        self.add_actor(walker, "Walker")
        self.wait(1)


class CollisionScenarioTester:
    def __init__(self, scene: Scenario, output_path: str) -> None:
        self.scene = scene
        self.world = self.scene.world
        self.client = self.scene.client
        self.scenario_name = self.scene.__class__.__name__
        self.output_path = output_path

    def compare_files(self, file_i: str, file_j: str) -> bool:
        if filecmp.cmp(file_i, file_j):
            return True
        data_i = np.loadtxt(file_i)
        data_j = np.loadtxt(file_j)
        return np.amax(np.abs(data_i - data_j)) < _MAX_ERROR

    def check_simulations(
        self, rep_prefixes: list[str], gen_prefix: str,
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
                    sim_check = sim_check and self.compare_files(file_i, file_j)
                mat_check[i][j] = int(sim_check)
                mat_check[j][i] = int(sim_check)

        determinism = np.sum(mat_check, axis=1)
        max_rep_equal_idx = np.argmax(determinism)
        min_rep_equal_idx = np.argmin(determinism)
        determinism_set = sorted(set(determinism), reverse=True)
        self.save_simulations(rep_prefixes, gen_prefix, max_rep_equal_idx, min_rep_equal_idx)
        return determinism_set

    def save_simulations(
        self, rep_prefixes: list[str], prefix: str, max_idx: int, min_idx: int,
    ) -> None:
        for actor in self.scene.actor_list:
            actor_id = actor[0]
            reference_id = "reference_" + actor_id
            file_rep = self.scene.get_filename_with_prefix(rep_prefixes[max_idx], actor_id)
            file_ref = self.scene.get_filename_with_prefix(prefix, reference_id)
            shutil.copyfile(file_rep, file_ref)

        if min_idx != max_idx:
            for actor in self.scene.actor_list:
                actor_id = actor[0]
                failed_id = "failed_" + actor_id
                file_rep = self.scene.get_filename_with_prefix(rep_prefixes[min_idx], actor_id)
                file_fail = self.scene.get_filename_with_prefix(prefix, failed_id)
                shutil.copyfile(file_rep, file_fail)

        for r_prefix in rep_prefixes:
            for actor in self.scene.actor_list:
                actor_id = actor[0]
                file_rep = self.scene.get_filename_with_prefix(r_prefix, actor_id)
                os.remove(file_rep)

    def test_scenario(
        self,
        fps: int = 20,
        fps_phys: int = 100,
        *,
        repetitions: int = 1,
        sim_tics: int = 100,
    ) -> str:
        output_str = (
            f"Testing Determinism in {self.scenario_name} for "
            f"{fps:3d} render FPS and {fps_phys:3d} physics FPS -> "
        )

        prefix = f"{self.output_path}{self.scenario_name}_{fps}_{fps_phys}"

        config_settings = self.world.get_settings()
        config_settings.synchronous_mode = True
        config_settings.fixed_delta_seconds = 1.0 / fps
        config_settings.substepping = True
        config_settings.max_substep_delta_time = 1.0 / fps_phys
        config_settings.max_substeps = _MAX_SUBSTEPS

        spectator_tr = carla.Transform(carla.Location(120, -256, 10), carla.Rotation(yaw=180))

        t_comp = 0.0
        sim_prefixes = []
        for i in range(repetitions):
            prefix_rep = f"{prefix}_rep{i}"
            t_comp += self.scene.run_simulation(prefix_rep, config_settings, spectator_tr, tics=sim_tics)
            sim_prefixes.append(prefix_rep)

        determ_repet = self.check_simulations(sim_prefixes, prefix)
        output_str += f"Deterministic Repetitions: {determ_repet!r} / {repetitions:2d}"
        output_str += f"  -> Comp. Time per frame: {t_comp / repetitions * sim_tics:.0f}"

        return output_str


def main(arg: object) -> None:
    """Main function of the script."""
    client = carla.Client(arg.host, arg.port)
    client.set_timeout(30.0)
    world = client.get_world()
    pre_settings = world.get_settings()
    world = client.load_world("Town03")

    spectator_transform = carla.Transform(carla.Location(120, -256, 5), carla.Rotation(yaw=180))
    spectator_transform.location.z += 5
    spectator = world.get_spectator()
    spectator.set_transform(spectator_transform)

    try:
        output_path = os.path.dirname(os.path.realpath(__file__))
        output_path = os.path.join(output_path, "_collisions") + os.path.sep
        if not os.path.exists(output_path):
            os.mkdir(output_path)

        test_list = [
            CollisionScenarioTester(TwoSpawnedCars(client, world, save_snapshots_mode=True), output_path),
            CollisionScenarioTester(TwoCarsSlowSpeedCollision(client, world, save_snapshots_mode=True), output_path),
            CollisionScenarioTester(TwoCarsHighSpeedCollision(client, world, save_snapshots_mode=True), output_path),
            CollisionScenarioTester(CarBikeCollision(client, world, save_snapshots_mode=True), output_path),
            CollisionScenarioTester(CarWalkerCollision(client, world, save_snapshots_mode=True), output_path),
            CollisionScenarioTester(
                ThreeCarsSlowSpeedCollision(client, world, save_snapshots_mode=True), output_path,
            ),
            CollisionScenarioTester(
                ThreeCarsHighSpeedCollision(client, world, save_snapshots_mode=True), output_path,
            ),
        ]

        for item in test_list:
            item.test_scenario(20, 100, repetitions=_DEFAULT_REPETITIONS)

    finally:
        world.apply_settings(pre_settings)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host", metavar="H", default="localhost",
        help="IP of the host CARLA Simulator (default: localhost)",
    )
    argparser.add_argument(
        "-p", "--port", metavar="P", default=2000, type=int,
        help="TCP port of CARLA Simulator (default: 2000)",
    )
    argparser.add_argument(
        "--filter", metavar="PATTERN", default="model3",
        help='actor filter (default: "vehicle.*")',
    )
    args = argparser.parse_args()

    with contextlib.suppress(KeyboardInterrupt):
        main(args)
