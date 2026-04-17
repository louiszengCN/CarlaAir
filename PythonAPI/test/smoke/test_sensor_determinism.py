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
from queue import Queue

import numpy as np

import carla

from . import SmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_SNAPSHOT_COLUMNS: int = 11
_MAX_SNAPSHOT_ERROR: float = 0.01
_WORLD_RELOAD_DELAY: float = 5.0
_DEFAULT_WAIT_FRAMES: int = 100
_DEFAULT_SIM_TICS: int = 200
_FIXED_DELTA: float = 0.05
_QUEUE_TIMEOUT: float = 15.0
_SENSOR_QUEUE_TIMEOUT: float = 15.0
_DEFAULT_REPETITIONS: int = 5
_DEFAULT_TEST_TICS: int = 100
_LIDAR_COLUMNS: int = 4


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
        self.sensor_list: list[tuple[str, object]] = []
        self.sensor_queue: Queue[tuple[int, str]] = Queue()

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
        self.sensor_list = []
        self.sensor_queue = Queue()

        self.reload_world(settings, spectator_tr)
        # workaround: give time to UE4 to clean memory after loading
        time.sleep(_WORLD_RELOAD_DELAY)

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
            if self.active:
                for _s in self.sensor_list:
                    self.sensor_queue.get(block=True, timeout=_SENSOR_QUEUE_TIMEOUT)

    def clear_scene(self) -> None:
        for sensor in self.sensor_list:
            sensor[1].destroy()
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
        if spectator_tr is not None:
            self.reset_spectator(spectator_tr)

        self.client.reload_world(reset_settings=False)
        # workaround: give time to UE4 to clean memory after loading
        time.sleep(_WORLD_RELOAD_DELAY)

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
            self.sensor_syncronization()
            self.save_snapshots()

        self.world.apply_settings(original_settings)
        self.save_snapshots_to_disk()
        self.clear_scene()

    def add_sensor(self, sensor: object, sensor_type: str) -> None:
        sen_idx = len(self.sensor_list)
        if sensor_type == "LiDAR":
            name = f"{sen_idx}_LiDAR"
            sensor.listen(lambda data: self.add_lidar_snapshot(data, name))
        elif sensor_type == "SemLiDAR":
            name = f"{sen_idx}_SemLiDAR"
            sensor.listen(lambda data: self.add_semlidar_snapshot(data, name))
        elif sensor_type == "Radar":
            name = f"{sen_idx}_Radar"
            sensor.listen(lambda data: self.add_radar_snapshot(data, name))
        else:
            return

        self.sensor_list.append((name, sensor))

    def add_lidar_snapshot(self, lidar_data: object, name: str = "LiDAR") -> None:
        if not self.active:
            return
        points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / _LIDAR_COLUMNS), _LIDAR_COLUMNS))
        frame = lidar_data.frame - self.init_timestamp["frame0"]
        np.savetxt(self.get_filename(name, frame), points)
        self.sensor_queue.put((lidar_data.frame, name))

    def add_semlidar_snapshot(self, lidar_data: object, name: str = "SemLiDAR") -> None:
        if not self.active:
            return
        data = np.frombuffer(lidar_data.raw_data, dtype=np.dtype([
            ("x", np.float32), ("y", np.float32), ("z", np.float32),
            ("CosAngle", np.float32), ("ObjIdx", np.uint32), ("ObjTag", np.uint32),
        ]))
        points = np.array([data["x"], data["y"], data["z"], data["CosAngle"], data["ObjTag"]]).T
        frame = lidar_data.frame - self.init_timestamp["frame0"]
        np.savetxt(self.get_filename(name, frame), points)
        self.sensor_queue.put((lidar_data.frame, name))

    def add_radar_snapshot(self, radar_data: object, name: str = "Radar") -> None:
        if not self.active:
            return
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / _LIDAR_COLUMNS), _LIDAR_COLUMNS))
        frame = radar_data.frame - self.init_timestamp["frame0"]
        np.savetxt(self.get_filename(name, frame), points)
        self.sensor_queue.put((radar_data.frame, name))

    def sensor_syncronization(self) -> None:
        w_frame = self.world.get_snapshot().frame
        for sensor in self.sensor_list:
            s_frame = self.sensor_queue.get(block=True, timeout=_SENSOR_QUEUE_TIMEOUT)[0]
            while s_frame < w_frame:
                s_frame = self.sensor_queue.get(block=True, timeout=_SENSOR_QUEUE_TIMEOUT)[0]
            if w_frame != s_frame:
                msg = (
                    f"FrameSyncError: Frames are not equal for sensor "
                    f"{sensor[0]}: {w_frame} {s_frame}"
                )
                raise DeterminismError(msg)


class SpawnAllRaycastSensors(Scenario):
    def init_scene(
        self,
        prefix: str,
        settings: object = None,
        spectator_tr: object = None,
    ) -> None:
        super().init_scene(prefix, settings, spectator_tr)

        blueprint_library = self.world.get_blueprint_library()

        vehicle00_tr = carla.Transform(
            carla.Location(140, -205, 0.1), carla.Rotation(yaw=181.5),
        )
        vehicle00 = self.world.spawn_actor(blueprint_library.filter("tt")[0], vehicle00_tr)
        vehicle00.set_target_velocity(carla.Vector3D(-25, 0, 0))

        vehicle01_tr = carla.Transform(
            carla.Location(50, -200, 0.1), carla.Rotation(yaw=1.5),
        )
        vehicle01 = self.world.spawn_actor(blueprint_library.filter("lincoln")[0], vehicle01_tr)
        vehicle01.set_target_velocity(carla.Vector3D(25, 0, 0))

        radar_bp = self.world.get_blueprint_library().find("sensor.other.radar")
        radar_bp.set_attribute("noise_seed", "54283")
        radar_tr = carla.Transform(carla.Location(z=2))
        radar = self.world.spawn_actor(radar_bp, radar_tr, attach_to=vehicle00)

        lidar01_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
        lidar01_bp.set_attribute("noise_seed", "12134")
        lidar01_tr = carla.Transform(carla.Location(x=1, z=2))
        lidar01 = self.world.spawn_actor(lidar01_bp, lidar01_tr, attach_to=vehicle00)

        lidar02_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
        lidar02_tr = carla.Transform(carla.Location(x=1, z=2))
        lidar02 = self.world.spawn_actor(lidar02_bp, lidar02_tr, attach_to=vehicle01)

        lidar03_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
        lidar03_bp.set_attribute("noise_seed", "23135")
        lidar03_tr = carla.Transform(carla.Location(z=2))
        lidar03 = self.world.spawn_actor(lidar03_bp, lidar03_tr, attach_to=vehicle01)

        self.add_sensor(radar, "Radar")
        self.add_sensor(lidar01, "LiDAR")
        self.add_sensor(lidar02, "SemLiDAR")
        self.add_sensor(lidar03, "LiDAR")
        self.add_actor(vehicle00, "Car")
        self.add_actor(vehicle01, "Car")

        self.wait(1)


class SensorScenarioTester:
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
        if data_i.shape != data_j.shape:
            return False

        max_error = np.amax(np.abs(data_i - data_j))
        return max_error < _MAX_SNAPSHOT_ERROR

    def check_simulations(
        self,
        rep_prefixes: list[str],
        sim_tics: int,
    ) -> list[int]:
        repetitions = len(rep_prefixes)
        mat_check = np.zeros((repetitions, repetitions), int)

        for i in range(repetitions):
            mat_check[i][i] = 1
            for j in range(i):
                sim_check = True
                for f_idx in range(1, sim_tics):
                    for sensor in self.scene.sensor_list:
                        file_i = self.scene.get_filename_with_prefix(
                            rep_prefixes[i], sensor[0], f_idx,
                        )
                        file_j = self.scene.get_filename_with_prefix(
                            rep_prefixes[j], sensor[0], f_idx,
                        )
                        check_ij = self.compare_files(file_i, file_j)
                        sim_check = sim_check and check_ij
                mat_check[i][j] = int(sim_check)
                mat_check[j][i] = int(sim_check)

        determinism = np.sum(mat_check, axis=1)
        return sorted(set(determinism), reverse=True)

    def test_scenario(
        self,
        *,
        repetitions: int = 1,
        sim_tics: int = 100,
    ) -> None:
        prefix = self.output_path + self.scenario_name

        config_settings = self.world.get_settings()
        config_settings.synchronous_mode = True
        config_settings.fixed_delta_seconds = _FIXED_DELTA

        spectator_tr = carla.Transform(
            carla.Location(160, -205, 10), carla.Rotation(yaw=180),
        )

        sim_prefixes = []
        for i in range(repetitions):
            prefix_rep = f"{prefix}_rep_{i:03d}"
            self.scene.run_simulation(
                prefix_rep, config_settings, spectator_tr, tics=sim_tics,
            )
            sim_prefixes.append(prefix_rep)

        determ_repet = self.check_simulations(sim_prefixes, sim_tics)

        if determ_repet[0] != repetitions:
            msg = (
                f"SensorOutputError: Scenario {self.scenario_name} "
                f"is not deterministic: {determ_repet[0]} / {repetitions}"
            )
            raise DeterminismError(msg)


class TestSensorDeterminism(SmokeTest):
    def test_all_sensors(self) -> None:
        orig_settings = self.world.get_settings()

        output_path = os.path.dirname(os.path.realpath(__file__))
        output_path = os.path.join(output_path, "_sensors") + os.path.sep
        if not os.path.exists(output_path):
            os.mkdir(output_path)

        try:
            test_sensors = SensorScenarioTester(
                SpawnAllRaycastSensors(self.client, self.world), output_path,
            )
            test_sensors.test_scenario(
                repetitions=_DEFAULT_REPETITIONS, sim_tics=_DEFAULT_TEST_TICS,
            )
        except DeterminismError as err:
            test_sensors.scene.clear_scene()
            shutil.rmtree(output_path)
            self.fail(err)

        self.world.apply_settings(orig_settings)
        shutil.rmtree(output_path)
