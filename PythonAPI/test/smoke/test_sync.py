# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Test synchronous mode functionality."""

from __future__ import annotations

import contextlib
import time
from queue import Empty, Queue

import carla

from . import SyncSmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

# World settings
_SYNC_DELTA: float = 0.05
_RELOAD_DELAY: float = 5.0
_RELOAD_COUNT: int = 4

# Camera test
_CAMERA_Z: float = 10.0
_CAMERA_TICK_COUNT: int = 100

# Sensor test
_SENSOR_CAR_BLUEPRINT: str = "vehicle.ford.mustang"
_SENSOR_X_OFFSET: float = 1.6
_SENSOR_Z_OFFSET: float = 1.7
_SENSOR_TICK_COUNT: int = 100
_SENSOR_QUEUE_TIMEOUT: float = 1.0
_SENSOR_SKIP_FRAMES: int = 1
_VEHICLE_THROTTLE: float = 0.75

# Sensor IDs (non-event sensors)
_SENSOR_IDS: list[str] = [
    "sensor.lidar.ray_cast",
    "sensor.lidar.ray_cast_semantic",
    "sensor.other.gnss",
    "sensor.other.radar",
    "sensor.other.imu",
]


# ──────────────────────────────────────────────────────────────────────────────
# Test Class
# ──────────────────────────────────────────────────────────────────────────────


class TestSynchronousMode(SyncSmokeTest):
    """Test CARLA synchronous mode behavior."""

    def test_reloading_map(self) -> None:
        """Verify map reload works correctly in sync mode."""
        settings = carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=_SYNC_DELTA,
        )
        for _ in range(_RELOAD_COUNT):
            self.world = self.client.reload_world()
            self.world.apply_settings(settings)
            # Workaround: give time to UE4 to clean memory after loading
            time.sleep(_RELOAD_DELAY)

    def _test_camera_on_synchronous_mode(self) -> None:
        """Verify camera frames sync correctly with world ticks."""

        cam_bp = self.world.get_blueprint_library().find(
            "sensor.camera.rgb",
        )
        t = carla.Transform(carla.Location(z=_CAMERA_Z))
        camera = self.world.spawn_actor(cam_bp, t)

        try:
            image_queue: Queue[object] = Queue()
            camera.listen(image_queue.put)

            frame: int | None = None

            for _ in range(_CAMERA_TICK_COUNT):
                self.world.tick()
                ts = self.world.get_snapshot().timestamp

                if frame is not None:
                    assert ts.frame == frame + 1

                frame = ts.frame

                image = image_queue.get()
                assert image.frame == ts.frame
                assert image.timestamp == ts.elapsed_seconds

        finally:
            camera.destroy()

    def test_sensor_transform_on_synchronous_mode(self) -> None:
        """Verify sensor transforms match snapshot in sync mode."""
        bp_lib = self.world.get_blueprint_library()

        spawn_points = self.world.get_map().get_spawn_points()
        assert len(spawn_points) != 0

        car_bp = bp_lib.find(_SENSOR_CAR_BLUEPRINT)
        car = self.world.spawn_actor(car_bp, spawn_points[0])

        sensor_bps = [bp_lib.find(n) for n in _SENSOR_IDS]
        trans = carla.Transform(
            carla.Location(x=_SENSOR_X_OFFSET, z=_SENSOR_Z_OFFSET),
        )
        sensors = [
            self.world.spawn_actor(sensor, trans, car)
            for sensor in sensor_bps
        ]
        queues: list[Queue[tuple[object, str]]] = [Queue() for _ in sensor_bps]
        car.apply_control(carla.VehicleControl(throttle=_VEHICLE_THROTTLE))

        def _sensor_callback(
            data: object, name: str, queue: Queue[tuple[object, str]],
        ) -> None:
            queue.put((data, name))

        try:
            for i in range(len(sensors)):
                sensors[i].listen(
                    lambda data, i=i: _sensor_callback(
                        data, _SENSOR_IDS[i], queues[i],
                    ),
                )

            local_frame = 0
            for _ in range(_SENSOR_TICK_COUNT):
                self.world.tick()
                snapshot_frame = self.world.get_snapshot().frame
                sensors_data: list[tuple[object, str]] = []

                with contextlib.suppress(Empty):
                    # Get the data once it's received
                    sensors_data.extend(
                        queue.get(block=True, timeout=_SENSOR_QUEUE_TIMEOUT)
                        for queue in queues
                    )

                for i in range(len(queues)):
                    assert queues[i].qsize() == 0, f"\nQueue {_SENSOR_IDS[i]} oversized"

                # Just in case some sensors do not have the correct
                # transform the same frame they are spawned, like IMU.
                if local_frame < _SENSOR_SKIP_FRAMES:
                    continue

                # All the data has been correctly retrieved
                assert len(sensors_data) == len(sensor_bps)

                # All the sensor frame numbers are the same
                for sensor_data in sensors_data:
                    assert sensor_data[0].frame == snapshot_frame

                # All the sensor transforms match in the snapshot
                # and the callback
                for i in range(len(sensors_data)):
                    assert sensors_data[i][0].transform == sensors[i].get_transform(), (
                        f"Sensor '{sensors_data[i][1]}' transform mismatch at frame {local_frame}"
                    )
                local_frame += 1

        finally:
            for sensor in sensors:
                if sensor is not None:
                    sensor.stop()
                    sensor.destroy()
            if car is not None:
                car.destroy()

    def _batch_scenario(
        self, *, batch_tick: bool, after_tick: bool,
    ) -> tuple[int, int]:
        """Run a batch spawn scenario and return frame numbers.

        Args:
            batch_tick: whether to tick during batch
            after_tick: whether to tick after batch

        Returns:
            tuple of (initial frame, final frame)
        """
        bp_veh = self.world.get_blueprint_library().filter(
            "vehicle.*",
        )[0]
        veh_transf = self.world.get_map().get_spawn_points()[0]

        frame_init = self.world.get_snapshot().frame

        batch = [carla.command.SpawnActor(bp_veh, veh_transf)]

        responses = self.client.apply_batch_sync(batch, batch_tick)
        if after_tick:
            self.world.tick()

        if len(responses) != 1 or responses[0].error:
            self.fail(
                f"{bp_veh.id}: The test car could not be "
                f"correctly spawned",
            )

        vehicle_id = responses[0].actor_id

        frame_after = self.world.get_snapshot().frame

        self.client.apply_batch_sync(
            [carla.command.DestroyActor(vehicle_id)],
        )

        return frame_init, frame_after

    def test_apply_batch_sync(self) -> None:
        """Verify apply_batch_sync frame behavior."""

        a_t0, a_t1 = self._batch_scenario(batch_tick=False, after_tick=False)
        assert a_t0 == a_t1, (
            f"Frames should be equal: {a_t0} {a_t1}"
        )

        a_t0, a_t1 = self._batch_scenario(batch_tick=True, after_tick=False)
        assert a_t0 + 1 == a_t1, (
            f"Frames should be consecutive: {a_t0} {a_t1}"
        )

        a_t0, a_t1 = self._batch_scenario(batch_tick=False, after_tick=True)
        assert a_t0 + 1 == a_t1, (
            f"Frames should be consecutive: {a_t0} {a_t1}"
        )
