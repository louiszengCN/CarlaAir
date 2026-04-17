# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import annotations

import time

import carla

from . import SyncSmokeTest

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_DEFAULT_TOLERANCE: float = 1e-5
_PHYSICS_TOLERANCE: float = 1e-3
_VELOCITY_TOLERANCE: float = 1e-3
_VELOCITY_TOLERANCE_COARSE: float = 1e-1
_WHEEL_TOLERANCE: float = 0.5
_STIFF_TOLERANCE: float = 1e-5
_WORLD_RELOAD_DELAY: float = 5.0
_WHEEL_COUNT: int = 4
_NUM_VEHICLES: int = 10
_KMH_TO_MS: float = 3.6
_VEL_100_KMH: float = 100.0 / 3.6
_VEL_50_KMH: float = 50.0 / 3.6
_BRAKE_FRAMES: int = 50

SpawnActor = carla.command.SpawnActor
FutureActor = carla.command.FutureActor
ApplyTargetVelocity = carla.command.ApplyTargetVelocity
SetEnableGravity = carla.command.SetEnableGravity
ApplyVehicleControl = carla.command.ApplyVehicleControl
ApplyVehiclePhysicsControl = carla.command.ApplyVehiclePhysicsControl


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────


_MIN_LIST_SIZE: int = 2


def list_equal_tol(objs: list[float], tol: float = _DEFAULT_TOLERANCE) -> bool:
    if len(objs) < _MIN_LIST_SIZE:
        return True
    return all(equal_tol(objs[0], objs[i], tol) for i in range(1, len(objs)))


def equal_tol(obj_a: object, obj_b: object, tol: float = _DEFAULT_TOLERANCE) -> bool:
    if isinstance(obj_a, list):
        return obj_a == obj_b
    if isinstance(obj_a, carla.libcarla.Vector3D):
        diff = abs(obj_a - obj_b)
        return diff.x < tol and diff.y < tol and diff.z < tol
    return abs(obj_a - obj_b) < tol


def equal_physics_control(
    pc_a: object,
    pc_b: object,
) -> tuple[bool, str]:
    error_msg = ""
    for key in dir(pc_a):
        if key.startswith("__") or key == "wheels":
            continue
        if not equal_tol(getattr(pc_a, key), getattr(pc_b, key), _PHYSICS_TOLERANCE):
            error_msg = (
                f"Car property: '{key}' does not match: "
                f"{getattr(pc_a, key):.4f} {getattr(pc_b, key):.4f}"
            )
            return False, error_msg

    if len(pc_a.wheels) != len(pc_b.wheels):
        error_msg = (
            f"Wheel count mismatch: {len(pc_a.wheels)} vs {len(pc_b.wheels)}"
        )
        return False, error_msg

    for w_idx in range(len(pc_a.wheels)):
        for key in dir(pc_a.wheels[w_idx]):
            if key.startswith("__") or key == "position":
                continue
            val_a = getattr(pc_a.wheels[w_idx], key)
            val_b = getattr(pc_b.wheels[w_idx], key)
            if not equal_tol(val_a, val_b, _PHYSICS_TOLERANCE):
                error_msg = (
                    f"Wheel property: '{key}' does not match: "
                    f"{val_a:.4f} {val_b:.4f}"
                )
                return False, error_msg

    return True, error_msg


def change_physics_control(
    vehicle: object,
    *,
    tire_friction: float | None = None,
    drag: float | None = None,
    wheel_sweep: bool | None = None,
    long_stiff: float | None = None,
) -> object:
    physics_control = vehicle.get_physics_control()

    if drag is not None:
        physics_control.drag_coefficient = drag
    if wheel_sweep is not None:
        physics_control.use_sweep_wheel_collision = wheel_sweep

    front_left_wheel = physics_control.wheels[0]
    front_right_wheel = physics_control.wheels[1]
    rear_left_wheel = physics_control.wheels[2]
    rear_right_wheel = physics_control.wheels[3]

    if tire_friction is not None:
        front_left_wheel.tire_friction = tire_friction
        front_right_wheel.tire_friction = tire_friction
        rear_left_wheel.tire_friction = tire_friction
        rear_right_wheel.tire_friction = tire_friction

    if long_stiff is not None:
        front_left_wheel.long_stiff_value = long_stiff
        front_right_wheel.long_stiff_value = long_stiff
        rear_left_wheel.long_stiff_value = long_stiff
        rear_right_wheel.long_stiff_value = long_stiff

    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
    physics_control.wheels = wheels

    return physics_control


# ──────────────────────────────────────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────────────────────────────────────


class TestApplyVehiclePhysics(SyncSmokeTest):
    def wait(self, frames: int = 100) -> None:
        for _i in range(frames):
            self.world.tick()

    def check_single_physics_control(self, bp_vehicle: object) -> None:
        veh_tranf = self.world.get_map().get_spawn_points()[0]
        vehicle = self.world.spawn_actor(bp_vehicle, veh_tranf)

        pc_a = change_physics_control(vehicle, drag=5)
        vehicle.apply_physics_control(pc_a)
        self.wait(2)
        pc_b = vehicle.get_physics_control()

        equal, msg = equal_physics_control(pc_a, pc_b)
        if not equal:
            self.fail(f"{bp_vehicle.id}: {msg}")

        self.wait(2)

        pc_a = change_physics_control(vehicle, tire_friction=5, long_stiff=987)
        vehicle.apply_physics_control(pc_a)
        self.wait(2)
        pc_b = vehicle.get_physics_control()

        equal, msg = equal_physics_control(pc_a, pc_b)
        if not equal:
            self.fail(f"{bp_vehicle.id}: {msg}")

        vehicle.destroy()

    def check_multiple_physics_control(
        self,
        bp_vehicles: object,
        index_bp: int | None = None,
    ) -> None:
        vehicles = []
        pc_a: list[object] = []
        pc_b: list[object] = []
        for i in range(_NUM_VEHICLES):
            veh_tranf = self.world.get_map().get_spawn_points()[i]
            bp_vehicle = bp_vehicles[index_bp] if index_bp is not None else bp_vehicles[i]
            vehicles.append(self.world.spawn_actor(bp_vehicle, veh_tranf))
            drag_coeff = 3.0 + 0.1 * i
            pc_a.append(change_physics_control(vehicles[i], drag=drag_coeff))
            vehicles[i].apply_physics_control(pc_a[i])

        self.wait(2)

        pc_b = [vehicles[i].get_physics_control() for i in range(_NUM_VEHICLES)]

        for i in range(_NUM_VEHICLES):
            equal, msg = equal_physics_control(pc_a[i], pc_b[i])
            if not equal:
                self.fail(f"{bp_vehicles[i].id}: {msg}")

        pc_a = []
        for i in range(_NUM_VEHICLES):
            friction = 1.0 + 0.1 * i
            lstiff = 500 + 100 * i
            pc_a.append(change_physics_control(vehicles[i], tire_friction=friction, long_stiff=lstiff))
            vehicles[i].apply_physics_control(pc_a[i])

        self.wait(2)

        pc_b = [vehicles[i].get_physics_control() for i in range(_NUM_VEHICLES)]

        for i in range(_NUM_VEHICLES):
            equal, msg = equal_physics_control(pc_a[i], pc_b[i])
            if not equal:
                self.fail(f"{bp_vehicles[i].id}: {msg}")

        for i in range(_NUM_VEHICLES):
            vehicles[i].destroy()

    def test_single_physics_control(self) -> None:
        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")
        bp_vehicles = self.filter_vehicles_for_old_towns(bp_vehicles)
        for bp_veh in bp_vehicles:
            self.check_single_physics_control(bp_veh)

    def test_multiple_physics_control(self) -> None:
        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")
        bp_vehicles = self.filter_vehicles_for_old_towns(bp_vehicles)
        for idx in range(len(bp_vehicles)):
            self.check_multiple_physics_control(bp_vehicles, idx)

        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")
        bp_vehicles = self.filter_vehicles_for_old_towns(bp_vehicles)
        self.check_multiple_physics_control(bp_vehicles)


class TestVehicleFriction(SyncSmokeTest):
    def wait(self, frames: int = 100) -> None:
        for _i in range(frames):
            self.world.tick()

    def test_vehicle_zero_friction(self) -> None:
        self.client.load_world("Town05_Opt", reset_settings=False)
        time.sleep(_WORLD_RELOAD_DELAY)

        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")
        bp_vehicles = self.filter_vehicles_for_old_towns(bp_vehicles)
        for bp_veh in bp_vehicles:
            veh_transf_00 = carla.Transform(
                carla.Location(33, -200, 0.2), carla.Rotation(yaw=90),
            )
            veh_transf_01 = carla.Transform(
                carla.Location(29, -200, 0.7), carla.Rotation(yaw=90),
            )

            batch = [
                SpawnActor(bp_veh, veh_transf_00)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0)))
                .then(SetEnableGravity(FutureActor, enabled=True)),
                SpawnActor(bp_veh, veh_transf_01)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0)))
                .then(SetEnableGravity(FutureActor, enabled=False)),
            ]

            responses = self.client.apply_batch_sync(batch)
            veh_ids = [x.actor_id for x in responses]
            veh_refs = [self.world.get_actor(x) for x in veh_ids]

            if (0 in veh_ids) or (None in veh_refs):
                self.fail(f"{bp_veh.id}: The test cars could not be correctly spawned")

            self.wait(10)

            self.client.apply_batch_sync([
                ApplyVehiclePhysicsControl(
                    veh_refs[0], change_physics_control(veh_refs[0], tire_friction=0.0, drag=0.0),
                ),
                ApplyVehiclePhysicsControl(
                    veh_refs[1], change_physics_control(veh_refs[1], drag=0.0),
                ),
            ])

            self.wait(1)

            self.client.apply_batch_sync([
                ApplyTargetVelocity(veh_refs[0], carla.Vector3D(0, _VEL_100_KMH, 0)),
                ApplyTargetVelocity(veh_refs[1], carla.Vector3D(0, _VEL_100_KMH, 0)),
            ])

            self.wait(1)

            vel_veh_00 = veh_refs[0].get_velocity().y
            vel_veh_01 = veh_refs[1].get_velocity().y

            if not list_equal_tol([_VEL_100_KMH, vel_veh_00, vel_veh_01], _VELOCITY_TOLERANCE):
                self.client.apply_batch_sync(
                    [carla.command.DestroyActor(x) for x in veh_ids],
                )
                self.fail(
                    f"{bp_veh.id}: Velocities not equal after init. "
                    f"Ref: {_VEL_100_KMH:.3f} -> [{vel_veh_00:.3f}, {vel_veh_01:.3f}]",
                )

            self.wait(100)

            vel_veh_00 = veh_refs[0].get_velocity().y
            vel_veh_01 = veh_refs[1].get_velocity().y

            if not list_equal_tol(
                [_VEL_100_KMH, vel_veh_00, vel_veh_01], _VELOCITY_TOLERANCE_COARSE,
            ):
                self.client.apply_batch_sync(
                    [carla.command.DestroyActor(x) for x in veh_ids],
                )
                self.fail(
                    f"{bp_veh.id}: Velocities not equal after sim. "
                    f"Ref: {_VEL_100_KMH:.3f} -> [{vel_veh_00:.3f}, {vel_veh_01:.3f}]",
                )

            self.client.apply_batch_sync(
                [carla.command.DestroyActor(x) for x in veh_ids],
            )

    def test_vehicle_friction_volume(self) -> None:
        self.client.load_world("Town05_Opt", reset_settings=False)
        time.sleep(_WORLD_RELOAD_DELAY)

        bp_vehicles = self.world.get_blueprint_library().filter("*charger_2020")

        value_vol_friction = 5.0
        friction_bp = self.world.get_blueprint_library().find("static.trigger.friction")
        friction_bp.set_attribute("friction", str(value_vol_friction))
        extent = carla.Location(300.0, 4500.0, 700.0)
        friction_bp.set_attribute("extent_x", str(extent.x))
        friction_bp.set_attribute("extent_y", str(extent.y))
        friction_bp.set_attribute("extent_z", str(extent.z))

        vol_transf = carla.Transform(carla.Location(27, -100, 1))

        self.world.debug.draw_box(
            box=carla.BoundingBox(vol_transf.location, extent * 1e-2),
            rotation=vol_transf.rotation,
            life_time=1000,
            thickness=0.5,
            color=carla.Color(r=0, g=255, b=0),
        )
        friction_trigger = self.world.spawn_actor(friction_bp, vol_transf)

        for bp_veh in bp_vehicles:
            veh_transf_00 = carla.Transform(
                carla.Location(36, -200, 0.2), carla.Rotation(yaw=90),
            )
            veh_transf_01 = carla.Transform(
                carla.Location(28, -200, 0.7), carla.Rotation(yaw=90),
            )

            batch = [
                SpawnActor(bp_veh, veh_transf_00)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
                SpawnActor(bp_veh, veh_transf_01)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
            ]

            responses = self.client.apply_batch_sync(batch)
            veh_ids = [x.actor_id for x in responses]
            veh_refs = [self.world.get_actor(x) for x in veh_ids]

            if (0 in veh_ids) or (None in veh_refs):
                self.fail(f"{bp_veh.id}: The test cars could not be correctly spawned")

            self.wait(10)

            friction_ref = 0.0

            self.client.apply_batch_sync([
                ApplyVehiclePhysicsControl(
                    veh_refs[0],
                    change_physics_control(veh_refs[0], tire_friction=friction_ref, drag=0.0),
                ),
                ApplyVehiclePhysicsControl(
                    veh_refs[1],
                    change_physics_control(veh_refs[1], tire_friction=friction_ref, drag=0.0),
                ),
            ])
            self.wait(1)

            self.client.apply_batch_sync([
                ApplyTargetVelocity(veh_refs[0], carla.Vector3D(0, _VEL_50_KMH, 0)),
                ApplyTargetVelocity(veh_refs[1], carla.Vector3D(0, _VEL_50_KMH, 0)),
            ])

            self.wait(4)

            bef_vel_veh_00 = veh_refs[0].get_velocity().y
            bef_vel_veh_01 = veh_refs[1].get_velocity().y

            extent_dbg = carla.Location(100.0, 100.0, 200.0)
            self.world.debug.draw_box(
                box=carla.BoundingBox(veh_refs[1].get_location(), extent_dbg * 1e-2),
                rotation=vol_transf.rotation, life_time=8, thickness=0.5,
                color=carla.Color(r=255, g=0, b=0),
            )

            if not equal_tol(bef_vel_veh_00, _VEL_50_KMH, _VELOCITY_TOLERANCE):
                self.fail(f"{bp_veh.id}: Ref vehicle changed before trigger")
            if not equal_tol(bef_vel_veh_01, _VEL_50_KMH, _VELOCITY_TOLERANCE):
                self.fail(f"{bp_veh.id}: Test vehicle changed before trigger")

            self.wait(100)

            ins_vel_veh_01 = veh_refs[1].get_velocity().y
            ins_tire_fr_01 = veh_refs[1].get_physics_control().wheels[0].tire_friction

            if ins_vel_veh_01 > _VEL_50_KMH:
                self.fail(f"{bp_veh.id}: Test vehicle too fast inside trigger")
            if not equal_tol(ins_tire_fr_01, value_vol_friction, _VELOCITY_TOLERANCE):
                self.fail(f"{bp_veh.id}: Test friction wrong inside trigger")

            self.wait(200)

            out_tire_fr_01 = veh_refs[1].get_physics_control().wheels[0].tire_friction
            if not equal_tol(out_tire_fr_01, friction_ref, _VELOCITY_TOLERANCE):
                self.fail(f"{bp_veh.id}: Test friction wrong after trigger")

            self.client.apply_batch_sync(
                [carla.command.DestroyActor(x) for x in veh_ids],
            )

        friction_trigger.destroy()

    def test_vehicle_friction_values(self) -> None:
        self.client.load_world("Town05_Opt", reset_settings=False)
        time.sleep(_WORLD_RELOAD_DELAY)

        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")

        for bp_veh in bp_vehicles:
            veh_transf_00 = carla.Transform(
                carla.Location(35, -200, 0.2), carla.Rotation(yaw=90),
            )
            veh_transf_01 = carla.Transform(
                carla.Location(29, -200, 0.7), carla.Rotation(yaw=90),
            )

            batch = [
                SpawnActor(bp_veh, veh_transf_00)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
                SpawnActor(bp_veh, veh_transf_01)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
            ]

            responses = self.client.apply_batch_sync(batch)
            veh_ids = [x.actor_id for x in responses]
            veh_refs = [self.world.get_actor(x) for x in veh_ids]

            if (0 in veh_ids) or (None in veh_refs):
                self.fail(f"{bp_veh.id}: The test cars could not be correctly spawned")

            self.wait(10)

            self.client.apply_batch_sync([
                ApplyVehiclePhysicsControl(
                    veh_refs[0],
                    change_physics_control(veh_refs[0], tire_friction=0.0, drag=0.0),
                ),
                ApplyVehiclePhysicsControl(
                    veh_refs[1],
                    change_physics_control(veh_refs[1], tire_friction=3.0, drag=0.0),
                ),
            ])

            self.wait(1)

            self.wait(1)
            self.client.apply_batch_sync([
                ApplyTargetVelocity(veh_refs[0], carla.Vector3D(0, _VEL_100_KMH, 0)),
                ApplyTargetVelocity(veh_refs[1], carla.Vector3D(0, _VEL_100_KMH, 0)),
            ])
            self.wait(20)

            loc_veh_00 = veh_refs[0].get_location().y
            loc_veh_01 = veh_refs[1].get_location().y

            for _i in range(_BRAKE_FRAMES):
                self.world.tick()
                self.client.apply_batch_sync([
                    ApplyVehicleControl(veh_refs[0], carla.VehicleControl(brake=1.0)),
                    ApplyVehicleControl(veh_refs[1], carla.VehicleControl(brake=1.0)),
                ])

            dist_veh_00 = veh_refs[0].get_location().y - loc_veh_00
            dist_veh_01 = veh_refs[1].get_location().y - loc_veh_01

            if dist_veh_01 > dist_veh_00:
                self.fail(
                    f"{bp_veh.id}: Friction test failed -> "
                    f"({dist_veh_00:f}, {dist_veh_01:f})",
                )

            self.client.apply_batch_sync(
                [carla.command.DestroyActor(x) for x in veh_ids],
            )


class TestVehicleTireConfig(SyncSmokeTest):
    def wait(self, frames: int = 100) -> None:
        for _i in range(frames):
            self.world.tick()

    def test_vehicle_wheel_collision(self) -> None:
        self.client.load_world("Town05_Opt", reset_settings=False)
        time.sleep(_WORLD_RELOAD_DELAY)

        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")
        bp_vehicles = [
            x for x in bp_vehicles
            if int(x.get_attribute("number_of_wheels")) == _WHEEL_COUNT
        ]

        for bp_veh in bp_vehicles:
            veh_transf_00 = carla.Transform(
                carla.Location(36, -200, 0.2), carla.Rotation(yaw=91),
            )
            veh_transf_01 = carla.Transform(
                carla.Location(31, -200, 0.7), carla.Rotation(yaw=91),
            )

            batch = [
                SpawnActor(bp_veh, veh_transf_00)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
                SpawnActor(bp_veh, veh_transf_01)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
            ]

            responses = self.client.apply_batch_sync(batch)
            veh_ids = [x.actor_id for x in responses]
            veh_refs = [self.world.get_actor(x) for x in veh_ids]

            if (0 in veh_ids) or (None in veh_refs):
                self.fail(f"{bp_veh.id}: The test cars could not be correctly spawned")

            self.wait(10)

            self.client.apply_batch_sync([
                ApplyVehiclePhysicsControl(
                    veh_refs[0], change_physics_control(veh_refs[0], wheel_sweep=False),
                ),
                ApplyVehiclePhysicsControl(
                    veh_refs[1], change_physics_control(veh_refs[1], wheel_sweep=True),
                ),
            ])
            self.wait(1)

            self.client.apply_batch_sync([
                ApplyTargetVelocity(veh_refs[0], carla.Vector3D(0, _VEL_100_KMH, 0)),
                ApplyTargetVelocity(veh_refs[1], carla.Vector3D(0, _VEL_100_KMH, 0)),
            ])
            self.wait(150)

            loc_veh_00 = veh_refs[0].get_location().y
            loc_veh_01 = veh_refs[1].get_location().y
            vel_veh_00 = veh_refs[0].get_velocity().y
            vel_veh_01 = veh_refs[1].get_velocity().y

            if not list_equal_tol([vel_veh_00, vel_veh_01], _WHEEL_TOLERANCE):
                self.client.apply_batch_sync(
                    [carla.command.DestroyActor(x) for x in veh_ids],
                )
                self.fail(
                    f"{bp_veh.id}: Velocities not equal. "
                    f"[{vel_veh_00:.3f}, {vel_veh_01:.3f}]",
                )

            if not list_equal_tol([loc_veh_00, loc_veh_01], _WHEEL_TOLERANCE):
                self.client.apply_batch_sync(
                    [carla.command.DestroyActor(x) for x in veh_ids],
                )
                self.fail(
                    f"{bp_veh.id}: Locations not equal. "
                    f"[{loc_veh_00:.3f}, {loc_veh_01:.3f}]",
                )

            self.client.apply_batch_sync(
                [carla.command.DestroyActor(x) for x in veh_ids],
            )

    def test_vehicle_tire_long_stiff(self) -> None:
        self.client.load_world("Town05_Opt", reset_settings=False)
        time.sleep(_WORLD_RELOAD_DELAY)

        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")
        bp_vehicles = [
            x for x in bp_vehicles
            if int(x.get_attribute("number_of_wheels")) == _WHEEL_COUNT
        ]

        for bp_veh in bp_vehicles:
            ref_pos = -200

            veh_transf_00 = carla.Transform(
                carla.Location(36, ref_pos, 0.2), carla.Rotation(yaw=90),
            )
            veh_transf_01 = carla.Transform(
                carla.Location(31, ref_pos, 0.2), carla.Rotation(yaw=90),
            )

            batch = [
                SpawnActor(bp_veh, veh_transf_00)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
                SpawnActor(bp_veh, veh_transf_01)
                .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
            ]

            responses = self.client.apply_batch_sync(batch)
            veh_ids = [x.actor_id for x in responses]
            veh_refs = [self.world.get_actor(x) for x in veh_ids]

            if (0 in veh_ids) or (None in veh_refs):
                self.fail(f"{bp_veh.id}: The test cars could not be correctly spawned")

            self.wait(10)

            self.client.apply_batch_sync([
                ApplyVehiclePhysicsControl(
                    veh_refs[0], change_physics_control(veh_refs[0], drag=0.0, long_stiff=100),
                ),
                ApplyVehiclePhysicsControl(
                    veh_refs[1], change_physics_control(veh_refs[1], drag=0.0, long_stiff=2000),
                ),
            ])

            self.wait(1)

            self.client.apply_batch_sync([
                ApplyVehicleControl(veh_refs[0], carla.VehicleControl(throttle=1.0)),
                ApplyVehicleControl(veh_refs[1], carla.VehicleControl(throttle=1.0)),
            ])

            self.wait(100)

            dist_veh_00 = veh_refs[0].get_location().y - ref_pos
            dist_veh_01 = veh_refs[1].get_location().y - ref_pos

            if dist_veh_01 < dist_veh_00:
                self.fail(
                    f"{bp_veh.id}: Long stiffness test failed. "
                    f"Veh00: [{dist_veh_00:f}] Veh01: [{dist_veh_01:f}]",
                )

            self.client.apply_batch_sync(
                [carla.command.DestroyActor(x) for x in veh_ids],
            )


class TestStickyControl(SyncSmokeTest):
    def wait(self, frames: int = 100) -> None:
        for _i in range(frames):
            self.world.tick()

    def run_scenario(
        self,
        bp_veh: object,
        veh_control: object,
        *,
        continous: bool = False,
        reset_after_first: bool = False,
        sticky: str | None = None,
    ) -> tuple[float, float]:
        ref_pos = -1
        veh_transf = carla.Transform(
            carla.Location(235, ref_pos, 0.2), carla.Rotation(yaw=90),
        )
        veh_transf.rotation.get_forward_vector()

        if sticky is not None:
            bp_veh.set_attribute("sticky_control", sticky)

        batch = [
            SpawnActor(bp_veh, veh_transf)
            .then(ApplyTargetVelocity(FutureActor, carla.Vector3D(0, 0, 0))),
        ]

        responses = self.client.apply_batch_sync(batch)

        if len(responses) != 1 or responses[0].error:
            self.fail(f"{bp_veh.id}: The test car could not be correctly spawned")

        vehicle_id = responses[0].actor_id
        vehicle_00 = self.world.get_actor(vehicle_id)

        for _i in range(10):
            self.world.tick()

        self.client.apply_batch_sync([ApplyVehicleControl(vehicle_00, veh_control)])
        self.world.tick()

        for _i in range(150):
            if continous:
                self.client.apply_batch_sync([ApplyVehicleControl(vehicle_00, veh_control)])
            if reset_after_first:
                self.client.apply_batch_sync(
                    [ApplyVehicleControl(vehicle_00, carla.VehicleControl())],
                )
            self.world.tick()

        loc_veh_00 = vehicle_00.get_location().y
        vel_veh_00 = vehicle_00.get_velocity().y
        dist_veh_00 = loc_veh_00 - ref_pos

        self.client.apply_batch([carla.command.DestroyActor(vehicle_id)])
        self.world.tick()

        return dist_veh_00, vel_veh_00

    def test_default(self) -> None:
        inp_control = carla.VehicleControl(throttle=1.0)
        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")

        bp_veh = bp_vehicles[0]
        d0, v0 = self.run_scenario(bp_veh, inp_control)
        d1, v1 = self.run_scenario(bp_veh, inp_control, continous=True)
        d2, v2 = self.run_scenario(bp_veh, inp_control, continous=True, sticky="False")

        if not equal_tol(d0, d1, _VELOCITY_TOLERANCE) or not equal_tol(v0, v1, _VELOCITY_TOLERANCE):
            self.fail(
                f"{bp_veh.id}: Default input not sticky: "
                f"[{d0:f}, {v0:f}] vs [{d1:f}, {v1:f}]",
            )

        if not equal_tol(d0, d2, _VELOCITY_TOLERANCE) or not equal_tol(v0, v2, _VELOCITY_TOLERANCE):
            self.fail(
                f"{bp_veh.id}: Default input not sticky: "
                f"[{d0:f}, {v0:f}] vs [{d2:f}, {v2:f}]",
            )

    def test_true(self) -> None:
        inp_control = carla.VehicleControl(throttle=1.0)
        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")

        bp_veh = bp_vehicles[0]
        d0, v0 = self.run_scenario(bp_veh, inp_control, sticky="True")
        d1, v1 = self.run_scenario(bp_veh, inp_control, continous=True)
        d2, v2 = self.run_scenario(bp_veh, inp_control, continous=True, sticky="False")

        if not equal_tol(d0, d1, _VELOCITY_TOLERANCE) or not equal_tol(v0, v1, _VELOCITY_TOLERANCE):
            self.fail(
                f"{bp_veh.id}: Input not sticky: "
                f"[{d0:f}, {v0:f}] vs [{d1:f}, {v1:f}]",
            )

        if not equal_tol(d0, d2, _VELOCITY_TOLERANCE) or not equal_tol(v0, v2, _VELOCITY_TOLERANCE):
            self.fail(
                f"{bp_veh.id}: Input not sticky: "
                f"[{d0:f}, {v0:f}] vs [{d2:f}, {v2:f}]",
            )

    def test_false(self) -> None:
        inp_control = carla.VehicleControl(throttle=1.0)
        bp_vehicles = self.world.get_blueprint_library().filter("vehicle.*")

        bp_veh = bp_vehicles[0]
        d0, v0 = self.run_scenario(bp_veh, inp_control, sticky="False")
        d1, v1 = self.run_scenario(bp_veh, inp_control, reset_after_first=True, sticky="True")
        d2, v2 = self.run_scenario(bp_veh, inp_control, reset_after_first=True, sticky="False")

        if not equal_tol(d0, d1, _STIFF_TOLERANCE) or not equal_tol(v0, v1, _STIFF_TOLERANCE):
            self.fail(
                f"{bp_veh.id}: Input is sticky: "
                f"[{d0:f}, {v0:f}] vs [{d1:f}, {v1:f}]",
            )

        if not equal_tol(d0, d2, _STIFF_TOLERANCE) or not equal_tol(v0, v2, _STIFF_TOLERANCE):
            self.fail(
                f"{bp_veh.id}: Input is sticky: "
                f"[{d0:f}, {v0:f}] vs [{d2:f}, {v2:f}]",
            )
