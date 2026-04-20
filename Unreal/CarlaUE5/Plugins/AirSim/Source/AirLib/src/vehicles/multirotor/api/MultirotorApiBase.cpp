// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include <functional>
#include <exception>
#include <vector>
#include <iostream>
#include <fstream>

namespace msr
{
namespace airlib
{

    void MultirotorApiBase::resetImplementation()
    {
        cancelLastTask();
        SingleTaskCall lock(this);
    }

    bool MultirotorApiBase::takeoff(float timeout_sec)
    {
        SingleTaskCall lock(this);

        auto kinematics = getKinematicsEstimated();
        if (kinematics.twist.linear.norm() > approx_zero_vel_) {
            throw VehicleMoveException(Utils::stringf(
                "Cannot perform takeoff because vehicle is already moving with velocity %f m/s",
                kinematics.twist.linear.norm()));
        }

        return moveToPosition(kinematics.pose.position.x(),
                              kinematics.pose.position.y(),
                              kinematics.pose.position.z() + getTakeoffZ(),
                              0.5f,
                              timeout_sec,
                              DrivetrainType::MaxDegreeOfFreedom,
                              YawMode::Zero(),
                              -1,
                              1);
    }

    bool MultirotorApiBase::land(float timeout_sec)
    {
        SingleTaskCall lock(this);

        int near_zero_vel_count = 0;

        return waitForFunction([&]() {
                   moveByVelocityInternal(0, 0, landing_vel_, YawMode::Zero());

                   float z_vel = getVelocity().z();
                   if (z_vel <= approx_zero_vel_)
                       ++near_zero_vel_count;
                   else
                       near_zero_vel_count = 0;

                   if (near_zero_vel_count > 10)
                       return true;

                   moveByVelocityInternal(0, 0, landing_vel_, YawMode::Zero());
                   return false;
               },
                               timeout_sec)
            .isComplete();
    }

    bool MultirotorApiBase::goHome(float timeout_sec)
    {
        SingleTaskCall lock(this);
        return moveToPosition(0, 0, 0, 0.5f, timeout_sec, DrivetrainType::MaxDegreeOfFreedom, YawMode::Zero(), -1, 1);
    }

    bool MultirotorApiBase::moveByVelocityBodyFrame(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        SingleTaskCall lock(this);

        if (duration <= 0)
            return true;

        float pitch, roll, yaw;
        VectorMath::toEulerianAngle(getKinematicsEstimated().pose.orientation, pitch, roll, yaw);
        float vx_new = (vx * (float)std::cos(yaw)) - (vy * (float)std::sin(yaw));
        float vy_new = (vx * (float)std::sin(yaw)) + (vy * (float)std::cos(yaw));

        YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
        adjustYaw(vx_new, vy_new, drivetrain, adj_yaw_mode);

        return waitForFunction([&]() {
                   moveByVelocityInternal(vx_new, vy_new, vz, adj_yaw_mode);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByVelocityZBodyFrame(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        SingleTaskCall lock(this);

        if (duration <= 0)
            return true;

        float pitch, roll, yaw;
        VectorMath::toEulerianAngle(getKinematicsEstimated().pose.orientation, pitch, roll, yaw);
        float vx_new = (vx * (float)std::cos(yaw)) - (vy * (float)std::sin(yaw));
        float vy_new = (vx * (float)std::sin(yaw)) + (vy * (float)std::cos(yaw));

        YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
        adjustYaw(vx_new, vy_new, drivetrain, adj_yaw_mode);

        return waitForFunction([&]() {
                   moveByVelocityZInternal(vx_new, vy_new, z, adj_yaw_mode);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByMotorPWMs(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        return waitForFunction([&]() {
                   commandMotorPWMs(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByRollPitchYawZ(float roll, float pitch, float yaw, float z, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        return waitForFunction([&]() {
                   moveByRollPitchYawZInternal(roll, pitch, yaw, z);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        return waitForFunction([&]() {
                   moveByRollPitchYawThrottleInternal(roll, pitch, yaw, throttle);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        return waitForFunction([&]() {
                   moveByRollPitchYawrateThrottleInternal(roll, pitch, yaw_rate, throttle);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        return waitForFunction([&]() {
                   moveByRollPitchYawrateZInternal(roll, pitch, yaw_rate, z);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        return waitForFunction([&]() {
                   moveByAngleRatesZInternal(roll_rate, pitch_rate, yaw_rate, z);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        return waitForFunction([&]() {
                   moveByAngleRatesThrottleInternal(roll_rate, pitch_rate, yaw_rate, throttle);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
        adjustYaw(vx, vy, drivetrain, adj_yaw_mode);
        return waitForFunction([&]() {
                   moveByVelocityInternal(vx, vy, vz, adj_yaw_mode);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return false;
        YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
        adjustYaw(vx, vy, drivetrain, adj_yaw_mode);
        return waitForFunction([&]() {
                   moveByVelocityZInternal(vx, vy, z, adj_yaw_mode);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    bool MultirotorApiBase::moveOnPath(const vector<Vector3r>& path, float velocity, float timeout_sec, DrivetrainType drivetrain, const YawMode& yaw_mode,
                                       float lookahead, float adaptive_lookahead)
    {
        SingleTaskCall lock(this);

        if (path.size() == 0) {
            Utils::log("moveOnPath terminated because path has no points", Utils::kLogLevelWarn);
            return true;
        }

        if (drivetrain == DrivetrainType::ForwardOnly && yaw_mode.is_rate)
            throw std::invalid_argument("Yaw cannot be specified as rate if drivetrain is ForwardOnly");

        float command_period_dist = velocity * getCommandPeriod();
        if (lookahead == 0)
            throw std::invalid_argument("lookahead distance cannot be 0");
        else if (lookahead > 0) {
            if (command_period_dist > lookahead)
                throw std::invalid_argument(Utils::stringf("lookahead value %f is too small for velocity %f. It must be at least %f", lookahead, velocity, command_period_dist));
            if (getDistanceAccuracy() > lookahead)
                throw std::invalid_argument(Utils::stringf("lookahead value %f is smaller than drone's distance accuracy %f.", lookahead, getDistanceAccuracy()));
        }
        else {
            lookahead = getAutoLookahead(velocity, adaptive_lookahead);
            Utils::log(Utils::stringf("lookahead = %f, adaptive_lookahead = %f", lookahead, adaptive_lookahead));
        }

        vector<Vector3r> path3d;
        vector<PathSegment> path_segs;
        path3d.push_back(getKinematicsEstimated().pose.position);

        Vector3r point;
        float path_length = 0;
        for (uint i = 0; i < path.size(); ++i) {
            point = path.at(i);
            PathSegment path_seg(path3d.at(i), point, velocity, path_length);
            path_length += path_seg.seg_length;
            path_segs.push_back(path_seg);
            path3d.push_back(point);
        }
        path_segs.push_back(PathSegment(point, point, velocity, path_length));

        float breaking_dist = 0;
        if (velocity > getMultirotorApiParams().breaking_vel) {
            breaking_dist = Utils::clip(velocity * getMultirotorApiParams().vel_to_breaking_dist,
                                        getMultirotorApiParams().min_breaking_dist,
                                        getMultirotorApiParams().max_breaking_dist);
        }

        PathPosition cur_path_loc, next_path_loc;
        cur_path_loc.seg_index = 0;
        cur_path_loc.offset = 0;
        cur_path_loc.position = path3d[0];

        float lookahead_error_increasing = 0;
        float lookahead_error = 0;
        Waiter waiter(getCommandPeriod(), timeout_sec, getCancelToken());

        setNextPathPosition(path3d, path_segs, cur_path_loc, lookahead + lookahead_error, next_path_loc);
        float overshoot = 0;
        float goal_dist = 0;

        while (!waiter.isTimeout() && (next_path_loc.seg_index < path_segs.size() - 1 || goal_dist > 0)) {
            float seg_velocity = path_segs.at(next_path_loc.seg_index).seg_velocity;
            float path_length_remaining = path_length - path_segs.at(cur_path_loc.seg_index).seg_path_length - cur_path_loc.offset;
            if (seg_velocity > getMultirotorApiParams().min_vel_for_breaking && path_length_remaining <= breaking_dist) {
                seg_velocity = getMultirotorApiParams().breaking_vel;
            }

            moveToPathPosition(next_path_loc.position, seg_velocity, drivetrain, yaw_mode, path_segs.at(cur_path_loc.seg_index).start_z);

            if (!waiter.sleep())
                return false;

            const Vector3r& goal_vect = next_path_loc.position - cur_path_loc.position;
            if (!goal_vect.isZero()) {
                const Vector3r& actual_vect = getPosition() - cur_path_loc.position;
                const Vector3r& goal_normalized = goal_vect.normalized();
                goal_dist = actual_vect.dot(goal_normalized);

                if (adaptive_lookahead) {
                    const Vector3r& actual_on_goal = goal_normalized * goal_dist;
                    float error = (actual_vect - actual_on_goal).norm() * adaptive_lookahead;
                    if (error > lookahead_error) {
                        lookahead_error_increasing++;
                        if (lookahead_error_increasing > 1E5) {
                            throw std::runtime_error("lookahead error is continually increasing so we do not have safe control, aborting moveOnPath operation");
                        }
                    }
                    else {
                        lookahead_error_increasing = 0;
                    }
                    lookahead_error = error;
                }
            }
            else {
                lookahead_error_increasing = 0;
                goal_dist = 0;
                lookahead_error = 0;
                waiter.complete();
            }

            if (goal_dist >= 0) {
                overshoot = setNextPathPosition(path3d, path_segs, cur_path_loc, goal_dist, cur_path_loc);
                if (overshoot)
                    Utils::log(Utils::stringf("overshoot=%f", overshoot));
            }

            overshoot = setNextPathPosition(path3d, path_segs, cur_path_loc, lookahead + lookahead_error, next_path_loc);
        }

        return waiter.isComplete();
    }

    bool MultirotorApiBase::moveToGPS(float latitude, float longitude, float altitude, float velocity, float timeout_sec, DrivetrainType drivetrain,
                                      const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        SingleTaskCall lock(this);
        GeoPoint target;
        target.latitude = latitude;
        target.longitude = longitude;
        target.altitude = altitude;
        if (!std::isnan(getHomeGeoPoint().latitude) && !std::isnan(getHomeGeoPoint().longitude) && !std::isnan(getHomeGeoPoint().altitude)) {
            vector<Vector3r> path{ msr::airlib::EarthUtils::GeodeticToNed(target, getHomeGeoPoint()) };
            return moveOnPath(path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead);
        }
        vector<Vector3r> path{ Vector3r(getPosition().x(), getPosition().y(), getPosition().z()) };
        return moveOnPath(path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead);
    }

    bool MultirotorApiBase::moveToPosition(float x, float y, float z, float velocity, float timeout_sec, DrivetrainType drivetrain,
                                           const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        SingleTaskCall lock(this);
        vector<Vector3r> path{ Vector3r(x, y, z) };
        return moveOnPath(path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead);
    }

    bool MultirotorApiBase::moveToZ(float z, float velocity, float timeout_sec, const YawMode& yaw_mode,
                                    float lookahead, float adaptive_lookahead)
    {
        SingleTaskCall lock(this);
        Vector2r cur_xy(getPosition().x(), getPosition().y());
        vector<Vector3r> path{ Vector3r(cur_xy.x(), cur_xy.y(), z) };
        return moveOnPath(path, velocity, timeout_sec, DrivetrainType::MaxDegreeOfFreedom, yaw_mode, lookahead, adaptive_lookahead);
    }

    bool MultirotorApiBase::moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        SingleTaskCall lock(this);
        const float kMaxMessageAge = 0.1f, kMaxRCValue = 10000;
        if (duration <= 0)
            return true;

        Quaternionr starting_quaternion = getKinematicsEstimated().pose.orientation;
        Waiter waiter(getCommandPeriod(), duration, getCancelToken());
        do {
            RCData rc_data = getRCData();
            TTimeDelta age = clock()->elapsedSince(rc_data.timestamp);
            if (rc_data.is_valid && (rc_data.timestamp == 0 || age <= kMaxMessageAge)) {
                if (rc_data_trims_.is_valid)
                    rc_data.subtract(rc_data_trims_);

                const Vector3r vel_word(rc_data.pitch * vy_max / kMaxRCValue, rc_data.roll * vx_max / kMaxRCValue, 0);
                Vector3r vel_body = VectorMath::transformToBodyFrame(vel_word, starting_quaternion, true);

                YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
                adj_yaw_mode.yaw_or_rate += rc_data.yaw * 100.0f / kMaxRCValue;
                adjustYaw(vel_body, drivetrain, adj_yaw_mode);

                try {
                    float vz = (rc_data.throttle / kMaxRCValue) * z_min + getPosition().z();
                    moveByVelocityZInternal(vel_body.x(), vel_body.y(), vz, adj_yaw_mode);
                }
                catch (const MultirotorApiBase::UnsafeMoveException& ex) {
                    Utils::log(Utils::stringf("Safety violation: %s", ex.result.message.c_str()), Utils::kLogLevelWarn);
                }
            }
            else {
                Utils::log(Utils::stringf("RCData had too old timestamp: %f", age));
            }
        } while (waiter.sleep());

        return waiter.isTimeout();
    }

    bool MultirotorApiBase::rotateToYaw(float yaw, float timeout_sec, float margin)
    {
        SingleTaskCall lock(this);
        if (timeout_sec <= 0)
            return true;

        auto start_pos = getPosition();
        float yaw_target = VectorMath::normalizeAngle(yaw);
        YawMode move_yaw_mode(false, yaw_target);
        YawMode stop_yaw_mode(true, 0);

        return waitForFunction([&]() {
                   if (isYawWithinMargin(yaw_target, margin)) {
                       moveToPositionInternal(start_pos, stop_yaw_mode);
                       auto yaw_rate = getKinematicsEstimated().twist.angular.z();
                       if (abs(yaw_rate) <= approx_zero_angular_vel_)
                           return true;
                   }
                   else {
                       moveToPositionInternal(start_pos, move_yaw_mode);
                   }
                   return false;
               },
                               timeout_sec)
            .isComplete();
    }

    bool MultirotorApiBase::rotateByYawRate(float yaw_rate, float duration)
    {
        SingleTaskCall lock(this);
        if (duration <= 0)
            return true;
        auto start_pos = getPosition();
        YawMode yaw_mode(true, yaw_rate);
        return waitForFunction([&]() {
                   moveToPositionInternal(start_pos, yaw_mode);
                   return false;
               },
                               duration)
            .isTimeout();
    }

    void MultirotorApiBase::setAngleLevelControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd)
    {
        uint8_t controller_type = 2;
        setControllerGains(controller_type, kp, ki, kd);
    }

    void MultirotorApiBase::setAngleRateControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd)
    {
        uint8_t controller_type = 3;
        setControllerGains(controller_type, kp, ki, kd);
    }

    void MultirotorApiBase::setVelocityControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd)
    {
        uint8_t controller_type = 4;
        setControllerGains(controller_type, kp, ki, kd);
    }

    void MultirotorApiBase::setPositionControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd)
    {
        uint8_t controller_type = 5;
        setControllerGains(controller_type, kp, ki, kd);
    }

    bool MultirotorApiBase::hover()
    {
        SingleTaskCall lock(this);
        return moveToZ(getPosition().z(), 0.5f, Utils::max<float>(), YawMode{ true, 0 }, 1.0f, false);
    }

    void MultirotorApiBase::moveByRC(const RCData& rc_data)
    {
        unused(rc_data);
        throw VehicleCommandNotImplementedException("moveByRC API is not implemented for this multirotor");
    }

    void MultirotorApiBase::moveByVelocityInternal(float vx, float vy, float vz, const YawMode& yaw_mode)
    {
        if (safetyCheckVelocity(Vector3r(vx, vy, vz)))
            commandVelocity(vx, vy, vz, yaw_mode);
    }

    void MultirotorApiBase::moveByVelocityZInternal(float vx, float vy, float z, const YawMode& yaw_mode)
    {
        if (safetyCheckVelocityZ(vx, vy, z))
            commandVelocityZ(vx, vy, z, yaw_mode);
    }

    void MultirotorApiBase::moveToPositionInternal(const Vector3r& dest, const YawMode& yaw_mode)
    {
        if (safetyCheckDestination(dest))
            commandPosition(dest.x(), dest.y(), dest.z(), yaw_mode);
    }

    void MultirotorApiBase::moveByRollPitchYawZInternal(float roll, float pitch, float yaw, float z)
    {
        if (safetyCheckVelocity(getVelocity()))
            commandRollPitchYawZ(roll, pitch, yaw, z);
    }

    void MultirotorApiBase::moveByRollPitchYawThrottleInternal(float roll, float pitch, float yaw, float throttle)
    {
        if (safetyCheckVelocity(getVelocity()))
            commandRollPitchYawThrottle(roll, pitch, yaw, throttle);
    }

    void MultirotorApiBase::moveByRollPitchYawrateThrottleInternal(float roll, float pitch, float yaw_rate, float throttle)
    {
        if (safetyCheckVelocity(getVelocity()))
            commandRollPitchYawrateThrottle(roll, pitch, yaw_rate, throttle);
    }

    void MultirotorApiBase::moveByRollPitchYawrateZInternal(float roll, float pitch, float yaw_rate, float z)
    {
        if (safetyCheckVelocity(getVelocity()))
            commandRollPitchYawrateZ(roll, pitch, yaw_rate, z);
    }

    void MultirotorApiBase::moveByAngleRatesZInternal(float roll_rate, float pitch_rate, float yaw_rate, float z)
    {
        if (safetyCheckVelocity(getVelocity()))
            commandAngleRatesZ(roll_rate, pitch_rate, yaw_rate, z);
    }

    void MultirotorApiBase::moveByAngleRatesThrottleInternal(float roll_rate, float pitch_rate, float yaw_rate, float throttle)
    {
        if (safetyCheckVelocity(getVelocity()))
            commandAngleRatesThrottle(roll_rate, pitch_rate, yaw_rate, throttle);
    }

    Waiter MultirotorApiBase::waitForFunction(WaitFunction function, float timeout_sec)
    {
        Waiter waiter(getCommandPeriod(), timeout_sec, getCancelToken());
        if (timeout_sec <= 0)
            return waiter;

        do {
            if (function()) {
                waiter.complete();
                break;
            }
        } while (waiter.sleep());
        return waiter;
    }

    bool MultirotorApiBase::waitForZ(float timeout_sec, float z, float margin)
    {
        float cur_z = 100000;
        return waitForFunction([&]() {
                   cur_z = getPosition().z();
                   return (std::abs(cur_z - z) <= margin);
               },
                               timeout_sec)
            .isComplete();
    }

    void MultirotorApiBase::setSafetyEval(const shared_ptr<SafetyEval> safety_eval_ptr)
    {
        SingleCall lock(this);
        safety_eval_ptr_ = safety_eval_ptr;
    }

    RCData MultirotorApiBase::estimateRCTrims(float trimduration, float minCountForTrim, float maxTrim)
    {
        rc_data_trims_ = RCData();
        Waiter waiter_trim(getCommandPeriod(), trimduration, getCancelToken());
        uint count = 0;
        do {
            const RCData rc_data = getRCData();
            if (rc_data.is_valid) {
                rc_data_trims_.add(rc_data);
                count++;
            }
        } while (waiter_trim.sleep());

        rc_data_trims_.is_valid = true;
        if (count < minCountForTrim) {
            rc_data_trims_.is_valid = false;
            Utils::log("Cannot compute RC trim because too few readings received");
        }

        rc_data_trims_.divideBy(static_cast<float>(count));
        if (rc_data_trims_.isAnyMoreThan(maxTrim)) {
            rc_data_trims_.is_valid = false;
            Utils::log(Utils::stringf("RC trims does not seem to be valid: %s", rc_data_trims_.toString().c_str()));
        }

        Utils::log(Utils::stringf("RCData Trims: %s", rc_data_trims_.toString().c_str()));
        return rc_data_trims_;
    }

    void MultirotorApiBase::moveToPathPosition(const Vector3r& dest, float velocity, DrivetrainType drivetrain, YawMode yaw_mode, float last_z)
    {
        unused(last_z);
        if (dest.hasNaN())
            throw std::invalid_argument(VectorMath::toString(dest, "dest vector cannot have NaN: "));

        float expected_dist = velocity * getCommandPeriod();
        const Vector3r cur = getPosition();
        const Vector3r cur_dest = dest - cur;
        float cur_dest_norm = cur_dest.norm();

        adjustYaw(cur_dest, drivetrain, yaw_mode);

        Vector3r velocity_vect;
        if (cur_dest_norm < getDistanceAccuracy())
            velocity_vect = Vector3r::Zero();
        else if (cur_dest_norm >= expected_dist)
            velocity_vect = (cur_dest / cur_dest_norm) * velocity;
        else {
            Utils::log(Utils::stringf("Too close dest: cur_dest_norm=%f, expected_dist=%f", cur_dest_norm, expected_dist));
            velocity_vect = (cur_dest / cur_dest_norm) * (cur_dest_norm / getCommandPeriod());
        }

        if (std::abs(cur.z() - dest.z()) <= getDistanceAccuracy())
            moveByVelocityInternal(velocity_vect.x(), velocity_vect.y(), 0, yaw_mode);
        else
            moveByVelocityInternal(velocity_vect.x(), velocity_vect.y(), velocity_vect.z(), yaw_mode);
    }

    bool MultirotorApiBase::setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
                                      float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
    {
        if (safety_eval_ptr_ == nullptr)
            throw std::invalid_argument("The setSafety call requires safety_eval_ptr_ to be set first");

        safety_eval_ptr_->setSafety(enable_reasons, obs_clearance, obs_startegy, origin, xy_length, max_z, min_z);
        obs_avoidance_vel_ = obs_avoidance_vel;
        Utils::log(Utils::stringf("obs_avoidance_vel: %f", obs_avoidance_vel_));
        return true;
    }

    bool MultirotorApiBase::emergencyManeuverIfUnsafe(const SafetyEval::EvalResult& result)
    {
        if (!result.is_safe) {
            if (result.reason == SafetyEval::SafetyViolationType_::Obstacle) {
                if (safety_eval_ptr_->getObsAvoidanceStrategy() != SafetyEval::ObsAvoidanceStrategy::RaiseException) {
                    Vector3r avoidance_vel = getObsAvoidanceVelocity(result.cur_risk_dist, obs_avoidance_vel_) * result.suggested_vec;
                    commandVelocityZ(avoidance_vel.x(), avoidance_vel.y(), getPosition().z(), YawMode::Zero());
                    return false;
                }
            }
            commandVelocity(0, 0, 0, YawMode::Zero());
            throw UnsafeMoveException(result);
        }
        return true;
    }

    bool MultirotorApiBase::safetyCheckVelocity(const Vector3r& velocity)
    {
        if (safety_eval_ptr_ == nullptr)
            return true;
        const auto& result = safety_eval_ptr_->isSafeVelocity(getPosition(), velocity, getOrientation());
        return emergencyManeuverIfUnsafe(result);
    }

    bool MultirotorApiBase::safetyCheckVelocityZ(float vx, float vy, float z)
    {
        if (safety_eval_ptr_ == nullptr)
            return true;
        const auto& result = safety_eval_ptr_->isSafeVelocityZ(getPosition(), vx, vy, z, getOrientation());
        return emergencyManeuverIfUnsafe(result);
    }

    bool MultirotorApiBase::safetyCheckDestination(const Vector3r& dest_pos)
    {
        if (safety_eval_ptr_ == nullptr)
            return true;
        const auto& result = safety_eval_ptr_->isSafeDestination(getPosition(), dest_pos, getOrientation());
        return emergencyManeuverIfUnsafe(result);
    }

    float MultirotorApiBase::setNextPathPosition(const vector<Vector3r>& path, const vector<PathSegment>& path_segs,
                                                 const PathPosition& cur_path_loc, float next_dist, PathPosition& next_path_loc)
    {
        uint i = cur_path_loc.seg_index;
        float offset = cur_path_loc.offset;
        while (i < path.size() - 1) {
            const PathSegment& seg = path_segs.at(i);
            if (seg.seg_length > 0 && seg.seg_length >= next_dist + offset) {
                next_path_loc.seg_index = i;
                next_path_loc.offset = next_dist + offset;
                next_path_loc.position = path.at(i) + seg.seg_normalized * next_path_loc.offset;
                return 0;
            }
            next_dist -= seg.seg_length - offset;
            offset = 0;
            if (&cur_path_loc == &next_path_loc)
                Utils::log(Utils::stringf("segment %d done: x=%f, y=%f, z=%f", i, path.at(i).x(), path.at(i).y(), path.at(i).z()));
            ++i;
        }

        next_path_loc.seg_index = i;
        next_path_loc.offset = 0;
        next_path_loc.position = path.at(i);
        return next_dist;
    }

    void MultirotorApiBase::adjustYaw(const Vector3r& heading, DrivetrainType drivetrain, YawMode& yaw_mode)
    {
        if (drivetrain == DrivetrainType::ForwardOnly && !yaw_mode.is_rate) {
            if (heading.norm() > getDistanceAccuracy()) {
                yaw_mode.yaw_or_rate = yaw_mode.yaw_or_rate + (std::atan2(heading.y(), heading.x()) * 180 / M_PIf);
                yaw_mode.yaw_or_rate = VectorMath::normalizeAngle(yaw_mode.yaw_or_rate);
            }
            else {
                yaw_mode.setZeroRate();
            }
        }
    }

    void MultirotorApiBase::adjustYaw(float x, float y, DrivetrainType drivetrain, YawMode& yaw_mode)
    {
        adjustYaw(Vector3r(x, y, 0), drivetrain, yaw_mode);
    }

    bool MultirotorApiBase::isYawWithinMargin(float yaw_target, float margin) const
    {
        const float yaw_current = VectorMath::getYaw(getOrientation()) * 180 / M_PIf;
        return std::abs(yaw_current - yaw_target) <= margin;
    }

    float MultirotorApiBase::getAutoLookahead(float velocity, float adaptive_lookahead, float max_factor, float min_factor) const
    {
        float command_period_dist = velocity * getCommandPeriod();
        float lookahead = command_period_dist * (adaptive_lookahead > 0 ? min_factor : max_factor);
        lookahead = std::max(lookahead, getDistanceAccuracy() * 1.5f);
        return lookahead;
    }

    float MultirotorApiBase::getObsAvoidanceVelocity(float risk_dist, float max_obs_avoidance_vel) const
    {
        unused(risk_dist);
        return max_obs_avoidance_vel;
    }
}
} //namespace
#endif