#pragma once

#include "CoreMinimal.h"

#include "PawnSimApi.h"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "MultirotorPawnEvents.h"
#include <future>

class MultirotorPawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::MultiRotorPhysicsBody MultiRotor;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;

    typedef MultirotorPawnEvents::RotorActuatorInfo RotorActuatorInfo;

public:
    virtual void initialize() override;

    virtual ~MultirotorPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    MultirotorPawnSimApi(const Params& params);
    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    //PhysicsBody interface
    //this just wrapped around MultiRotor physics body
    virtual void resetImplementation() override;
    virtual void update() override;
    virtual void reportState(StateReporter& reporter) override;
    virtual UpdatableObject* getPhysicsBody() override;

    virtual void setPose(const Pose& pose, bool ignore_collision) override;
    virtual void setKinematics(const Kinematics::State& state, bool ignore_collision) override;
    virtual void pawnTick(float dt) override;

    msr::airlib::MultirotorApiBase* getVehicleApi() const
    {
        return vehicle_api_.get();
    }

    virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
    {
        return vehicle_api_.get();
    }

    // Physics collision mode: true = bounce off buildings + ground clamp, false = pass through
    void setPhysicsCollisionEnabled(bool enabled) { bPhysicsCollisionEnabled_ = enabled; }
    bool isPhysicsCollisionEnabled() const { return bPhysicsCollisionEnabled_; }
    bool isBouncing() const { return BounceTimeRemaining_ > 0.0f; }
    FVector getBounceVelocity() const { return BounceVelocityUU_; }

private:
    bool bPhysicsCollisionEnabled_ = true;

    // Bounce-back state for realistic collision response
    FVector BounceVelocityUU_ = FVector::ZeroVector;  // UU velocity during bounce
    float BounceTimeRemaining_ = 0.0f;                // seconds left in bounce animation
    float BounceCooldown_ = 0.0f;                     // cooldown to prevent repeated hits
    static constexpr float BOUNCE_DURATION = 0.4f;    // bounce animation duration (seconds)
    static constexpr float BOUNCE_COOLDOWN = 0.3f;    // min time between bounces
    static constexpr float BOUNCE_RESTITUTION = 0.6f; // energy retention on bounce (0-1)
    static constexpr float TRACE_DISTANCE = 120.0f;   // collision probe distance (cm, ~1.2m)
    static constexpr float MIN_GROUND_CLEARANCE = 5.0f; // cm above ground
    std::unique_ptr<msr::airlib::MultirotorApiBase> vehicle_api_;
    std::unique_ptr<msr::airlib::MultiRotorParams> vehicle_params_;

    std::unique_ptr<MultiRotor> multirotor_physics_body_;
    unsigned int rotor_count_;
    std::vector<RotorActuatorInfo> rotor_actuator_info_;

    //show info on collision response from physics engine
    CollisionResponse collision_response;

    MultirotorPawnEvents* pawn_events_;

    //when pose needs to set from non-physics thread, we set it as pending
    bool pending_pose_collisions_;
    enum class PendingPoseStatus
    {
        NonePending,
        RenderPending
    } pending_pose_status_;
    Pose pending_phys_pose_; //force new pose through API

    //reset must happen while World is locked so its async task initiated from API thread
    bool reset_pending_;
    bool did_reset_;
    std::packaged_task<void()> reset_task_;

    Pose last_phys_pose_; //for trace lines showing vehicle path
    std::vector<std::string> vehicle_api_messages_;
    RotorStates rotor_states_;
};
