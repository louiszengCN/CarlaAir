#include "AirSimCameraDirector.h"
#include "GameFramework/PlayerController.h"
#include "AirBlueprintLib.h"

AAirSimCameraDirector::AAirSimCameraDirector()
{
    PrimaryActorTick.bCanEverTick = true;

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetRelativeLocation(FVector(0.0f, 0.0f, 34.0f));
    SpringArm->SetWorldRotation(FRotator(-20.0f, 0.0f, 0.0f));
    SpringArm->TargetArmLength = 125.0f;
    SpringArm->bEnableCameraLag = false;
    SpringArm->bEnableCameraRotationLag = false;
    SpringArm->CameraRotationLagSpeed = 10.0f;
    SpringArm->bInheritPitch = true;
    SpringArm->bInheritYaw = true;
    SpringArm->bInheritRoll = true;
}

void AAirSimCameraDirector::BeginPlay()
{
    Super::BeginPlay();
}

void AAirSimCameraDirector::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
        if (manual_pose_controller_)
            manual_pose_controller_->updateActorPose(DeltaTime);
    }
    else if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) {
    }
    else if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY) {
    }
    else {
        UAirBlueprintLib::FollowActor(ExternalCamera, follow_actor_, initial_ground_obs_offset_, ext_obs_fixed_z_);
    }
}

ECameraDirectorMode AAirSimCameraDirector::getMode()
{
    return mode_;
}

void AAirSimCameraDirector::initializeForBeginPlay(ECameraDirectorMode view_mode,
                                             AActor* follow_actor, APIPCamera* fpv_camera, APIPCamera* front_camera, APIPCamera* back_camera)
{
    manual_pose_controller_ = NewObject<UManualPoseController>(this, "CameraDirector_ManualPoseController");
    manual_pose_controller_->initializeForPlay();

    setupInputBindings();

    mode_ = view_mode;
    mode_before_front_ = view_mode;

    follow_actor_ = follow_actor;
    fpv_camera_ = fpv_camera;
    front_camera_ = front_camera;
    backup_camera_ = back_camera;
    camera_start_location_ = ExternalCamera->GetActorLocation();
    camera_start_rotation_ = ExternalCamera->GetActorRotation();
    initial_ground_obs_offset_ = camera_start_location_ -
                                 (follow_actor_ ? follow_actor_->GetActorLocation() : FVector::ZeroVector);

    switch (mode_) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME:
        inputEventFlyWithView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV:
        inputEventFpvView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER:
        inputEventGroundView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL:
        inputEventManualView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE:
        inputEventSpringArmChaseView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP:
        inputEventBackupView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY:
        inputEventNoDisplayView();
        break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT:
        inputEventFrontView();
        break;
    default:
        throw std::out_of_range("Unsupported view mode specified in CameraDirector::initializeForBeginPlay");
    }
}

void AAirSimCameraDirector::attachSpringArm(bool attach)
{
    if (attach) {
        if (follow_actor_ && ExternalCamera->GetRootComponent()->GetAttachParent() != SpringArm) {
            SpringArm->bEnableCameraRotationLag = camera_rotation_lag_enabled_;
            SpringArm->AttachToComponent(follow_actor_->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
            SpringArm->SetRelativeLocation(FVector(0.0f, 0.0f, 34.0f));
            last_parent_ = ExternalCamera->GetRootComponent()->GetAttachParent();
            ExternalCamera->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            ExternalCamera->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepRelativeTransform);
        }

        ExternalCamera->SetActorRelativeLocation(FVector(follow_distance_ * 100.0f, 0.0f, 0.0f));
        ExternalCamera->SetActorRelativeRotation(FRotator(10.0f, 0.0f, 0.0f));
    }
    else {
        if (last_parent_ && ExternalCamera->GetRootComponent()->GetAttachParent() == SpringArm) {
            ExternalCamera->DetachFromActor(FDetachmentTransformRules::KeepRelativeTransform);
            ExternalCamera->AttachToComponent(last_parent_, FAttachmentTransformRules::KeepRelativeTransform);
        }
    }
}

void AAirSimCameraDirector::setMode(ECameraDirectorMode mode)
{
    {
        if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE &&
            mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) {
            attachSpringArm(false);
        }

        if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY &&
            mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY) {
            UAirBlueprintLib::enableViewportRendering(this, true);
        }

        if (mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
            if (ExternalCamera != nullptr && manual_pose_controller_ && manual_pose_controller_->getActor() == ExternalCamera) {
                manual_pose_controller_->setActor(nullptr);
            }
        }
    }

    {
        switch (mode) {
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL:
            manual_pose_controller_->setActor(ExternalCamera);
            break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE:
            attachSpringArm(true);
            break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY:
            UAirBlueprintLib::enableViewportRendering(this, false);
            break;
        default:
            break;
        }
    }

    mode_ = mode;
}

void AAirSimCameraDirector::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventFpvView", EKeys::F, this, &AAirSimCameraDirector::inputEventFpvView);
    UAirBlueprintLib::BindActionToKey("inputEventFlyWithView", EKeys::B, this, &AAirSimCameraDirector::inputEventFlyWithView);
    UAirBlueprintLib::BindActionToKey("inputEventGroundView", EKeys::Backslash, this, &AAirSimCameraDirector::inputEventGroundView);
    UAirBlueprintLib::BindActionToKey("inputEventManualView", EKeys::M, this, &AAirSimCameraDirector::inputEventManualView);
    UAirBlueprintLib::BindActionToKey("inputEventSpringArmChaseView", EKeys::Slash, this, &AAirSimCameraDirector::inputEventSpringArmChaseView);
    UAirBlueprintLib::BindActionToKey("inputEventBackupView", EKeys::K, this, &AAirSimCameraDirector::inputEventBackupView);
    UAirBlueprintLib::BindActionToKey("inputEventNoDisplayView", EKeys::Hyphen, this, &AAirSimCameraDirector::inputEventNoDisplayView);
    UAirBlueprintLib::BindActionToKey("inputEventFrontView", EKeys::I, this, &AAirSimCameraDirector::inputEventFrontView);
}

void AAirSimCameraDirector::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    manual_pose_controller_ = nullptr;
    SpringArm = nullptr;
    ExternalCamera = nullptr;
}

APIPCamera* AAirSimCameraDirector::getFpvCamera() const
{
    return fpv_camera_;
}

APIPCamera* AAirSimCameraDirector::getExternalCamera() const
{
    return ExternalCamera;
}

APIPCamera* AAirSimCameraDirector::getBackupCamera() const
{
    return backup_camera_;
}

void AAirSimCameraDirector::inputEventSpringArmChaseView()
{
    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE);
        ExternalCamera->showToScreen();
        disableCameras(true, true, false, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventGroundView()
{
    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER);
        ExternalCamera->showToScreen();
        disableCameras(true, true, false, true);
        ext_obs_fixed_z_ = true;
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventManualView()
{
    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL);
        ExternalCamera->showToScreen();
        disableCameras(true, true, false, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventNoDisplayView()
{
    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY);
        disableCameras(true, true, true, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventBackupView()
{
    if (backup_camera_) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP);
        backup_camera_->showToScreen();
        disableCameras(true, false, true, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "backup_camera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventFrontView()
{
    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT
        || mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV) {
        switch (mode_before_front_) {
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER:
            inputEventGroundView();
            break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL:
            inputEventManualView();
            break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE:
            inputEventSpringArmChaseView();
            break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP:
            inputEventBackupView();
            break;
        default:
            inputEventFlyWithView();
            break;
        }
        return;
    }

    mode_before_front_ = mode_;

    if (front_camera_) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT);
        front_camera_->showToScreen();
        disableCameras(true, true, true, false);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "front_camera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventFlyWithView()
{
    if (ExternalCamera) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME);
        ExternalCamera->showToScreen();

        if (follow_actor_)
            ExternalCamera->SetActorLocationAndRotation(
                follow_actor_->GetActorLocation() + initial_ground_obs_offset_, camera_start_rotation_);
        disableCameras(true, true, false, true);
        ext_obs_fixed_z_ = false;
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "ExternalCamera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::inputEventFpvView()
{
    if (fpv_camera_) {
        setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV);
        fpv_camera_->showToScreen();
        disableCameras(false, true, true, true);
    }
    else
        UAirBlueprintLib::LogMessageString("Camera is not available: ", "fpv_camera", LogDebugLevel::Failure);

    notifyViewModeChanged();
}

void AAirSimCameraDirector::disableCameras(bool fpv, bool backup, bool external, bool front)
{
    if (fpv && fpv_camera_)
        fpv_camera_->disableMain();
    if (backup && backup_camera_)
        backup_camera_->disableMain();
    if (external && ExternalCamera)
        ExternalCamera->disableMain();
    if (front && front_camera_)
        front_camera_->disableMain();
}

void AAirSimCameraDirector::notifyViewModeChanged()
{
    bool nodisplay = ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY == mode_;

    UWorld* world = GetWorld();
    UGameViewportClient* gameViewport = world->GetGameViewport();
    gameViewport->bDisableWorldRendering = nodisplay;
}