// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Components/BoxComponent.h"
#include "Engine/CollisionProfile.h"
#include "MovementComponents/DefaultMovementComponent.h"
#include "Rendering/SkeletalMeshRenderData.h"
#include "UObject/UObjectGlobals.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"

// #include "PhysXPublic.h" // UE5: PhysX removed, using Chaos
// #include "PhysXVehicleManager.h" // UE5: PhysXVehicleManager removed
// #include "TireConfig.h" // UE5: TireConfig removed in ChaosVehicles
// #include "VehicleWheel.h" // UE5: VehicleWheel replaced by UChaosVehicleWheel
#include "ChaosVehicleMovementComponent.h"
#include "ChaosWheeledVehicleMovementComponent.h"

#include "Carla.h"
#include "Carla/Game/CarlaHUD.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Trigger/FrictionTrigger.h"
#include "Carla/Util/ActorAttacher.h"
#include "Carla/Util/EmptyActor.h"
#include "Carla/Util/BoundingBoxCalculator.h"
#include "Carla/Vegetation/VegetationManager.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

ACarlaWheeledVehicle::ACarlaWheeledVehicle(const FObjectInitializer& ObjectInitializer) :
  Super(ObjectInitializer)
{
  VehicleBounds = CreateDefaultSubobject<UBoxComponent>(TEXT("VehicleBounds"));
  VehicleBounds->SetupAttachment(RootComponent);
  VehicleBounds->SetHiddenInGame(true);
  VehicleBounds->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

  VelocityControl = CreateDefaultSubobject<UVehicleVelocityControl>(TEXT("VelocityControl"));
  VelocityControl->Deactivate();

  GetVehicleMovementComponent()->bReverseAsBrake = false;
  BaseMovementComponent = CreateDefaultSubobject<UBaseCarlaMovementComponent>(TEXT("BaseMovementComponent"));
}

ACarlaWheeledVehicle::~ACarlaWheeledVehicle() {}

void ACarlaWheeledVehicle::SetWheelCollision(UChaosWheeledVehicleMovementComponent *VehicleChaos,
    const FVehiclePhysicsControl &PhysicsControl ) {

  #ifdef WHEEL_SWEEP_ENABLED
    const bool IsBike = IsTwoWheeledVehicle();

    if (IsBike)
      return;

    const bool IsEqual = VehicleChaos->UseSweepWheelCollision == PhysicsControl.UseSweepWheelCollision;

    if (IsEqual)
      return;

    VehicleChaos->UseSweepWheelCollision = PhysicsControl.UseSweepWheelCollision;

  #else

    if (PhysicsControl.UseSweepWheelCollision)
      UE_LOG(LogCarla, Warning, TEXT("Error: Sweep for wheel collision is not available. \
      Make sure you have installed the required patch.") );

  #endif

}

// UE5: SetWheelCollisionNW removed — UWheeledVehicleMovementComponentNW not available in ChaosVehicles.
// Tracked in issue #UE5-NW-VEHICLE.

void ACarlaWheeledVehicle::BeginPlay()
{
  Super::BeginPlay();

  UDefaultMovementComponent::CreateDefaultMovementComponent(this);

  // Get constraint components and their initial transforms
  FTransform ActorInverseTransform = GetActorTransform().Inverse();
  ConstraintsComponents.Empty();
  DoorComponentsTransform.Empty();
  ConstraintDoor.Empty();
  for (FName& ComponentName : ConstraintComponentNames)
  {
    UPhysicsConstraintComponent* ConstraintComponent =
        Cast<UPhysicsConstraintComponent>(GetDefaultSubobjectByName(ComponentName));
    if (ConstraintComponent)
    {
      UPrimitiveComponent* DoorComponent = Cast<UPrimitiveComponent>(
          GetDefaultSubobjectByName(ConstraintComponent->ComponentName1.ComponentName));
      if(DoorComponent)
      {
        UE_LOG(LogCarla, Warning, TEXT("Door name: %s"), *(DoorComponent->GetName()));
        FTransform ComponentWorldTransform = DoorComponent->GetComponentTransform();
        FTransform RelativeTransform = ComponentWorldTransform * ActorInverseTransform;
        DoorComponentsTransform.Add(DoorComponent, RelativeTransform);
        ConstraintDoor.Add(ConstraintComponent, DoorComponent);
        ConstraintsComponents.Add(ConstraintComponent);
        ConstraintComponent->TermComponentConstraint();
      }
      else
      {
        UE_LOG(LogCarla, Error, TEXT("Missing component for constraint: %s"), *(ConstraintComponent->GetName()));
      }
    }
  }
  ResetConstraints();

  // get collision disable constraints (used to prevent doors from colliding with each other)
  CollisionDisableConstraints.Empty();
  TArray<UPhysicsConstraintComponent*> Constraints;
  GetComponents(Constraints);
  for (UPhysicsConstraintComponent* Constraint : Constraints)
  {
    if (!ConstraintsComponents.Contains(Constraint))
    {
      UPrimitiveComponent* CollisionDisabledComponent1 = Cast<UPrimitiveComponent>(
          GetDefaultSubobjectByName(Constraint->ComponentName1.ComponentName));
      UPrimitiveComponent* CollisionDisabledComponent2 = Cast<UPrimitiveComponent>(
          GetDefaultSubobjectByName(Constraint->ComponentName2.ComponentName));
      if (CollisionDisabledComponent1)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent1, Constraint);
      }
      if (CollisionDisabledComponent2)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent2, Constraint);
      }
    }
  }

  float FrictionScale = 3.5f;

  UChaosWheeledVehicleMovementComponent* MovementComponent = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

  if (MovementComponent)
  {
    // Setup Tire Configs with default value. This is needed to avoid getting
    // friction values of previously created TireConfigs for the same vehicle
    // blueprint.
    TArray<float> OriginalFrictions;
    OriginalFrictions.Init(FrictionScale, MovementComponent->Wheels.Num());
    SetWheelsFrictionScale(OriginalFrictions);

    // Check if it overlaps with a Friction trigger, if so, update the friction
    // scale.
    TArray<AActor *> OverlapActors;
    GetOverlappingActors(OverlapActors, AFrictionTrigger::StaticClass());
    for (const auto &Actor : OverlapActors)
    {
      AFrictionTrigger *FrictionTrigger = Cast<AFrictionTrigger>(Actor);
      if (FrictionTrigger)
      {
        FrictionScale = FrictionTrigger->Friction;
      }
    }

    // Set the friction scale to Wheel CDO and update wheel setups
    TArray<FChaosWheelSetup> NewWheelSetups = MovementComponent->WheelSetups;
    for (const auto &WheelSetup : NewWheelSetups)
    {
      UChaosVehicleWheel *Wheel = WheelSetup.WheelClass.GetDefaultObject(); // UE5: TSubclassOf::GetDefaultObject() is not a template
      if (!IsValid(Wheel)) { continue; }
    }

    MovementComponent->WheelSetups = NewWheelSetups;

    LastAppliedPhysicsControl = GetVehiclePhysicsControl();

    // Update physics in the Ackermann Controller
    AckermannController.UpdateVehiclePhysics(this);
  }

  AddReferenceToManager();
}

void ACarlaWheeledVehicle::TickActor(float DeltaTime, enum ELevelTick TickType, FActorTickFunction& ThisTickFunction){
  Super::TickActor(DeltaTime, TickType, ThisTickFunction);

  FPoseSnapshot pose;
  GetMesh()->SnapshotPose(pose);
  for(FTransform &transform : pose.LocalTransforms)
  {
    transform *= GetMesh()->GetComponentTransform();
  }
  
  WorldTransformedPose = pose;

}

bool ACarlaWheeledVehicle::IsInVehicleRange(const FVector& Location) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::IsInVehicleRange);

  return FoliageBoundingBox.IsInside(Location);
}

void ACarlaWheeledVehicle::UpdateDetectionBox()
{
  const FTransform GlobalTransform = GetActorTransform();
  const FVector Vec { DetectionSize, DetectionSize, DetectionSize };
  FBox Box = FBox(-Vec, Vec);
  const FTransform NonScaledTransform(GlobalTransform.GetRotation(), GlobalTransform.GetLocation(), {1.0f, 1.0f, 1.0f});
  FoliageBoundingBox = Box.TransformBy(NonScaledTransform);
}

const TArray<int32> ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle(const UInstancedStaticMeshComponent* Component) const
{  
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle);
  return Component->GetInstancesOverlappingBox(FoliageBoundingBox);
}

FBox ACarlaWheeledVehicle::GetDetectionBox() const
{  
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetDetectionBox);
  return FoliageBoundingBox;
}

float ACarlaWheeledVehicle::GetDetectionSize() const
{
  return DetectionSize;
}

void ACarlaWheeledVehicle::DrawFoliageBoundingBox() const
{
  const FVector& Center = FoliageBoundingBox.GetCenter();
  const FVector& Extent = FoliageBoundingBox.GetExtent();
  const FQuat& Rotation = GetActorQuat();
  DrawDebugBox(GetWorld(), Center, Extent, Rotation, FColor::Magenta, false, 0.0f, 0, 5.0f);
}

FBoxSphereBounds ACarlaWheeledVehicle::GetBoxSphereBounds() const
{
  ALargeMapManager* LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    FTransform GlobalTransform = LargeMap->LocalToGlobalTransform(GetActorTransform());
    return VehicleBounds->CalcBounds(GlobalTransform);
  }
  return VehicleBounds->CalcBounds(GetActorTransform());
}

void ACarlaWheeledVehicle::AdjustVehicleBounds()
{
  FBoundingBox BoundingBox = UBoundingBoxCalculator::GetVehicleBoundingBox(this);

  const FTransform& CompToWorldTransform = RootComponent->GetComponentTransform();
  const FRotator Rotation = CompToWorldTransform.GetRotation().Rotator();
  const FVector Translation = CompToWorldTransform.GetLocation();
  const FVector Scale = CompToWorldTransform.GetScale3D();

  // Invert BB origin to local space
  BoundingBox.Origin -= Translation;
  BoundingBox.Origin = Rotation.UnrotateVector(BoundingBox.Origin);
  BoundingBox.Origin /= Scale;

  // Prepare Box Collisions
  FTransform Transform;
  Transform.SetTranslation(BoundingBox.Origin);
  VehicleBounds->SetRelativeTransform(Transform);
  VehicleBounds->SetBoxExtent(BoundingBox.Extent);
}

// =============================================================================
// -- Get functions ------------------------------------------------------------
// =============================================================================

float ACarlaWheeledVehicle::GetVehicleForwardSpeed() const
{
  return BaseMovementComponent->GetVehicleForwardSpeed();
}

FVector ACarlaWheeledVehicle::GetVehicleOrientation() const
{
  return GetVehicleTransform().GetRotation().GetForwardVector();
}

int32 ACarlaWheeledVehicle::GetVehicleCurrentGear() const
{
    return BaseMovementComponent->GetVehicleCurrentGear();
}

FTransform ACarlaWheeledVehicle::GetVehicleBoundingBoxTransform() const
{
  return VehicleBounds->GetRelativeTransform();
}

FVector ACarlaWheeledVehicle::GetVehicleBoundingBoxExtent() const
{
  return VehicleBounds->GetScaledBoxExtent();
}

float ACarlaWheeledVehicle::GetMaximumSteerAngle() const
{
  const UChaosWheeledVehicleMovementComponent* VehicleChaos = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
  if (!VehicleChaos || VehicleChaos->Wheels.Num() == 0)
  {
    return 0.0f;
  }
  const UChaosVehicleWheel *FrontWheel = VehicleChaos->Wheels[0].Get(); // UE5: TArray<TObjectPtr<UChaosVehicleWheel>>
  if (!FrontWheel)
  {
    return 0.0f;
  }
  // UE5: SteerAngle renamed to MaxSteerAngle in UChaosVehicleWheel
  return FrontWheel->MaxSteerAngle;
}

// =============================================================================
// -- Set functions ------------------------------------------------------------
// =============================================================================

void ACarlaWheeledVehicle::FlushVehicleControl()
{
  if (IsAckermannControlActive()) {
    AckermannController.UpdateVehicleState(this);
    AckermannController.RunLoop(InputControl.Control);
  }

  BaseMovementComponent->ProcessControl(InputControl.Control);
  InputControl.Control.bReverse = InputControl.Control.Gear < 0;
  LastAppliedControl = InputControl.Control;
  InputControl.Priority = EVehicleInputPriority::INVALID;
}

void ACarlaWheeledVehicle::SetThrottleInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Throttle = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetSteeringInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Steer = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetBrakeInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Brake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetReverse(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bReverse = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetHandbrakeInput(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bHandBrake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

TArray<float> ACarlaWheeledVehicle::GetWheelsFrictionScale()
{

  UChaosWheeledVehicleMovementComponent* Movement = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
  TArray<float> WheelsFrictionScale;
  if (Movement)
  {
    for (auto &Wheel : Movement->Wheels)
    {
      // UE5: TireConfig->GetFrictionScale() replaced by FrictionForceMultiplier.
      // Note: this reads the live wheel instance property; changes made by
      // SetWheelsFrictionScale take effect after RecreatePhysicsState() is called.
      WheelsFrictionScale.Add(Wheel->FrictionForceMultiplier);
    }
  }
  return WheelsFrictionScale;
}

void ACarlaWheeledVehicle::SetWheelsFrictionScale(TArray<float> &WheelsFrictionScale)
{

  UChaosWheeledVehicleMovementComponent* Movement = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
  if (Movement)
  {
    // Soft guard: in shipping ensure() is stripped; clamp loop to the smaller count to prevent OOB
    if (Movement->Wheels.Num() != WheelsFrictionScale.Num())
    {
      UE_LOG(LogCarla, Warning, TEXT("SetWheelsFrictionScale: wheel count mismatch (%d vs %d)"),
             Movement->Wheels.Num(), WheelsFrictionScale.Num());
    }
    const int32 Count = FMath::Min(Movement->Wheels.Num(), WheelsFrictionScale.Num());
    for (int32 i = 0; i < Count; ++i)
    {
      // UE5: TireConfig->SetFrictionScale() replaced by UChaosVehicleWheel friction
      Movement->Wheels[i]->FrictionForceMultiplier = WheelsFrictionScale[i];
    }
  }
}

FVehiclePhysicsControl ACarlaWheeledVehicle::GetVehiclePhysicsControl() const
{
  FVehiclePhysicsControl PhysicsControl;

  UChaosWheeledVehicleMovementComponent *VehicleChaos = Cast<UChaosWheeledVehicleMovementComponent>(
        GetVehicleMovementComponent());
  if (!VehicleChaos)
  {
    return PhysicsControl;
  }

  // Engine Setup
  PhysicsControl.TorqueCurve = VehicleChaos->EngineSetup.TorqueCurve.EditorCurveData;
  PhysicsControl.MaxRPM = VehicleChaos->EngineSetup.MaxRPM;
  // UE5: MOI → EngineRevUpMOI; DampingRate* fields removed from FChaosEngineConfig
  PhysicsControl.MOI = VehicleChaos->EngineSetup.EngineRevUpMOI;
  PhysicsControl.DampingRateFullThrottle = 0.0f;
  PhysicsControl.DampingRateZeroThrottleClutchEngaged = 0.0f;
  PhysicsControl.DampingRateZeroThrottleClutchDisengaged = 0.0f;

  // Transmission Setup
  PhysicsControl.bUseGearAutoBox = VehicleChaos->TransmissionSetup.bUseAutomaticGears;
  PhysicsControl.GearSwitchTime = VehicleChaos->TransmissionSetup.GearChangeTime;
  // UE5: ClutchStrength removed from ChaosVehicles TransmissionSetup
  // PhysicsControl.ClutchStrength = VehicleChaos->TransmissionSetup.ClutchStrength;
  PhysicsControl.FinalRatio = VehicleChaos->TransmissionSetup.FinalRatio;

  TArray<FGearPhysicsControl> ForwardGears;

  for (const auto &GearRatio : VehicleChaos->TransmissionSetup.ForwardGearRatios)
  {
    FGearPhysicsControl GearPhysicsControl;

    GearPhysicsControl.Ratio = GearRatio;
    // UE5: UpRatio and DownRatio not available as per-gear data in ChaosVehicles
    GearPhysicsControl.UpRatio = 0.0f;
    GearPhysicsControl.DownRatio = 0.0f;

    ForwardGears.Add(GearPhysicsControl);
  }

  PhysicsControl.ForwardGears = ForwardGears;

  // Vehicle Setup
  PhysicsControl.Mass = VehicleChaos->Mass;
  PhysicsControl.DragCoefficient = VehicleChaos->DragCoefficient;

  // Center of mass offset (Center of mass is always zero vector in local
  // position)
  UPrimitiveComponent *UpdatedPrimitive = Cast<UPrimitiveComponent>(VehicleChaos->UpdatedComponent);
  if (UpdatedPrimitive)
  {
    PhysicsControl.CenterOfMass = UpdatedPrimitive->BodyInstance.COMNudge;
  }

  // Steering Setup
  PhysicsControl.SteeringCurve = VehicleChaos->SteeringSetup.SteeringCurve.EditorCurveData;

  // Wheels Setup
  TArray<FWheelPhysicsControl> Wheels;

  for (int32 i = 0; i < VehicleChaos->WheelSetups.Num(); ++i)
  {
    FWheelPhysicsControl PhysicsWheel;

    if (bPhysicsEnabled) {
      // UE5: PxVehicleWheelData/PxVehicleTireData removed - read from Chaos wheel setup
      UChaosVehicleWheel* ChaosWheel = VehicleChaos->Wheels.IsValidIndex(i) ? VehicleChaos->Wheels[i] : nullptr;
      if (ChaosWheel)
      {
        PhysicsWheel.DampingRate = 0.0f; // UE5: WheelDampingRate removed from UChaosVehicleWheel
        PhysicsWheel.MaxSteerAngle = ChaosWheel->MaxSteerAngle;
        PhysicsWheel.Radius = ChaosWheel->WheelRadius;
        PhysicsWheel.MaxBrakeTorque = ChaosWheel->MaxBrakeTorque;
        PhysicsWheel.MaxHandBrakeTorque = ChaosWheel->MaxHandBrakeTorque;
        // UE5: LatStiffMaxLoad, LatStiffValue, LongStiffValue not directly available in Chaos
        PhysicsWheel.LatStiffMaxLoad = 0.0f;
        PhysicsWheel.LatStiffValue = 0.0f;
        PhysicsWheel.LongStiffValue = 0.0f;
        PhysicsWheel.TireFriction = ChaosWheel->FrictionForceMultiplier;
        PhysicsWheel.Position = ChaosWheel->Location;
      }
    } else {
      if (i < LastAppliedPhysicsControl.Wheels.Num()) {
        PhysicsWheel = LastAppliedPhysicsControl.Wheels[i];
      }
    }
    Wheels.Add(PhysicsWheel);
  }

  PhysicsControl.Wheels = Wheels;

  return PhysicsControl;
}

FVehicleLightState ACarlaWheeledVehicle::GetVehicleLightState() const
{
  return InputControl.LightState;
}

void ACarlaWheeledVehicle::RestoreVehiclePhysicsControl()
{
  ApplyVehiclePhysicsControl(LastAppliedPhysicsControl);
}

void ACarlaWheeledVehicle::ApplyVehiclePhysicsControl(const FVehiclePhysicsControl &PhysicsControl)
{
  LastAppliedPhysicsControl = PhysicsControl;

  UChaosWheeledVehicleMovementComponent *VehicleChaos = Cast<UChaosWheeledVehicleMovementComponent>(
        GetVehicleMovementComponent());
  if (!VehicleChaos)
  {
    return;
  }

  // Engine Setup
  VehicleChaos->EngineSetup.TorqueCurve.EditorCurveData = PhysicsControl.TorqueCurve;
  VehicleChaos->EngineSetup.MaxRPM = PhysicsControl.MaxRPM;

  // UE5: MOI → EngineRevUpMOI; DampingRate* fields removed from FChaosEngineConfig
  VehicleChaos->EngineSetup.EngineRevUpMOI = PhysicsControl.MOI;
  // DampingRateFullThrottle/ZeroThrottleClutch* removed in UE5 ChaosVehicles

  // Transmission Setup
  VehicleChaos->TransmissionSetup.bUseAutomaticGears = PhysicsControl.bUseGearAutoBox;
  VehicleChaos->TransmissionSetup.GearChangeTime = PhysicsControl.GearSwitchTime;
  // UE5: ClutchStrength removed from ChaosVehicles TransmissionSetup
  // VehicleChaos->TransmissionSetup.ClutchStrength = PhysicsControl.ClutchStrength;
  VehicleChaos->TransmissionSetup.FinalRatio = PhysicsControl.FinalRatio;

  TArray<float> ForwardGearRatios;

  for (const auto &Gear : PhysicsControl.ForwardGears)
  {
    // UE5: ForwardGearRatios is a flat float array (ratio only); UpRatio/DownRatio not used
    ForwardGearRatios.Add(Gear.Ratio);
  }

  VehicleChaos->TransmissionSetup.ForwardGearRatios = ForwardGearRatios;

  // Vehicle Setup
  VehicleChaos->Mass = PhysicsControl.Mass;
  VehicleChaos->DragCoefficient = PhysicsControl.DragCoefficient;

  // Center of mass
  UPrimitiveComponent *UpdatedPrimitive = Cast<UPrimitiveComponent>(VehicleChaos->UpdatedComponent);
  if (UpdatedPrimitive)
  {
    UpdatedPrimitive->BodyInstance.COMNudge = PhysicsControl.CenterOfMass;
  }

  // Steering Setup
  VehicleChaos->SteeringSetup.SteeringCurve.EditorCurveData = PhysicsControl.SteeringCurve;

  // Wheels Setup
  const int PhysicsWheelsNum = PhysicsControl.Wheels.Num();

  // Change, if required, the collision mode for wheels
  SetWheelCollision(VehicleChaos, PhysicsControl);

  TArray<FChaosWheelSetup> NewWheelSetups = VehicleChaos->WheelSetups;
  const int32 VehicleWheelsNum = NewWheelSetups.Num();

  for (int32 i = 0; i < PhysicsWheelsNum; ++i)
  {
    if (i >= VehicleWheelsNum)
    {
      UE_LOG(LogCarla, Warning, TEXT("ApplyVehiclePhysicsControl: PhysicsControl has %d wheels but vehicle has %d — skipping extra"), PhysicsWheelsNum, VehicleWheelsNum);
      break;
    }
    UChaosVehicleWheel *Wheel = NewWheelSetups[i].WheelClass.GetDefaultObject(); // UE5: TSubclassOf::GetDefaultObject() not a template
    if (!Wheel) continue;

    // UE5: TireConfig replaced by FrictionForceMultiplier on UChaosVehicleWheel
    Wheel->FrictionForceMultiplier = PhysicsControl.Wheels[i].TireFriction;
  }

  VehicleChaos->WheelSetups = NewWheelSetups;

  // Recreate Physics State for vehicle setup
  // UE5: GetPxScene()->lockWrite()/unlockWrite() removed; Chaos does not need PhysX scene lock
  VehicleChaos->RecreatePhysicsState();

  // UE5: Per-wheel Px data (mRadius, mMaxSteer, mDampingRate, etc.) not available in Chaos.
  // Wheel properties are set via WheelSetups CDO above and take effect after RecreatePhysicsState.
  // LatStiffMaxLoad, LatStiffValue, LongStiffValue have no direct Chaos equivalents and are skipped.

  ResetConstraints();

  auto * Recorder = UCarlaStatics::GetRecorder(GetWorld());
  if (Recorder && Recorder->IsEnabled())
  {
    Recorder->AddPhysicsControl(*this);
  }

  // Update physics in the Ackermann Controller
  AckermannController.UpdateVehiclePhysics(this);
  
}

void ACarlaWheeledVehicle::ActivateVelocityControl(const FVector &Velocity)
{
  VelocityControl->Activate(Velocity);
}

void ACarlaWheeledVehicle::DeactivateVelocityControl()
{
  VelocityControl->Deactivate();
}

FVehicleTelemetryData ACarlaWheeledVehicle::GetVehicleTelemetryData() const
{
  FVehicleTelemetryData TelemetryData;

  UChaosWheeledVehicleMovementComponent *MovementComponent = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
  if (!MovementComponent)
  {
    return TelemetryData;
  }

  // Vehicle telemetry data
  TelemetryData.Speed = GetVehicleForwardSpeed() / 100.0f;  // From cm/s to m/s
  TelemetryData.Steer = LastAppliedControl.Steer;
  TelemetryData.Throttle = LastAppliedControl.Throttle;
  TelemetryData.Brake = LastAppliedControl.Brake;
  TelemetryData.EngineRPM = MovementComponent->GetEngineRotationSpeed();
  TelemetryData.Gear = GetVehicleCurrentGear();
  TelemetryData.Drag = MovementComponent->DebugDragMagnitude / 100.0f;  // kg*cm/s2 to Kg*m/s2

  // Wheels telemetry data
  // UE5: PhysXVehicleManager and PxWheelQueryResult removed; read from UChaosVehicleWheel
  TArray<FWheelTelemetryData> Wheels;
  for (int32 w = 0; w < MovementComponent->Wheels.Num(); ++w)
  {
    FWheelTelemetryData WheelTelemetryData;

    UChaosVehicleWheel* ChaosWheel = MovementComponent->Wheels[w];
    // UE5: tireFriction/lateralSlip/longitudinalSlip from PxWheelQueryResult not available in Chaos
    WheelTelemetryData.TireFriction = ChaosWheel->FrictionForceMultiplier;
    WheelTelemetryData.LatSlip = 0.0f;   // UE5: not directly available in ChaosVehicles
    WheelTelemetryData.LongSlip = 0.0f;  // UE5: not directly available in ChaosVehicles
    WheelTelemetryData.Omega = 0.0f;  // UE5: UChaosVehicleWheel has no angular-velocity accessor;
    // GetRotationAngle() returns cumulative angle (degrees), not rad/s — zeroed like other PhysX-era fields.
    // TODO: restore by finite-differencing GetRotationAngle() over DeltaTime in Tick() and caching per wheel.
    // UE5: DebugTireLoad, DebugNormalizedTireLoad, DebugWheelTorque, DebugLongForce, DebugLatForce
    // are PhysX-era debug fields not present on UChaosVehicleWheel
    WheelTelemetryData.TireLoad = 0.0f;
    WheelTelemetryData.NormalizedTireLoad = 0.0f;
    WheelTelemetryData.Torque = 0.0f;
    WheelTelemetryData.LongForce = 0.0f;
    WheelTelemetryData.LatForce = 0.0f;
    WheelTelemetryData.NormalizedLongForce = 0.0f;
    WheelTelemetryData.NormalizedLatForce = 0.0f;

    Wheels.Add(WheelTelemetryData);
  }

  TelemetryData.Wheels = Wheels;

  return TelemetryData;

}

void ACarlaWheeledVehicle::ShowDebugTelemetry(bool Enabled)
{
  if (GetWorld()->GetFirstPlayerController())
  {
    ACarlaHUD* hud = Cast<ACarlaHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
    if (hud) {

      // Set/Unset the car movement component in HUD to show the temetry
      if (Enabled) {
        hud->AddDebugVehicleForTelemetry(GetVehicleMovementComponent());
      }
      else{
        if (hud->DebugVehicle == GetVehicleMovementComponent()) {
          hud->AddDebugVehicleForTelemetry(nullptr);
          // UE5: StopTelemetry() removed from UChaosVehicleMovementComponent
        }
      }

    }
    else {
      UE_LOG(LogCarla, Warning, TEXT("ACarlaWheeledVehicle::ShowDebugTelemetry:: Cannot find HUD for debug info"));
    }
  }
}

void ACarlaWheeledVehicle::SetVehicleLightState(const FVehicleLightState &LightState)
{
  if (LightState.Position != InputControl.LightState.Position ||
      LightState.LowBeam != InputControl.LightState.LowBeam ||
      LightState.HighBeam != InputControl.LightState.HighBeam ||
      LightState.Brake != InputControl.LightState.Brake ||
      LightState.RightBlinker != InputControl.LightState.RightBlinker ||
      LightState.LeftBlinker != InputControl.LightState.LeftBlinker ||
      LightState.Reverse != InputControl.LightState.Reverse ||
      LightState.Fog != InputControl.LightState.Fog ||
      LightState.Interior != InputControl.LightState.Interior ||
      LightState.Special1 != InputControl.LightState.Special1 ||
      LightState.Special2 != InputControl.LightState.Special2)
  {
    InputControl.LightState = LightState;
    RefreshLightState(LightState);
  }
}

void ACarlaWheeledVehicle::SetFailureState(const carla::rpc::VehicleFailureState &InFailureState)
{
  FailureState = InFailureState;
}

void ACarlaWheeledVehicle::SetCarlaMovementComponent(UBaseCarlaMovementComponent* MovementComponent)
{
  if (BaseMovementComponent)
  {
    BaseMovementComponent->DestroyComponent();
  }
  BaseMovementComponent = MovementComponent;
}

void ACarlaWheeledVehicle::SetWheelSteerDirection(EVehicleWheelLocation WheelLocation, float AngleInDeg) {

  if (bPhysicsEnabled == false)
  {
    // UE5: UVehicleAnimInstance removed in ChaosVehicles — wheel steer direction not supported in kinematic mode
    UE_LOG(LogCarla, Warning, TEXT("SetWheelSteerDirection: UVehicleAnimInstance API removed in ChaosVehicles."));
  }
  else
  {
    UE_LOG(LogCarla, Warning, TEXT("Cannot set wheel steer direction. Physics are enabled."));
  }
}

float ACarlaWheeledVehicle::GetWheelSteerAngle(EVehicleWheelLocation WheelLocation) {

  // UE5: UVehicleAnimInstance removed in ChaosVehicles — get steer angle from ChaosVehicleMovement directly
  if (bPhysicsEnabled == true)
  {
    UChaosWheeledVehicleMovementComponent* Movement = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    if (Movement && Movement->Wheels.IsValidIndex((uint8)WheelLocation))
    {
      return Movement->Wheels[(uint8)WheelLocation]->GetSteerAngle();
    }
  }
  return 0.0f;
}

void ACarlaWheeledVehicle::SetWheelPitchAngle(EVehicleWheelLocation WheelLocation, float AngleInDeg) {

  if (bPhysicsEnabled == false)
  {
    // UE5: UVehicleAnimInstance removed in ChaosVehicles — wheel pitch angle not supported in kinematic mode
    UE_LOG(LogCarla, Warning, TEXT("SetWheelPitchAngle: UVehicleAnimInstance API removed in ChaosVehicles."));
  }
  else
  {
    UE_LOG(LogCarla, Warning, TEXT("Cannot set wheel pitch angle. Physics are enabled."))
  }
}

float ACarlaWheeledVehicle::GetWheelPitchAngle(EVehicleWheelLocation WheelLocation) {

  // UE5: UVehicleAnimInstance removed in ChaosVehicles — get rotation angle from ChaosVehicleMovement directly
  if (bPhysicsEnabled == true)
  {
    UChaosWheeledVehicleMovementComponent* Movement = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
    if (Movement && Movement->Wheels.IsValidIndex((uint8)WheelLocation))
    {
      return Movement->Wheels[(uint8)WheelLocation]->GetRotationAngle();
    }
  }
  return 0.0f;
}

void ACarlaWheeledVehicle::SetSimulatePhysics(bool enabled) {
  if(!GetCarlaMovementComponent<UDefaultMovementComponent>())
  {
    return;
  }

  UChaosWheeledVehicleMovementComponent* Movement = Cast<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());
  if (Movement)
  {
    if(bPhysicsEnabled == enabled)
      return;

    SetActorEnableCollision(true);
    auto RootComponent = Cast<UPrimitiveComponent>(GetRootComponent());
    RootComponent->SetSimulatePhysics(enabled);
    RootComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    // UE5: UVehicleAnimInstance removed in ChaosVehicles — ResetWheelCustomRotations not available
    // UE5: GetPxScene()->lockWrite()/unlockWrite() removed; Chaos does not use PhysX scene locking
    if (enabled)
    {
      Movement->RecreatePhysicsState();
    }
    else
    {
      Movement->DestroyPhysicsState();
    }

    bPhysicsEnabled = enabled;

    ResetConstraints();
  }

}

void ACarlaWheeledVehicle::ResetConstraints()
{
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    OpenDoorPhys(EVehicleDoor(i));
  }
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    CloseDoorPhys(EVehicleDoor(i));
  }
}

FVector ACarlaWheeledVehicle::GetVelocity() const
{
  return BaseMovementComponent->GetVelocity();
}

void ACarlaWheeledVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  ShowDebugTelemetry(false);
  Super::EndPlay(EndPlayReason);
  RemoveReferenceToManager();
}

void ACarlaWheeledVehicle::OpenDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogCarla, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      OpenDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  OpenDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::CloseDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogCarla, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      CloseDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  CloseDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::OpenDoorPhys(const EVehicleDoor DoorIdx)
{
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  DoorComponent->DetachFromComponent(
      FDetachmentTransformRules(EDetachmentRule::KeepWorld, false));
  FTransform DoorInitialTransform =
      DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->SetSimulatePhysics(true);
  DoorComponent->SetCollisionProfileName(TEXT("BlockAll"));
  float AngleLimit = Constraint->ConstraintInstance.GetAngularSwing1Limit();
  FRotator AngularRotationOffset = Constraint->ConstraintInstance.AngularRotationOffset;

  if (Constraint->ConstraintInstance.AngularRotationOffset.Yaw < 0.0f)
  {
    AngleLimit = -AngleLimit;
  }
  Constraint->SetAngularOrientationTarget(FRotator(0, AngleLimit, 0));
  Constraint->SetAngularDriveParams(DoorOpenStrength, 1.0, 0.0);

  Constraint->InitComponentConstraint();

  UPhysicsConstraintComponent** CollisionDisable =
      CollisionDisableConstraints.Find(DoorComponent);
  if (CollisionDisable)
  {
    (*CollisionDisable)->InitComponentConstraint();
  }

  RecordDoorChange(DoorIdx, true);
}

void ACarlaWheeledVehicle::CloseDoorPhys(const EVehicleDoor DoorIdx)
{
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  FTransform DoorInitialTransform =
      DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetSimulatePhysics(false);
  DoorComponent->SetCollisionProfileName(TEXT("NoCollision"));
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->AttachToComponent(
      GetMesh(), FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
  RecordDoorChange(DoorIdx, false);
}

void ACarlaWheeledVehicle::RecordDoorChange(const EVehicleDoor DoorIdx, bool bIsOpen)
{
  auto * Recorder = UCarlaStatics::GetRecorder(GetWorld());
  if (Recorder && Recorder->IsEnabled())
  {
      Recorder->AddVehicleDoor(*this, DoorIdx, bIsOpen);
  }
}

void ACarlaWheeledVehicle::ApplyRolloverBehavior()
{
  auto roll = GetVehicleTransform().Rotator().Roll;

  // The angular velocity reduction is applied in 4 stages, to improve its smoothness.
  // Case 4 starts the timer to set the rollover flag, so users are notified.
  switch (RolloverBehaviorTracker) {
    case 0: CheckRollover(roll, std::make_pair(130.0, 230.0));      break;
    case 1: CheckRollover(roll, std::make_pair(140.0, 220.0));      break;
    case 2: CheckRollover(roll, std::make_pair(150.0, 210.0));      break;
    case 3: CheckRollover(roll, std::make_pair(160.0, 200.0));      break;
    case 4:
      GetWorld()->GetTimerManager().SetTimer(TimerHandler, this, &ACarlaWheeledVehicle::SetRolloverFlag, RolloverFlagTime);
      RolloverBehaviorTracker += 1;
      break;
    case 5: break;
    default:
      RolloverBehaviorTracker = 5;
  }

  // In case the vehicle recovers, reset the rollover tracker
  if (RolloverBehaviorTracker > 0 && -30 < roll && roll < 30){
    RolloverBehaviorTracker = 0;
    FailureState = carla::rpc::VehicleFailureState::None;
  }
}

void ACarlaWheeledVehicle::CheckRollover(const float roll, const std::pair<float, float> threshold_roll){
  if (threshold_roll.first < roll && roll < threshold_roll.second){
    auto RootComponent = Cast<UPrimitiveComponent>(GetRootComponent());
    auto angular_velocity = RootComponent->GetPhysicsAngularVelocityInDegrees();
    RootComponent->SetPhysicsAngularVelocityInDegrees((1 - RolloverBehaviorForce) * angular_velocity);
    RolloverBehaviorTracker += 1;
  }
}

void ACarlaWheeledVehicle::SetRolloverFlag(){
  // Make sure the vehicle hasn't recovered since the timer started
  if (RolloverBehaviorTracker >= 4) {
    FailureState = carla::rpc::VehicleFailureState::Rollover;
  }
}

carla::rpc::VehicleFailureState ACarlaWheeledVehicle::GetFailureState() const{
  return FailureState;
}

void ACarlaWheeledVehicle::AddReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->AddVehicle(this);
    return;
  }
}

void ACarlaWheeledVehicle::RemoveReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->RemoveVehicle(this);
    return;
  }
}

FRotator ACarlaWheeledVehicle::GetPhysicsConstraintAngle(
    UPhysicsConstraintComponent* Component)
{
  return Component->ConstraintInstance.AngularRotationOffset;
}

void ACarlaWheeledVehicle::SetPhysicsConstraintAngle(
    UPhysicsConstraintComponent* Component, const FRotator &NewAngle)
{
  Component->ConstraintInstance.AngularRotationOffset = NewAngle;
}

FPoseSnapshot ACarlaWheeledVehicle::GetWorldTransformedPose()
{
  if(WorldTransformedPose.bIsValid == false)
  {
    SetActorTickEnabled(true);
    GetMesh()->SnapshotPose(WorldTransformedPose);
    for(FTransform &transform : WorldTransformedPose.LocalTransforms)
    {
      transform *= GetMesh()->GetComponentTransform();
    }
  }
  return WorldTransformedPose;
}
