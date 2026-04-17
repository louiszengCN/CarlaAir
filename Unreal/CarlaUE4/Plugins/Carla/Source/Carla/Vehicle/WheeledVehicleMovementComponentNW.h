// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

// UE5: WheeledVehicleMovementComponentNW is not available in ChaosVehicles.
// This file is kept as a stub to avoid breaking includes.
// The NW vehicle movement component is disabled for UE5.7 compatibility.

#include "ChaosWheeledVehicleMovementComponent.h"
#include "WheeledVehicleMovementComponentNW.generated.h"

USTRUCT()
struct FVehicleNWWheelDifferentialData
{
  GENERATED_USTRUCT_BODY()
  UPROPERTY(EditAnywhere, Category = Setup)
  bool bDriven = true;
};

USTRUCT()
struct FVehicleNWEngineData
{
  GENERATED_USTRUCT_BODY()
  UPROPERTY(EditAnywhere, Category = Setup)
  FRuntimeFloatCurve TorqueCurve;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float MaxRPM = 4500.0f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float MOI = 1.0f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float DampingRateFullThrottle = 0.15f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float DampingRateZeroThrottleClutchEngaged = 2.0f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float DampingRateZeroThrottleClutchDisengaged = 0.35f;
};

USTRUCT()
struct FVehicleNWGearData
{
  GENERATED_USTRUCT_BODY()
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float Ratio = 1.0f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float DownRatio = 0.5f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float UpRatio = 0.65f;
};

USTRUCT()
struct FVehicleNWTransmissionData
{
  GENERATED_USTRUCT_BODY()
  UPROPERTY(EditAnywhere, Category = Setup)
  bool bUseGearAutoBox = true;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float GearSwitchTime = 0.5f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float GearAutoBoxLatency = 1.0f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float FinalRatio = 4.0f;
  UPROPERTY(EditAnywhere, Category = Setup)
  TArray<FVehicleNWGearData> ForwardGears;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float NeutralGearUpRatio = 0.15f;
  UPROPERTY(EditAnywhere, Category = Setup, meta=(ClampMin="0.0"))
  float ClutchStrength = 10.0f;
};

// UE5: UWheeledVehicleMovementComponentNW stubbed as UChaosWheeledVehicleMovementComponent subclass.
// All NW-specific functionality is disabled.
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class CARLA_API UWheeledVehicleMovementComponentNW : public UChaosWheeledVehicleMovementComponent
{
  GENERATED_UCLASS_BODY()
};
