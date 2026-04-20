// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/CarlaActorFactoryBlueprint.h"
#include "Carla/Actor/VehicleParameters.h"

#include "VehicleActorFactory.generated.h"

/// Compatibility parent for legacy vehicle factory blueprints.
UCLASS(Abstract, BlueprintType, Blueprintable)
class CARLA_API AVehicleActorFactory : public ACarlaActorFactoryBlueprint
{
  GENERATED_BODY()

public:

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Compatibility")
  TArray<FVehicleParameters> VehiclesParams;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Compatibility")
  TArray<FVehicleParameters> MineVehiclesParams;

  UFUNCTION(BlueprintCallable, Category = "CARLA|Compatibility")
  bool SaveVehicleParametersArrayToFile(
      const FString &FileName,
      const TArray<FVehicleParameters> &VehicleParamsArray);

  UFUNCTION(BlueprintCallable, Category = "CARLA|Compatibility")
  bool LoadVehicleParametersArrayFromFile(
      const FString &FileName,
      TArray<FVehicleParameters> &OutVehicleParamsArray);
};