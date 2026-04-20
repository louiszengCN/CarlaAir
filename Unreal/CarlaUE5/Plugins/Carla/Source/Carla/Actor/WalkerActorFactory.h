// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/CarlaActorFactoryBlueprint.h"
#include "Carla/Actor/PedestrianParameters.h"

#include "WalkerActorFactory.generated.h"

/// Compatibility parent for legacy walker factory blueprints.
UCLASS(Abstract, BlueprintType, Blueprintable)
class CARLA_API AWalkerActorFactory : public ACarlaActorFactoryBlueprint
{
  GENERATED_BODY()

public:

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Compatibility")
  TArray<FPedestrianParameters> WalkersParams;

  UFUNCTION(BlueprintCallable, Category = "CARLA|Compatibility")
  bool SaveWalkerParametersArrayToFile(
      const FString &FileName,
      const TArray<FPedestrianParameters> &WalkerParamsArray);

  UFUNCTION(BlueprintCallable, Category = "CARLA|Compatibility")
  bool LoadWalkerParametersArrayFromFile(
      const FString &FileName,
      TArray<FPedestrianParameters> &OutWalkerParamsArray);
};