// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/CarlaActorFactoryBlueprint.h"
#include "Carla/Actor/PropParameters.h"

#include "PropActorFactory.generated.h"

/// Compatibility parent for legacy prop factory blueprints.
UCLASS(Abstract, BlueprintType, Blueprintable)
class CARLA_API APropActorFactory : public ACarlaActorFactoryBlueprint
{
  GENERATED_BODY()

public:

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Compatibility")
  TArray<FPropParameters> PropsParams;

  UFUNCTION(BlueprintCallable, Category = "CARLA|Compatibility")
  bool SavePropParametersArrayToFile(
      const FString &FileName,
      const TArray<FPropParameters> &PropParamsArray);

  UFUNCTION(BlueprintCallable, Category = "CARLA|Compatibility")
  bool LoadPropParametersArrayFromFile(
      const FString &FileName,
      TArray<FPropParameters> &OutPropParamsArray);
};