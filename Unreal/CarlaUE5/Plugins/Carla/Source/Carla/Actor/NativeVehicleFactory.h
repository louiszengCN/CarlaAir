// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/CarlaActorFactory.h"

#include "NativeVehicleFactory.generated.h"

/// Pure-C++ vehicle factory that reads Content/Carla/Config/VehicleParameters.json
/// and returns definitions without relying on any Blueprint graph.
///
/// Uses LAZY class loading: GetDefinitions() does NOT call LoadClass (avoids
/// triggering Nanite DDC builds which crash for certain malformed skeletal meshes).
/// SpawnActor() loads the class on first use, so only actually-requested vehicles
/// trigger a mesh build.
UCLASS()
class CARLA_API ANativeVehicleFactory : public ACarlaActorFactory
{
  GENERATED_BODY()

public:

  TArray<FActorDefinition> GetDefinitions() override;

  FActorSpawnResult SpawnActor(
      const FTransform &SpawnAtTransform,
      const FActorDescription &ActorDescription) override;

private:

  /// Maps blueprint ID (e.g. "vehicle.lincoln.mkz") → UE class path
  /// populated in GetDefinitions(), consumed in SpawnActor().
  TMap<FString, FString> ClassPaths_;
};
