// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaWheeledVehicleNW.h"
#include "WheeledVehicleMovementComponentNW.h" // UE5 note: not available in ChaosVehicles

ACarlaWheeledVehicleNW::ACarlaWheeledVehicleNW(const FObjectInitializer& ObjectInitializer) :
  // UE5: WheeledVehicleMovementComponentNW stubbed out - not available in ChaosVehicles
  // Super(ObjectInitializer.SetDefaultSubobjectClass<UWheeledVehicleMovementComponentNW>(AWheeledVehicle::VehicleMovementComponentName))
  Super(ObjectInitializer)
{
  bIsNWVehicle = true;
}

ACarlaWheeledVehicleNW::~ACarlaWheeledVehicleNW() {}
