// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Commandlets/Commandlet.h"
#include "FixBlueprintsCommandlet.generated.h"

/**
 * Fixes Blueprint assets whose StreetMap type references were lost (null import
 * entries) due to repeated failed saves during the UE5 migration.
 *
 * Run with:
 *   UnrealEditor CarlaUE5.uproject -run=FixBlueprints -unattended -nosplash
 */
UCLASS()
class CARLATOOLS_API UFixBlueprintsCommandlet : public UCommandlet
{
    GENERATED_BODY()
public:
    UFixBlueprintsCommandlet();
    virtual int32 Main(const FString& Params) override;
};
