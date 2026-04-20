// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Actor/NativeVehicleFactory.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Actor/ActorSpawnResult.h"
#include "Carla/Actor/VehicleParameters.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/WheeledVehicleAIController.h"

#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

static TArray<FColor> ParseColors(const TArray<TSharedPtr<FJsonValue>> &ColorValues)
{
  TArray<FColor> Colors;
  for (const auto &Val : ColorValues)
  {
    const TSharedPtr<FJsonObject> Obj = Val->AsObject();
    if (!Obj.IsValid()) continue;
    FColor C;
    C.R = static_cast<uint8>(Obj->GetIntegerField(TEXT("R")));
    C.G = static_cast<uint8>(Obj->GetIntegerField(TEXT("G")));
    C.B = static_cast<uint8>(Obj->GetIntegerField(TEXT("B")));
    C.A = static_cast<uint8>(Obj->GetIntegerField(TEXT("A")));
    Colors.Add(C);
  }
  return Colors;
}

TArray<FActorDefinition> ANativeVehicleFactory::GetDefinitions()
{
  ClassPaths_.Empty();
  TArray<FActorDefinition> Definitions;

  const FString JsonPath =
      FPaths::ProjectContentDir() / TEXT("Carla/Config/VehicleParameters.json");

  FString JsonString;
  if (!FFileHelper::LoadFileToString(JsonString, *JsonPath))
  {
    UE_LOG(LogCarla, Warning,
        TEXT("NativeVehicleFactory: could not read %s"), *JsonPath);
    return Definitions;
  }

  TSharedPtr<FJsonObject> Root;
  TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
  {
    UE_LOG(LogCarla, Warning,
        TEXT("NativeVehicleFactory: failed to parse %s"), *JsonPath);
    return Definitions;
  }

  const TArray<TSharedPtr<FJsonValue>> *Vehicles = nullptr;
  if (!Root->TryGetArrayField(TEXT("Vehicles"), Vehicles) || Vehicles == nullptr)
  {
    UE_LOG(LogCarla, Warning,
        TEXT("NativeVehicleFactory: no 'Vehicles' array in %s"), *JsonPath);
    return Definitions;
  }

  for (const TSharedPtr<FJsonValue> &Val : *Vehicles)
  {
    const TSharedPtr<FJsonObject> Obj = Val->AsObject();
    if (!Obj.IsValid()) continue;

    FVehicleParameters Params;
    Params.Make            = Obj->GetStringField(TEXT("Make"));
    Params.Model           = Obj->GetStringField(TEXT("Model"));
    Params.NumberOfWheels  = Obj->GetIntegerField(TEXT("NumberOfWheels"));
    Params.Generation      = Obj->GetIntegerField(TEXT("Generation"));
    Params.ObjectType      = Obj->GetStringField(TEXT("ObjectType"));
    Params.BaseType        = Obj->GetStringField(TEXT("BaseType"));
    Params.SpecialType     = Obj->GetStringField(TEXT("SpecialType"));
    Obj->TryGetBoolField(TEXT("HasDynamicDoors"), Params.HasDynamicDoors);
    Obj->TryGetBoolField(TEXT("HasLights"),       Params.HasLights);

    const TArray<TSharedPtr<FJsonValue>> *ColorArr = nullptr;
    if (Obj->TryGetArrayField(TEXT("RecommendedColors"), ColorArr) && ColorArr)
      Params.RecommendedColors = ParseColors(*ColorArr);

    const TArray<TSharedPtr<FJsonValue>> *DriverArr = nullptr;
    if (Obj->TryGetArrayField(TEXT("SupportedDrivers"), DriverArr) && DriverArr)
      for (const auto &D : *DriverArr)
        Params.SupportedDrivers.Add(static_cast<int32>(D->AsNumber()));

    // Do NOT call LoadClass here — loading vehicle Blueprint classes triggers
    // Nanite skeletal mesh DDC builds which crash for some malformed assets.
    // Instead, store the class path and load lazily in SpawnActor().
    FString ClassPath;
    Obj->TryGetStringField(TEXT("Class"), ClassPath);

    // Use base class as placeholder so CheckActorDefinition passes;
    // SpawnActor() will replace it with the real class at spawn time.
    Params.Class = TSubclassOf<ACarlaWheeledVehicle>(ACarlaWheeledVehicle::StaticClass());

    FActorDefinition Def;
    bool Success = false;
    UActorBlueprintFunctionLibrary::MakeVehicleDefinition(Params, Success, Def);
    if (Success)
    {
      Definitions.Add(Def);
      ClassPaths_.Add(Def.Id, ClassPath);
    }
  }

  UE_LOG(LogCarla, Log,
      TEXT("NativeVehicleFactory: %d vehicle definitions registered (lazy class loading)"),
      Definitions.Num());

  return Definitions;
}

FActorSpawnResult ANativeVehicleFactory::SpawnActor(
    const FTransform &SpawnAtTransform,
    const FActorDescription &ActorDescription)
{
  auto *World = GetWorld();
  if (World == nullptr)
  {
    UE_LOG(LogCarla, Error,
        TEXT("NativeVehicleFactory: cannot spawn into an empty world."));
    return {};
  }

  // Load the real Blueprint class on demand.
  const FString *ClassPathPtr = ClassPaths_.Find(ActorDescription.Id);
  if (ClassPathPtr == nullptr || ClassPathPtr->IsEmpty())
  {
    UE_LOG(LogCarla, Error,
        TEXT("NativeVehicleFactory: no class path for '%s'."), *ActorDescription.Id);
    return {};
  }

  UClass *VehicleClass = LoadClass<ACarlaWheeledVehicle>(nullptr, **ClassPathPtr);
  if (VehicleClass == nullptr)
  {
    UE_LOG(LogCarla, Error,
        TEXT("NativeVehicleFactory: could not load class '%s'."), **ClassPathPtr);
    return {};
  }

  FActorSpawnParameters SpawnParams;
  SpawnParams.SpawnCollisionHandlingOverride =
      ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

  auto *Vehicle = World->SpawnActor<ACarlaWheeledVehicle>(
      VehicleClass, SpawnAtTransform, SpawnParams);

  // The legacy BP vehicle factory path created drivable vehicles with the
  // CARLA wheeled vehicle controller available. The native replacement must
  // preserve that or client control never gets flushed into the movement
  // component.
  if (Vehicle != nullptr)
  {
    Vehicle->AIControllerClass = AWheeledVehicleAIController::StaticClass();
    Vehicle->AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;
    if (Vehicle->GetController() == nullptr)
    {
      Vehicle->SpawnDefaultController();
    }

    auto *Controller = Cast<AWheeledVehicleAIController>(Vehicle->GetController());
    if (Controller != nullptr)
    {
      Controller->SetStickyControl(true);
      UE_LOG(LogCarla, Log,
          TEXT("NativeVehicleFactory: spawned %s with controller %s possessing=%s"),
          *Vehicle->GetName(),
          *Controller->GetName(),
          Controller->GetPossessedVehicle() == Vehicle ? TEXT("true") : TEXT("false"));
    }
    else
    {
      UE_LOG(LogCarla, Warning,
          TEXT("NativeVehicleFactory: spawned %s without wheeled vehicle controller"),
          *Vehicle->GetName());
    }
  }

  return FActorSpawnResult(Vehicle);
}
