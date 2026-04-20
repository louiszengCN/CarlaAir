// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Actor/VehicleActorFactory.h"

#include "Dom/JsonObject.h"
#include "JsonObjectConverter.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
  static const TCHAR *VehicleParamsArrayField = TEXT("VehicleParamsArray");

  static bool SaveVehicleParametersToJson(
      const FString &FileName,
      const TArray<FVehicleParameters> &VehicleParamsArray)
  {
    TArray<TSharedPtr<FJsonValue>> JsonValues;
    for (const FVehicleParameters &VehicleParams : VehicleParamsArray)
    {
      TSharedRef<FJsonObject> JsonObject = MakeShared<FJsonObject>();
      if (!FJsonObjectConverter::UStructToJsonObject(FVehicleParameters::StaticStruct(), &VehicleParams, JsonObject, 0, 0))
      {
        return false;
      }
      JsonValues.Add(MakeShared<FJsonValueObject>(JsonObject));
    }

    TSharedRef<FJsonObject> RootObject = MakeShared<FJsonObject>();
    RootObject->SetArrayField(VehicleParamsArrayField, JsonValues);

    FString JsonString;
    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
    if (!FJsonSerializer::Serialize(RootObject, Writer))
    {
      return false;
    }

    return FFileHelper::SaveStringToFile(JsonString, *FileName, FFileHelper::EEncodingOptions::ForceUTF8);
  }

  static bool LoadVehicleParametersFromJson(
      const FString &FileName,
      TArray<FVehicleParameters> &OutVehicleParamsArray)
  {
    FString JsonString;
    if (!FFileHelper::LoadFileToString(JsonString, *FileName))
    {
      return false;
    }

    TSharedPtr<FJsonObject> RootObject;
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
    if (!FJsonSerializer::Deserialize(Reader, RootObject) || !RootObject.IsValid())
    {
      return false;
    }

    const TArray<TSharedPtr<FJsonValue>> *JsonValues = nullptr;
    if (!RootObject->TryGetArrayField(VehicleParamsArrayField, JsonValues) || JsonValues == nullptr)
    {
      return false;
    }

    OutVehicleParamsArray.Reset();
    for (const TSharedPtr<FJsonValue> &JsonValue : *JsonValues)
    {
      if (!JsonValue.IsValid())
      {
        return false;
      }

      const TSharedPtr<FJsonObject> JsonObject = JsonValue->AsObject();
      if (!JsonObject.IsValid())
      {
        return false;
      }

      FVehicleParameters VehicleParams;
      if (!FJsonObjectConverter::JsonObjectToUStruct(JsonObject.ToSharedRef(), FVehicleParameters::StaticStruct(), &VehicleParams, 0, 0))
      {
        return false;
      }

      OutVehicleParamsArray.Add(MoveTemp(VehicleParams));
    }

    return true;
  }
}

bool AVehicleActorFactory::SaveVehicleParametersArrayToFile(
    const FString &FileName,
    const TArray<FVehicleParameters> &VehicleParamsArray)
{
  return SaveVehicleParametersToJson(FileName, VehicleParamsArray);
}

bool AVehicleActorFactory::LoadVehicleParametersArrayFromFile(
    const FString &FileName,
    TArray<FVehicleParameters> &OutVehicleParamsArray)
{
  return LoadVehicleParametersFromJson(FileName, OutVehicleParamsArray);
}