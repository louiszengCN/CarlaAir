// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Actor/PropActorFactory.h"

#include "Dom/JsonObject.h"
#include "JsonObjectConverter.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
  static const TCHAR *PropParamsArrayField = TEXT("PropParamsArray");

  static bool SavePropParametersToJson(
      const FString &FileName,
      const TArray<FPropParameters> &PropParamsArray)
  {
    TArray<TSharedPtr<FJsonValue>> JsonValues;
    for (const FPropParameters &PropParams : PropParamsArray)
    {
      TSharedRef<FJsonObject> JsonObject = MakeShared<FJsonObject>();
      if (!FJsonObjectConverter::UStructToJsonObject(FPropParameters::StaticStruct(), &PropParams, JsonObject, 0, 0))
      {
        return false;
      }
      JsonValues.Add(MakeShared<FJsonValueObject>(JsonObject));
    }

    TSharedRef<FJsonObject> RootObject = MakeShared<FJsonObject>();
    RootObject->SetArrayField(PropParamsArrayField, JsonValues);

    FString JsonString;
    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
    if (!FJsonSerializer::Serialize(RootObject, Writer))
    {
      return false;
    }

    return FFileHelper::SaveStringToFile(JsonString, *FileName, FFileHelper::EEncodingOptions::ForceUTF8);
  }

  static bool LoadPropParametersFromJson(
      const FString &FileName,
      TArray<FPropParameters> &OutPropParamsArray)
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
    if (!RootObject->TryGetArrayField(PropParamsArrayField, JsonValues) || JsonValues == nullptr)
    {
      return false;
    }

    OutPropParamsArray.Reset();
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

      FPropParameters PropParams;
      if (!FJsonObjectConverter::JsonObjectToUStruct(JsonObject.ToSharedRef(), FPropParameters::StaticStruct(), &PropParams, 0, 0))
      {
        return false;
      }

      OutPropParamsArray.Add(MoveTemp(PropParams));
    }

    return true;
  }
}

bool APropActorFactory::SavePropParametersArrayToFile(
    const FString &FileName,
    const TArray<FPropParameters> &PropParamsArray)
{
  return SavePropParametersToJson(FileName, PropParamsArray);
}

bool APropActorFactory::LoadPropParametersArrayFromFile(
    const FString &FileName,
    TArray<FPropParameters> &OutPropParamsArray)
{
  return LoadPropParametersFromJson(FileName, OutPropParamsArray);
}