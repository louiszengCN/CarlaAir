// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Actor/WalkerActorFactory.h"

#include "Dom/JsonObject.h"
#include "JsonObjectConverter.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
  static const TCHAR *WalkerParamsArrayField = TEXT("WalkerParamsArray");

  static bool SaveWalkerParametersToJson(
      const FString &FileName,
      const TArray<FPedestrianParameters> &WalkerParamsArray)
  {
    TArray<TSharedPtr<FJsonValue>> JsonValues;
    for (const FPedestrianParameters &WalkerParams : WalkerParamsArray)
    {
      TSharedRef<FJsonObject> JsonObject = MakeShared<FJsonObject>();
      if (!FJsonObjectConverter::UStructToJsonObject(FPedestrianParameters::StaticStruct(), &WalkerParams, JsonObject, 0, 0))
      {
        return false;
      }
      JsonValues.Add(MakeShared<FJsonValueObject>(JsonObject));
    }

    TSharedRef<FJsonObject> RootObject = MakeShared<FJsonObject>();
    RootObject->SetArrayField(WalkerParamsArrayField, JsonValues);

    FString JsonString;
    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
    if (!FJsonSerializer::Serialize(RootObject, Writer))
    {
      return false;
    }

    return FFileHelper::SaveStringToFile(JsonString, *FileName, FFileHelper::EEncodingOptions::ForceUTF8);
  }

  static bool LoadWalkerParametersFromJson(
      const FString &FileName,
      TArray<FPedestrianParameters> &OutWalkerParamsArray)
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
    if (!RootObject->TryGetArrayField(WalkerParamsArrayField, JsonValues) || JsonValues == nullptr)
    {
      return false;
    }

    OutWalkerParamsArray.Reset();
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

      FPedestrianParameters WalkerParams;
      if (!FJsonObjectConverter::JsonObjectToUStruct(JsonObject.ToSharedRef(), FPedestrianParameters::StaticStruct(), &WalkerParams, 0, 0))
      {
        return false;
      }

      OutWalkerParamsArray.Add(MoveTemp(WalkerParams));
    }

    return true;
  }
}

bool AWalkerActorFactory::SaveWalkerParametersArrayToFile(
    const FString &FileName,
    const TArray<FPedestrianParameters> &WalkerParamsArray)
{
  return SaveWalkerParametersToJson(FileName, WalkerParamsArray);
}

bool AWalkerActorFactory::LoadWalkerParametersArrayFromFile(
    const FString &FileName,
    TArray<FPedestrianParameters> &OutWalkerParamsArray)
{
  return LoadWalkerParametersFromJson(FileName, OutWalkerParamsArray);
}