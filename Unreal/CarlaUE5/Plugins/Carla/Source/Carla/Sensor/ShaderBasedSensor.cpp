// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/ShaderBasedSensor.h"

#include "ConstructorHelpers.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Actor/ActorBlueprintFunctionLibrary.h"
#include "Misc/PackageName.h"

namespace
{
  bool CarlaPackageExists(const FString &ObjectPath)
  {
    const FSoftObjectPath SoftPath(ObjectPath);
    const FString PackageName = SoftPath.GetLongPackageName();
    return !PackageName.IsEmpty() && FPackageName::DoesPackageExist(PackageName);
  }
}

bool AShaderBasedSensor::AddPostProcessingMaterial(const FString &Path)
{
  // During CDO construction the plugin content directory is not yet mounted, so
  // DoesPackageExist returns false for plugin assets.  Skip silently — the
  // per-instance constructor (called at actual spawn time) will load correctly.
  if (IsTemplate())
    return true;

  UMaterial *LoadedMaterial = nullptr;
  if (CarlaPackageExists(Path))
  {
    LoadedMaterial = LoadObject<UMaterial>(nullptr, *Path);
  }
  if (LoadedMaterial != nullptr)
  {
    MaterialsFound.Add(LoadedMaterial);
    return true;
  }

  UE_LOG(LogCarla, Warning, TEXT("Post-process material not found: %s"), *Path);
  return false;
}

void AShaderBasedSensor::SetUpSceneCaptureComponent(USceneCaptureComponent2D &SceneCapture)
{
  for (const auto &MaterialFound : MaterialsFound)
  {
    // Create a dynamic instance of the Material (Shader)
    AddShader({UMaterialInstanceDynamic::Create(MaterialFound, this), 1.0});
  }

  for (const auto &Shader : Shaders)
  {
    // Attach the instance into the blendables
    SceneCapture.PostProcessSettings.AddBlendable(Shader.PostProcessMaterial, Shader.Weight);
  }

  // Set the value for each Float parameter in the shader
  for (const auto &ParameterValue : FloatShaderParams)
  {
    Shaders[ParameterValue.ShaderIndex].PostProcessMaterial->SetScalarParameterValue(
        ParameterValue.ParameterName,
        ParameterValue.Value);
  }
}

void AShaderBasedSensor::Set(const FActorDescription &Description)
{
  Super::Set(Description);
  UActorBlueprintFunctionLibrary::SetCamera(Description, this);
}

void AShaderBasedSensor::SetFloatShaderParameter(
    uint8_t ShaderIndex,
    const FName &ParameterName,
    float Value)
{
  FloatShaderParams.Add({ShaderIndex, ParameterName, Value});
}
