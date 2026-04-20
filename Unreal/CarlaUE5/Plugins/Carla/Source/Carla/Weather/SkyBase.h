// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Components/ChildActorComponent.h"
#include "Components/PostProcessComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/DirectionalLight.h"
#include "Engine/StaticMeshActor.h"

#include "SkyBase.generated.h"

/// Compatibility parent for legacy CARLA sky blueprints.
UCLASS(BlueprintType, Blueprintable)
class CARLA_API ASkyBase : public AActor
{
  GENERATED_BODY()

public:

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  USceneComponent *SceneRoot = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  UDirectionalLightComponent *DirectionalLightComponentSun = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  UDirectionalLightComponent *DirectionalLightComponentMoon = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  USkyAtmosphereComponent *SkyAtmosphereComponent = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  UVolumetricCloudComponent *VolumetricCloudComponent = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  UExponentialHeightFogComponent *ExponentialHeightFogComponent = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  USkyLightComponent *SkyLightComponent = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  UPostProcessComponent *PostProcessComponent = nullptr;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Sky")
  ADirectionalLight *SunActor = nullptr;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Sky")
  ADirectionalLight *MoonActor = nullptr;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Sky")
  AStaticMeshActor *SkySphere = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  UChildActorComponent *SkySphereChildActor = nullptr;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA|Sky")
  UStaticMeshComponent *SkySphereMesh = nullptr;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Sky")
  ADirectionalLight *DirectionalLightActor = nullptr;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA|Sky")
  float CloudOpacity = 1.0f;

  ASkyBase(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
  {
    PrimaryActorTick.bCanEverTick = false;
    SceneRoot = ObjectInitializer.CreateDefaultSubobject<USceneComponent>(this, TEXT("RootComponent"));
    RootComponent = SceneRoot;
    DirectionalLightComponentSun = ObjectInitializer.CreateDefaultSubobject<UDirectionalLightComponent>(this, TEXT("DirectionalLightComponentSun"));
    DirectionalLightComponentSun->SetupAttachment(SceneRoot);
    DirectionalLightComponentMoon = ObjectInitializer.CreateDefaultSubobject<UDirectionalLightComponent>(this, TEXT("DirectionalLightComponentMoon"));
    DirectionalLightComponentMoon->SetupAttachment(SceneRoot);
    SkyAtmosphereComponent = ObjectInitializer.CreateDefaultSubobject<USkyAtmosphereComponent>(this, TEXT("SkyAtmosphereComponent"));
    SkyAtmosphereComponent->SetupAttachment(SceneRoot);
    VolumetricCloudComponent = ObjectInitializer.CreateDefaultSubobject<UVolumetricCloudComponent>(this, TEXT("VolumetricCloudComponent"));
    VolumetricCloudComponent->SetupAttachment(SceneRoot);
    ExponentialHeightFogComponent = ObjectInitializer.CreateDefaultSubobject<UExponentialHeightFogComponent>(this, TEXT("ExponentialHeightFogComponent"));
    ExponentialHeightFogComponent->SetupAttachment(SceneRoot);
    SkyLightComponent = ObjectInitializer.CreateDefaultSubobject<USkyLightComponent>(this, TEXT("SkyLightComponent"));
    SkyLightComponent->SetupAttachment(SceneRoot);
    PostProcessComponent = ObjectInitializer.CreateDefaultSubobject<UPostProcessComponent>(this, TEXT("PostProcessComponent"));
    PostProcessComponent->SetupAttachment(SceneRoot);
    SkySphereChildActor = ObjectInitializer.CreateDefaultSubobject<UChildActorComponent>(this, TEXT("SkySphereChildActor"));
    SkySphereChildActor->SetupAttachment(SceneRoot);
    SkySphereMesh = ObjectInitializer.CreateDefaultSubobject<UStaticMeshComponent>(this, TEXT("SkySphereMesh"));
    SkySphereMesh->SetupAttachment(SceneRoot);
    Tags.Add(TEXT("CARLA_SKY"));
  }

  UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = "CARLA|Sky")
  void DayTimeChangeEvent(bool bIsDay);
};