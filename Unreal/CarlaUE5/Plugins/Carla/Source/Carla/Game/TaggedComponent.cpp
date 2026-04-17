#include "Carla.h"
#include "TaggedComponent.h"
#include "TaggedMaterials.h"

#include "Rendering/SkeletalMeshRenderData.h"
#include "SkeletalRenderPublic.h"
#include "GPUSkinVertexFactory.h" // UE5: FGPUBaseSkinVertexFactory::GetMaxGPUSkinBones()

//
// UTaggedComponent
//
UTaggedComponent::UTaggedComponent(const FObjectInitializer& ObjectInitializer) :
  UPrimitiveComponent(ObjectInitializer),
  Color(1, 1, 1, 1)
{
  PrimaryComponentTick.bCanEverTick = true;
  PrimaryComponentTick.bStartWithTickEnabled = false;
}

void UTaggedComponent::OnRegister()
{
  Super::OnRegister();

  TaggedMID = UTaggedMaterialsRegistry::Get()->GetTaggedMaterial();

  if (!IsValid(TaggedMID))
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to create MID!"));
  }

  if(USceneComponent* ParentSceneComponent = GetAttachParent()) {
    UPrimitiveComponent* ParentComponent = CastChecked<UPrimitiveComponent>(ParentSceneComponent);
    TArray<UMaterialInterface*> UsedMaterials;
    ParentComponent->GetUsedMaterials(UsedMaterials);
    for (UMaterialInterface* UsedMaterial : UsedMaterials) {
      UMaterialInstanceDynamic* TaggedMaterial = UTaggedMaterialsRegistry::Get()->GetTaggedMaterial(UsedMaterial);
      if (TaggedMaterial) {
        TaggedMaterials.Add(UsedMaterial, TaggedMaterial);
      }
    }
  }

  SetColor(Color);
}

void UTaggedComponent::SetColor(FLinearColor NewColor)
{
  Color = NewColor;

  if (IsValid(TaggedMID))
  {
    TaggedMID->SetVectorParameterValue("AnnotationColor", Color);
  }

  for (auto& Pair : TaggedMaterials) {
    Pair.Value->SetVectorParameterValue("AnnotationColor", Color);
  }
}

FLinearColor UTaggedComponent::GetColor()
{
  return Color;
}

TArray<UMaterialInstanceDynamic*> UTaggedComponent::GetTaggedMaterials()
{
  TArray<UMaterialInstanceDynamic*> Ret;
  for (auto& Pair : TaggedMaterials) {
    Ret.Add(Pair.Value);
  }
  return Ret;
}

FBoxSphereBounds UTaggedComponent::CalcBounds(const FTransform & LocalToWorld) const
{
  USceneComponent * ParentComponent = GetAttachParent();
  if(ParentComponent)
  {
    return ParentComponent->CalcBounds(LocalToWorld);
  }
  return FBoxSphereBounds();
}

FPrimitiveSceneProxy * UTaggedComponent::CreateSceneProxy()
{
  USceneComponent * ParentComponent = GetAttachParent();

  if (!IsValid(ParentComponent))
  {
    UE_LOG(LogCarla, Error, TEXT("Invalid parent component"));
    return nullptr;
  }

  USkeletalMeshComponent * SkeletalMeshComponent = Cast<USkeletalMeshComponent>(ParentComponent);
  UStaticMeshComponent * StaticMeshComponent = Cast<UStaticMeshComponent>(ParentComponent);
  UHierarchicalInstancedStaticMeshComponent * HierarchicalInstancedStaticMeshComponent =
      Cast<UHierarchicalInstancedStaticMeshComponent>(ParentComponent);
  UInstancedStaticMeshComponent* InstancedStaticMeshComponent = Cast<UInstancedStaticMeshComponent>(ParentComponent);
  if (HierarchicalInstancedStaticMeshComponent)
  {
    return CreateSceneProxy(HierarchicalInstancedStaticMeshComponent);
  }
  else if (InstancedStaticMeshComponent)
  {
    return CreateSceneProxy(InstancedStaticMeshComponent);
  }
  else
  if (IsValid(StaticMeshComponent))
  {
    return CreateSceneProxy(StaticMeshComponent);
  }
  else if (IsValid(SkeletalMeshComponent))
  {
    bSkeletalMesh = true;
    return CreateSceneProxy(SkeletalMeshComponent);
  }

  UE_LOG(LogCarla, Error, TEXT("Unknown type of parent component: %s"), *ParentComponent->GetClass()->GetName());

  return nullptr;
}

FPrimitiveSceneProxy * UTaggedComponent::CreateSceneProxy(UStaticMeshComponent * StaticMeshComponent)
{
  // Make sure static mesh has render data
  UStaticMesh * StaticMesh = StaticMeshComponent->GetStaticMesh();

  if (StaticMesh == nullptr)
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to create scene proxy for static mesh component (because static mesh is null): %s"), *StaticMeshComponent->GetReadableName());
    return nullptr;
  }

  if (StaticMesh->GetRenderData() == nullptr) // UE5: RenderData private
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to create scene proxy for static mesh component (because render data is null): %s"), *StaticMeshComponent->GetReadableName());
    return nullptr;
  }


  if (StaticMesh->GetRenderData()->LODResources.Num() == 0) // UE5: GetRenderData()
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to create scene proxy for static mesh component (because num LOD resources is 0): %s"), *StaticMeshComponent->GetReadableName());
    return nullptr;
  }

  // UE5: FSplineMeshSceneProxy is final in UE5.7 and cannot be subclassed.
  // Spline mesh components fall back to FTaggedStaticMeshSceneProxy which skips spline
  // deformation — road/guardrail splines will render flat/undeformed.
  // TODO(#UE5-SPLINE-TAG): restore per-spline tagging once UE5 exposes a non-final proxy.
  return new FTaggedStaticMeshSceneProxy(StaticMeshComponent, true, TaggedMID, TaggedMaterials);
}

FPrimitiveSceneProxy * UTaggedComponent::CreateSceneProxy(USkeletalMeshComponent * SkeletalMeshComponent)
{
  if (bShouldWaitFrame)
  {
    return nullptr;
  }
  ERHIFeatureLevel::Type SceneFeatureLevel = GetWorld()->FeatureLevel;
	FSkeletalMeshRenderData* SkelMeshRenderData = SkeletalMeshComponent->GetSkeletalMeshRenderData();

	// Only create a scene proxy for rendering if properly initialized
	if (SkelMeshRenderData &&
		SkelMeshRenderData->LODRenderData.IsValidIndex(SkeletalMeshComponent->PredictedLODLevel) &&
		!SkeletalMeshComponent->bHideSkin &&
		SkeletalMeshComponent->MeshObject)
	{
		// UE5: GetFeatureLevelMaxNumberOfBones() removed; use FGPUBaseSkinVertexFactory::GetMaxGPUSkinBones()
		int32 MinLODIndex = SkeletalMeshComponent->ComputeMinLOD();
		int32 MaxBonesPerChunk = SkelMeshRenderData->GetMaxBonesPerSection(MinLODIndex);
		const bool bCPUSkinned = SkeletalMeshComponent->MeshObject->IsCPUSkinned();
		const int32 MaxSupportedNumBones = bCPUSkinned ? MAX_int32 : FGPUBaseSkinVertexFactory::GetMaxGPUSkinBones();
		if (MaxBonesPerChunk <= MaxSupportedNumBones)
		{
			return new FTaggedSkeletalMeshSceneProxy(SkeletalMeshComponent, SkelMeshRenderData, TaggedMID, TaggedMaterials);
		}
	}
  return nullptr;
}

FPrimitiveSceneProxy * UTaggedComponent::CreateSceneProxy(UHierarchicalInstancedStaticMeshComponent * MeshComponent)
{
	// Verify that the mesh is valid before using it.
	const bool bMeshIsValid =
		// make sure we have an actual staticmesh
		MeshComponent->GetStaticMesh() &&
		MeshComponent->GetStaticMesh()->HasValidRenderData(false); // UE5: PerInstanceRenderData removed from public API

	if (bMeshIsValid)
	{
		bool bIsGrass = !MeshComponent->PerInstanceSMData.Num();
		return new FTaggedHierarchicalStaticMeshSceneProxy(MeshComponent, bIsGrass, GetWorld()->FeatureLevel, TaggedMID, TaggedMaterials);
	}
	return nullptr;
}

FPrimitiveSceneProxy * UTaggedComponent::CreateSceneProxy(UInstancedStaticMeshComponent * MeshComponent)
{
	// Verify that the mesh is valid before using it.
	const bool bMeshIsValid =
		// make sure we have an actual staticmesh
		MeshComponent->GetStaticMesh() &&
		MeshComponent->GetStaticMesh()->HasValidRenderData(false); // UE5: PerInstanceRenderData removed from public API

	if (bMeshIsValid)
	{
		return new FTaggedInstancedStaticMeshSceneProxy(MeshComponent, GetWorld()->FeatureLevel, TaggedMID, TaggedMaterials);
	}
	return nullptr;
}

void UTaggedComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction * ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

  // // TODO: Try removing this
  if (bSkeletalMesh)
  {
    // MarkRenderTransformDirty();
    MarkRenderStateDirty();
    if(bShouldWaitFrame)
    {
      if(NumFramesToWait < 0)
      {
        bShouldWaitFrame = false;
      }
      NumFramesToWait--;
    }
  }
}

//
// FTaggedStaticMeshSceneProxy
//
FTaggedStaticMeshSceneProxy::FTaggedStaticMeshSceneProxy(UStaticMeshComponent * Component, bool bForceLODsShareStaticLighting, UMaterialInstance * MaterialInstance, TMap<UMaterialInterface*, UMaterialInstanceDynamic*> TaggedMaterials) :
  FStaticMeshSceneProxy(Component, bForceLODsShareStaticLighting)
{
  TaggedMaterialInstance = MaterialInstance;

  // Replace materials with tagged material
  bVerifyUsedMaterials = false;

  for (FLODInfo& LODInfo : LODs) {
    for (FLODInfo::FSectionInfo& SectionInfo : LODInfo.Sections) {
      UMaterialInstanceDynamic** TaggedMaterial = TaggedMaterials.Find(SectionInfo.Material);
      if (TaggedMaterial) {
        SectionInfo.Material = *TaggedMaterial;
      } else {
        SectionInfo.Material = TaggedMaterialInstance;
      }
    }
  }
}

FPrimitiveViewRelevance FTaggedStaticMeshSceneProxy::GetViewRelevance(const FSceneView * View) const
{
  FPrimitiveViewRelevance ViewRelevance = FStaticMeshSceneProxy::GetViewRelevance(View);

  // UE5: NotDrawTaggedComponents was a CARLA engine patch, unavailable in stock UE5.7.
  // Tagged components now draw in ALL views — semantic segmentation isolation may break.
  // TODO(#UE5-NOTAG-FLAG): re-add engine patch or use a custom FEngineShowFlags extension.
  ViewRelevance.bShadowRelevance = false;

  return ViewRelevance;
}

// UE5: FTaggedSplineMeshSceneProxy removed — FSplineMeshSceneProxy is final in UE5.7
// Spline meshes use FTaggedStaticMeshSceneProxy instead.

//
// FTaggedSkeletalMeshSceneProxy
//
FTaggedSkeletalMeshSceneProxy::FTaggedSkeletalMeshSceneProxy(const USkinnedMeshComponent * Component, FSkeletalMeshRenderData * InSkeletalMeshRenderData, UMaterialInstance * MaterialInstance, TMap<UMaterialInterface*, UMaterialInstanceDynamic*> TaggedMaterials) :
  FSkeletalMeshSceneProxy(Component, InSkeletalMeshRenderData)
{
  TaggedMaterialInstance = MaterialInstance;

  // Replace materials with tagged material
  // UE5: bVerifyUsedMaterials not in FSkeletalMeshSceneProxy

  for (FLODSectionElements& LODSection : LODSections) {
    for (FSectionElementInfo& ElementInfo : LODSection.SectionElements) {
      UMaterialInstanceDynamic** TaggedMaterial = TaggedMaterials.Find(ElementInfo.Material);
      if (TaggedMaterial) {
        ElementInfo.Material = *TaggedMaterial;
      } else {
        ElementInfo.Material = TaggedMaterialInstance;
      }
    }
  }
}

FPrimitiveViewRelevance FTaggedSkeletalMeshSceneProxy::GetViewRelevance(const FSceneView * View) const
{
  FPrimitiveViewRelevance ViewRelevance = FSkeletalMeshSceneProxy::GetViewRelevance(View);

  // UE5: NotDrawTaggedComponents was a CARLA engine patch, unavailable in stock UE5.7.
  // Tagged components now draw in ALL views — semantic segmentation isolation may break.
  // TODO(#UE5-NOTAG-FLAG): re-add engine patch or use a custom FEngineShowFlags extension.
  ViewRelevance.bShadowRelevance = false;

  return ViewRelevance;
}

FTaggedInstancedStaticMeshSceneProxy::FTaggedInstancedStaticMeshSceneProxy(
    UInstancedStaticMeshComponent * Component, ERHIFeatureLevel::Type InFeatureLevel, UMaterialInstance * MaterialInstance, TMap<UMaterialInterface*, UMaterialInstanceDynamic*> TaggedMaterials)
  : FInstancedStaticMeshSceneProxy(Component, InFeatureLevel)
{
  TaggedMaterialInstance = MaterialInstance;

  // Replace materials with tagged material
  bVerifyUsedMaterials = false;

  for (FLODInfo& LODInfo : LODs) {
    for (FLODInfo::FSectionInfo& SectionInfo : LODInfo.Sections) {
      UMaterialInstanceDynamic** TaggedMaterial = TaggedMaterials.Find(SectionInfo.Material);
      if (TaggedMaterial) {
        SectionInfo.Material = *TaggedMaterial;
      } else {
        SectionInfo.Material = TaggedMaterialInstance;
      }
    }
  }
}

FPrimitiveViewRelevance FTaggedInstancedStaticMeshSceneProxy::GetViewRelevance(const FSceneView * View) const
{
  FPrimitiveViewRelevance ViewRelevance = FInstancedStaticMeshSceneProxy::GetViewRelevance(View);

  // UE5: NotDrawTaggedComponents was a CARLA engine patch, unavailable in stock UE5.7.
  // Tagged components now draw in ALL views — semantic segmentation isolation may break.
  // TODO(#UE5-NOTAG-FLAG): re-add engine patch or use a custom FEngineShowFlags extension.
  ViewRelevance.bShadowRelevance = false;

  return ViewRelevance;
}


FTaggedHierarchicalStaticMeshSceneProxy::FTaggedHierarchicalStaticMeshSceneProxy(
    UHierarchicalInstancedStaticMeshComponent * Component, bool bInIsGrass, ERHIFeatureLevel::Type InFeatureLevel, UMaterialInstance * MaterialInstance, TMap<UMaterialInterface*, UMaterialInstanceDynamic*> TaggedMaterials)
  : FInstancedStaticMeshSceneProxy(Component, InFeatureLevel) // UE5: FHierarchicalStaticMeshSceneProxy is final; fall back to instanced proxy
{
  TaggedMaterialInstance = MaterialInstance;

  // Replace materials with tagged material
  bVerifyUsedMaterials = false;

  for (FLODInfo& LODInfo : LODs) {
    for (FLODInfo::FSectionInfo& SectionInfo : LODInfo.Sections) {
      UMaterialInstanceDynamic** TaggedMaterial = TaggedMaterials.Find(SectionInfo.Material);
      if (TaggedMaterial) {
        SectionInfo.Material = *TaggedMaterial;
      } else {
        SectionInfo.Material = TaggedMaterialInstance;
      }
    }
  }
}

FPrimitiveViewRelevance FTaggedHierarchicalStaticMeshSceneProxy::GetViewRelevance(const FSceneView * View) const
{
  FPrimitiveViewRelevance ViewRelevance = FInstancedStaticMeshSceneProxy::GetViewRelevance(View); // UE5: base changed to FInstancedStaticMeshSceneProxy

  // UE5: NotDrawTaggedComponents was a CARLA engine patch, unavailable in stock UE5.7.
  // Tagged components now draw in ALL views — semantic segmentation isolation may break.
  // TODO(#UE5-NOTAG-FLAG): re-add engine patch or use a custom FEngineShowFlags extension.
  ViewRelevance.bShadowRelevance = false;

  return ViewRelevance;
}

// Specific code for tagging landscapes (terrain)

// Map each landscape in the world to a new Guid, which will be used for the tagged scene proxies
static TMap<FGuid, FGuid> TaggedLandscapeGuidsMap;

FBoxSphereBounds UTaggedLandscapeComponent::CalcBounds(const FTransform & LocalToWorld) const
{
  USceneComponent * ParentComponent = GetAttachParent();
  if(ParentComponent)
  {
    return ParentComponent->CalcBounds(LocalToWorld);
  }
  return FBoxSphereBounds();
}

FPrimitiveSceneProxy * UTaggedLandscapeComponent::CreateSceneProxy()
{
  USceneComponent * ParentComponent = GetAttachParent();

  if (!IsValid(ParentComponent))
  {
    UE_LOG(LogCarla, Error, TEXT("Invalid parent component"));
    return nullptr;
  }
  ULandscapeComponent* LandscapeComponent = Cast<ULandscapeComponent>(ParentComponent);

  if (!IsValid(LandscapeComponent))
  {
    UE_LOG(LogCarla, Error, TEXT("UTaggedLandscapeComponent falsely attached to parent component of type: %s"), *ParentComponent->GetClass()->GetName());
    return nullptr;
  }

  // Unreal's landscape rendering code contains lots of logic to cache SceneProxies, so that a ULandscapeComponent essentially has one fixed SceneProxy.
  // Since we want to create a second SceneProxy for the given ULandscapeComponent, we modify the LandscapeGuid before creating the SceneProxy
  // and reset it afterwards. This results in a new FLandscapeKey in the SceneProxy, again resulting in a new LandscapeRenderingSystem.
  // With this small "hack", the secondary SceneProxies are rendered correctly.
  FGuid OriginalFGuid = LandscapeComponent->GetLandscapeProxy()->GetLandscapeGuid();
  FGuid NewFGuid = TaggedLandscapeGuidsMap.FindOrAdd(OriginalFGuid, FGuid::NewGuid());

  LandscapeComponent->GetLandscapeProxy()->SetLandscapeGuid(NewFGuid);
  FTaggedLandscapeComponentSceneProxy* SceneProxyTemp = new FTaggedLandscapeComponentSceneProxy(LandscapeComponent);
  // Check if the annotation material was actually set, otherwise fall back to invisible terrain (behavior of <=0.9.15)
  // Note: This should not happen, only if .umap and .uasset files are missing to some unknown reasons.
  if (!SceneProxyTemp->TaggedLandscapeMaterialInstance)
  {
    return nullptr;
  }
  // Create pointer to SceneProxy's material here (in game-thread object), to prevent material being garbage collected
  TaggedLMIC = SceneProxyTemp->TaggedLandscapeMaterialInstance;
  LandscapeComponent->GetLandscapeProxy()->SetLandscapeGuid(OriginalFGuid);
  return SceneProxyTemp;
}

FTaggedLandscapeComponentSceneProxy::FTaggedLandscapeComponentSceneProxy(ULandscapeComponent * Component)
  : FLandscapeComponentSceneProxy(Component)
{
  // The following code loads an annotated landscape from a dedicated level (.umap), retrieves the material
  // (an ULandscapeMaterialInstanceConstant) and overwrites the materials of this SceneProxy with this
  // annotation material. Without the material being already assigned to a landscape when building CARLA,
  // the required shaders for Landscapes are not cooked, therefore this procedure is taken.
  // This code is part of the SceneProxy (and not the UTaggedLandscapeComponent), because otherwise CARLA
  // may crash during startup since the loading of a UWorld seem to happen too early, having parts of the engine
  // not initialized yet. The SceneProxies are created much later, thus not leading to this crash.

  // Find and load the world "AnnotationColorLandscape" containing a landscape with annotation material
  if (!LandscapeAnnotationWorldPath.IsValid())
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to find AnnotationColorLandscape world."));
    return;
  }
  UObject* LandscapeAnnotationObject = LandscapeAnnotationWorldPath.ResolveObject();
  if (!LandscapeAnnotationObject)
  {
    LandscapeAnnotationObject = LandscapeAnnotationWorldPath.TryLoad();
    if (!LandscapeAnnotationObject)
    {
      UE_LOG(LogCarla, Error, TEXT("Failed to load AnnotationColorLandscape world."));
      return;
    }
  }
  UWorld* LandscapeAnnotationWorld = CastChecked<UWorld>(LandscapeAnnotationObject);

  // Find the landscape actor
  ALandscape* AnnotationLandscape = nullptr;
  for (AActor* Actor : LandscapeAnnotationWorld->PersistentLevel->Actors)
  {
    if (Actor && Actor->IsA(ALandscape::StaticClass()))
    {
      AnnotationLandscape = CastChecked<ALandscape>(Actor);
      break;
    }
  }
  if (!AnnotationLandscape)
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to find annotated landscape in AnnotationColorLandscape world."));
    return;
  }

  // Get and check the texture
  TaggedLandscapeMaterialInstance = Cast<ULandscapeMaterialInstanceConstant>(AnnotationLandscape->LandscapeComponents[0]->MaterialInstances[0]);
  if (!TaggedLandscapeMaterialInstance)
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to find annotated material on landscape in AnnotationColorLandscape world."));
    return;
  }

  // Replace materials with tagged material
  bVerifyUsedMaterials = false;
  for (int32 i = 0; i < AvailableMaterials.Num(); ++i)
  {
    AvailableMaterials[i] = TaggedLandscapeMaterialInstance->GetRenderProxy(); // UE5: AvailableMaterials is TArray<FMaterialRenderProxy*>
  }
}

FPrimitiveViewRelevance FTaggedLandscapeComponentSceneProxy::GetViewRelevance(const FSceneView * View) const
{
  FPrimitiveViewRelevance ViewRelevance = FLandscapeComponentSceneProxy::GetViewRelevance(View);

  // UE5: NotDrawTaggedComponents was a CARLA engine patch, unavailable in stock UE5.7.
  // Tagged components now draw in ALL views — semantic segmentation isolation may break.
  // TODO(#UE5-NOTAG-FLAG): re-add engine patch or use a custom FEngineShowFlags extension.
  ViewRelevance.bShadowRelevance = false;

  return ViewRelevance;
}
