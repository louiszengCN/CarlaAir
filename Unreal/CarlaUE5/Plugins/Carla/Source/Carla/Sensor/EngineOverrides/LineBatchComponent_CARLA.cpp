// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.


#include "Sensor/EngineOverrides/LineBatchComponent_CARLA.h"
#include "Sensor/CosmosControlSensor.h"
#include "ConstructorHelpers.h"
#include "DynamicMeshBuilder.h" // UE5: FDynamicMeshBuilder
#include "Misc/PackageName.h"
#include "UObject/SoftObjectPath.h"

namespace
{
	template <typename TObject>
	TObject* LoadOptionalObject(const TCHAR* ObjectPath)
	{
		const FSoftObjectPath SoftPath(ObjectPath);
		const FString PackageName = SoftPath.GetLongPackageName();
		if (PackageName.IsEmpty() || !FPackageName::DoesPackageExist(PackageName))
		{
			return nullptr;
		}
		return LoadObject<TObject>(nullptr, ObjectPath);
	}
}

FLineBatcherSceneProxy_CARLA::FLineBatcherSceneProxy_CARLA(const ULineBatchComponent_CARLA* InComponent) :
	// UE5: FLineBatcherSceneProxy is private; use FPrimitiveSceneProxy directly
	FPrimitiveSceneProxy(InComponent), Lines(InComponent->BatchedLines),
	Points(InComponent->BatchedPoints), Meshes(InComponent->BatchedMeshes)
{
	bWillEverBeLit = false;
}

FLineBatcherSceneProxy_CARLA::~FLineBatcherSceneProxy_CARLA()
{
	for (auto& Pair : CachedMaterialProxies)
	{
		delete Pair.Value;
	}
	CachedMaterialProxies.Empty();
}

ULineBatchComponent_CARLA::ULineBatchComponent_CARLA(const FObjectInitializer& ObjectInitializer /*= FObjectInitializer::Get()*/)
	: Super(ObjectInitializer)
{
	// CosmosMeshMaterial is loaded lazily in CreateSceneProxy() to avoid CDO-time
	// asset loading failures (package filesystem not ready during module startup).
}

FPrimitiveSceneProxy* ULineBatchComponent_CARLA::CreateSceneProxy()
{
	if (!CosmosMeshMaterial)
	{
		CosmosMeshMaterial = LoadOptionalObject<UMaterial>(
			TEXT("Material'/Carla/PostProcessingMaterials/DebugCosmosMeshMaterial.DebugCosmosMeshMaterial'"));
	}
	FLineBatcherSceneProxy_CARLA* proxy = new FLineBatcherSceneProxy_CARLA(this);
	proxy->CosmosMeshMaterial = CosmosMeshMaterial;

	return proxy;
}

SIZE_T FLineBatcherSceneProxy_CARLA::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}
/**
*  Returns a struct that describes to the renderer when to draw this proxy.
*	@param		Scene view to use to determine our relevence.
*  @return		View relevance struct
*/
FPrimitiveViewRelevance FLineBatcherSceneProxy_CARLA::GetViewRelevance(const FSceneView* View) const
{
	FPrimitiveViewRelevance ViewRelevance;

	ViewRelevance.bDrawRelevance = Cast<ACosmosControlSensor>(View->ViewActor.Get()) != nullptr; // UE5: ViewActor is FSceneViewOwner, use .Get() for raw pointer
	ViewRelevance.bDynamicRelevance = true;
	// ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
	ViewRelevance.bSeparateTranslucency = ViewRelevance.bNormalTranslucency = true;
	return ViewRelevance;
}

void FLineBatcherSceneProxy_CARLA::GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_LineBatcherSceneProxy_GetDynamicMeshElements);

	for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
	{
		if (VisibilityMap & (1 << ViewIndex))
		{
			const FSceneView* View = Views[ViewIndex];
			FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);

			for (int32 i = 0; i < Lines.Num(); i++)
			{
				FLinearColor LinearColor = FLinearColor(Lines[i].Color);
				PDI->DrawLine(Lines[i].Start, Lines[i].End, LinearColor, Lines[i].DepthPriority, Lines[i].Thickness, 0.0f, true);
			}

			for (int32 i = 0; i < Points.Num(); i++)
			{
				FLinearColor LinearColor = FLinearColor(Points[i].Color);
				PDI->DrawPoint(Points[i].Position, LinearColor, Points[i].PointSize, Points[i].DepthPriority);
			}

			for (int32 i = 0; i < Meshes.Num(); i++)
			{
				// UE5: DynamicMeshBuilder::AddVertex requires FVector3f/FVector2f
				static FVector3f const PosX(1.f, 0, 0);
				static FVector3f const PosY(0, 1.f, 0);
				static FVector3f const PosZ(0, 0, 1.f);

				FBatchedMesh const& M = Meshes[i];

				// this seems far from optimal in terms of perf, but it's for debugging
				FDynamicMeshBuilder MeshBuilder(View->GetFeatureLevel());

				// set up geometry
				for (int32 VertIdx = 0; VertIdx < M.MeshVerts.Num(); ++VertIdx)
				{
					// UE5: AddVertex now takes FVector3f/FVector2f; cast MeshVerts element (FVector→FVector3f)
					MeshBuilder.AddVertex(FVector3f(M.MeshVerts[VertIdx]), FVector2f::ZeroVector, PosX, PosY, PosZ, FColor::White);
				}
				//MeshBuilder.AddTriangles(M.MeshIndices);
				for (int32 Idx = 0; Idx < M.MeshIndices.Num(); Idx += 3)
				{
					MeshBuilder.AddTriangle(M.MeshIndices[Idx], M.MeshIndices[Idx + 1], M.MeshIndices[Idx + 2]);
				}

				// Create a unique key from the color
				uint32 ColorKey = M.Color.DWColor();
				FColoredMaterialRenderProxy** CachedProxy = CachedMaterialProxies.Find(ColorKey);
				FMaterialRenderProxy* MaterialRenderProxy = nullptr;
				
				if (CachedProxy)
				{
					MaterialRenderProxy = *CachedProxy;
				}
				else
				{
					// Create new proxy and cache it
					FMaterialRenderProxy* BaseMaterialProxy = CosmosMeshMaterial == nullptr ? GEngine->DebugMeshMaterial->GetRenderProxy() : CosmosMeshMaterial->GetRenderProxy();
					
					FLinearColor LinearColor = M.Color.ReinterpretAsLinear();
					FColoredMaterialRenderProxy* NewProxy = new FColoredMaterialRenderProxy(
						BaseMaterialProxy,
						LinearColor);
					
					CachedMaterialProxies.Add(ColorKey, NewProxy);
					MaterialRenderProxy = NewProxy;
				}
				
				MeshBuilder.GetMesh(FMatrix::Identity, MaterialRenderProxy, M.DepthPriority, false, false, ViewIndex, Collector);
			}
		}
	}
}
