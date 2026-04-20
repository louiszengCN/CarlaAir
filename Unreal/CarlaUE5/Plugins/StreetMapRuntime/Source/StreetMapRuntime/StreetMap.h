#pragma once
#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Components/MeshComponent.h"
#include "GameFramework/Actor.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstance.h"
#include "StreetMap.generated.h"

/* ──────────────────────────────────────────────────────────────────────────
 * StreetMap stub plugin (UE5 migration).
 *
 * Module name is "StreetMapRuntime" so that the package path
 * /Script/StreetMapRuntime.* matches what the CarlaTools Blueprint .uasset
 * files serialised when they were originally compiled against the CARLA UE4
 * StreetMap plugin fork.
 *
 * All classes/structs are no-ops; actual map import goes via LibCarla/OpenDrive.
 *
 * Field and parameter names MUST match the original plugin exactly — the
 * Blueprint asset serialises struct-break pin names by C++ property name.
 * ─────────────────────────────────────────────────────────────────────── */

// ── Node (road vertex) ───────────────────────────────────────────────────────
USTRUCT(BlueprintType)
struct STREETMAPRUNTIME_API FStreetMapNode
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    FVector2D Location = FVector2D::ZeroVector;
};

// ── Road ────────────────────────────────────────────────────────────────────
USTRUCT(BlueprintType)
struct STREETMAPRUNTIME_API FStreetMapRoad
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    FString RoadName;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<FStreetMapNode> Nodes;
};

// ── Building  ────────────────────────────────────────────────────────────────
// Pin names on Blueprint "Break FStreetMapBuilding" nodes are derived from
// UPROPERTY names (CamelCase split by capital letters). Names below match the
// original CARLA UE4 plugin exactly so existing Blueprint wiring resolves correctly.
//
// Basic geometric fields match ue4plugins/StreetMap original.
// CARLA-extended fields (BuildingStyleHolder, Doors, etc.) are stubbed with
// closest UE5 C++ types so the Break/Make node pins exist and connections load.
USTRUCT(BlueprintType)
struct STREETMAPRUNTIME_API FStreetMapBuilding
{
    GENERATED_BODY()

    /** Name of the building (may be empty) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    FString BuildingName;

    /** OSM building:use or building tag value */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    FString BuildingCategory;

    /** 2-D footprint vertices (local space, cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<FVector2D> BuildingPoints;

    /** Total height in cm */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float Height = 0.f;

    /** Number of floors / levels */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    int32 BuildingLevels = 0;

    /** Per-level height factor (cm/floor) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float BuildingLevelFloorFactor = 300.f;

    // ── Individual float bounds (pin names: "Bounds Min X" etc.) ──────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float BoundsMinX = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float BoundsMinY = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float BoundsMaxX = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float BoundsMaxY = 0.f;

    // ── Vector2D bounds (pin names: "Bounds Min" / "Bounds Max") ──────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    FVector2D BoundsMin = FVector2D::ZeroVector;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    FVector2D BoundsMax = FVector2D::ZeroVector;

    // ── CARLA-extended fields — stub types to preserve Blueprint pin wiring ─
    /** Building style data asset (stubbed as generic UObject) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    UObject* BuildingStyleHolder = nullptr;

    /** Door meshes */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<UStaticMesh*> Doors;

    /** Window meshes */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<UStaticMesh*> Windows;

    /** Roof slope meshes */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<UStaticMesh*> Slopes;

    /** Flat plane/ceiling mesh */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    UStaticMesh* PlaneMesh = nullptr;

    /** Building material instances */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<UMaterialInstance*> MaterialInstances;

    /** Minimum building height filter */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float MinHeight = 0.f;

    /** Maximum building height filter */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float MaxHeight = 0.f;

    /** Minimum building floor area filter */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float MinArea = 0.f;

    /** Maximum building floor area filter */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float MaxArea = 0.f;
};

// ── UStreetMap data asset ────────────────────────────────────────────────────
UCLASS(BlueprintType)
class STREETMAPRUNTIME_API UStreetMap : public UDataAsset
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<FStreetMapRoad> Roads;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<FStreetMapBuilding> Buildings;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    TArray<FStreetMapNode> Nodes;
};

// ── Import settings ──────────────────────────────────────────────────────────
USTRUCT(BlueprintType)
struct STREETMAPRUNTIME_API FStreetMapImportSettings
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    float RoadOffsetZ = 0.f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    bool bWantSmoothStreets = true;
};

// ── UStreetMapComponent ──────────────────────────────────────────────────────
UCLASS(ClassGroup=(Rendering), meta=(BlueprintSpawnableComponent))
class STREETMAPRUNTIME_API UStreetMapComponent : public UMeshComponent
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=StreetMap)
    UStreetMap* StreetMap = nullptr;

    UFUNCTION(BlueprintCallable, Category=StreetMap)
    UStreetMap* GetStreetMap() const { return StreetMap; }

    UFUNCTION(BlueprintCallable, Category=StreetMap)
    void InvalidateMesh() {}

    // Minimal UPrimitiveComponent overrides
    virtual FPrimitiveSceneProxy* CreateSceneProxy() override { return nullptr; }
    virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override
    {
        return FBoxSphereBounds(ForceInitToZero);
    }
    virtual int32 GetNumMaterials() const override { return 0; }
};

// ── AStreetMapActor ──────────────────────────────────────────────────────────
UCLASS(BlueprintType)
class STREETMAPRUNTIME_API AStreetMapActor : public AActor
{
    GENERATED_BODY()
public:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=StreetMap)
    UStreetMapComponent* StreetMapComponent = nullptr;

    AStreetMapActor()
    {
        StreetMapComponent = CreateDefaultSubobject<UStreetMapComponent>(
            TEXT("StreetMapComponent"));
        RootComponent = StreetMapComponent;
    }

    UFUNCTION(BlueprintCallable, Category=StreetMap)
    TArray<FStreetMapBuilding> GetBuildings() const
    {
        return StreetMapComponent && StreetMapComponent->StreetMap
               ? StreetMapComponent->StreetMap->Buildings
               : TArray<FStreetMapBuilding>();
    }

    /** Stub — originally a CARLA-specific function to procedurally generate building tops.
     *  Returns an actor reference (null stub) — original return type was AActor* or similar. */
    UFUNCTION(BlueprintCallable, Category=StreetMap)
    AActor* GenerateTopOfBuilding(int32 Index, const FString& MapName) { return nullptr; }
};

// ── UStreetMapImportBlueprintLibrary ─────────────────────────────────────────
// Parameter names MUST match what the Blueprint node pins were serialised with.
UCLASS()
class STREETMAPRUNTIME_API UStreetMapImportBlueprintLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    /** Stub — always returns nullptr. StreetMap plugin not available for UE5. */
    UFUNCTION(BlueprintCallable, Category=StreetMap)
    static UStreetMap* ImportStreetMap(
        const FString& Path,
        const FString& DestinationAssetPath,
        const FVector2D& OriginLatLon)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("StreetMap: ImportStreetMap() is a no-op stub. Returning nullptr."));
        return nullptr;
    }
};
