// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "StaticMeshEditorSubsystem.h"
#include "StaticMeshEditorSubsystemHelpers.h" // FMergeStaticMeshActorsOptions
#include "CarlaEditorFunctionLibrary.generated.h"

class AStaticMeshActor;
class UStaticMesh;

/**
 * Non-deprecated drop-in replacements for the EditorScriptingUtilities
 * functions deprecated in UE5.  Named identically to the originals so that
 * FixBlueprintsCommandlet can redirect Blueprint call-nodes by name without
 * breaking any pin wiring.
 */
UCLASS()
class CARLATOOLS_API UCarlaEditorFunctionLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    /** Replacement for deprecated UEditorLevelLibrary::GetAllLevelActors. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static TArray<AActor*> GetAllLevelActors();

    /** Replacement for deprecated UEditorLevelLibrary::DestroyActor. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static bool DestroyActor(AActor* ActorToDestroy);

    /** Replacement for deprecated UEditorLevelLibrary::SaveCurrentLevel. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static bool SaveCurrentLevel();

    /** Replacement for deprecated UEditorLevelLibrary::GetEditorWorld. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static UWorld* GetEditorWorld();

    /** Replacement for deprecated UEditorLevelLibrary::LoadLevel. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static bool LoadLevel(const FString& AssetPath);

    /** Replacement for deprecated UEditorLevelLibrary::NewLevel. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static bool NewLevel(const FString& AssetPath);

    /** Replacement for deprecated UEditorLevelLibrary::NewLevelFromTemplate. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static bool NewLevelFromTemplate(const FString& AssetPath, const FString& TemplateAssetPath);

    /**
     * Replacement for deprecated UEditorLevelLibrary::MergeStaticMeshActors
     * (the overload accepting FMergeStaticMeshActorsOptions).
     * Uses UStaticMeshEditorSubsystem internally.
     */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static bool MergeStaticMeshActors(
        const TArray<AStaticMeshActor*>& ActorsToMerge,
        const FMergeStaticMeshActorsOptions& MergeOptions,
        AStaticMeshActor*& OutMergedActor);

    /** Replacement for deprecated UEditorStaticMeshLibrary::AddSimpleCollisions. */
    UFUNCTION(BlueprintCallable, Category="Carla|Editor")
    static int32 AddSimpleCollisions(UStaticMesh* StaticMesh, EScriptCollisionShapeType ShapeType);
};
