// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaEditorFunctionLibrary.h"

#include "Editor.h"
#include "Subsystems/EditorActorSubsystem.h"
#include "LevelEditorSubsystem.h"
#include "Subsystems/UnrealEditorSubsystem.h"
#include "StaticMeshEditorSubsystem.h"

// ---------------------------------------------------------------------------

TArray<AActor*> UCarlaEditorFunctionLibrary::GetAllLevelActors()
{
    if (auto* S = GEditor->GetEditorSubsystem<UEditorActorSubsystem>())
        return S->GetAllLevelActors();
    return {};
}

bool UCarlaEditorFunctionLibrary::DestroyActor(AActor* ActorToDestroy)
{
    auto* S = GEditor->GetEditorSubsystem<UEditorActorSubsystem>();
    return S && S->DestroyActor(ActorToDestroy);
}

bool UCarlaEditorFunctionLibrary::SaveCurrentLevel()
{
    auto* S = GEditor->GetEditorSubsystem<ULevelEditorSubsystem>();
    return S && S->SaveCurrentLevel();
}

UWorld* UCarlaEditorFunctionLibrary::GetEditorWorld()
{
    if (auto* S = GEditor->GetEditorSubsystem<UUnrealEditorSubsystem>())
        return S->GetEditorWorld();
    return nullptr;
}

bool UCarlaEditorFunctionLibrary::LoadLevel(const FString& AssetPath)
{
    auto* S = GEditor->GetEditorSubsystem<ULevelEditorSubsystem>();
    return S && S->LoadLevel(AssetPath);
}

bool UCarlaEditorFunctionLibrary::NewLevel(const FString& AssetPath)
{
    auto* S = GEditor->GetEditorSubsystem<ULevelEditorSubsystem>();
    return S && S->NewLevel(AssetPath);
}

bool UCarlaEditorFunctionLibrary::NewLevelFromTemplate(const FString& AssetPath, const FString& TemplateAssetPath)
{
    auto* S = GEditor->GetEditorSubsystem<ULevelEditorSubsystem>();
    return S && S->NewLevelFromTemplate(AssetPath, TemplateAssetPath);
}

bool UCarlaEditorFunctionLibrary::MergeStaticMeshActors(
    const TArray<AStaticMeshActor*>& ActorsToMerge,
    const FMergeStaticMeshActorsOptions& MergeOptions,
    AStaticMeshActor*& OutMergedActor)
{
    auto* S = GEditor->GetEditorSubsystem<UStaticMeshEditorSubsystem>();
    return S && S->MergeStaticMeshActors(ActorsToMerge, MergeOptions, OutMergedActor);
}

int32 UCarlaEditorFunctionLibrary::AddSimpleCollisions(UStaticMesh* StaticMesh, EScriptCollisionShapeType ShapeType)
{
    auto* S = GEditor->GetEditorSubsystem<UStaticMeshEditorSubsystem>();
    return S ? S->AddSimpleCollisions(StaticMesh, ShapeType) : 0;
}
