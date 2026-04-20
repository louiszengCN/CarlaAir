// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Commandlet/FixBlueprintsCommandlet.h"
#include "CarlaEditorFunctionLibrary.h"         // UCarlaEditorFunctionLibrary

#include "Engine/Blueprint.h"
#include "EdGraph/EdGraph.h"
#include "EdGraph/EdGraphNode.h"
#include "EdGraph/EdGraphPin.h"
#include "EdGraphSchema_K2.h"
#include "K2Node_Variable.h"
#include "K2Node_VariableGet.h"
#include "K2Node_BreakStruct.h"
#include "K2Node_MakeStruct.h"
#include "K2Node_DynamicCast.h"
#include "K2Node_CallFunction.h"
#include "K2Node_GetClassDefaults.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Modules/ModuleManager.h"
#include "Misc/PackageName.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "UObject/UObjectGlobals.h"
#include "Carla/Actor/PropActorFactory.h"
#include "Carla/Actor/VehicleActorFactory.h"
#include "Carla/Actor/WalkerActorFactory.h"
#include "Carla/Weather/SkyBase.h"
#include "Carla/Weather/Weather.h"
#include "Components/ChildActorComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/InstancedStaticMeshComponent.h" // AddInstance (replaces AddInstanceWorldSpace)
#include "Components/LightComponent.h"
#include "Components/LightComponentBase.h"
#include "Components/MeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/PostProcessComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Particles/ParticleSystemComponent.h"
#include "EditorUtilityWidget.h"                    // UEditorUtilityWidget (reparent WB_* widgets)

DEFINE_LOG_CATEGORY_STATIC(LogFixBlueprints, Log, All);

namespace
{
    struct FCompatibilityTargetPath
    {
        FName RootMember = NAME_None;
        FName NestedMember = NAME_None;
    };

    FName GetCompatibilitySkyReferenceMemberName(const FString& BPPath, const FString& MemberName)
    {
        const bool bIsGeneralSceneSettings = BPPath == TEXT("/Game/Carla/Blueprints/BP_GeneralSceneSettings");
        const bool bIsWeather = BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather");

        if (!(bIsGeneralSceneSettings || bIsWeather))
            return NAME_None;

        if (MemberName.Equals(TEXT("CarlaSky"), ESearchCase::IgnoreCase)
            || MemberName.Equals(TEXT("BP_CarlaSky"), ESearchCase::IgnoreCase))
        {
            return bIsGeneralSceneSettings ? FName(TEXT("CarlaSky")) : FName(TEXT("BP_CarlaSky"));
        }

        return NAME_None;
    }

    bool IsLegacySkyVariableName(const FString& MemberName)
    {
        return MemberName.Equals(TEXT("SkySphere"), ESearchCase::IgnoreCase)
            || MemberName.Equals(TEXT("Sky Sphere"), ESearchCase::IgnoreCase);
    }

    bool IsLegacyWeatherSkyReferenceName(const FString& MemberName)
    {
        return MemberName.Equals(TEXT("CarlaSky"), ESearchCase::IgnoreCase)
            || MemberName.Equals(TEXT("BP_CarlaSky"), ESearchCase::IgnoreCase);
    }

    FName GetCompatibilitySkyMemberName(const FString& MemberName)
    {
        if (MemberName.Equals(TEXT("SkySphereMesh"), ESearchCase::IgnoreCase))
            return TEXT("SkySphereMesh");
        if (MemberName.Equals(TEXT("PostProcessComponent"), ESearchCase::IgnoreCase))
            return TEXT("PostProcessComponent");
        if (MemberName.Equals(TEXT("Directional light actor"), ESearchCase::IgnoreCase)
            || MemberName.Equals(TEXT("DirectionalLightActor"), ESearchCase::IgnoreCase))
            return TEXT("DirectionalLightActor");
        if (MemberName.Equals(TEXT("Cloud opacity"), ESearchCase::IgnoreCase)
            || MemberName.Equals(TEXT("CloudOpacity"), ESearchCase::IgnoreCase))
            return TEXT("CloudOpacity");
        if (MemberName.Equals(TEXT("ExponentialHeightFogComponent"), ESearchCase::IgnoreCase))
            return TEXT("ExponentialHeightFogComponent");
        if (MemberName.Equals(TEXT("SkyAtmosphereComponent"), ESearchCase::IgnoreCase))
            return TEXT("SkyAtmosphereComponent");
        if (MemberName.Equals(TEXT("DirectionalLightComponentMoon"), ESearchCase::IgnoreCase))
            return TEXT("DirectionalLightComponentMoon");
        if (MemberName.Equals(TEXT("DirectionalLightComponentSun"), ESearchCase::IgnoreCase))
            return TEXT("DirectionalLightComponentSun");
        if (MemberName.Equals(TEXT("SkyLightComponent"), ESearchCase::IgnoreCase))
            return TEXT("SkyLightComponent");
        if (MemberName.Equals(TEXT("VolumetricCloudComponent"), ESearchCase::IgnoreCase))
            return TEXT("VolumetricCloudComponent");
        return NAME_None;
    }

    FName GetCompatibilityTargetMemberName(const FString& BPPath, const UClass* OwnerClass, const FString& GraphName, const FString& SymbolName)
    {
        if (!OwnerClass)
            return NAME_None;

        if (OwnerClass->GetName().Contains(TEXT("BP_Carla_Sky"))
            || OwnerClass->IsChildOf(ASkyBase::StaticClass()))
        {
            const FName SkyReferenceMember = GetCompatibilitySkyReferenceMemberName(BPPath, TEXT("BP_CarlaSky"));
            return SkyReferenceMember.IsNone() ? NAME_None : SkyReferenceMember;
        }

        if (OwnerClass->GetName().Contains(TEXT("BP_Sky_Sphere")))
            return TEXT("SkySphere");

        if (OwnerClass->IsChildOf(UChildActorComponent::StaticClass()))
            return TEXT("SkySphereChildActor");

        if (OwnerClass->IsChildOf(UFXSystemComponent::StaticClass()))
            return TEXT("RainTarget");
        if (OwnerClass->IsChildOf(UPostProcessComponent::StaticClass()))
            return TEXT("PostProcessComponent");

        if (OwnerClass->IsChildOf(UVolumetricCloudComponent::StaticClass()))
            return TEXT("VolumetricCloudComponent");

        if (OwnerClass->IsChildOf(UExponentialHeightFogComponent::StaticClass()))
            return TEXT("ExponentialHeightFogComponent");

        if (OwnerClass->IsChildOf(USkyAtmosphereComponent::StaticClass()))
            return TEXT("SkyAtmosphereComponent");

        if (OwnerClass->IsChildOf(USkyLightComponent::StaticClass()))
            return TEXT("SkyLightComponent");

        if (OwnerClass->IsChildOf(UMeshComponent::StaticClass())
            || OwnerClass->IsChildOf(UPrimitiveComponent::StaticClass()))
        {
            if (GraphName.Contains(TEXT("Cloud"), ESearchCase::IgnoreCase)
                || SymbolName.Contains(TEXT("Cloud"), ESearchCase::IgnoreCase))
                return TEXT("VolumetricCloudComponent");

            return TEXT("SkySphereMesh");
        }

        if (OwnerClass->IsChildOf(UDirectionalLightComponent::StaticClass())
            || OwnerClass->IsChildOf(ULightComponent::StaticClass())
            || OwnerClass->IsChildOf(ULightComponentBase::StaticClass())
            || OwnerClass->IsChildOf(USceneComponent::StaticClass()))
        {
            if (GraphName.Contains(TEXT("Moon"), ESearchCase::IgnoreCase)
                || SymbolName.Contains(TEXT("Moon"), ESearchCase::IgnoreCase))
                return TEXT("DirectionalLightComponentMoon");

            return TEXT("DirectionalLightComponentSun");
        }

        return NAME_None;
    }

    FCompatibilityTargetPath GetCompatibilityTargetPath(const FString& BPPath, const UClass* OwnerClass, const FString& GraphName, const FString& SymbolName)
    {
        const FName TargetMember = GetCompatibilityTargetMemberName(BPPath, OwnerClass, GraphName, SymbolName);
        if (TargetMember.IsNone())
            return {};

        if (BPPath == TEXT("/Game/Carla/Blueprints/BP_GeneralSceneSettings"))
        {
            if (TargetMember == TEXT("CarlaSky"))
                return { TEXT("CarlaSky"), NAME_None };

            return { TEXT("CarlaSky"), TargetMember };
        }

        return { TargetMember, NAME_None };
    }

    const UClass* GetCompatibilityMemberOwnerClass(const FName MemberName)
    {
        if (MemberName == TEXT("RainTarget") || MemberName == TEXT("Target"))
            return AWeather::StaticClass();

        return ASkyBase::StaticClass();
    }
}

// ---------------------------------------------------------------------------

UFixBlueprintsCommandlet::UFixBlueprintsCommandlet()
{
    IsClient  = false;
    IsServer  = false;
    IsEditor  = true;
    LogToConsole = true;
}

// ---------------------------------------------------------------------------

int32 UFixBlueprintsCommandlet::Main(const FString& /*Params*/)
{
    // ── 1. Look up StreetMap reflection objects ─────────────────────────────

    UScriptStruct* StreetMapBuildingStruct =
        FindObject<UScriptStruct>(nullptr, TEXT("/Script/StreetMapRuntime.StreetMapBuilding"));

    UClass* StreetMapClass =
        FindObject<UClass>(nullptr, TEXT("/Script/StreetMapRuntime.StreetMap"));

    UClass* StreetMapComponentClass =
        FindObject<UClass>(nullptr, TEXT("/Script/StreetMapRuntime.StreetMapComponent"));

    UClass* StreetMapActorClass =
        FindObject<UClass>(nullptr, TEXT("/Script/StreetMapRuntime.StreetMapActor"));

    UE_LOG(LogFixBlueprints, Display, TEXT("FStreetMapBuilding  : %s"),
        StreetMapBuildingStruct ? TEXT("OK") : TEXT("NOT FOUND - StreetMap module may not be loaded"));
    UE_LOG(LogFixBlueprints, Display, TEXT("UStreetMap          : %s"),
        StreetMapClass          ? TEXT("OK") : TEXT("NOT FOUND"));
    UE_LOG(LogFixBlueprints, Display, TEXT("UStreetMapComponent : %s"),
        StreetMapComponentClass ? TEXT("OK") : TEXT("NOT FOUND"));
    UE_LOG(LogFixBlueprints, Display, TEXT("AStreetMapActor     : %s"),
        StreetMapActorClass     ? TEXT("OK") : TEXT("NOT FOUND"));

    if (!StreetMapBuildingStruct)
    {
        UE_LOG(LogFixBlueprints, Error, TEXT("Cannot fix Blueprints: StreetMapBuilding struct not found. "
               "Make sure the StreetMapRuntime plugin is enabled and built."));
        return 1;
    }

    // ── 2. Look up additional structs needed for UE5 deprecation fixes ──────

    // FMeshMergingSettings — the stale 'bCreateMergedMaterial' pin lives here
    UScriptStruct* MeshMergingSettingsStruct =
        FindObject<UScriptStruct>(nullptr, TEXT("/Script/Engine.MeshMergingSettings"));

    UE_LOG(LogFixBlueprints, Display, TEXT("FMeshMergingSettings: %s"),
        MeshMergingSettingsStruct ? TEXT("OK") : TEXT("NOT FOUND"));

    // Wrapper class — non-deprecated drop-in replacements
    UClass* WrapperClass = UCarlaEditorFunctionLibrary::StaticClass();

    // ── 3. Blueprints to fix ────────────────────────────────────────────────
    //
    // bFixStreetMapTypes = true  → also redirect null DynamicCast → AStreetMapActor.
    //   (Only valid for OnroadMapGenerator BPs that had StreetMap casts.)
    // bFixStreetMapTypes = false → function-redirect only; skip DynamicCast fix
    //   so that other null casts (e.g. missing BP_Weather game content) are left alone.

    struct FBlueprintFixEntry { FString Path; bool bFixStreetMapTypes; };
    TArray<FBlueprintFixEntry> BlueprintEntries =
    {
        { TEXT("/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky"),              false },
        { TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather"),               false },
        { TEXT("/Game/Carla/Blueprints/BP_GeneralSceneSettings"),               false },
        { TEXT("/Game/Carla/Blueprints/Vehicles/BP_VehicleFactory"),            false },
        { TEXT("/Game/Carla/Blueprints/Walkers/BP_WalkerFactory"),              false },
        { TEXT("/Game/Carla/Blueprints/Props/BP_PropFactory"),                  false },
        { TEXT("/CarlaTools/OnroadMapGenerator/Blueprints/BP_BuildingGenerator"), true  },
        { TEXT("/CarlaTools/OnroadMapGenerator/BP_OpenDriveToMap"),               true  },
        { TEXT("/CarlaTools/OnroadMapGenerator/Blueprints/BP_Veg_Scatter"),       true  },
        { TEXT("/CarlaTools/OnroadMapGenerator/Blueprints/BP_RoofPropsGenerator"),true  },
        { TEXT("/CarlaTools/MapGenerator/UWB_CARLA"),                             false },
        { TEXT("/CarlaTools/OnroadMapGenerator/LevelCreator"),                    false },
        { TEXT("/CarlaTools/OnroadMapGenerator/UW_HoudiniBuildingImporter"),      false },
        { TEXT("/CarlaTools/OnroadMapGenerator/UW_OnRoadMainWidget"),             false },
        { TEXT("/CarlaTools/USDImporter/UW_USDVehicleImporterEditorWidget"),      false },
        { TEXT("/CarlaTools/USDImporter/UW_USDPropImporterEditorWidget"),         false },
    };

    int32 TotalFixed = 0;

    for (const FBlueprintFixEntry& Entry : BlueprintEntries)
    {
        const FString& BPPath = Entry.Path;
        UBlueprint* BP = LoadObject<UBlueprint>(nullptr, *BPPath);
        if (!BP)
        {
            UE_LOG(LogFixBlueprints, Warning, TEXT("Could not load Blueprint: %s (may not exist)"), *BPPath);
            continue;
        }

        UE_LOG(LogFixBlueprints, Display, TEXT("Processing: %s"), *BPPath);
        bool bModified = false;

        auto RemoveMemberVariableIfPresent = [&](const FName VariableName)
        {
            const int32 VariableIndex = BP->NewVariables.IndexOfByPredicate(
                [&](const FBPVariableDescription& Var)
                {
                    return Var.VarName == VariableName;
                });

            if (VariableIndex != INDEX_NONE)
            {
                BP->NewVariables.RemoveAt(VariableIndex);
                UE_LOG(LogFixBlueprints, Display,
                    TEXT("  Removed duplicate Blueprint member '%s'"),
                    *VariableName.ToString());
                bModified = true;
            }
        };

        auto FindDataPin = [](UEdGraphNode* Node, EEdGraphPinDirection Direction, const TArray<FName>& PreferredNames)
            -> UEdGraphPin*
        {
            if (!Node)
                return nullptr;

            for (const FName& PreferredName : PreferredNames)
            {
                if (UEdGraphPin* PreferredPin = Node->FindPin(PreferredName, Direction))
                    return PreferredPin;
            }

            for (UEdGraphPin* Pin : Node->Pins)
            {
                if (!Pin || Pin->Direction != Direction)
                    continue;
                if (Pin->PinType.PinCategory == UEdGraphSchema_K2::PC_Exec)
                    continue;
                if (Pin->PinName == UEdGraphSchema_K2::PN_Self)
                    continue;
                if (Pin->PinName == UEdGraphSchema_K2::PN_Then)
                    continue;
                if (Pin->PinName == UEdGraphSchema_K2::PN_Execute)
                    continue;
                return Pin;
            }

            return nullptr;
        };

        auto PurgeOrphanedPins = [](UEdGraphNode* Node)
        {
            bool bRemovedAny = false;
            if (!Node)
                return bRemovedAny;

            for (int32 Idx = Node->Pins.Num() - 1; Idx >= 0; --Idx)
            {
                if (UEdGraphPin* Pin = Node->Pins[Idx])
                {
                    if (!Pin->bOrphanedPin)
                        continue;

                    Pin->BreakAllPinLinks();
                    Node->Pins.RemoveAt(Idx);
                    bRemovedAny = true;
                }
            }

            return bRemovedAny;
        };

        auto FindTargetPin = [](UEdGraphNode* Node) -> UEdGraphPin*
        {
            if (!Node)
                return nullptr;

            if (UEdGraphPin* SelfPin = Node->FindPin(UEdGraphSchema_K2::PN_Self, EGPD_Input))
                return SelfPin;

            return Node->FindPin(TEXT("Target"), EGPD_Input);
        };

        auto SpawnSelfMemberGetter = [&](UEdGraph* Graph, UEdGraphNode* AnchorNode, const FName MemberName) -> UEdGraphPin*
        {
            if (!Graph || MemberName.IsNone())
                return nullptr;

            FGraphNodeCreator<UK2Node_VariableGet> NodeCreator(*Graph);
            UK2Node_VariableGet* GetterNode = NodeCreator.CreateNode();
            GetterNode->VariableReference.SetSelfMember(MemberName);
            GetterNode->NodePosX = AnchorNode ? AnchorNode->NodePosX - 320 : 0;
            GetterNode->NodePosY = AnchorNode ? AnchorNode->NodePosY : 0;
            GetterNode->AllocateDefaultPins();
            GetterNode->ReconstructNode();
            NodeCreator.Finalize();

            return FindDataPin(GetterNode, EGPD_Output, {});
        };

        auto SpawnExternalMemberGetter = [&](UEdGraph* Graph, UEdGraphNode* AnchorNode, const UClass* OwnerClass, const FName MemberName, UEdGraphPin* TargetSourcePin) -> UEdGraphPin*
        {
            if (!Graph || !OwnerClass || MemberName.IsNone())
                return nullptr;

            FProperty* MemberProperty = FindFProperty<FProperty>(const_cast<UClass*>(OwnerClass), MemberName);
            if (!MemberProperty)
                return nullptr;

            FGraphNodeCreator<UK2Node_VariableGet> NodeCreator(*Graph);
            UK2Node_VariableGet* GetterNode = NodeCreator.CreateNode();
            GetterNode->VariableReference.SetFromField<FProperty>(MemberProperty, false, const_cast<UClass*>(OwnerClass));
            GetterNode->NodePosX = AnchorNode ? AnchorNode->NodePosX - 160 : 0;
            GetterNode->NodePosY = AnchorNode ? AnchorNode->NodePosY : 0;
            GetterNode->AllocateDefaultPins();
            GetterNode->ReconstructNode();
            NodeCreator.Finalize();

            if (TargetSourcePin)
            {
                if (UEdGraphPin* GetterTargetPin = FindTargetPin(GetterNode))
                {
                    if (const UEdGraphSchema* Schema = GetterNode->GetSchema())
                        Schema->TryCreateConnection(TargetSourcePin, GetterTargetPin);
                    else
                        TargetSourcePin->MakeLinkTo(GetterTargetPin);
                }
            }

            return FindDataPin(GetterNode, EGPD_Output, {});
        };

        auto ReplaceVariableGetterWithNestedMember = [&](UEdGraph* Graph, UK2Node_VariableGet* VariableNode, const FCompatibilityTargetPath& TargetPath)
        {
            if (!Graph || !VariableNode || TargetPath.RootMember.IsNone())
                return;

            UEdGraphPin* OldOutputPin = FindDataPin(VariableNode, EGPD_Output, {});
            if (!OldOutputPin)
                return;

            TArray<UEdGraphPin*> SavedLinks = OldOutputPin->LinkedTo;
            OldOutputPin->BreakAllPinLinks();

            UEdGraphPin* SourcePin = SpawnSelfMemberGetter(Graph, VariableNode, TargetPath.RootMember);
            if (SourcePin && !TargetPath.NestedMember.IsNone())
            {
                SourcePin = SpawnExternalMemberGetter(
                    Graph,
                    VariableNode,
                    GetCompatibilityMemberOwnerClass(TargetPath.NestedMember),
                    TargetPath.NestedMember,
                    SourcePin);
            }

            if (!SourcePin)
                return;

            const UEdGraphSchema* Schema = VariableNode->GetSchema();
            bool bConnected = SavedLinks.Num() == 0;
            for (UEdGraphPin* SavedLink : SavedLinks)
            {
                if (!SavedLink)
                    continue;

                if (Schema)
                    bConnected |= Schema->TryCreateConnection(SourcePin, SavedLink);
                else
                {
                    SourcePin->MakeLinkTo(SavedLink);
                    bConnected = true;
                }
            }

            if (!bConnected)
                return;

            for (UEdGraphPin* Pin : VariableNode->Pins)
            {
                if (Pin)
                    Pin->BreakAllPinLinks();
            }

            Graph->RemoveNode(VariableNode);
            UE_LOG(LogFixBlueprints, Display,
                TEXT("  Removed stale node '%s' from '%s' (%s)"),
                *VariableNode->GetName(),
                *Graph->GetName(),
                TEXT("replaced self-context sky variable"));
            bModified = true;
        };

        auto RemoveNodeSafely = [&](UEdGraph* Graph, UEdGraphNode* Node, const TCHAR* Reason)
        {
            if (!Graph || !Node)
                return;

            for (UEdGraphPin* Pin : Node->Pins)
            {
                if (Pin)
                    Pin->BreakAllPinLinks();
            }

            Graph->RemoveNode(Node);
            UE_LOG(LogFixBlueprints, Display,
                TEXT("  Removed stale node '%s' from '%s' (%s)"),
                *Node->GetName(),
                *Graph->GetName(),
                Reason);
            bModified = true;
        };

        auto IsLegacyUwbCarlaVariableName = [](const FString& MemberName)
        {
            return MemberName.Equals(TEXT("Default Weathers"), ESearchCase::IgnoreCase)
                || MemberName.Equals(TEXT("DefaultWeathers"), ESearchCase::IgnoreCase)
                || MemberName.Equals(TEXT("LookAtTownName"), ESearchCase::IgnoreCase)
                || MemberName.Equals(TEXT("SelectedWeather"), ESearchCase::IgnoreCase);
        };

        auto WireMissingTargetFromSelfMember = [&](UEdGraph* Graph, UEdGraphNode* Node, const UClass* OwnerClass, const FString& SymbolName)
        {
            UEdGraphPin* TargetPin = FindTargetPin(Node);
            if (!TargetPin || !OwnerClass)
                return;

            const FCompatibilityTargetPath TargetPath =
                GetCompatibilityTargetPath(BPPath, OwnerClass, Graph ? Graph->GetName() : FString(), SymbolName);
            if (TargetPath.RootMember.IsNone())
                return;

            TargetPin->BreakAllPinLinks();

            UEdGraphPin* SourcePin = SpawnSelfMemberGetter(Graph, Node, TargetPath.RootMember);
            if (SourcePin && !TargetPath.NestedMember.IsNone())
            {
                SourcePin = SpawnExternalMemberGetter(
                    Graph,
                    Node,
                    GetCompatibilityMemberOwnerClass(TargetPath.NestedMember),
                    TargetPath.NestedMember,
                    SourcePin);
            }
            if (!SourcePin)
                return;

            const UEdGraphSchema* Schema = Node->GetSchema();
            bool bConnected = false;
            if (Schema)
            {
                bConnected = Schema->TryCreateConnection(SourcePin, TargetPin);
            }
            else
            {
                SourcePin->MakeLinkTo(TargetPin);
                bConnected = true;
            }
            if (!bConnected && TargetPath.NestedMember != TEXT("DirectionalLightComponentSun")
                && OwnerClass->IsChildOf(USceneComponent::StaticClass()))
            {
                UEdGraphPin* FallbackRootPin = SpawnSelfMemberGetter(Graph, Node, TargetPath.RootMember);
                UEdGraphPin* FallbackPin = FallbackRootPin;
                if (FallbackPin && BPPath == TEXT("/Game/Carla/Blueprints/BP_GeneralSceneSettings"))
                {
                    FallbackPin = SpawnExternalMemberGetter(
                        Graph,
                        Node,
                        ASkyBase::StaticClass(),
                        TEXT("DirectionalLightComponentSun"),
                        FallbackRootPin);
                }

                if (FallbackPin)
                {
                    if (Schema)
                        bConnected = Schema->TryCreateConnection(FallbackPin, TargetPin);
                    else
                    {
                        FallbackPin->MakeLinkTo(TargetPin);
                        bConnected = true;
                    }
                }
            }

            if (bConnected)
            {
                UE_LOG(LogFixBlueprints, Display,
                    TEXT("  Rewired target pin on '%s' in '%s' -> self.%s"),
                    *Node->GetName(),
                    Graph ? *Graph->GetName() : TEXT("<null>"),
                    *TargetPath.RootMember.ToString());
                bModified = true;
            }
        };

        auto RetargetVariableNode = [&](UK2Node_Variable* VariableNode, const FName NewMemberName)
        {
            if (!VariableNode)
                return;

            // Break ALL connections on ALL pins, then remove orphaned pins.
            // Orphaned pins persist through ReconstructNode if not removed here.
            // We intentionally do NOT restore connections: the old wire type was
            // ABP_CarlaSky*/ABP_Weather* while the new C++ property is
            // AStaticMeshActor*/AActor* — type-incompatible; restoring causes
            // "Can't connect pins" compile errors.
            for (UEdGraphPin* Pin : VariableNode->Pins)
            {
                if (Pin) Pin->BreakAllPinLinks();
            }
            PurgeOrphanedPins(VariableNode);

            VariableNode->VariableReference.SetSelfMember(NewMemberName);
            VariableNode->ReconstructNode();

            // Some stale pins are only marked orphaned after the first reconstruct.
            if (PurgeOrphanedPins(VariableNode))
                VariableNode->ReconstructNode();

            bModified = true;
        };

        auto ReplaceLegacyDayTimeNode = [&](UEdGraph* Graph, UEdGraphNode* OldNode)
        {
            if (!Graph || !OldNode)
                return;

            TArray<UEdGraphPin*> SavedExecLinks;
            TArray<UEdGraphPin*> SavedThenLinks;
            TArray<UEdGraphPin*> SavedBoolLinks;
            FString SavedDefaultValue;

            for (UEdGraphPin* Pin : OldNode->Pins)
            {
                if (!Pin)
                    continue;

                if (Pin->PinName == UEdGraphSchema_K2::PN_Execute)
                    SavedExecLinks.Append(Pin->LinkedTo);
                else if (Pin->PinName == UEdGraphSchema_K2::PN_Then)
                    SavedThenLinks.Append(Pin->LinkedTo);
                else if (Pin->PinName == TEXT("bIsDay") || Pin->PinName == TEXT("Is Day"))
                {
                    SavedBoolLinks.Append(Pin->LinkedTo);
                    if (!Pin->DefaultValue.IsEmpty())
                        SavedDefaultValue = Pin->DefaultValue;
                }

                Pin->BreakAllPinLinks();
            }

            FGraphNodeCreator<UK2Node_CallFunction> NodeCreator(*Graph);
            UK2Node_CallFunction* ReplacementNode = NodeCreator.CreateNode();
            ReplacementNode->FunctionReference.SetExternalMember(TEXT("DayTimeChangeEvent"), AWeather::StaticClass());
            ReplacementNode->NodePosX = OldNode->NodePosX;
            ReplacementNode->NodePosY = OldNode->NodePosY;
            ReplacementNode->AllocateDefaultPins();
            ReplacementNode->ReconstructNode();
            NodeCreator.Finalize();

            if (UEdGraphPin* ExecPin = ReplacementNode->FindPin(UEdGraphSchema_K2::PN_Execute, EGPD_Input))
            {
                for (UEdGraphPin* SavedLink : SavedExecLinks)
                    if (SavedLink) ExecPin->MakeLinkTo(SavedLink);
            }

            if (UEdGraphPin* ThenPin = ReplacementNode->FindPin(UEdGraphSchema_K2::PN_Then, EGPD_Output))
            {
                for (UEdGraphPin* SavedLink : SavedThenLinks)
                    if (SavedLink) ThenPin->MakeLinkTo(SavedLink);
            }

            if (UEdGraphPin* NewBoolPin = FindDataPin(ReplacementNode, EGPD_Input, { TEXT("bIsDay"), TEXT("Is Day") }))
            {
                const UEdGraphSchema* Schema = ReplacementNode->GetSchema();
                bool bAnyBoolConnected = false;
                for (UEdGraphPin* SavedLink : SavedBoolLinks)
                {
                    if (!SavedLink)
                        continue;
                    if (Schema ? Schema->TryCreateConnection(SavedLink, NewBoolPin) : (SavedLink->MakeLinkTo(NewBoolPin), true))
                        bAnyBoolConnected = true;
                }

                if (!bAnyBoolConnected && !SavedDefaultValue.IsEmpty())
                    NewBoolPin->DefaultValue = SavedDefaultValue;
            }

            Graph->RemoveNode(OldNode);
            bModified = true;

            UE_LOG(LogFixBlueprints, Display,
                TEXT("  Replaced legacy DayTimeChangeEvent node '%s' in '%s'"),
                *OldNode->GetName(),
                *Graph->GetName());
        };

        auto ScrubLegacyBoolPins = [&](UEdGraphNode* Node)
        {
            if (!Node)
                return;

            bool bTouched = false;
            for (UEdGraphPin* Pin : Node->Pins)
            {
                if (!Pin)
                    continue;

                if (Pin->PinName == TEXT("bIsDay") || Pin->PinName == TEXT("Is Day"))
                {
                    Pin->BreakAllPinLinks();
                    bTouched = true;
                }
            }

            if (!bTouched)
                return;

            PurgeOrphanedPins(Node);
            Node->ReconstructNode();
            if (PurgeOrphanedPins(Node))
                Node->ReconstructNode();
            bModified = true;
        };

        auto RetargetDayTimeChangeEventCall = [&](UK2Node_CallFunction* CallNode)
        {
            if (!CallNode)
                return;

            TArray<UEdGraphPin*> SavedExecLinks;
            TArray<UEdGraphPin*> SavedThenLinks;
            FString SavedDefaultValue;
            for (UEdGraphPin* Pin : CallNode->Pins)
            {
                if (!Pin)
                    continue;

                const FName PN = Pin->PinName;
                if (PN == UEdGraphSchema_K2::PN_Execute)
                    SavedExecLinks.Append(Pin->LinkedTo);
                else if (PN == UEdGraphSchema_K2::PN_Then)
                    SavedThenLinks.Append(Pin->LinkedTo);

                if (PN == TEXT("bIsDay") || PN == TEXT("Is Day"))
                {
                    if (!Pin->DefaultValue.IsEmpty())
                        SavedDefaultValue = Pin->DefaultValue;
                }

                Pin->BreakAllPinLinks();
            }
            PurgeOrphanedPins(CallNode);

            // BP_CarlaWeather inherits AWeather (not ASkyBase), so resolve via AWeather.
            // Both AWeather and ASkyBase declare DayTimeChangeEvent, but using the direct
            // parent class ensures the pin type resolves correctly.
            CallNode->FunctionReference.SetExternalMember(TEXT("DayTimeChangeEvent"), AWeather::StaticClass());
            CallNode->ReconstructNode();

            // As with variable nodes, some stale pins only become orphaned after the
            // first reconstruct against the new function signature.
            if (PurgeOrphanedPins(CallNode))
                CallNode->ReconstructNode();

            if (UEdGraphPin* ExecPin = CallNode->FindPin(UEdGraphSchema_K2::PN_Execute, EGPD_Input))
            {
                for (UEdGraphPin* SavedLink : SavedExecLinks)
                    if (SavedLink) ExecPin->MakeLinkTo(SavedLink);
            }

            if (UEdGraphPin* ThenPin = CallNode->FindPin(UEdGraphSchema_K2::PN_Then, EGPD_Output))
            {
                for (UEdGraphPin* SavedLink : SavedThenLinks)
                    if (SavedLink) ThenPin->MakeLinkTo(SavedLink);
            }

            if (UEdGraphPin* NewBoolPin = FindDataPin(CallNode, EGPD_Input, { TEXT("bIsDay"), TEXT("Is Day") }))
            {
                if (!SavedDefaultValue.IsEmpty())
                    NewBoolPin->DefaultValue = SavedDefaultValue;
            }

            bModified = true;
        };

        struct FPinLinkSnapshot
        {
            FName PinName;
            EEdGraphPinDirection Direction;
            TArray<UEdGraphPin*> LinkedTo;
            FString DefaultValue;
        };

        auto RefreshNodePreservingLinks = [&](UEdGraphNode* Node)
        {
            if (!Node)
                return;

            TArray<FPinLinkSnapshot> SavedPins;
            SavedPins.Reserve(Node->Pins.Num());

            for (UEdGraphPin* Pin : Node->Pins)
            {
                if (!Pin)
                    continue;

                FPinLinkSnapshot& Snapshot = SavedPins.AddDefaulted_GetRef();
                Snapshot.PinName = Pin->PinName;
                Snapshot.Direction = Pin->Direction;
                Snapshot.LinkedTo = Pin->LinkedTo;
                Snapshot.DefaultValue = Pin->DefaultValue;
                Pin->BreakAllPinLinks();
            }

            PurgeOrphanedPins(Node);
            Node->ReconstructNode();
            if (PurgeOrphanedPins(Node))
                Node->ReconstructNode();

            const UEdGraphSchema* Schema = Node->GetSchema();
            for (const FPinLinkSnapshot& Snapshot : SavedPins)
            {
                UEdGraphPin* NewPin = Node->FindPin(Snapshot.PinName, Snapshot.Direction);
                if (!NewPin)
                    continue;

                bool bConnected = false;
                for (UEdGraphPin* LinkedPin : Snapshot.LinkedTo)
                {
                    if (!LinkedPin)
                        continue;

                    const bool bMadeConnection = Schema
                        ? Schema->TryCreateConnection(NewPin, LinkedPin)
                        : (NewPin->MakeLinkTo(LinkedPin), true);
                    bConnected = bConnected || bMadeConnection;
                }

                if (!bConnected && !Snapshot.DefaultValue.IsEmpty())
                    NewPin->DefaultValue = Snapshot.DefaultValue;
            }

            bModified = true;
        };

        if (BPPath == TEXT("/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky"))
        {
            RemoveMemberVariableIfPresent(TEXT("SkySphere"));
            if (BP->ParentClass != ASkyBase::StaticClass())
            {
                BP->ParentClass = ASkyBase::StaticClass();
                UE_LOG(LogFixBlueprints, Display, TEXT("  Restored parent class → ASkyBase"));
                bModified = true;
            }
        }
        else if (BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather"))
        {
            RemoveMemberVariableIfPresent(TEXT("Target"));
            RemoveMemberVariableIfPresent(TEXT("RainTarget"));
            if (BP->ParentClass != AWeather::StaticClass())
            {
                BP->ParentClass = AWeather::StaticClass();
                UE_LOG(LogFixBlueprints, Display, TEXT("  Restored parent class → AWeather"));
                bModified = true;
            }
        }
        else if (BPPath == TEXT("/Game/Carla/Blueprints/Vehicles/BP_VehicleFactory"))
        {
            if (BP->ParentClass != AVehicleActorFactory::StaticClass())
            {
                BP->ParentClass = AVehicleActorFactory::StaticClass();
                UE_LOG(LogFixBlueprints, Display, TEXT("  Restored parent class → AVehicleActorFactory"));
                bModified = true;
            }
        }
        else if (BPPath == TEXT("/Game/Carla/Blueprints/Walkers/BP_WalkerFactory"))
        {
            if (BP->ParentClass != AWalkerActorFactory::StaticClass())
            {
                BP->ParentClass = AWalkerActorFactory::StaticClass();
                UE_LOG(LogFixBlueprints, Display, TEXT("  Restored parent class → AWalkerActorFactory"));
                bModified = true;
            }
        }
        else if (BPPath == TEXT("/Game/Carla/Blueprints/Props/BP_PropFactory"))
        {
            if (BP->ParentClass != APropActorFactory::StaticClass())
            {
                BP->ParentClass = APropActorFactory::StaticClass();
                UE_LOG(LogFixBlueprints, Display, TEXT("  Restored parent class → APropActorFactory"));
                bModified = true;
            }
        }

        // ── 3a. Fix Blueprint variable types ───────────────────────────────
        for (FBPVariableDescription& Var : BP->NewVariables)
        {
            FEdGraphPinType& T = Var.VarType;
            FName N = Var.VarName;

            if ((N == TEXT("StreetMapData") || N == TEXT("StreetMap")) && StreetMapClass)
            {
                if (T.PinSubCategoryObject != StreetMapClass)
                {
                    T.PinSubCategoryObject = StreetMapClass;
                    UE_LOG(LogFixBlueprints, Display, TEXT("  Fixed var '%s' → UStreetMap"), *N.ToString());
                    bModified = true;
                }
            }
            else if (N == TEXT("StreetMapComponent") && StreetMapComponentClass)
            {
                if (T.PinSubCategoryObject != StreetMapComponentClass)
                {
                    T.PinSubCategoryObject = StreetMapComponentClass;
                    UE_LOG(LogFixBlueprints, Display, TEXT("  Fixed var '%s' → UStreetMapComponent"), *N.ToString());
                    bModified = true;
                }
            }
            else if (N == TEXT("StreetMapActor") && StreetMapActorClass)
            {
                if (T.PinSubCategoryObject != StreetMapActorClass)
                {
                    T.PinSubCategoryObject = StreetMapActorClass;
                    UE_LOG(LogFixBlueprints, Display, TEXT("  Fixed var '%s' → AStreetMapActor"), *N.ToString());
                    bModified = true;
                }
            }
            else if (T.PinSubCategoryObject == nullptr
                     && T.PinCategory == FName(TEXT("struct"))
                     && (N == TEXT("Buildings") || N == TEXT("BuildingList") || N == TEXT("StreetMapBuildings"))
                     && StreetMapBuildingStruct)
            {
                T.PinSubCategoryObject = StreetMapBuildingStruct;
                UE_LOG(LogFixBlueprints, Display,
                    TEXT("  Fixed null struct var '%s' → FStreetMapBuilding"), *N.ToString());
                bModified = true;
            }
        }

        // ── 3b-pre. Pre-compile to regenerate the SkeletonGeneratedClass.
        //           Without a valid up-to-date skeleton, ReconstructNode()
        //           in the loop below cannot resolve C++ properties by name.
        //           Note: no BatchCompile so the skeleton is fully generated now.
        FKismetEditorUtilities::CompileBlueprint(BP,
            EBlueprintCompileOptions::SkipGarbageCollection);

        // Diagnostic: verify skeleton state after pre-compile
        {
            UClass* SkelClass = BP->SkeletonGeneratedClass;
            UE_LOG(LogFixBlueprints, Display, TEXT("  [diag] SkeletonGeneratedClass: %s"),
                SkelClass ? *SkelClass->GetName() : TEXT("NULL"));
            if (SkelClass)
            {
                UE_LOG(LogFixBlueprints, Display, TEXT("  [diag] Skeleton super: %s"),
                    SkelClass->GetSuperClass() ? *SkelClass->GetSuperClass()->GetName() : TEXT("NULL"));
                FProperty* SkySp = FindFProperty<FProperty>(SkelClass, TEXT("SkySphere"));
                FProperty* TargetP = FindFProperty<FProperty>(SkelClass, TEXT("Target"));
                UE_LOG(LogFixBlueprints, Display,
                    TEXT("  [diag] FindFProperty(SkySphere)=%s  FindFProperty(Target)=%s"),
                    SkySp ? TEXT("FOUND") : TEXT("NULL"),
                    TargetP ? TEXT("FOUND") : TEXT("NULL"));
            }
            UE_LOG(LogFixBlueprints, Display, TEXT("  [diag] NewVariables:"));
            for (const FBPVariableDescription& Var : BP->NewVariables)
            {
                UE_LOG(LogFixBlueprints, Display, TEXT("    VarName='%s' FriendlyName='%s'"),
                    *Var.VarName.ToString(), *Var.FriendlyName);
            }
        }

        // ── 3b. Fix K2Nodes in all graphs ──────────────────────────────────
        TArray<UEdGraph*> AllGraphs;
        BP->GetAllGraphs(AllGraphs);

        for (UEdGraph* Graph : AllGraphs)
        {
            if (!Graph) continue;

            if (BPPath == TEXT("/Game/Carla/Blueprints/Props/BP_PropFactory")
                && Graph->GetName() == TEXT("PostProcessProp"))
            {
                UEdGraphNode* EntryNode = nullptr;
                UEdGraphNode* ReturnNode = nullptr;
                TArray<UEdGraphNode*> NodesToRemove;

                for (UEdGraphNode* GraphNode : Graph->Nodes)
                {
                    if (!GraphNode)
                        continue;

                    if (!EntryNode && GraphNode->GetClass()->GetName() == TEXT("K2Node_FunctionEntry"))
                    {
                        EntryNode = GraphNode;
                        continue;
                    }

                    if (GraphNode->GetClass()->GetName() == TEXT("K2Node_FunctionResult"))
                    {
                        if (!ReturnNode)
                        {
                            ReturnNode = GraphNode;
                            continue;
                        }
                    }

                    NodesToRemove.Add(GraphNode);
                }

                if (EntryNode && ReturnNode)
                {
                    for (UEdGraphNode* GraphNode : NodesToRemove)
                    {
                        for (UEdGraphPin* Pin : GraphNode->Pins)
                        {
                            if (Pin)
                                Pin->BreakAllPinLinks();
                        }
                        Graph->RemoveNode(GraphNode);
                    }

                    if (UEdGraphPin* ThenPin = EntryNode->FindPin(UEdGraphSchema_K2::PN_Then, EGPD_Output))
                    {
                        ThenPin->BreakAllPinLinks();
                        if (UEdGraphPin* ExecPin = ReturnNode->FindPin(UEdGraphSchema_K2::PN_Execute, EGPD_Input))
                        {
                            ExecPin->BreakAllPinLinks();
                            if (const UEdGraphSchema* Schema = Graph->GetSchema())
                                Schema->TryCreateConnection(ThenPin, ExecPin);
                            else
                                ThenPin->MakeLinkTo(ExecPin);
                        }
                    }

                    UE_LOG(LogFixBlueprints, Display,
                        TEXT("  Collapsed stale graph '%s' to entry/return no-op"),
                        *Graph->GetName());
                    bModified = true;
                }
            }

            const TArray<UEdGraphNode*> GraphNodes = Graph->Nodes;
            for (UEdGraphNode* Node : GraphNodes)
            {
                if (!Node) continue;

                if (BPPath == TEXT("/CarlaTools/MapGenerator/UWB_CARLA"))
                {
                    if (UK2Node_Variable* VariableNode = Cast<UK2Node_Variable>(Node))
                    {
                        if (IsLegacyUwbCarlaVariableName(VariableNode->VariableReference.GetMemberName().ToString()))
                        {
                            RemoveNodeSafely(Graph, Node, TEXT("legacy variable reference"));
                            continue;
                        }
                    }

                    const FString NodeClassName = Node->GetClass()->GetName();
                    if (NodeClassName == TEXT("K2Node_DynamicCast") || NodeClassName == TEXT("K2Node_ClassDynamicCast"))
                    {
                        if (UK2Node_DynamicCast* CastNode = Cast<UK2Node_DynamicCast>(Node))
                        {
                            const bool bBadTargetType = CastNode->TargetType == nullptr
                                || CastNode->TargetType->GetName().Contains(TEXT("BP_Weather"));
                            if (bBadTargetType)
                            {
                                RemoveNodeSafely(Graph, Node, TEXT("legacy weather cast"));
                                continue;
                            }
                        }
                    }

                    if (UK2Node_GetClassDefaults* GetClassDefaultsNode = Cast<UK2Node_GetClassDefaults>(Node))
                    {
                        bool bRemoveNode = false;
                        for (UEdGraphPin* Pin : GetClassDefaultsNode->Pins)
                        {
                            if (!Pin)
                                continue;
                            if (IsLegacyUwbCarlaVariableName(Pin->PinName.ToString()))
                            {
                                bRemoveNode = true;
                                break;
                            }
                        }

                        if (bRemoveNode)
                        {
                            RemoveNodeSafely(Graph, Node, TEXT("legacy class defaults node"));
                            continue;
                        }
                    }
                }

                if (BPPath == TEXT("/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky")
                    || BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                    || BPPath == TEXT("/Game/Carla/Blueprints/BP_GeneralSceneSettings"))
                {
                    if (UK2Node_Variable* VariableNode = Cast<UK2Node_Variable>(Node))
                    {
                        FProperty* ResolvedProperty = VariableNode->GetPropertyForVariable();
                        const UClass* OwnerClass = ResolvedProperty ? ResolvedProperty->GetOwnerClass() : nullptr;
                        const FString MemberName = VariableNode->VariableReference.GetMemberName().ToString();
                        UE_LOG(LogFixBlueprints, Display,
                            TEXT("  [scan-var] BP='%s' Graph='%s' Node='%s' Member='%s' SelfCtx=%d Resolved=%s Owner=%s Pins=%d"),
                            *BPPath,
                            *Graph->GetName(),
                            *Node->GetName(),
                            *MemberName,
                            (int32)VariableNode->VariableReference.IsSelfContext(),
                            ResolvedProperty ? TEXT("YES") : TEXT("NO"),
                            OwnerClass ? *OwnerClass->GetName() : TEXT("NULL"),
                            VariableNode->Pins.Num());

                        if (BPPath == TEXT("/Game/Carla/Blueprints/BP_GeneralSceneSettings")
                            && Cast<UK2Node_VariableGet>(VariableNode))
                        {
                            if (UK2Node_VariableGet* VariableGetNode = Cast<UK2Node_VariableGet>(VariableNode))
                            {
                                const FName SkyMemberName = GetCompatibilitySkyMemberName(MemberName);
                                const bool bReplaceSkyGetter = !SkyMemberName.IsNone()
                                    && (VariableNode->VariableReference.IsSelfContext()
                                        || (OwnerClass && OwnerClass->IsChildOf(ASkyBase::StaticClass())));
                                if (bReplaceSkyGetter)
                                {
                                    ReplaceVariableGetterWithNestedMember(
                                        Graph,
                                        VariableGetNode,
                                        { TEXT("CarlaSky"), SkyMemberName });
                                    continue;
                                }
                            }
                        }

                        if (!VariableNode->VariableReference.IsSelfContext())
                        {
                            WireMissingTargetFromSelfMember(Graph, VariableNode, OwnerClass,
                                MemberName);
                        }
                    }
                    else if (UK2Node_CallFunction* ScanCallNode = Cast<UK2Node_CallFunction>(Node))
                    {
                        UFunction* TargetFunction = ScanCallNode->GetTargetFunction();
                        const UClass* OwnerClass = TargetFunction ? TargetFunction->GetOwnerClass() : nullptr;
                        UE_LOG(LogFixBlueprints, Display,
                            TEXT("  [scan-call] BP='%s' Graph='%s' Node='%s' Func='%s' Owner=%s Pins=%d"),
                            *BPPath,
                            *Graph->GetName(),
                            *Node->GetName(),
                            *ScanCallNode->GetFunctionName().ToString(),
                            OwnerClass ? *OwnerClass->GetName() : TEXT("NULL"),
                            ScanCallNode->Pins.Num());

                        WireMissingTargetFromSelfMember(Graph, ScanCallNode, OwnerClass,
                            ScanCallNode->GetFunctionName().ToString());
                    }
                }

                if (BPPath == TEXT("/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky")
                    || BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                    || BPPath == TEXT("/Game/Carla/Blueprints/BP_GeneralSceneSettings"))
                {
                    if (BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                        && Graph->GetName() == TEXT("Rain"))
                    {
                        RefreshNodePreservingLinks(Node);
                    }

                    if (BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                        && !Cast<UK2Node_CallFunction>(Node)
                        && Node->GetNodeTitle(ENodeTitleType::ListView).ToString().Contains(TEXT("DayTimeChangeEvent")))
                    {
                        ReplaceLegacyDayTimeNode(Graph, Node);
                        continue;
                    }

                    ScrubLegacyBoolPins(Node);

                    if (UK2Node_Variable* VariableNode = Cast<UK2Node_Variable>(Node))
                    {
                        const FString MemberName = VariableNode->VariableReference.GetMemberName().ToString();
                        const FName ReplacementMemberName =
                            IsLegacySkyVariableName(MemberName) ? FName(TEXT("SkySphere")) :
                            (GetCompatibilitySkyReferenceMemberName(BPPath, MemberName) != NAME_None)
                                ? GetCompatibilitySkyReferenceMemberName(BPPath, MemberName) :
                                                        ((BPPath == TEXT("/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky")
                                                            || BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather"))
                                                                ? GetCompatibilitySkyMemberName(MemberName)
                                                                : NAME_None) != NAME_None
                                                                ? GetCompatibilitySkyMemberName(MemberName)
                                                                :
                            (BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                                && MemberName.Equals(TEXT("Rain Target"), ESearchCase::IgnoreCase))
                                ? FName(TEXT("RainTarget"))
                                :
                            (BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                                && MemberName.Equals(TEXT("RainTarget"), ESearchCase::IgnoreCase))
                                ? FName(TEXT("RainTarget"))
                                :
                            (BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                                && MemberName.Equals(TEXT("Target"), ESearchCase::IgnoreCase))
                                ? FName(TEXT("Target"))
                                : NAME_None;
                        if (ReplacementMemberName != NAME_None)
                        {
                            UE_LOG(LogFixBlueprints, Display,
                                TEXT("  [diag] Retarget: MemberName='%s' bSelfCtx=%d Graph='%s'"),
                                *MemberName,
                                (int32)VariableNode->VariableReference.IsSelfContext(),
                                *Graph->GetName());
                            RetargetVariableNode(VariableNode, ReplacementMemberName);
                            UE_LOG(LogFixBlueprints, Display,
                                TEXT("  [diag] After retarget: MemberName='%s' bSelfCtx=%d Pins=%d"),
                                *VariableNode->VariableReference.GetMemberName().ToString(),
                                (int32)VariableNode->VariableReference.IsSelfContext(),
                                VariableNode->Pins.Num());
                            for (UEdGraphPin* DbgPin : VariableNode->Pins)
                            {
                                if (DbgPin)
                                    UE_LOG(LogFixBlueprints, Display,
                                        TEXT("  [diag]   pin='%s' orphan=%d linked=%d"),
                                        *DbgPin->PinName.ToString(),
                                        (int32)DbgPin->bOrphanedPin,
                                        DbgPin->LinkedTo.Num());
                            }
                        }
                    }
                }

                if (BPPath == TEXT("/Game/Carla/Blueprints/Props/BP_PropFactory"))
                {
                    RefreshNodePreservingLinks(Node);
                }

                // ── StreetMap: null StructType → FStreetMapBuilding ────────
                if (UK2Node_BreakStruct* BreakNode = Cast<UK2Node_BreakStruct>(Node))
                {
                    if (BreakNode->StructType == nullptr && StreetMapBuildingStruct)
                    {
                        BreakNode->StructType = StreetMapBuildingStruct;
                        BreakNode->ReconstructNode();
                        UE_LOG(LogFixBlueprints, Display,
                            TEXT("  Fixed K2Node_BreakStruct → FStreetMapBuilding in '%s'"),
                            *Graph->GetName());
                        bModified = true;
                    }
                }
                // ── MakeStruct: StreetMap null fix + stale-pin fix ─────────
                else if (UK2Node_MakeStruct* MakeNode = Cast<UK2Node_MakeStruct>(Node))
                {
                    if (MakeNode->StructType == nullptr && StreetMapBuildingStruct)
                    {
                        // Null StructType → FStreetMapBuilding
                        MakeNode->StructType = StreetMapBuildingStruct;
                        MakeNode->ReconstructNode();
                        UE_LOG(LogFixBlueprints, Display,
                            TEXT("  Fixed K2Node_MakeStruct → FStreetMapBuilding in '%s'"),
                            *Graph->GetName());
                        bModified = true;
                    }
                    else if (MeshMergingSettingsStruct
                             && MakeNode->StructType == MeshMergingSettingsStruct)
                    {
                        // Fix orphaned pins whose fields were removed from FMeshMergingSettings
                        // (e.g. bCreateMergedMaterial → bCreateMergedMaterial_DEPRECATED, no UPROPERTY).
                        // Reset DefaultValue to autogenerated so ValidateOrphanPin doesn't warn.
                        for (UEdGraphPin* Pin : MakeNode->Pins)
                        {
                            if (Pin && Pin->bOrphanedPin
                                && Pin->DefaultValue != Pin->AutogeneratedDefaultValue)
                            {
                                UE_LOG(LogFixBlueprints, Display,
                                    TEXT("  Reset orphaned pin '%s' on Make Mesh Merging Settings"),
                                    *Pin->PinName.ToString());
                                Pin->DefaultValue = Pin->AutogeneratedDefaultValue;
                                bModified = true;
                            }
                        }
                    }
                }
                // ── DynamicCast: null TargetType → AStreetMapActor ─────────
                // Only applied for OnroadMapGenerator BPs; other null casts
                // (e.g. missing game content) must not be redirected here.
                else if (UK2Node_DynamicCast* CastNode = Cast<UK2Node_DynamicCast>(Node))
                {
                    if (Entry.bFixStreetMapTypes && CastNode->TargetType == nullptr && StreetMapActorClass)
                    {
                        CastNode->TargetType = StreetMapActorClass;
                        CastNode->ReconstructNode();
                        UE_LOG(LogFixBlueprints, Display,
                            TEXT("  Fixed K2Node_DynamicCast → AStreetMapActor in '%s'"),
                            *Graph->GetName());
                        bModified = true;
                    }
                }
                // ── CallFunction: redirect deprecated calls ─────────────────
                if (UK2Node_CallFunction* CallNode = Cast<UK2Node_CallFunction>(Node))
                {
                    if (BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
                        && CallNode->GetFunctionName() == TEXT("DayTimeChangeEvent"))
                    {
                        RetargetDayTimeChangeEventCall(CallNode);
                    }

                    UFunction* Func = CallNode->GetTargetFunction();
                    if (!Func || !Func->HasMetaData(TEXT("DeprecatedFunction")))
                        continue;

                    const FName FuncName = CallNode->GetFunctionName();

                    if (FuncName == TEXT("AddInstanceWorldSpace"))
                    {
                        // Member function: AddInstanceWorldSpace(WorldTransform) →
                        //                  AddInstance(InstanceTransform, bWorldSpace=true)
                        // Pin name changes: WorldTransform → InstanceTransform
                        //
                        // IMPORTANT: break old connections BEFORE ReconstructNode so the
                        // WorldTransform pin becomes an unconnected orphan (no error).
                        // Without this, ReconstructNode keeps the orphaned pin connected
                        // and the compiler reports "In use pin no longer exists".
                        UEdGraphPin* OldPin = CallNode->FindPin(TEXT("WorldTransform"), EGPD_Input);
                        TArray<UEdGraphPin*> SourcePins; // other-end pins to reconnect later
                        if (OldPin)
                        {
                            SourcePins = OldPin->LinkedTo; // save before breaking
                            OldPin->BreakAllPinLinks();    // detaches both sides cleanly
                        }

                        CallNode->FunctionReference.SetExternalMember(
                            TEXT("AddInstance"), UInstancedStaticMeshComponent::StaticClass());
                        CallNode->ReconstructNode();

                        // Reconnect saved source pins → new InstanceTransform pin
                        UEdGraphPin* NewPin = CallNode->FindPin(TEXT("InstanceTransform"), EGPD_Input);
                        if (NewPin)
                            for (UEdGraphPin* Src : SourcePins)
                                if (Src) NewPin->MakeLinkTo(Src);

                        // Set bWorldSpace = true (replaces the implicit world-space semantic)
                        if (UEdGraphPin* WSPin = CallNode->FindPin(TEXT("bWorldSpace"), EGPD_Input))
                            WSPin->DefaultValue = TEXT("true");

                        UE_LOG(LogFixBlueprints, Display,
                            TEXT("  Redirected AddInstanceWorldSpace → AddInstance(bWorldSpace=true) in '%s'"),
                            *Graph->GetName());
                        bModified = true;
                    }
                    else if (WrapperClass && WrapperClass->FindFunctionByName(FuncName))
                    {
                        // Static library function: redirect to UCarlaEditorFunctionLibrary
                        // (same name, same signature → all pins reconnect automatically)
                        CallNode->FunctionReference.SetExternalMember(FuncName, WrapperClass);
                        CallNode->ReconstructNode();
                        UE_LOG(LogFixBlueprints, Display,
                            TEXT("  Redirected deprecated '%s' → UCarlaEditorFunctionLibrary in '%s'"),
                            *FuncName.ToString(), *Graph->GetName());
                        bModified = true;
                    }
                }
            }
        }

        if (!bModified)
            UE_LOG(LogFixBlueprints, Display, TEXT("  No fixable issues found — forcing recompile"));

        if (bModified)
        {
            FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
        }

        if (BPPath == TEXT("/Game/Carla/Blueprints/LevelDesign/BP_Carla_Sky")
            || BPPath == TEXT("/Game/Carla/Blueprints/Weather/BP_CarlaWeather")
            || BPPath == TEXT("/Game/Carla/Blueprints/Props/BP_PropFactory")
            || BPPath == TEXT("/CarlaTools/MapGenerator/UWB_CARLA"))
        {
            UE_LOG(LogFixBlueprints, Display, TEXT("  Final RefreshAllNodes..."));
            FBlueprintEditorUtils::RefreshAllNodes(BP);
        }

        // ── 3c. Recompile ──────────────────────────────────────────────────
        // NOTE: No second RefreshAllNodes here — the pre-loop RefreshAllNodes
        // already updated the skeleton class.  A second call would re-reconstruct
        // all nodes against stale GUIDs and undo the SetSelfMember fixes above.
        UE_LOG(LogFixBlueprints, Display, TEXT("  Recompiling..."));
        FKismetEditorUtilities::CompileBlueprint(BP,
            EBlueprintCompileOptions::SkipGarbageCollection |
            EBlueprintCompileOptions::BatchCompile);

        // ── 3e. Save ───────────────────────────────────────────────────────
        UPackage* Package = BP->GetOutermost();
        Package->MarkPackageDirty();

        FString FilePath;
        if (FPackageName::TryConvertLongPackageNameToFilename(
                Package->GetName(), FilePath, FPackageName::GetAssetPackageExtension()))
        {
            FSavePackageArgs SaveArgs;
            SaveArgs.SaveFlags = SAVE_None;
            const bool bSaved = UPackage::SavePackage(Package, nullptr, *FilePath, SaveArgs);

            if (bSaved)
            {
                UE_LOG(LogFixBlueprints, Display, TEXT("  Saved: %s"), *FilePath);
                ++TotalFixed;
            }
            else
            {
                UE_LOG(LogFixBlueprints, Error, TEXT("  Save FAILED for: %s"), *FilePath);
            }
        }
        else
        {
            UE_LOG(LogFixBlueprints, Error, TEXT("  Could not resolve file path for package: %s"),
                   *Package->GetName());
        }
    }

    // ── 4. Widget Blueprint reparenting ────────────────────────────────────────
    // WB_BaseRegionWidget and its subclasses use editor-only UObjectPropertyEntryBox::SetObject
    // inside a standard UUserWidget, which the compiler rejects.  The fix is to reparent
    // these widgets to UEditorUtilityWidget (editor-only base), which allows editor APIs.

    UClass* EditorUtilityWidgetClass = FindObject<UClass>(nullptr, TEXT("/Script/Blutility.EditorUtilityWidget"));
    if (!EditorUtilityWidgetClass)
        EditorUtilityWidgetClass = LoadObject<UClass>(nullptr, TEXT("/Script/Blutility.EditorUtilityWidget"));

    TArray<FString> WidgetReparentPaths =
    {
        TEXT("/CarlaTools/MapGenerator/Misc/RegionWidget/WB_BaseRegionWidget"),
        TEXT("/CarlaTools/MapGenerator/Misc/RegionWidget/WB_TerrainMaterialWidget"),
        TEXT("/CarlaTools/MapGenerator/Misc/RegionWidget/WB_VegetationGeneralSettings"),
    };

    if (!EditorUtilityWidgetClass)
    {
        UE_LOG(LogFixBlueprints, Warning,
            TEXT("UEditorUtilityWidget class not found — skipping widget reparenting."));
    }
    else
    {
        for (const FString& WBPath : WidgetReparentPaths)
        {
            UBlueprint* WB = LoadObject<UBlueprint>(nullptr, *WBPath);
            if (!WB)
            {
                UE_LOG(LogFixBlueprints, Warning, TEXT("Could not load Widget Blueprint: %s"), *WBPath);
                continue;
            }

            UE_LOG(LogFixBlueprints, Display, TEXT("Widget reparent: %s"), *WBPath);

            // Only reparent if still using a runtime base (UUserWidget or subclass
            // that is NOT already editor-only).
            const bool bAlreadyEditorOnly = WB->ParentClass &&
                WB->ParentClass->IsChildOf(EditorUtilityWidgetClass);
            if (bAlreadyEditorOnly)
            {
                UE_LOG(LogFixBlueprints, Display, TEXT("  Already an editor widget — skipping."));
                continue;
            }

            WB->Modify();
            WB->ParentClass = EditorUtilityWidgetClass;
            UE_LOG(LogFixBlueprints, Display,
                TEXT("  Reparented to UEditorUtilityWidget."));

            FBlueprintEditorUtils::RefreshAllNodes(WB);

            UE_LOG(LogFixBlueprints, Display, TEXT("  Recompiling..."));
            FKismetEditorUtilities::CompileBlueprint(WB,
                EBlueprintCompileOptions::SkipGarbageCollection |
                EBlueprintCompileOptions::BatchCompile);

            UPackage* WBPackage = WB->GetOutermost();
            WBPackage->MarkPackageDirty();

            FString WBFilePath;
            if (FPackageName::TryConvertLongPackageNameToFilename(
                    WBPackage->GetName(), WBFilePath, FPackageName::GetAssetPackageExtension()))
            {
                FSavePackageArgs SaveArgs;
                SaveArgs.SaveFlags = SAVE_None;
                if (UPackage::SavePackage(WBPackage, nullptr, *WBFilePath, SaveArgs))
                {
                    UE_LOG(LogFixBlueprints, Display, TEXT("  Saved: %s"), *WBFilePath);
                    ++TotalFixed;
                }
                else
                {
                    UE_LOG(LogFixBlueprints, Error, TEXT("  Save FAILED for: %s"), *WBFilePath);
                }
            }
        }
    }

    UE_LOG(LogFixBlueprints, Display, TEXT("Done. Fixed and saved %d Blueprint(s)."), TotalFixed);
    return 0;
}
