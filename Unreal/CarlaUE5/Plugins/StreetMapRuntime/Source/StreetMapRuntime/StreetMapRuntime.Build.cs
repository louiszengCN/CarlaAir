// StreetMapRuntime stub — satisfies Blueprint type references from CarlaTools Blueprints.
// Module name MUST be "StreetMapRuntime" to match the original ue4plugins/StreetMapRuntime package path
// (/Script/StreetMapRuntime.*) serialised into the Blueprint .uasset files.
using UnrealBuildTool;
public class StreetMapRuntime : ModuleRules
{
    public StreetMapRuntime(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "ProceduralMeshComponent"
        });
    }
}
