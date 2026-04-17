// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

using UnrealBuildTool;

public class CarlaExporter : ModuleRules
{
  public CarlaExporter(ReadOnlyTargetRules Target) : base(Target)
  {
    PCHUsage = ModuleRules.PCHUsageMode.UseSharedPCHs; // UE5: disable IWYU enforcement for legacy includes

    PublicIncludePaths.AddRange(
      new string[] {
        // ... add public include paths required here ...
      }
      );


    PrivateIncludePaths.AddRange(
      new string[] {
        // ... add other private include paths required here ...
      }
      );


    PublicDependencyModuleNames.AddRange(
      new string[]
      {
        "Core",
        // ... add other public dependencies that you statically link with here ...
      }
      );


    PrivateDependencyModuleNames.AddRange(
      new string[]
      {
        "Projects",
        "InputCore",
        "UnrealEd",
        "LevelEditor",
        "CoreUObject",
        "Engine",
        "Slate",
        "SlateCore",
        // "Physx",  // UE5: PhysX removed, using Chaos
        "EditorStyle"
        // ... add private dependencies that you statically link with here ...
      }
      );


    DynamicallyLoadedModuleNames.AddRange(
      new string[]
      {
        // ... add any modules that your module loads dynamically here ...
      }
      );
  }
}
