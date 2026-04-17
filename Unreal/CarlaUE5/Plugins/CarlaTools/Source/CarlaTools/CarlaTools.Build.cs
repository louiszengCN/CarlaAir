// Copyright Epic Games, Inc. All Rights Reserved.

using System;
using System.IO;
using UnrealBuildTool;

public class CarlaTools : ModuleRules
{
  bool bUsingSimReadyPlugins = false;
  private bool IsWindows(ReadOnlyTargetRules Target)
  {
    return (Target.Platform == UnrealTargetPlatform.Win64); // UE5: Win32 removed
  }

	public CarlaTools(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseSharedPCHs; // UE5: disable IWYU enforcement for legacy includes

    // PrivatePCHHeaderFile = "Carla.h";

    if (IsWindows(Target))
    {
      bEnableExceptions = true;
    }

    string CarlaPluginPath = Path.GetFullPath( ModuleDirectory );
    string ConfigDir =  Path.GetFullPath(Path.Combine(CarlaPluginPath, "../../../../Config/"));
    string OptionalModulesFile = Path.Combine(ConfigDir, "OptionalModules.ini");
    string[] text = System.IO.File.ReadAllLines(OptionalModulesFile);
    foreach (string line in text)
    {
      if (line.Contains("SimReady ON"))
      {
        Console.WriteLine("Enabling SimReady Converter Plugins");
        bUsingSimReadyPlugins = true;
        PublicDefinitions.Add("WITH_SIMREADY");
        PrivateDefinitions.Add("WITH_SIMREADY");
      }
    }

		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);


		PrivateIncludePaths.AddRange(
			new string[] {
				// UE5: FoliageEdMode.h (private) removed — FEdModeFoliage::AddInstances is a hidden symbol.
				// Replaced with public API (FoliageTrace + FFoliageInfo::AddInstances) in MapGeneratorWidget.cpp.
			}
			);


		PublicDependencyModuleNames.AddRange(
			new string[]
			{
        "Core",
        "ProceduralMeshComponent",
        "MeshDescription",
        "RawMesh",
        "AssetTools"
				// ... add other public dependencies that you statically link with here ...
			}
			);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"UnrealEd",
				"DesktopPlatform", // UE5: for IDesktopPlatform / file dialogs
				"StaticMeshEditor", // UE5: for UStaticMeshEditorSubsystem (AddSimpleCollisions)
				"Blutility",
				"UMG",
				"EditorScriptingUtilities",
				"Landscape",
				"Foliage",
				"FoliageEdit",
        "MeshMergeUtilities",
				"Carla",
        "StaticMeshDescription",
				"ChaosVehicles",
        "Json",
        "JsonUtilities",
        "Networking",
        "Sockets",
        "HTTP",
        "RHI",
        "RenderCore",
        "MeshMergeUtilities",
        // "StreetMapImporting",  // UE5: StreetMap plugin unavailable
        // "StreetMapRuntime"     // UE5: StreetMap plugin unavailable
				// ... add private dependencies that you statically link with here ...
			}
			);
    if(bUsingSimReadyPlugins)
    {
      PrivateDependencyModuleNames.AddRange(
        new string[]
        {
          "SimReadyUSD",
          "SimReadyRuntime"
        });
    }

		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
		AddCarlaServerDependency(Target);
	}

  private bool UseDebugLibs(ReadOnlyTargetRules Target)
  {
    if (IsWindows(Target))
    {
      // In Windows, Unreal uses the Release C++ Runtime (CRT) even in debug
      // mode, so unless we recompile the engine we cannot link the debug
      // libraries.
      return false;
    }
    else
    {
      return false;
    }
  }

  delegate string ADelegate(string s);

  private void AddBoostLibs(string LibPath)
  {
    string [] files = Directory.GetFiles(LibPath, "*boost*.lib");
    foreach (string file in files)
    {
      PublicAdditionalLibraries.Add(file);
    }
  }


	private void AddCarlaServerDependency(ReadOnlyTargetRules Target)
	{
		string LibCarlaInstallPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../Carla/CarlaDependencies"));

		ADelegate GetLibName = (string BaseName) => {
			if (IsWindows(Target))
			{
				return BaseName + ".lib";
			}
			else
			{
				return "lib" + BaseName + ".a";
			}
		};

    // Link dependencies. UE5/macOS: guard with File.Exists — CarlaDependencies may not be built yet.
    if (IsWindows(Target))
    {
      AddBoostLibs(Path.Combine(LibCarlaInstallPath, "lib"));
      string RpcLib = Path.Combine(LibCarlaInstallPath, "lib", GetLibName("rpc"));
      if (File.Exists(RpcLib)) PublicAdditionalLibraries.Add(RpcLib);

      string ServerLib = Path.Combine(LibCarlaInstallPath, "lib", GetLibName(UseDebugLibs(Target) ? "carla_server_debug" : "carla_server"));
      if (File.Exists(ServerLib)) PublicAdditionalLibraries.Add(ServerLib);
    }
    else
    {
      string RpcLib = Path.Combine(LibCarlaInstallPath, "lib", GetLibName("rpc"));
      if (File.Exists(RpcLib)) PublicAdditionalLibraries.Add(RpcLib);

      string ServerLib = Path.Combine(LibCarlaInstallPath, "lib", GetLibName(UseDebugLibs(Target) ? "carla_server_debug" : "carla_server"));
      if (File.Exists(ServerLib)) PublicAdditionalLibraries.Add(ServerLib);
    }
    // Include path — only add if it exists to avoid UBT warnings.
    string LibCarlaIncludePath = Path.Combine(LibCarlaInstallPath, "include");
    if (Directory.Exists(LibCarlaIncludePath))
    {
      PublicIncludePaths.Add(LibCarlaIncludePath);
      PrivateIncludePaths.Add(LibCarlaIncludePath);
    }

    PublicDefinitions.Add("ASIO_NO_EXCEPTIONS");
    PublicDefinitions.Add("BOOST_NO_EXCEPTIONS");
    // PublicDefinitions.Add("LIBCARLA_NO_EXCEPTIONS");
    PublicDefinitions.Add("PUGIXML_NO_EXCEPTIONS");
    PublicDefinitions.Add("BOOST_DISABLE_ABI_HEADERS");
    PublicDefinitions.Add("BOOST_TYPE_INDEX_FORCE_NO_RTTI_COMPATIBILITY");
	}
}
