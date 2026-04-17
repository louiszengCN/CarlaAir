// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using System;
using System.IO;
using UnrealBuildTool;

public class Carla : ModuleRules
{
  bool UsingCarSim = false;
  bool UsingChrono = false;
  bool UsingPytorch = false;
  bool UsingRos2 = false;
  private bool IsWindows(ReadOnlyTargetRules Target)
  {
    return (Target.Platform == UnrealTargetPlatform.Win64); // UE5: Win32 removed
  }

  public Carla(ReadOnlyTargetRules Target) : base(Target)
  {
    PCHUsage = PCHUsageMode.UseSharedPCHs; // UE5: disable IWYU enforcement for legacy includes
    bLegacyPublicIncludePaths = true; // UE5: restore module src-dir in include path for Carla.h resolution
    PrivatePCHHeaderFile = "Carla.h";
    CppStandard = CppStandardVersion.Cpp20;

    if (IsWindows(Target))
    {
      bEnableExceptions = true;
    }

    // Eigen3 - required for vehicle physics computations
    string CarlaPluginPath = Path.GetFullPath(ModuleDirectory);
    // UE5: Use the engine's bundled Eigen module (includes all required defines)
    AddEngineThirdPartyPrivateStaticDependencies(Target, "Eigen");

    // Read config about carsim
    string ConfigDir =  Path.GetFullPath(Path.Combine(CarlaPluginPath, "../../../../Config/"));
    string OptionalModulesFile = Path.Combine(ConfigDir, "OptionalModules.ini");
    string[] text = System.IO.File.ReadAllLines(OptionalModulesFile);
    foreach (string line in text)
    {
      if (line.Contains("CarSim ON"))
      {
        Console.WriteLine("Enabling carsim");
        UsingCarSim = true;
        PublicDefinitions.Add("WITH_CARSIM");
        PrivateDefinitions.Add("WITH_CARSIM");
      }
      if (line.Contains("Chrono ON"))
      {
        Console.WriteLine("Enabling chrono");
        UsingChrono = true;
        PublicDefinitions.Add("WITH_CHRONO");
        PrivateDefinitions.Add("WITH_CHRONO");
      }
      if (line.Contains("Pytorch ON"))
      {
        Console.WriteLine("Enabling pytorch");
        UsingPytorch = true;
        PublicDefinitions.Add("WITH_PYTORCH");
        PrivateDefinitions.Add("WITH_PYTORCH");
      }

      if (line.Contains("Ros2 ON"))
      {
        Console.WriteLine("Enabling ros2");
        UsingRos2 = true;
        PublicDefinitions.Add("WITH_ROS2");
        PrivateDefinitions.Add("WITH_ROS2");
      }
    }

    PublicIncludePaths.AddRange(
      new string[] {
        ModuleDirectory, // UE5: explicitly export module source dir so Carla.h is found by dependents
        "/opt/homebrew/include", // UE5: Boost headers from homebrew (CarlaDependencies missing)
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
        "RenderCore",
        "RHI",
        "Renderer",
        "ProceduralMeshComponent",
        "MeshDescription",
        "Projects"
        // ... add other public dependencies that you statically link with here ...
      }
      );
    if (UsingCarSim)
    {
      PublicDependencyModuleNames.AddRange(new string[] { "CarSim" });
    }

	 if (Target.Type == TargetType.Editor)
	 {
		PublicDependencyModuleNames.AddRange(new string[] { "UnrealEd", "DeveloperToolSettings" }); // UE5: UProjectPackagingSettings moved to DeveloperToolSettings
	 }

    PrivateDependencyModuleNames.AddRange(
      new string[]
      {
        "AIModule",
        "AssetRegistry",
        "CoreUObject",
        "Engine",
        "Foliage",
        "HTTP",
        "StaticMeshDescription",
        "ImageWriteQueue",
        "Json",
        "JsonUtilities",
        "Landscape",
        "ChaosVehicles",
        "Chaos",
        "Projects",
        "Slate",
        "SlateCore",
        "PhysicsCore",
        "Projects"
        // ... add private dependencies that you statically link with here ...
      }
      );
    if (UsingCarSim)
    {
      PrivateDependencyModuleNames.AddRange(new string[] { "CarSim" });
      PrivateIncludePathModuleNames.AddRange(new string[] { "CarSim" });
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

  private void AddDynamicLibrary(string library)
  {
    PublicAdditionalLibraries.Add(library);
    RuntimeDependencies.Add(library);
    PublicDelayLoadDLLs.Add(library);
  }
  private void AddDllDependency(string PathToFolder, string DllName)
  {
    string Source = Path.Combine(PathToFolder, DllName);
    string Destination = Path.Combine("$(BinaryOutputDir)", DllName);
    RuntimeDependencies.Add(Destination, Source);
  }

  delegate string ADelegate(string s);

  private void AddBoostLibs(string LibPath)
  {
    string [] files = Directory.GetFiles(LibPath, "*boost*.lib");
    foreach (string file in files) PublicAdditionalLibraries.Add(file);
  }

  private void AddCarlaServerDependency(ReadOnlyTargetRules Target)
  {
    string LibCarlaInstallPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../CarlaDependencies"));

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

    // Link dependencies.
    if (IsWindows(Target))
    {
      AddBoostLibs(Path.Combine(LibCarlaInstallPath, "lib"));
      PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("rpc")));

      if (UseDebugLibs(Target))
      {
        PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("carla_server_debug")));
      }
      else
      {
        PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("carla_server")));
      }
      if (UsingChrono)
      {
        PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("ChronoEngine")));
        PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("ChronoEngine_vehicle")));
        PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("ChronoModels_vehicle")));
        PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("ChronoModels_robot")));
        AddDllDependency(Path.Combine(LibCarlaInstallPath, "dll"), "ChronoEngine.dll");
        AddDllDependency(Path.Combine(LibCarlaInstallPath, "dll"), "ChronoEngine_vehicle.dll");
        AddDllDependency(Path.Combine(LibCarlaInstallPath, "dll"), "ChronoModels_vehicle.dll");
        AddDllDependency(Path.Combine(LibCarlaInstallPath, "dll"), "ChronoModels_robot.dll");
        bUseRTTI = true;
      }

      //OsmToODR
      PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", "sqlite3.lib"));
      PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", "xerces-c_3.lib"));
      PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", "proj.lib"));
      PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", "osm2odr.lib"));
      PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", "zlibstatic.lib"));
    }
    else
    {
      // UE5/macOS: Only add libraries that actually exist on disk to avoid linker errors
      // for missing pre-built CARLA server libraries. These need to be built via the
      // CARLA build system (make LibCarla.client or make LibCarla.server for macOS ARM).
      string RpcLib = Path.Combine(LibCarlaInstallPath, "lib", GetLibName("rpc"));
      if (File.Exists(RpcLib)) PublicAdditionalLibraries.Add(RpcLib);

      string CarlaServerLib = Path.Combine(LibCarlaInstallPath, "lib", GetLibName(UseDebugLibs(Target) ? "carla_server_debug" : "carla_server"));
      if (File.Exists(CarlaServerLib)) PublicAdditionalLibraries.Add(CarlaServerLib);

      if (UsingChrono)
      {
        string[] ChronoLibs = { "libChronoEngine.so", "libChronoEngine_vehicle.so", "libChronoModels_vehicle.so", "libChronoModels_robot.so" };
        foreach (string lib in ChronoLibs)
        {
          string libPath = Path.Combine(LibCarlaInstallPath, "lib", lib);
          if (File.Exists(libPath)) AddDynamicLibrary(libPath);
        }
        bUseRTTI = true;
      }

      if (UsingPytorch)
      {
        // PyTorch + CUDA libs: Linux/CUDA only, skip on macOS
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
          PublicAdditionalLibraries.Add(Path.Combine(LibCarlaInstallPath, "lib", GetLibName("carla_pytorch")));
          PublicAdditionalLibraries.Add("stdc++");
          // Note: remaining PyTorch/CUDA lib additions omitted — requires Linux CUDA build environment
        }
      }

      if (UsingRos2)
      {
        string[] Ros2Libs = {
          GetLibName("carla_fastdds"), "libfoonathan_memory-0.7.3.a", "libfastcdr.a", "libfastrtps.a"
        };
        foreach (string lib in Ros2Libs)
        {
          string libPath = Path.Combine(LibCarlaInstallPath, "lib", lib);
          if (File.Exists(libPath)) PublicAdditionalLibraries.Add(libPath);
        }
      }

      // OsmToODR — Linux uses libc.so explicitly; macOS links libc automatically.
      if (Target.Platform == UnrealTargetPlatform.Linux)
      {
        PublicAdditionalLibraries.Add("/usr/lib/x86_64-linux-gnu/libc.so");
      }
      // sqlite3: Linux uses .so, macOS uses system sqlite or .a
      string Sqlite3Lib = Path.Combine(LibCarlaInstallPath, "lib", "libsqlite3.so");
      if (!File.Exists(Sqlite3Lib)) Sqlite3Lib = Path.Combine(LibCarlaInstallPath, "lib", "libsqlite3.a");
      if (File.Exists(Sqlite3Lib)) PublicAdditionalLibraries.Add(Sqlite3Lib);

      string[] OsmLibs = { "libxerces-c.a", "libproj.a", "libosm2odr.a" };
      foreach (string lib in OsmLibs)
      {
        string libPath = Path.Combine(LibCarlaInstallPath, "lib", lib);
        if (File.Exists(libPath)) PublicAdditionalLibraries.Add(libPath);
      }
    }
    bEnableExceptions = true;

    // Include path.
    string LibCarlaIncludePath = Path.Combine(LibCarlaInstallPath, "include");
    // UE5: if CarlaDependencies/include is missing, use LibCarla source + old rpc headers
    if (!Directory.Exists(LibCarlaIncludePath))
    {
      // Use LibCarla source for carla/ and compiler/ headers (new API)
      LibCarlaIncludePath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../../../LibCarla/source"));
    }

    // Add LibCarla source first (takes priority for carla/ headers)
    PublicIncludePaths.Add(LibCarlaIncludePath);
    PrivateIncludePaths.Add(LibCarlaIncludePath);

    // Add rpc headers from old CarlaDependencies (provides rpc/msgpack.hpp)
    // These must come AFTER LibCarlaIncludePath so new carla/ headers take priority
    string OldCarlaRpc = "/Users/vincent/carla-air-source/CarlaAir/Unreal/CarlaUE4/Plugins/Carla/CarlaDependencies/include";
    if (Directory.Exists(OldCarlaRpc))
    {
      PublicIncludePaths.Add(OldCarlaRpc);
      PrivateIncludePaths.Add(OldCarlaRpc);
    }

    PublicDefinitions.Add("ASIO_NO_EXCEPTIONS");
    PublicDefinitions.Add("BOOST_NO_EXCEPTIONS");
    PublicDefinitions.Add("LIBCARLA_NO_EXCEPTIONS");
    PublicDefinitions.Add("PUGIXML_NO_EXCEPTIONS");
    PublicDefinitions.Add("BOOST_DISABLE_ABI_HEADERS");
    PublicDefinitions.Add("BOOST_TYPE_INDEX_FORCE_NO_RTTI_COMPATIBILITY");
  }
}
