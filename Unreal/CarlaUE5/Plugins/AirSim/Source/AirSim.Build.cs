// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using UnrealBuildTool;
using System.IO;

public class AirSim : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string AirLibPath
    {
        get { return Path.Combine(ModulePath, "AirLib"); }
    }
    private string AirSimPluginPath
    {
        get { return Directory.GetParent(ModulePath).FullName; }
    }
    private string ProjectBinariesPath
    {
        get { return Path.Combine(
                Directory.GetParent(AirSimPluginPath).Parent.FullName, "Binaries");
        }
    }
    private string AirSimPluginDependencyPath
    {
        get { return Path.Combine(AirSimPluginPath, "Dependencies"); }
    }
    private string CarlaDependenciesPath
    {
        get
        {
            return Path.GetFullPath(Path.Combine(
                AirSimPluginPath,
                "..",
                "Carla",
                "CarlaDependencies"));
        }
    }

    private enum CompileMode
    {
        HeaderOnlyNoRpc,
        HeaderOnlyWithRpc,
        CppCompileNoRpc,
        CppCompileWithRpc
    }

    private void SetupCompileMode(CompileMode mode, ReadOnlyTargetRules Target)
    {
        switch (mode)
        {
            case CompileMode.HeaderOnlyNoRpc:
                PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");
                PublicDefinitions.Add("AIRLIB_NO_RPC=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                break;

            case CompileMode.HeaderOnlyWithRpc:
                PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;

            case CompileMode.CppCompileNoRpc:
                LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");
                PublicDefinitions.Add("AIRLIB_NO_RPC=1");
                break;

            case CompileMode.CppCompileWithRpc:
                LoadAirSimDependency(Target, "rpclib", "rpc");
                break;

            default:
                throw new System.Exception("CompileMode specified in plugin's Build.cs file is not recognized");
        }

    }

    public AirSim(ReadOnlyTargetRules Target) : base(Target)
    {
        //bEnforceIWYU = true; //to support 4.16
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "RHI", "AssetRegistry", "PhysicsCore", "Landscape", "CinematicCamera", "ChaosVehicles" }); // UE5: PhysXVehicles/PhysXVehicleLib/PhysX/APEX removed
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore", "Carla", "Foliage" });
        AddEngineThirdPartyPrivateStaticDependencies(Target, "Eigen");

        //suppress VC++ proprietary warnings
        PublicDefinitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("HMD_MODULE_INCLUDED=0");

        PublicIncludePaths.Add(Path.Combine(AirLibPath, "include"));
        AddOSLibDependencies(Target);

        SetupCompileMode(CompileMode.CppCompileWithRpc, Target);
    }

    private void AddOSLibDependencies(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // for SHGetFolderPath.
            PublicAdditionalLibraries.Add("Shell32.lib");

            //for joystick support
            PublicAdditionalLibraries.Add("dinput8.lib");
            PublicAdditionalLibraries.Add("dxguid.lib");
        }

		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			// needed when packaging
			PublicAdditionalLibraries.Add("stdc++");
			PublicAdditionalLibraries.Add("supc++");
		}
    }

    static void CopyFileIfNewer(string srcFilePath, string destFolder)
    {
        FileInfo srcFile = new FileInfo(srcFilePath);
        FileInfo destFile = new FileInfo(Path.Combine(destFolder, srcFile.Name));
        if (!destFile.Exists || srcFile.LastWriteTime > destFile.LastWriteTime)
        {
            srcFile.CopyTo(destFile.FullName, true);
        }
        //else skip
    }

    private bool LoadAirSimDependency(ReadOnlyTargetRules Target, string LibName, string LibFileName)
    {
        string LibrariesPath = Path.Combine(AirLibPath, "deps", LibName, "lib");
        return AddLibDependency(LibName, LibrariesPath, LibFileName, Target, true);
    }

    private bool AddLibDependency(string LibName, string LibPath, string LibFileName, ReadOnlyTargetRules Target, bool IsAddLibInclude)
    {
        string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string ConfigurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";
        bool isLibrarySupported = false;
        string libraryPath = "";
        string includePath = Path.Combine(AirLibPath, "deps", LibName, "include");

        if (LibName == "rpclib")
        {
            string carlaRpcLib = Path.Combine(CarlaDependenciesPath, "lib", "librpc.a");
            string carlaRpcInclude = Path.Combine(CarlaDependenciesPath, "include");

            if (Target.Platform == UnrealTargetPlatform.Win64)
            {
                libraryPath = Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib");
            }
            else if (File.Exists(carlaRpcLib))
            {
                libraryPath = carlaRpcLib;
                includePath = carlaRpcInclude;
            }
            else
            {
                libraryPath = Path.Combine(LibPath, "lib" + LibFileName + ".a");
            }
        }
        else if (LibName == "MavLinkCom")
        {
            if (Target.Platform == UnrealTargetPlatform.Win64)
            {
                libraryPath = Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib");
            }
            else
            {
                libraryPath = Path.Combine(LibPath, "lib" + LibFileName + ".a");
            }
        }
        else if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            libraryPath = Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib");
        }
        else
        {
            libraryPath = Path.Combine(LibPath, "lib" + LibFileName + ".a");
        }

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            isLibrarySupported = File.Exists(libraryPath);
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            isLibrarySupported = File.Exists(libraryPath);
        }

        if (isLibrarySupported)
        {
            PublicAdditionalLibraries.Add(libraryPath);
        }

        if (isLibrarySupported && IsAddLibInclude)
        {
            PublicIncludePaths.Add(includePath);
        }
        PublicDefinitions.Add(string.Format("WITH_" + LibName.ToUpper() + "_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}
