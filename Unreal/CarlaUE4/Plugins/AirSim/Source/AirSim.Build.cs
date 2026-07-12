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
    private string MavLinkComPath
    {
        get { return Path.Combine(ModulePath, "MavLinkCom"); }
    }
    private string AirSimPluginPath
    {
        get { return Directory.GetParent(ModulePath).FullName; }
    }
    private string CarlaAirRootPath
    {
        get { return Path.GetFullPath(Path.Combine(AirSimPluginPath, "..", "..", "..", "..")); }
    }
    private string RpclibInstallPath
    {
        get { return Path.Combine(CarlaAirRootPath, "Build", "rpclib-install"); }
    }
    private string EigenInstallIncludePath
    {
        get { return Path.Combine(CarlaAirRootPath, "Build", "eigen-install", "include"); }
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

    private enum CompileMode
    {
        HeaderOnlyNoRpc,
        HeaderOnlyWithRpc,
        CppCompileNoRpc,
        CppCompileWithRpc
    }

    private void SetupCompileMode(CompileMode mode, ReadOnlyTargetRules Target)
    {
        LoadMavLinkComDependency(Target);

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
                LoadRpclibDependency(Target);
                break;

            case CompileMode.CppCompileNoRpc:
                PublicDefinitions.Add("AIRLIB_NO_RPC=1");
                break;

            case CompileMode.CppCompileWithRpc:
                LoadRpclibDependency(Target);
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

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "RHI", "AssetRegistry", "PhysicsCore", "PhysXVehicles", "PhysXVehicleLib", "PhysX", "APEX", "Landscape", "CinematicCamera" });
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore", "Carla", "Foliage" });

        //suppress VC++ proprietary warnings
        PublicDefinitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("HMD_MODULE_INCLUDED=0");
        PublicDefinitions.Add("NOMINMAX");

        PublicIncludePaths.Add(Path.Combine(AirLibPath, "include"));
        PublicIncludePaths.Add(EigenInstallIncludePath);
        PublicIncludePaths.Add(Path.Combine(ModulePath, "MavLinkCom", "include"));
        PublicIncludePaths.Add(Path.Combine(ModulePath, "MavLinkCom", "common_utils"));
        AddOSLibDependencies(Target);

        SetupCompileMode(CompileMode.HeaderOnlyWithRpc, Target);
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

    private bool LoadMavLinkComDependency(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string ConfigurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";
            string LibraryPath = Path.Combine(MavLinkComPath, "lib", "x64", ConfigurationString, "MavLinkCom.lib");
            if (!File.Exists(LibraryPath))
            {
                throw new BuildException("MavLinkCom library was not found at its Windows build output: " + LibraryPath);
            }

            PublicAdditionalLibraries.Add(LibraryPath);
            PublicDefinitions.Add("WITH_MAVLINKCOM_BINDING=1");
            return true;
        }

        return LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");
    }

    private bool LoadRpclibDependency(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string IncludePath = Path.Combine(RpclibInstallPath, "include");
            string LibraryPath = Path.Combine(RpclibInstallPath, "lib", "rpc.lib");
            if (!Directory.Exists(IncludePath) || !File.Exists(LibraryPath))
            {
                throw new BuildException("rpclib was not found at its Windows install output: " + RpclibInstallPath);
            }

            PublicIncludePaths.Add(IncludePath);
            PublicAdditionalLibraries.Add(LibraryPath);
            PublicDefinitions.Add("WITH_RPCLIB_BINDING=1");
            return true;
        }

        return LoadAirSimDependency(Target, "rpclib", "rpc");
    }

    private bool AddLibDependency(string LibName, string LibPath, string LibFileName, ReadOnlyTargetRules Target, bool IsAddLibInclude)
    {
        string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string ConfigurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";
        bool isLibrarySupported = false;


        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            isLibrarySupported = true;

            PublicAdditionalLibraries.Add(Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib"));
        } else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac) {
            isLibrarySupported = true;
            PublicAdditionalLibraries.Add(Path.Combine(LibPath, "lib" + LibFileName + ".a"));
        }

        if (isLibrarySupported && IsAddLibInclude)
        {
            // Include path
            PublicIncludePaths.Add(Path.Combine(AirLibPath, "deps", LibName, "include"));
        }
        PublicDefinitions.Add(string.Format("WITH_" + LibName.ToUpper() + "_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}
