// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.Collections.Generic;
using System;
using System.IO;

public class CarlaUE5EditorTarget : TargetRules
{
	public CarlaUE5EditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V6;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
		CppStandard = CppStandardVersion.Cpp20;
		ExtraModuleNames.Add("CarlaUE5");

		string ConfigDir = Path.GetDirectoryName(ProjectFile.ToString()) + "/Config/";
		string OptionalModulesFile = Path.Combine(ConfigDir, "OptionalModules.ini");
		string[] text = System.IO.File.ReadAllLines(OptionalModulesFile);

		bool UnityOn = true;

		foreach (string line in text) {
			if (line.Contains("Unity OFF"))
			{
				UnityOn = false;
			}
		}

		if (!UnityOn) {
			Console.WriteLine("Disabling unity");
			bUseUnityBuild = false;
			bForceUnityBuild = false;
			bUseAdaptiveUnityBuild = false;
		}
	}
}
