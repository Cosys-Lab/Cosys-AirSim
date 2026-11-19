// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class BlocksEditorTarget : TargetRules
{
	public BlocksEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		ExtraModuleNames.AddRange(new string[] { "Blocks" });
        DefaultBuildSettings = BuildSettingsVersion.V2;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_2;
		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			string Msvc = FindMsvcForUE52();
			if (Msvc != null)
				WindowsPlatform.CompilerVersion = Msvc;
			else
				System.Console.WriteLine("Warning: Unreal Engine 5.2 needs an MSVC toolset older than 14.40, for example \"MSVC v143 - VS 2022 C++ x64/x86 build tools (v14.38-17.8)\" from the Visual Studio Installer");
		}
        //bUseUnityBuild = false;
        //bUsePCHFiles = false;
    }

	// Unreal Engine 5.2 does not compile with MSVC 14.40 (VS 2022 17.10) or newer. Use the toolset set in the
	// AIRSIM_VCTOOLSVERSION environment variable, else the newest installed Visual Studio 2022 toolset older than 14.40.
	static string FindMsvcForUE52()
	{
		string Pinned = System.Environment.GetEnvironmentVariable("AIRSIM_VCTOOLSVERSION");
		if (!string.IsNullOrEmpty(Pinned))
			return Pinned;
		System.Version Best = null;
		foreach (System.Environment.SpecialFolder Root in new[] { System.Environment.SpecialFolder.ProgramFiles, System.Environment.SpecialFolder.ProgramFilesX86 })
		{
			string VsDir = System.IO.Path.Combine(System.Environment.GetFolderPath(Root), "Microsoft Visual Studio", "2022");
			if (!System.IO.Directory.Exists(VsDir))
				continue;
			foreach (string Edition in System.IO.Directory.GetDirectories(VsDir))
			{
				string MsvcDir = System.IO.Path.Combine(Edition, "VC", "Tools", "MSVC");
				if (!System.IO.Directory.Exists(MsvcDir))
					continue;
				foreach (string Dir in System.IO.Directory.GetDirectories(MsvcDir))
				{
					System.Version V;
					if (System.Version.TryParse(System.IO.Path.GetFileName(Dir), out V) && V.Major == 14 && V.Minor < 40 && (Best == null || V > Best))
						Best = V;
				}
			}
		}
		return Best != null ? Best.ToString() : null;
	}
}
