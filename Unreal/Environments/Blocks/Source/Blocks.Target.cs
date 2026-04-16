// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class BlocksTarget : TargetRules
{
	public BlocksTarget(TargetInfo Target) : base(Target)
	{
        DefaultBuildSettings = BuildSettingsVersion.V6;
        IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
        Type = TargetType.Game;
		ExtraModuleNames.AddRange(new string[] { "Blocks" });
		if (Target.Platform == UnrealTargetPlatform.Linux)
			bUsePCHFiles = false;
	}
}
