// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class BlocksEditorTarget : TargetRules
{
	public BlocksEditorTarget(TargetInfo Target) : base(Target)
	{
	    DefaultBuildSettings = BuildSettingsVersion.V5;
        Type = TargetType.Editor;
		ExtraModuleNames.AddRange(new string[] { "Blocks" });
        IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
        //bUseUnityBuild = false;
        //bUsePCHFiles = false;
    }
}
