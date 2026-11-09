// Copyright (c) Cosys-Lab, University of Antwerp. 2026

using UnrealBuildTool;
using System;
using System.IO;

public class AirSim : ModuleRules
{
    // This module's own directory: Source/AirSim/
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    // AirLib's own public headers, staged here by build.cmd/build.sh (AirLib/include ->
    // Public/AirLib). AirLib's .cpp sources are NOT staged/compiled here - AIRLIB_HEADER_ONLY
    // (below) guards them down to no-ops when included this way, and the real implementation
    // comes from the precompiled AirLib.lib linked in via AddAirLibBinary().
    private string AirLibIncludePath
    {
        get { return Path.Combine(ModulePath, "Public", "AirLib"); }
    }

    // The precompiled AirLib static library, staged by build.cmd/build.sh (AirLib/lib -> Lib/AirLib).
    private string AirLibLibPath
    {
        get { return Path.Combine(ModulePath, "Lib", "AirLib"); }
    }

    // Unreal/Plugins/AirSim (two levels up from Source/AirSim)
    private string AirSimPluginPath
    {
        get { return Directory.GetParent(ModulePath).Parent.FullName; }
    }
    private string ProjectBinariesPath
    {
        get
        {
            return Path.Combine(
                Directory.GetParent(AirSimPluginPath).Parent.FullName, "Binaries");
        }
    }
    private string AirSimPluginDependencyPath
    {
        get { return Path.Combine(AirSimPluginPath, "Dependencies"); }
    }

    public AirSim(ReadOnlyTargetRules Target) : base(Target)
    {
        //bEnforceIWYU = true; //to support 4.16
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        bEnableExceptions = true;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "RHI", "AssetRegistry", "PhysicsCore", "ChaosVehicles", "Landscape", "CinematicCamera" });
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore", "RenderCore", "ChaosVehicles" });

        // ThirdParty dependencies - see Source/ThirdParty/*/ (Eigen, rpclib, MavLinkCom)
        PrivateDependencyModuleNames.AddRange(new string[] { "Eigen", "rpclib", "MavLinkCom" });

        //suppress VC++ proprietary warnings
        PublicDefinitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("HMD_MODULE_INCLUDED=0");

        // AirLib compiles here in header-only mode (see AirLibIncludePath's comment above);
        // the actual implementation is linked in via the precompiled AirLib.lib.
        PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");
        PublicIncludePaths.Add(AirLibIncludePath);
        AddAirLibBinary(Target);

        AddOSLibDependencies(Target);

        PrivateIncludePaths.AddRange(
            new string[] {
            Path.Combine(GetModuleDirectory("Renderer"), "Private"),
            Path.Combine(GetModuleDirectory("Renderer"), "Internal"),
            }
        );
    }

    private void AddAirLibBinary(ReadOnlyTargetRules Target)
    {
        string platformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string configurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicAdditionalLibraries.Add(Path.Combine(AirLibLibPath, platformString, configurationString, "AirLib.lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            PublicAdditionalLibraries.Add(Path.Combine(AirLibLibPath, "libAirLib.a"));
        }
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
}
