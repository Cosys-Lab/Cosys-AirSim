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

        // ThirdParty dependencies - see Source/ThirdParty/*/ (AirLib, Eigen, rpclib, MavLinkCom, and the small
        // header-only libraries AirLib uses in AirSimVendor: nlohmann/json and CTPL).
        // Module names are prefixed with "AirSim" because UBT module names are global: the engine
        // already ships its own "Eigen" module, and other AirSim-derived plugins may ship rpclib/MavLinkCom.
        // AirLib is public: the AirSim headers include AirLib headers.
        PublicDependencyModuleNames.Add("AirSimAirLib");
        PrivateDependencyModuleNames.AddRange(new string[] { "AirSimEigen", "AirSimRpclib", "AirSimMavLinkCom", "AirSimVendor" });

        //suppress VC++ proprietary warnings
        PublicDefinitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("HMD_MODULE_INCLUDED=0");

        AddOSLibDependencies(Target);

        PrivateIncludePaths.AddRange(
            new string[] {
            Path.Combine(GetModuleDirectory("Renderer"), "Private"),
            Path.Combine(GetModuleDirectory("Renderer"), "Internal"),
            }
        );
    }

    private void AddOSLibDependencies(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // OS libraries are resolved from the SDK's library paths, so they go in
            // PublicSystemLibraries (PublicAdditionalLibraries expects full file paths).
            // for SHGetFolderPath.
            PublicSystemLibraries.Add("Shell32.lib");

            //for joystick support
            PublicSystemLibraries.Add("dinput8.lib");
            PublicSystemLibraries.Add("dxguid.lib");
        }

        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // needed when packaging
            PublicSystemLibraries.Add("stdc++");
            PublicSystemLibraries.Add("supc++");
        }
    }
}
