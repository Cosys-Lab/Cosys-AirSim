// Copyright (c) Cosys-Lab, University of Antwerp. 2026

using UnrealBuildTool;
using System.IO;

// MavLinkCom - MAVLink communication library used for PX4/ArduPilot hardware/software-in-the-loop
// support, consumed here as a prebuilt headers+library dependency (its own .cpp sources are built
// separately, outside the Unreal plugin, via build.cmd/build.sh). Bundles the generated MAVLink
// common.xml protocol headers - see THIRD_PARTY_LICENSES.md at the repository root.
public class MavLinkCom : ModuleRules
{
    public MavLinkCom(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
        PublicDefinitions.Add("WITH_MAVLINKCOM_BINDING=1");

        string platformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string configurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", platformString, configurationString, "MavLinkCom.lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "libMavLinkCom.a"));
        }
    }
}
