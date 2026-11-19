// Copyright (c) Cosys-Lab, University of Antwerp. 2026

using UnrealBuildTool;
using System.IO;

// AirLib - the engine-independent simulation library of Cosys-AirSim (vehicle physics, sensors, the RPC API).
// It is built outside Unreal by build.cmd/build.sh and staged here: its headers in include/ and the precompiled
// static library in lib/. AirLib's .cpp sources are not shipped; AIRLIB_HEADER_ONLY guards them down to no-ops when
// their headers are included, and the implementation comes from the static library.
public class AirSimAirLib : ModuleRules
{
    public AirSimAirLib(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
        PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");

        // AirLib's headers include Eigen, rpclib, MavLinkCom, nlohmann/json and CTPL
        PublicDependencyModuleNames.AddRange(new string[] { "AirSimEigen", "AirSimRpclib", "AirSimMavLinkCom", "AirSimVendor" });

        string platformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string configurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", platformString, configurationString, "AirLib.lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            // build.sh puts the library in a Linux or Mac subfolder, so one plugin can hold the libraries of every platform
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", Target.Platform == UnrealTargetPlatform.Mac ? "Mac" : "Linux", "libAirLib.a"));
        }
    }
}
