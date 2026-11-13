// Copyright (c) Cosys-Lab, University of Antwerp. 2026

using UnrealBuildTool;
using System.IO;

// rpclib (https://github.com/rpclib/rpclib) - msgpack-rpc client/server library used for the
// AirSim client<->simulator API transport. See THIRD_PARTY_LICENSES.md at the repository root
// for license details (rpclib itself also bundles asio, {fmt}, and msgpack-c).
public class AirSimRpclib : ModuleRules
{
    public AirSimRpclib(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
        PublicDefinitions.Add("WITH_RPCLIB_BINDING=1");

        string platformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string configurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", platformString, configurationString, "rpc.lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            // build.sh puts the library in a Linux or Mac subfolder, older builds put it directly in lib
            string platformLib = Path.Combine(ModuleDirectory, "lib", Target.Platform == UnrealTargetPlatform.Mac ? "Mac" : "Linux", "librpc.a");
            PublicAdditionalLibraries.Add(File.Exists(platformLib) ? platformLib : Path.Combine(ModuleDirectory, "lib", "librpc.a"));
        }
    }
}
