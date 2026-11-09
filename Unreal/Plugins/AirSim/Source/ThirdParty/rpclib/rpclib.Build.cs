// Copyright (c) Cosys-Lab, University of Antwerp. 2026

using UnrealBuildTool;
using System.IO;

// rpclib (https://github.com/rpclib/rpclib) - msgpack-rpc client/server library used for the
// AirSim client<->simulator API transport. See THIRD_PARTY_LICENSES.md at the repository root
// for license details (rpclib itself also bundles asio, {fmt}, and msgpack-c).
public class rpclib : ModuleRules
{
    public rpclib(ReadOnlyTargetRules Target) : base(Target)
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
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "librpc.a"));
        }
    }
}
