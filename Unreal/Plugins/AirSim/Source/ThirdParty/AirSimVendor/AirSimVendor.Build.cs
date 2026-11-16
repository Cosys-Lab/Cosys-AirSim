// Copyright (c) Cosys-Lab, University of Antwerp. 2026

using UnrealBuildTool;
using System.IO;

// Small header-only third-party libraries used by AirLib: nlohmann/json (include/common_utils/json.hpp)
// and CTPL (include/ctpl_stl.h). The build scripts copy them here from AirLib/include/common/common_utils/.
// See THIRD_PARTY_LICENSES.md at the repository root for license details.
public class AirSimVendor : ModuleRules
{
    public AirSimVendor(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        // AirLib includes these as "common_utils/json.hpp" and "ctpl_stl.h"
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
    }
}
