// Copyright (c) Cosys-Lab, University of Antwerp. 2026

using UnrealBuildTool;

// Eigen (https://eigen.tuxfamily.org) - header-only linear algebra library used throughout
// AirLib. See THIRD_PARTY_LICENSES.md at the repository root for license details.
public class AirSimEigen : ModuleRules
{
    public AirSimEigen(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        // Headers live under ./Eigen/ (the library's own root folder, e.g. Eigen/Dense),
        // so the include path is this module's own directory, not the Eigen/ subfolder.
        PublicIncludePaths.Add(ModuleDirectory);
    }
}
