// Copyright (c) Cosys-Lab, University of Antwerp. 2026

// Private precompiled header of the AirSim module. It is kept this small on purpose: every compile loads the
// whole precompiled header into memory, and the engine's shared PCH needs about 2.4 GB per compile, which runs
// out of memory on build machines that compile many files in parallel.
#pragma once

#include "CoreMinimal.h"
