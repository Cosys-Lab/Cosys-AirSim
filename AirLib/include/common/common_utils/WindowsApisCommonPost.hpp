// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright (c) Cosys-Lab, University of Antwerp. 2026

//there is no #pragma once in this header

//following is required to support Unreal Build System
#if (defined _WIN32 || defined _WIN64) && (defined UE_GAME || defined UE_EDITOR)
#pragma warning(pop)
#include "Windows/HideWindowsPlatformAtomics.h"
#include "Windows/HideWindowsPlatformTypes.h"
#include "Windows/WindowsHWrapper.h"
#endif
