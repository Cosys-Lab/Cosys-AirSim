// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright (c) Cosys-Lab, University of Antwerp. 2026

//there is no #pragma once in this header

//following is required to support Unreal Build System
#if (defined _WIN32 || defined _WIN64) && (defined UE_GAME || defined UE_EDITOR)
#include "CoreTypes.h"
#include "Windows/WindowsHWrapper.h"
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/AllowWindowsPlatformAtomics.h"
//remove warnings for VC++
#pragma warning(push)
#pragma warning(disable : 4191 6000 28251)
#pragma warning(disable : 4996) //warning C4996: This function or variable may be unsafe. Consider using xxx instead.
#pragma warning(disable : 4005) //warning C4005: 'TEXT': macro redefinition
#endif
