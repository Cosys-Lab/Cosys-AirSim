#pragma once

#include "CoreMinimal.h"

class AIRSIM_API UrdfOrigin
{
public:
	FVector Origin = FVector::ZeroVector;
	FRotator RollPitchYaw = FRotator::ZeroRotator;
};