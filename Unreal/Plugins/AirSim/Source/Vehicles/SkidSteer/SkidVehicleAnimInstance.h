// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin

#pragma once

#include "CoreMinimal.h"
#include "VehicleAnimInstance.h"
#include "SkidVehicleAnimInstance.generated.h"

/**
 * 
 */
UCLASS(transient)
class AIRSIM_API USkidVehicleAnimInstance : public UVehicleAnimInstance
{
	GENERATED_UCLASS_BODY()

	/** Makes a montage jump to the end of a named section. */
	UFUNCTION(BlueprintCallable, Category = "Animation")
	class ASkidVehicle * GetSkidVehicle();
	
};
