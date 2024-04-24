// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin
// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "Vehicles/SkidSteer/SkidVehicle.h"
#include "VehicleAnimationInstance.h"
#include "SkidVehicleAnimInstance.generated.h"

/**
 * 
 */
UCLASS(transient)
class AIRSIM_API USkidVehicleAnimInstance : public UVehicleAnimationInstance
{
	GENERATED_UCLASS_BODY()

	/** Makes a montage jump to the end of a named section. */
	UFUNCTION(BlueprintCallable, Category = "Animation")
	class ASkidVehicle * GetSkidVehicle();
	
};
