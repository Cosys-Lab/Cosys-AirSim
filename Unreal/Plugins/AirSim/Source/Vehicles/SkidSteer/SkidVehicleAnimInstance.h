// Fill out your copyright notice in the Description page of Project Settings.

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
