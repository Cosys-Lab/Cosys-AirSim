// Fill out your copyright notice in the Description page of Project Settings.


#include "SkidVehicleAnimInstance.h"
#include "SkidVehicle.h"
#include "AnimationRuntime.h"

USkidVehicleAnimInstance::USkidVehicleAnimInstance(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

class ASkidVehicle* USkidVehicleAnimInstance::GetSkidVehicle()
{
	return Cast<ASkidVehicle>(GetOwningActor());
}