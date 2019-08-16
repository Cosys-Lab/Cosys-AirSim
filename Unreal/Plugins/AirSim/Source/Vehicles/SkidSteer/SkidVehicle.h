// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicle.h"
#include "SkidVehicle.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API ASkidVehicle : public AWheeledVehicle
{
	GENERATED_UCLASS_BODY()
	
public:

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	virtual void PostInitializeComponents() override;

public:

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetLeftThrust(float LeftThrust);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetRightThrust(float RightThrust);
};

