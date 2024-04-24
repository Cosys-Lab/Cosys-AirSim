// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin
// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "WheeledVehiclePawn.h"
#include "SkidVehicle.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API ASkidVehicle : public AWheeledVehiclePawn
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

	class USkidVehicleMovementComponent* GetSkidVehicleMovement() const;

public:

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetXJoy(float XThrust);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetYJoy(float YThrust);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetBreaksOn();

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetBreaksOff();
};

