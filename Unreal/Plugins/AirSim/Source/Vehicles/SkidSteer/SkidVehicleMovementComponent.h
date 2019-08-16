// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "Curves/CurveFloat.h"
#include "UObject/ObjectMacros.h"
#include "SkidVehicleMovementComponent.generated.h"

UENUM(BlueprintType)
enum class SkidVehicleDriveControlModel : uint8
{
	STANDARD = 0, //Left/Right thrust range [0,1]
	SPECIAL //Left/Right thrust range [-1,1]
};

UENUM(BlueprintType)
enum class SkidVehicleDriveControlMethod : uint8
{
	SingleStick,
	DualStick,
};

UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent), hidecategories = (PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class AIRSIM_API USkidVehicleMovementComponent : public UWheeledVehicleMovementComponent
{
	GENERATED_UCLASS_BODY()
	
public:

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetAcceleration(float Acceleration);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetLeftBreak(float LeftBreak);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetRightBreak(float RightBreak);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetLeftThrust(float LeftThrust);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetRightThrust(float RightThrust);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetAcceleration() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetLeftBreak() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetRightBreak() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetLeftThrust() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetRightThrust() const;

public:


	/** Engine */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup)
		FVehicleEngineData EngineSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup)
		FVehicleTransmissionData TransmissionSetup;

	UPROPERTY(EditAnywhere, Category = "Skid Setup")
		uint32 NumOfWheels;




	UPROPERTY(EditAnywhere, Category = "Skid Setup")
		SkidVehicleDriveControlModel SkidControlModel;

	UPROPERTY(EditAnywhere, Category = "Skid Setup")
		SkidVehicleDriveControlMethod SkidControlMethod;

	UPROPERTY(EditAnywhere, Category = "Skid Setup")
		bool bUseDualBreak;

	/** Maximum steering versus forward speed (km/h) */
	UPROPERTY(EditAnywhere, Category = SteeringSetup)
		FRuntimeFloatCurve SteeringCurve;

	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRate LeftThrustRate;
	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRate RightThrustRate;
	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRate RightBrakeRate;
	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRate LeftBrakeRate;

public:

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

protected:

#if WITH_PHYSX && PHYSICS_INTERFACE_PHYSX

	virtual void SetupVehicle() override;

	virtual void SetupWheels(physx::PxVehicleWheelsSimData* PWheelsSimData) override;

	virtual void UpdateSimulation(float DeltaTime) override;

#endif // WITH_PHYSX

protected:


	UPROPERTY(Transient)
		float Acceleration;
	UPROPERTY(Transient)
		float LeftBreak;
	UPROPERTY(Transient)
		float RightBreak;
	UPROPERTY(Transient)
		float LeftThrust;
	UPROPERTY(Transient)
		float RightThrust;

private:



};