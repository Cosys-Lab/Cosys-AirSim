// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin
// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "Curves/CurveFloat.h"
#include "UObject/ObjectMacros.h"
#include "SkidVehicleMovementComponent.generated.h"

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
		void SetYJoy(float nJoyY);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetXJoy(float nJoyX);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetAcceleration() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetLeftBreak() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetRightBreak() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetYJoy() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		float GetXJoy() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetBreaksOn();

	UFUNCTION(BlueprintCallable, Category = "Game|Components|SkidVehicleMovement")
		void SetBreaksOff();


public:


	/** Engine */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup)
		FVehicleEngineData EngineSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup)
		FVehicleTransmissionData TransmissionSetup;

	UPROPERTY(EditAnywhere, Category = "Skid Setup")
		uint32 NumOfWheels;

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
		float nJoyX;
	UPROPERTY(Transient)
		float nJoyY;
	UPROPERTY(Transient)
		bool toggleBreak;

private:



};