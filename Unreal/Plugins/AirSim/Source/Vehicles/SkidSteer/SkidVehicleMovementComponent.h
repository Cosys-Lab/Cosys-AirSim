// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin
// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Curves/CurveFloat.h"
#include "UObject/ObjectMacros.h"
#include "SkidVehicleMovementComponent.generated.h"


UENUM()
namespace EVehicleDifferential4WSkid
{
	enum Type
	{
		LimitedSlip_4W,
		LimitedSlip_FrontDrive,
		LimitedSlip_RearDrive,
		Open_4W,
		Open_FrontDrive,
		Open_RearDrive,
	};
}

USTRUCT()
struct FVehicleEngineDataSkid
{
	GENERATED_BODY()

	FVehicleEngineDataSkid();

	/** Torque (Nm) at a given RPM*/
	UPROPERTY(EditAnywhere, Category = Setup)
	FRuntimeFloatCurve TorqueCurve;

	/** Maximum revolutions per minute of the engine */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float MaxRPM;

	/** Moment of inertia of the engine around the axis of rotation (Kgm^2). */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float MOI;

	/** Damping rate of engine when full throttle is applied (Kgm^2/s) */
	UPROPERTY(EditAnywhere, Category = Setup, AdvancedDisplay, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float DampingRateFullThrottle;

	/** Damping rate of engine in at zero throttle when the clutch is engaged (Kgm^2/s)*/
	UPROPERTY(EditAnywhere, Category = Setup, AdvancedDisplay, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float DampingRateZeroThrottleClutchEngaged;

	/** Damping rate of engine in at zero throttle when the clutch is disengaged (in neutral gear) (Kgm^2/s)*/
	UPROPERTY(EditAnywhere, Category = Setup, AdvancedDisplay, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float DampingRateZeroThrottleClutchDisengaged;

	/** Find the peak torque produced by the TorqueCurve */
	float FindPeakTorque() const;
};


USTRUCT()
struct FVehicleGearDataSkid
{
	GENERATED_BODY()

	FVehicleGearDataSkid();

	/** Determines the amount of torque multiplication*/
	UPROPERTY(EditAnywhere, Category = Setup)
	float Ratio;

	/** Value of engineRevs/maxEngineRevs that is low enough to gear down*/
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"), Category = Setup)
	float DownRatio;

	/** Value of engineRevs/maxEngineRevs that is high enough to gear up*/
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"), Category = Setup)
	float UpRatio;
};

USTRUCT()
struct FVehicleTransmissionDataSkid
{
	GENERATED_BODY()

	FVehicleTransmissionDataSkid();

	/** Whether to use automatic transmission */
	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta = (DisplayName = "Automatic Transmission"))
	bool bUseGearAutoBox;

	/** Time it takes to switch gears (seconds) */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float GearSwitchTime;

	/** Minimum time it takes the automatic transmission to initiate a gear change (seconds)*/
	UPROPERTY(EditAnywhere, Category = Setup, meta = (editcondition = "bUseGearAutoBox", ClampMin = "0.0", UIMin = "0.0"))
	float GearAutoBoxLatency;

	/** The final gear ratio multiplies the transmission gear ratios.*/
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float FinalRatio;

	/** Forward gear ratios (up to 30) */
	UPROPERTY(EditAnywhere, Category = Setup, AdvancedDisplay)
	TArray<FVehicleGearDataSkid> ForwardGears;

	/** Reverse gear ratio */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float ReverseGearRatio;

	/** Value of engineRevs/maxEngineRevs that is high enough to increment gear*/
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float NeutralGearUpRatio;

	/** Strength of clutch (Kgm^2/s)*/
	UPROPERTY(EditAnywhere, Category = Setup, AdvancedDisplay, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float ClutchStrength;
};

UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent), hidecategories = (PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class AIRSIM_API USkidVehicleMovementComponent : public UChaosWheeledVehicleMovementComponent
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
		FVehicleEngineDataSkid EngineSetupLocal;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup)
		FVehicleTransmissionDataSkid TransmissionSetupLocal;

	UPROPERTY(EditAnywhere, Category = "Skid Setup")
		uint32 NumOfWheels;

	/** Maximum steering versus forward speed (km/h) */
	UPROPERTY(EditAnywhere, Category = SteeringSetup)
		FRuntimeFloatCurve SteeringCurve;

	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRateConfig LeftThrustRate;
	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRateConfig RightThrustRate;
	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRateConfig RightBrakeRate;
	UPROPERTY(EditAnywhere, Category = SkidInput, AdvancedDisplay)
		FVehicleInputRateConfig LeftBrakeRate;

public:

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

protected:

	virtual void SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle) override;

	void UpdateSimulation(float DeltaTime, const FChaosVehicleDefaultAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle);

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