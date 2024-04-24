// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin
// Developed by Cosys-Lab, University of Antwerp

#include "SkidVehicleMovementComponent.h"
#include "Components/PrimitiveComponent.h"


FVehicleTransmissionDataSkid::FVehicleTransmissionDataSkid()
	: bUseGearAutoBox(true)
	, GearSwitchTime(0.0f)
	, GearAutoBoxLatency(0.0f)
	, FinalRatio(1.0f)
	, ReverseGearRatio(0.0f)
	, NeutralGearUpRatio(0.0f)
	, ClutchStrength(0.0f)
{

}

FVehicleGearDataSkid::FVehicleGearDataSkid()
	: Ratio(1.0f)
	, DownRatio(0.0f)
	, UpRatio(1.0f)
{

}

FVehicleEngineDataSkid::FVehicleEngineDataSkid()
	: MaxRPM(0.0f)
	, MOI(0.0f)
	, DampingRateFullThrottle(0.0f)
	, DampingRateZeroThrottleClutchEngaged(0.0f)
	, DampingRateZeroThrottleClutchDisengaged(0.0f)
{

}

float FVehicleEngineDataSkid::FindPeakTorque() const
{
	// Find max torque
	float PeakTorque = 0.f;
	TArray<FRichCurveKey> TorqueKeys = TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
	for (int32 KeyIdx = 0; KeyIdx < TorqueKeys.Num(); KeyIdx++)
	{
		FRichCurveKey& Key = TorqueKeys[KeyIdx];
		PeakTorque = FMath::Max(PeakTorque, Key.Value);
	}
	return PeakTorque;
}


USkidVehicleMovementComponent::USkidVehicleMovementComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	// Initialize WheelSetups array with 4 wheels
	WheelSetups.SetNum(4);
}

float FindSkidPeakTorque(const FVehicleEngineDataSkid& Setup)
{
	// Find max torque
	float PeakTorque = 0.f;
	TArray<FRichCurveKey> TorqueKeys = Setup.TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
	for (int32 KeyIdx = 0; KeyIdx < TorqueKeys.Num(); KeyIdx++)
	{
		FRichCurveKey& Key = TorqueKeys[KeyIdx];
		PeakTorque = FMath::Max(PeakTorque, Key.Value);
	}
	return PeakTorque;
}

void USkidVehicleMovementComponent::SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle)
{

	if (!UpdatedPrimitive)
	{
		return;
	}

	if (WheelSetups.Num() % 2 != 0 && WheelSetups.Num() > 20)
	{
		PVehicle = NULL;
		return;
	}



	for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx)
	{
		const FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];
		if (WheelSetup.BoneName == NAME_None)
		{
			return;
		}
	}

	Super::SetupVehicle(PVehicle);
}

#if WITH_EDITOR
void USkidVehicleMovementComponent::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

void USkidVehicleMovementComponent::SetAcceleration(float OtherAcceleration)
{
	this->Acceleration = OtherAcceleration;	
}

void USkidVehicleMovementComponent::SetLeftBreak(float OtherLeftBreak)
{
	this->LeftBreak = OtherLeftBreak;
	SetBrakeInput(GetLeftBreak());
}

void USkidVehicleMovementComponent::SetRightBreak(float OtherRightBreak)
{
	this->RightBreak = OtherRightBreak;
	SetBrakeInput(GetRightBreak());

}

void USkidVehicleMovementComponent::SetYJoy(float OtherNJoyY)
{
	this->nJoyY = OtherNJoyY;
}

void USkidVehicleMovementComponent::SetXJoy(float OtherNJoyX)
{
	this->nJoyX = OtherNJoyX;
	float accel = FMath::Min((float)sqrt(nJoyX * nJoyX + nJoyY * nJoyY), 1.0f);
	SetThrottleInput(GetAcceleration());
	SetYawInput(nJoyX);
}

float USkidVehicleMovementComponent::GetAcceleration() const
{
	return Acceleration;
}

float USkidVehicleMovementComponent::GetLeftBreak() const
{
	return LeftBreak;
}

float USkidVehicleMovementComponent::GetRightBreak() const
{
	return RightBreak;
}

float USkidVehicleMovementComponent::GetYJoy() const
{
	return nJoyY;
}

float USkidVehicleMovementComponent::GetXJoy() const
{
	return nJoyX;
}

void USkidVehicleMovementComponent::SetBreaksOn()
{
	this->toggleBreak = true;
	SetAcceleration(0);
	SetLeftBreak(1);
	SetParked(1);
}

void USkidVehicleMovementComponent::SetBreaksOff()
{
	this->toggleBreak = false;
	SetParked(0);
}