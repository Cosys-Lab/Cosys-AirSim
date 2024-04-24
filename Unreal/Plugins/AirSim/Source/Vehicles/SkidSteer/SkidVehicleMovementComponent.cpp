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
	RightThrustRate.FallRate = 5.f;
	RightThrustRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	RightThrustRate.RiseRate = 2.5f;
	LeftThrustRate.FallRate = 5.f;
	LeftThrustRate.RiseRate = 2.5f;
	LeftThrustRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	LeftBrakeRate.RiseRate = 6.f;
	LeftBrakeRate.FallRate = 10.f;
	LeftBrakeRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	RightBrakeRate.RiseRate = 6.f;
	RightBrakeRate.FallRate = 10.f;
	RightBrakeRate.InputCurveFunction = EInputFunctionType::LinearFunction;

	// Init steering speed curve
	FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
	SteeringCurveData->AddKey(0.f, 1.f);
	SteeringCurveData->AddKey(20.f, 0.9f);
	SteeringCurveData->AddKey(60.f, 0.8f);
	SteeringCurveData->AddKey(120.f, 0.7f);

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


void USkidVehicleMovementComponent::UpdateSimulation(float DeltaTime, const FChaosVehicleDefaultAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle)
{

	float accel = FMath::Min((float)sqrt(nJoyX*nJoyX + nJoyY*nJoyY) , 1.0f);

	// Differential Steering Joystick Algorithm
	// ========================================
	//   by Calvin Hass
	//   https://www.impulseadventure.com/elec/
	//
	// Converts a single dual-axis joystick into a differential
	// drive motor control, with support for both drive, turn
	// and pivot operations.
	//

	// INPUTS
	//	nJoyX;              // Joystick X input                     (-1..+1)
	//	nJoyY;              // Joystick Y input                     (-1..+1)

	// OUTPUTS
	float     nMotMixL;           // Motor (left)  mixed output           (-1..+1)
	float     nMotMixR;           // Motor (right) mixed output           (-1..+1)

	// CONFIG
	// - fPivYLimt  : The threshold at which the pivot action starts
	//                This threshold is measured in units on the Y-axis
	//                away from the X-axis (Y=0). A greater value will assign
	//                more of the joystick's range to pivot actions.
	//                Allowable range: (0..+1)
	float fPivYLimit = 1.00;

	// TEMP VARIABLES
	float   nMotPremixL;    // Motor (left)  premixed output        (-1..+1)
	float   nMotPremixR;    // Motor (right) premixed output        (-1..+1)
	float   nPivSpeed;      // Pivot Speed                          (-1..+1)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )


	// Calculate Drive Turn output due to Joystick X input
	if (nJoyY >= 0) {
		// Forward
		nMotPremixL = (nJoyX >= 0) ? 1 : (1 + nJoyX);
		nMotPremixR = (nJoyX >= 0) ? (1 - nJoyX) : 1;
	}
	else {
		// Reverse
		nMotPremixL = (nJoyX >= 0) ? (1 - nJoyX) : 1;
		nMotPremixR = (nJoyX >= 0) ? 1 : (1 + nJoyX);
	}

	// Scale Drive output due to Joystick Y input (throttle)
	nMotPremixL = nMotPremixL * nJoyY / 1;
	nMotPremixR = nMotPremixR * nJoyY / 1;

	// Now calculate pivot amount
	// - Strength of pivot (nPivSpeed) based on Joystick X input
	// - Blending of pivot vs drive (fPivScale) based on Joystick Y input
	nPivSpeed = nJoyX;
	fPivScale = (abs(nJoyY) >= fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

	// Calculate final mix of Drive and Pivot
	nMotMixL = (1.0 - fPivScale)*nMotPremixL + fPivScale * (nPivSpeed);
	nMotMixR = (1.0 - fPivScale)*nMotPremixR + fPivScale * (-nPivSpeed);
   

	FChaosVehicleDefaultAsyncInput VehicleInputData;	
	VehicleInputData.ControlInputs.YawInput = nJoyX;
	VehicleInputData.ControlInputs.ThrottleInput = accel;

	if (!Super::GetUseAutoGears())
	{
		VehicleInputData.ControlInputs.GearUpInput = bRawGearUpInput;
		VehicleInputData.ControlInputs.GearDownInput = bRawGearDownInput;
	}

	// Toggle on/off digital (hand) break
	if (toggleBreak) {
		VehicleInputData.ControlInputs.BrakeInput = LeftBreak;
		VehicleInputData.ControlInputs.ThrottleInput = 0;
		VehicleInputData.ControlInputs.HandbrakeInput = LeftBreak;
		//UE_LOG(LogTemp, Warning, TEXT("Braking %f %f "), VehicleInputData.getAnalogLeftBrake(), VehicleInputData.getAnalogRightBrake());
	}
	VehicleSimulationPT->UpdateSimulation(DeltaTime, VehicleInputData, Handle);
}

#if WITH_EDITOR
void USkidVehicleMovementComponent::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	if (PropertyName == TEXT("DownRatio"))
	{
		for (int32 GearIdx = 0; GearIdx < TransmissionSetupLocal.ForwardGears.Num(); ++GearIdx)
		{
			FVehicleGearDataSkid& GearData = TransmissionSetupLocal.ForwardGears[GearIdx];
			GearData.DownRatio = FMath::Min(GearData.DownRatio, GearData.UpRatio);
		}
	}
	else if (PropertyName == TEXT("UpRatio"))
	{
		for (int32 GearIdx = 0; GearIdx < TransmissionSetupLocal.ForwardGears.Num(); ++GearIdx)
		{
			FVehicleGearDataSkid& GearData = TransmissionSetupLocal.ForwardGears[GearIdx];
			GearData.UpRatio = FMath::Max(GearData.DownRatio, GearData.UpRatio);
		}
	}
	else if (PropertyName == TEXT("SteeringCurve"))
	{
		//make sure values are capped between 0 and 1
		TArray<FRichCurveKey> SteerKeys = SteeringCurve.GetRichCurve()->GetCopyOfKeys();
		for (int32 KeyIdx = 0; KeyIdx < SteerKeys.Num(); ++KeyIdx)
		{
			float NewValue = FMath::Clamp(SteerKeys[KeyIdx].Value, 0.f, 1.f);
			SteeringCurve.GetRichCurve()->UpdateOrAddKey(SteerKeys[KeyIdx].Time, NewValue);
		}
	}
}
#endif

void USkidVehicleMovementComponent::SetAcceleration(float OtherAcceleration)
{
	this->Acceleration = OtherAcceleration;
}

void USkidVehicleMovementComponent::SetLeftBreak(float OtherLeftBreak)
{
	this->LeftBreak = OtherLeftBreak;
}

void USkidVehicleMovementComponent::SetRightBreak(float OtherRightBreak)
{
	this->RightBreak = OtherRightBreak;
}

void USkidVehicleMovementComponent::SetYJoy(float OtherNJoyY)
{
	this->nJoyY = OtherNJoyY;
}

void USkidVehicleMovementComponent::SetXJoy(float OtherNJoyX)
{
	this->nJoyX = OtherNJoyX;
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
}

void USkidVehicleMovementComponent::SetBreaksOff()
{
	this->toggleBreak = false;
}