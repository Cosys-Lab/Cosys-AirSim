// Based on the work by Leon Rosengarten and Boone Adkins.
// https://github.com/b-adkins/UE4-TankVehiclePlugin
// Developed by Cosys-Lab, University of Antwerp

#include "SkidVehicleMovementComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Runtime/Engine/Private/PhysicsEngine/PhysXSupport.h"
#include "PhysXPublic.h"
#include "PhysXIncludes.h"
USkidVehicleMovementComponent::USkidVehicleMovementComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
#if WITH_PHYSX

	RightThrustRate.FallRate = 5.f;
	RightThrustRate.RiseRate = 2.5f;
	LeftThrustRate.FallRate = 5.f;
	LeftThrustRate.RiseRate = 2.5f;
	LeftBrakeRate.RiseRate = 6.f;
	LeftBrakeRate.FallRate = 10.f;
	RightBrakeRate.RiseRate = 6.f;
	RightBrakeRate.FallRate = 10.f;

	PxVehicleEngineData DefEngineData;
	EngineSetup.MOI = DefEngineData.mMOI;
	EngineSetup.MaxRPM = OmegaToRPM(DefEngineData.mMaxOmega);
	EngineSetup.DampingRateFullThrottle = DefEngineData.mDampingRateFullThrottle;
	EngineSetup.DampingRateZeroThrottleClutchEngaged = DefEngineData.mDampingRateZeroThrottleClutchEngaged;
	EngineSetup.DampingRateZeroThrottleClutchDisengaged = DefEngineData.mDampingRateZeroThrottleClutchDisengaged;

	// Convert from PhysX curve to ours
	FRichCurve* TorqueCurveData = EngineSetup.TorqueCurve.GetRichCurve();
	for (PxU32 KeyIdx = 0; KeyIdx < DefEngineData.mTorqueCurve.getNbDataPairs(); KeyIdx++)
	{
		float Input = DefEngineData.mTorqueCurve.getX(KeyIdx) * EngineSetup.MaxRPM;
		float Output = DefEngineData.mTorqueCurve.getY(KeyIdx) * DefEngineData.mPeakTorque;
		TorqueCurveData->AddKey(Input, Output);
	}

	PxVehicleClutchData DefClutchData;
	TransmissionSetup.ClutchStrength = DefClutchData.mStrength;

	PxVehicleGearsData DefGearSetup;
	TransmissionSetup.GearSwitchTime = DefGearSetup.mSwitchTime;
	TransmissionSetup.ReverseGearRatio = DefGearSetup.mRatios[PxVehicleGearsData::eREVERSE];
	TransmissionSetup.FinalRatio = DefGearSetup.mFinalRatio;

	PxVehicleAutoBoxData DefAutoBoxSetup;
	TransmissionSetup.NeutralGearUpRatio = DefAutoBoxSetup.mUpRatios[PxVehicleGearsData::eNEUTRAL];
	TransmissionSetup.GearAutoBoxLatency = DefAutoBoxSetup.getLatency();
	TransmissionSetup.bUseGearAutoBox = true;

	for (uint32 i = PxVehicleGearsData::eFIRST; i < DefGearSetup.mNbRatios; i++)
	{
		FVehicleGearData GearData;
		GearData.DownRatio = DefAutoBoxSetup.mDownRatios[i];
		GearData.UpRatio = DefAutoBoxSetup.mUpRatios[i];
		GearData.Ratio = DefGearSetup.mRatios[i];
		TransmissionSetup.ForwardGears.Add(GearData);
	}

	// Init steering speed curve
	FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
	SteeringCurveData->AddKey(0.f, 1.f);
	SteeringCurveData->AddKey(20.f, 0.9f);
	SteeringCurveData->AddKey(60.f, 0.8f);
	SteeringCurveData->AddKey(120.f, 0.7f);

	// Initialize WheelSetups array with 4 wheels
	WheelSetups.SetNum(4);
#endif // WITH_PHYSX
}

float FindSkidPeakTorque(const FVehicleEngineData& Setup)
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

#if WITH_PHYSX
static void GetSkidVehicleEngineSetup(const FVehicleEngineData& Setup, PxVehicleEngineData& PxSetup)
{
	PxSetup.mMOI = M2ToCm2(Setup.MOI);
	PxSetup.mMaxOmega = RPMToOmega(Setup.MaxRPM);
	PxSetup.mDampingRateFullThrottle = M2ToCm2(Setup.DampingRateFullThrottle);
	PxSetup.mDampingRateZeroThrottleClutchEngaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchEngaged);
	PxSetup.mDampingRateZeroThrottleClutchDisengaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchDisengaged);

	float PeakTorque = FindSkidPeakTorque(Setup); // In Nm
	PxSetup.mPeakTorque = M2ToCm2(PeakTorque);	// convert Nm to (kg cm^2/s^2)

	// Convert from our curve to PhysX
	PxSetup.mTorqueCurve.clear();
	TArray<FRichCurveKey> TorqueKeys = Setup.TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
	int32 NumTorqueCurveKeys = FMath::Min<int32>(TorqueKeys.Num(), PxVehicleEngineData::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES);
	for (int32 KeyIdx = 0; KeyIdx < NumTorqueCurveKeys; KeyIdx++)
	{
		FRichCurveKey& Key = TorqueKeys[KeyIdx];
		const float KeyFloat = FMath::IsNearlyZero(Setup.MaxRPM) ? 0.f : Key.Time / Setup.MaxRPM;
		const float ValueFloat = FMath::IsNearlyZero(PeakTorque) ? 0.f : Key.Value / PeakTorque;
		PxSetup.mTorqueCurve.addPair(FMath::Clamp(KeyFloat, 0.f, 1.f), FMath::Clamp(ValueFloat, 0.f, 1.f)); // Normalize torque to 0-1 range
	}
}

static void GetSkidVehicleGearSetup(const FVehicleTransmissionData& Setup, PxVehicleGearsData& PxSetup)
{
	PxSetup.mSwitchTime = Setup.GearSwitchTime;
	PxSetup.mRatios[PxVehicleGearsData::eREVERSE] = Setup.ReverseGearRatio;
	for (int32 i = 0; i < Setup.ForwardGears.Num(); i++)
	{
		PxSetup.mRatios[i + PxVehicleGearsData::eFIRST] = Setup.ForwardGears[i].Ratio;
	}
	PxSetup.mFinalRatio = Setup.FinalRatio;
	PxSetup.mNbRatios = Setup.ForwardGears.Num() + PxVehicleGearsData::eFIRST;
}

static void GetSkidVehicleAutoBoxSetup(const FVehicleTransmissionData& Setup, PxVehicleAutoBoxData& PxSetup)
{
	for (int32 i = 0; i < Setup.ForwardGears.Num(); i++)
	{
		const FVehicleGearData& GearData = Setup.ForwardGears[i];
		PxSetup.mUpRatios[i + PxVehicleGearsData::eFIRST] = GearData.UpRatio;
		PxSetup.mDownRatios[i + PxVehicleGearsData::eFIRST] = GearData.DownRatio;
	}
	PxSetup.mUpRatios[PxVehicleGearsData::eNEUTRAL] = Setup.NeutralGearUpRatio;
	PxSetup.setLatency(Setup.GearAutoBoxLatency);
}

void SetupSkidDriveHelper(const USkidVehicleMovementComponent* VehicleData, const PxVehicleWheelsSimData* PWheelsSimData, PxVehicleDriveSimData4W& DriveData, uint32 NumOfWheels)
{
	PxVehicleEngineData EngineSetup;
	GetSkidVehicleEngineSetup(VehicleData->EngineSetup, EngineSetup);
	DriveData.setEngineData(EngineSetup);

	PxVehicleClutchData ClutchSetup;
	ClutchSetup.mStrength = M2ToCm2(VehicleData->TransmissionSetup.ClutchStrength);
	DriveData.setClutchData(ClutchSetup);

	PxVehicleGearsData GearSetup;
	GetSkidVehicleGearSetup(VehicleData->TransmissionSetup, GearSetup);
	DriveData.setGearsData(GearSetup);

	PxVehicleAutoBoxData AutoBoxSetup;
	GetSkidVehicleAutoBoxSetup(VehicleData->TransmissionSetup, AutoBoxSetup);
	DriveData.setAutoBoxData(AutoBoxSetup);
}
#endif // WITH_PHYSX

#if WITH_PHYSX_VEHICLES
void USkidVehicleMovementComponent::SetupVehicle()
{

	if (!UpdatedPrimitive)
	{
		return;
	}

	if (WheelSetups.Num() % 2 != 0 && WheelSetups.Num() > 20)
	{
		PVehicle = NULL;
		PVehicleDrive = NULL;
		return;
	}

	for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx)
	{
		const FWheelSetup& WheelSetup = WheelSetups[WheelIdx];
		if (WheelSetup.BoneName == NAME_None)
		{
			return;
		}
	}

	// Setup the chassis and wheel shapes
	SetupVehicleShapes();

	// Setup mass properties
	SetupVehicleMass();

	// Setup the wheels
	PxVehicleWheelsSimData* PWheelsSimData = PxVehicleWheelsSimData::allocate(WheelSetups.Num());

	SetupWheels(PWheelsSimData);

	// Setup drive data
	PxVehicleDriveSimData4W DriveData;
	SetupSkidDriveHelper(this, PWheelsSimData, DriveData, WheelSetups.Num());

	// Create the vehicle
	PxVehicleDriveTank* PVehicleDriveTank = PxVehicleDriveTank::allocate(WheelSetups.Num());
	check(PVehicleDriveTank);

	FPhysicsCommand::ExecuteWrite(UpdatedPrimitive->GetBodyInstance()->ActorHandle, [&](const FPhysicsActorHandle& Actor)
	{
#if WITH_CHAOS || WITH_IMMEDIATE_PHYSX
		PxRigidActor* PRigidActor = nullptr;
#else
		PxRigidActor* PRigidActor = Actor.SyncActor;
#endif

		if (PRigidActor)
		{
			if (PxRigidDynamic* PRigidDynamic = PRigidActor->is<PxRigidDynamic>())
			{
				PVehicleDriveTank->setup(GPhysXSDK, PRigidDynamic, *PWheelsSimData, DriveData, WheelSetups.Num());
				PVehicleDriveTank->setToRestState();

				// cleanup
				PWheelsSimData->free();
			}
		}
	});
	PWheelsSimData = NULL;
	// cache values
	PVehicle = PVehicleDriveTank;
	PVehicleDrive = PVehicleDriveTank;

	SetUseAutoGears(TransmissionSetup.bUseGearAutoBox);

}

void USkidVehicleMovementComponent::SetupWheels(physx::PxVehicleWheelsSimData* PWheelsSimData)
{
	Super::SetupWheels(PWheelsSimData);
}
#endif // WITH_PHYSX

#if WITH_PHYSX_VEHICLES

void USkidVehicleMovementComponent::UpdateSimulation(float DeltaTime)
{
	if (PVehicleDrive == NULL)
		return;

	PxVehicleDriveTankRawInputData VehicleInputData(PxVehicleDriveTankControlModel::eSPECIAL);

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
   
	VehicleInputData.setAnalogLeftThrust(nMotMixL);
	VehicleInputData.setAnalogRightThrust(nMotMixR);
	VehicleInputData.setAnalogAccel(accel);

	if (!PVehicleDrive->mDriveDynData.getUseAutoGears())
	{
		VehicleInputData.setGearUp(bRawGearUpInput);
		VehicleInputData.setGearDown(bRawGearDownInput);
	}

	// Toggle on/off digital (hand) break
	if (toggleBreak) {
		VehicleInputData.setAnalogLeftBrake(LeftBreak);
		VehicleInputData.setAnalogRightBrake(RightBreak);
		VehicleInputData.setAnalogAccel(0);
		//UE_LOG(LogTemp, Warning, TEXT("Braking %f %f "), VehicleInputData.getAnalogLeftBrake(), VehicleInputData.getAnalogRightBrake());
	}
	// Convert from our curve to PxFixedSizeLookupTable

	PxVehiclePadSmoothingData SmoothData = {
		{ ThrottleInputRate.RiseRate, LeftBrakeRate.RiseRate, RightBrakeRate.RiseRate, LeftThrustRate.RiseRate, RightThrustRate.RiseRate },
		{ ThrottleInputRate.FallRate, LeftBrakeRate.FallRate, RightBrakeRate.FallRate, LeftThrustRate.FallRate, RightThrustRate.FallRate }
	};

	PxVehicleDriveTank* PVehicleDriveTank = (PxVehicleDriveTank*)PVehicleDrive;
	PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs(SmoothData, VehicleInputData, DeltaTime, *PVehicleDriveTank);

}
#endif // WITH_PHYSX_VEHICLES

#if WITH_EDITOR
void USkidVehicleMovementComponent::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	if (PropertyName == TEXT("DownRatio"))
	{
		for (int32 GearIdx = 0; GearIdx < TransmissionSetup.ForwardGears.Num(); ++GearIdx)
		{
			FVehicleGearData & GearData = TransmissionSetup.ForwardGears[GearIdx];
			GearData.DownRatio = FMath::Min(GearData.DownRatio, GearData.UpRatio);
		}
	}
	else if (PropertyName == TEXT("UpRatio"))
	{
		for (int32 GearIdx = 0; GearIdx < TransmissionSetup.ForwardGears.Num(); ++GearIdx)
		{
			FVehicleGearData & GearData = TransmissionSetup.ForwardGears[GearIdx];
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
#if WITH_PHYSX_VEHICLES
	this->Acceleration = OtherAcceleration;
#endif
}

void USkidVehicleMovementComponent::SetLeftBreak(float OtherLeftBreak)
{
#if WITH_PHYSX_VEHICLES
	this->LeftBreak = OtherLeftBreak;
#endif
}

void USkidVehicleMovementComponent::SetRightBreak(float OtherRightBreak)
{
#if WITH_PHYSX_VEHICLES
	this->RightBreak = OtherRightBreak;
#endif
}

void USkidVehicleMovementComponent::SetYJoy(float OtherNJoyY)
{
#if WITH_PHYSX_VEHICLES
	this->nJoyY = OtherNJoyY;
#endif
}

void USkidVehicleMovementComponent::SetXJoy(float OtherNJoyX)
{
#if WITH_PHYSX_VEHICLES
	this->nJoyX = OtherNJoyX;
#endif
}

float USkidVehicleMovementComponent::GetAcceleration() const
{
#if WITH_PHYSX_VEHICLES
	return Acceleration;
#else
	return 0.0f;
#endif
}

float USkidVehicleMovementComponent::GetLeftBreak() const
{
#if WITH_PHYSX_VEHICLES
	return LeftBreak;
#else
	return 0.0f;
#endif
}

float USkidVehicleMovementComponent::GetRightBreak() const
{
#if WITH_PHYSX_VEHICLES
	return RightBreak;
#else
	return 0.0f;
#endif
}

float USkidVehicleMovementComponent::GetYJoy() const
{
#if WITH_PHYSX_VEHICLES
	return nJoyY;
#else
	return 0.0f;
#endif
}

float USkidVehicleMovementComponent::GetXJoy() const
{
#if WITH_PHYSX_VEHICLES
	return nJoyX;
#else
	return 0.0f;
#endif
}

void USkidVehicleMovementComponent::SetBreaksOn()
{
#if WITH_PHYSX_VEHICLES
	this->toggleBreak = true;
#endif
}

void USkidVehicleMovementComponent::SetBreaksOff()
{
#if WITH_PHYSX_VEHICLES
	this->toggleBreak = false;
#endif
}