// Fill out your copyright notice in the Description page of Project Settings.


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

	SkidControlMethod = SkidVehicleDriveControlMethod::SingleStick;

	SkidControlModel = SkidVehicleDriveControlModel::STANDARD;


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

float FVehicleEngineData::FindPeakTorque() const
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

#if WITH_PHYSX
static void GetSkidVehicleEngineSetup(const FVehicleEngineData& Setup, PxVehicleEngineData& PxSetup)
{
	PxSetup.mMOI = M2ToCm2(Setup.MOI);
	PxSetup.mMaxOmega = RPMToOmega(Setup.MaxRPM);
	PxSetup.mDampingRateFullThrottle = M2ToCm2(Setup.DampingRateFullThrottle);
	PxSetup.mDampingRateZeroThrottleClutchEngaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchEngaged);
	PxSetup.mDampingRateZeroThrottleClutchDisengaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchDisengaged);

	float PeakTorque = Setup.FindPeakTorque(); // In Nm
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

	PxVehicleDriveTankRawInputData VehicleInputData((PxVehicleDriveTankControlModel::Enum)SkidControlModel);

	//switch (SkidControlModel)
	//{
	//case SkidVehicleDriveControlModel::STANDARD:

	//	switch (SkidControlMethod)
	//	{
	//	case SkidVehicleDriveControlMethod::SingleStick: {

	//		float scaledSteering = SteeringInput * 100;
	//		float scaledThrottle = ThrottleInput * 100;
	//		float invSteering = -scaledSteering;
	//		float v = (100 - FMath::Abs(invSteering)) * (scaledThrottle / 100) + scaledThrottle;
	//		float w = (100 - FMath::Abs(scaledThrottle))* (invSteering / 100) + invSteering;
	//		float r = (v + w) / 2.f;
	//		float l = (v - w) / 2.f;
	//		UE_LOG(LogTemp, Warning, TEXT("SteeringInput:  %+0.3f, ThrottleInput:  %+.3f, v: %+.3f, w: %+.3f, r: %+.3f, l: %+.3f"), SteeringInput, ThrottleInput, v, w, r, l);
	//		VehicleInputData.setAnalogLeftThrust(l / 100.f);
	//		VehicleInputData.setAnalogRightThrust(r / 100.f);
	//		VehicleInputData.setAnalogLeftBrake(BrakeInput);
	//		VehicleInputData.setAnalogRightBrake(BrakeInput);
	//		if (ThrottleInput != 0)
	//		{
	//			VehicleInputData.setAnalogAccel(ThrottleInput);
	//		}
	//		else
	//		{
	//			if (scaledSteering != 0) {
	//				VehicleInputData.setAnalogAccel(abs(scaledSteering));
	//			}
	//			else {
	//				VehicleInputData.setAnalogAccel(0);
	//			}
	//		}
	//	}
	//	break;
	//	case SkidVehicleDriveControlMethod::DualStick:

	//		float range = sqrt(LeftThrust*LeftThrust + RightThrust * RightThrust) * 100;
	//		float theta = tanhf(RightThrust / LeftThrust);
	//		float theta_norm = fmod(theta + 180, 360) - 180;
	//		float v_a = range * (45 - fmod(theta_norm, 90)) / 45;
	//		float v_b = FMath::Min(FMath::Min(100.0f, 2 * range + v_a), 2 * range - v_a);
	//		UE_LOG(LogTemp, Warning, TEXT("LeftThrust: %+0.3f, RightThrust: %+.3f, range: %+.3f, theta_norm: %+.3f, v_a: %+.3f, v_b: %+.3f"), LeftThrust, RightThrust, range, theta_norm, v_a, v_b);
	//		if (theta_norm < -90)
	//		{
	//			VehicleInputData.setAnalogLeftThrust(-v_a / 100);
	//			VehicleInputData.setAnalogRightThrust(-v_b / 100);
	//		}
	//		else if (theta_norm < 0) {
	//			VehicleInputData.setAnalogLeftThrust(-v_a / 100);
	//			VehicleInputData.setAnalogRightThrust(v_b / 100);
	//		}
	//		else if (theta_norm < 90)
	//		{
	//			VehicleInputData.setAnalogLeftThrust(v_a / 100);
	//			VehicleInputData.setAnalogRightThrust(v_b / 100);
	//		}
	//		else {
	//			VehicleInputData.setAnalogLeftThrust(v_a / 100);
	//			VehicleInputData.setAnalogRightThrust(-v_b / 100);
	//		}
	//		VehicleInputData.setAnalogAccel((v_a + v_b) / 100);
	//		break;
	//	default:
	//		break;
	//	}

	//	break;
	//case SkidVehicleDriveControlModel::SPECIAL:

	//		VehicleInputData.setAnalogLeftThrust(LeftThrust);
	//		VehicleInputData.setAnalogRightThrust(RightThrust);
	//		VehicleInputData.setAnalogLeftBrake(BrakeInput);
	//		VehicleInputData.setAnalogRightBrake(BrakeInput);
	//		VehicleInputData.setAnalogAccel(ThrottleInput);

	//	break;
	//default:
	//	break;
	//}

	float range = sqrt(LeftThrust*LeftThrust + RightThrust * RightThrust) * 100;
	float theta = atan2(RightThrust, LeftThrust) / PI * 180;
	float theta_norm = fmod(theta + 180, 360) - 180;
	float v_a = range * (45 - fmod(theta_norm, 90)) / 45;
	float v_b = FMath::Min(FMath::Min(100.0f, 2 * range + v_a), 2 * range - v_a);
	if (theta_norm < -90)
	{
		UE_LOG(LogTemp, Warning, TEXT("LeftThrust: %+0.3f, RightThrust: %+.3f, acc: %+.3f"), -v_a / 100, -v_b / 100, (v_a + v_b) / 100);
		VehicleInputData.setAnalogLeftThrust(-v_a / 100);
		VehicleInputData.setAnalogRightThrust(-v_b / 100);
	}
	else if (theta_norm < 0) {
		UE_LOG(LogTemp, Warning, TEXT("LeftThrust: %+0.3f, RightThrust: %+.3f, acc: %+.3f"), -v_a / 100, -v_b / 100, (v_a + v_b) / 100);
		VehicleInputData.setAnalogLeftThrust(-v_a / 100);
		VehicleInputData.setAnalogRightThrust(v_b / 100);
	}
	else if (theta_norm < 90)
	{
		UE_LOG(LogTemp, Warning, TEXT("LeftThrust: %+0.3f, RightThrust: %+.3f, acc: %+.3f"), -v_a / 100, -v_b / 100, (v_a + v_b) / 100);
		VehicleInputData.setAnalogLeftThrust(v_a / 100);
		VehicleInputData.setAnalogRightThrust(v_b / 100);
	}
	else 
	{
		UE_LOG(LogTemp, Warning, TEXT("LeftThrust: %+0.3f, RightThrust: %+.3f, acc: %+.3f"), -v_a / 100, -v_b / 100, (v_a + v_b) / 100);
		VehicleInputData.setAnalogLeftThrust(v_a / 100);
		VehicleInputData.setAnalogRightThrust(-v_b / 100);
	}
	VehicleInputData.setAnalogAccel((v_a + v_b) / 100);

	if (!PVehicleDrive->mDriveDynData.getUseAutoGears())
	{
		VehicleInputData.setGearUp(bRawGearUpInput);
		VehicleInputData.setGearDown(bRawGearDownInput);
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

void USkidVehicleMovementComponent::SetLeftThrust(float OtherLeftThrust)
{
	this->LeftThrust = OtherLeftThrust;
}

void USkidVehicleMovementComponent::SetRightThrust(float OtherRightThrust)
{
	this->RightThrust = OtherRightThrust;
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

float USkidVehicleMovementComponent::GetLeftThrust() const
{
	return LeftThrust;
}

float USkidVehicleMovementComponent::GetRightThrust() const
{
	return RightThrust;
}