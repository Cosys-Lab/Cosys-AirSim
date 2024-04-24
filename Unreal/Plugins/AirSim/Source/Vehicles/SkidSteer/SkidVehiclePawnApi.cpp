// Developed by Cosys-Lab, University of Antwerp

#include "SkidVehiclePawnApi.h"
#include "AirBlueprintLib.h"
#include "ChaosVehicleManager.h"

SkidVehiclePawnApi::SkidVehiclePawnApi(ASkidVehiclePawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, msr::airlib::CarApiBase* vehicle_api)
	: pawn_(pawn), pawn_kinematics_(pawn_kinematics), vehicle_api_(vehicle_api)
{
	movement_ = pawn->GetSkidVehicleMovement();
}

void SkidVehiclePawnApi::updateMovement(const msr::airlib::CarApiBase::CarControls& controls)
{
	last_controls_ = controls;

	if (!controls.is_manual_gear && movement_->GetTargetGear() < 0)
		movement_->SetTargetGear(0, true); //in auto gear we must have gear >= 0
	if (controls.is_manual_gear && movement_->GetTargetGear() != controls.manual_gear)
		movement_->SetTargetGear(controls.manual_gear, controls.gear_immediate);

	movement_->SetYJoy(controls.throttle);
	movement_->SetXJoy(controls.steering);
	movement_->SetLeftBreak(controls.brake);
	movement_->SetRightBreak(controls.brake);
	if (controls.brake || controls.handbrake) {
		movement_->SetBreaksOn();
		movement_->SetYJoy(0);
		movement_->SetXJoy(0);
	}
	else {
		movement_->SetBreaksOff();
	}
	movement_->SetUseAutomaticGears(!controls.is_manual_gear);

	//float accel = FMath::Min((float)sqrt(controls.steering * controls.steering + controls.throttle * controls.throttle), 1.0f);
	//movement_->SetThrottleInput(accel);
	//movement_->SetSteeringInput(controls.steering);
	//movement_->SetBrakeInput(controls.brake);
	//movement_->SetHandbrakeInput(controls.handbrake);
	//movement_->SetUseAutomaticGears(!controls.is_manual_gear);
}

msr::airlib::CarApiBase::CarState SkidVehiclePawnApi::getCarState() const
{
	msr::airlib::CarApiBase::CarState state(
		movement_->GetForwardSpeed() / 100, //cm/s -> m/s
		movement_->GetCurrentGear(),
		movement_->GetEngineRotationSpeed(),
		movement_->GetEngineMaxRotationSpeed(),
		last_controls_.handbrake,
		*pawn_kinematics_,
		vehicle_api_->clock()->nowNanos());
	return state;
}

void SkidVehiclePawnApi::reset()
{
	vehicle_api_->reset();

	last_controls_ = msr::airlib::CarApiBase::CarControls();
	auto phys_comps = UAirBlueprintLib::getPhysicsComponents(pawn_);
	UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
		for (auto* phys_comp : phys_comps) {
			phys_comp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
			phys_comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
			//phys_comp->SetSimulatePhysics(false);
		}
		movement_->ResetMoveState();
		movement_->SetActive(false);
		movement_->SetActive(true, true);
		updateMovement(msr::airlib::CarApiBase::CarControls());

		//auto pv = movement_->PVehicle;
		//if (pv) {
		//	pv->mWheelsDynData.setToRestState();
		//}
		//auto pvd = movement_->PVehicleDrive;
		//if (pvd) {
		//	pvd->mDriveDynData.setToRestState();
		//}
	}, true);

	//UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
	//	for (auto* phys_comp : phys_comps)
	//		phys_comp->SetSimulatePhysics(true);
	//}, true);
}

void SkidVehiclePawnApi::update(float delta)
{
	vehicle_api_->updateCarState(getCarState());
	vehicle_api_->update(delta);
}

SkidVehiclePawnApi::~SkidVehiclePawnApi() = default;
