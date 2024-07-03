// Developed by Cosys-Lab, University of Antwerp

#include "SkidVehiclePawnApi.h"
#include "AirBlueprintLib.h"
#include "ChaosVehicleManager.h"

SkidVehiclePawnApi::SkidVehiclePawnApi(ASkidVehiclePawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, msr::airlib::CarApiBase* vehicle_api)
	: pawn_(pawn), pawn_kinematics_(pawn_kinematics), vehicle_api_(vehicle_api)
{
	movement_ = CastChecked<UChaosWheeledVehicleMovementComponent>(pawn->GetVehicleMovement());
}

void SkidVehiclePawnApi::updateMovement(const msr::airlib::CarApiBase::CarControls& controls)
{		
	if (last_controls_.steering == 0 && controls.steering != 0)
		turn_started_ = true;
	if (last_controls_.throttle == 0 && controls.throttle != 0)
		move_started_ = true;
	if (last_controls_.steering != 0 && controls.steering == 0)
		turn_completed_ = true;
	if (last_controls_.throttle != 0 && controls.throttle == 0)
		move_completed_ = true;

	last_controls_ = controls;

	msr::airlib::CarApiBase::CarControls to_set_controls_ = controls;
	to_set_controls_.brake = 0;
	bool set_throttle = false;
	bool set_steering = false;
	if (controls.handbrake) {
		to_set_controls_.throttle = 0;
		to_set_controls_.steering = 0;
		to_set_controls_.brake = 0;
		to_set_controls_.handbrake = controls.handbrake;
		set_throttle = true;
		set_steering = true;
	}
	else
	{
		if (move_completed_ || controls.throttle != 0){
			if (move_completed_){
				set_throttle = true;
				move_completed_ = false;
			}
			if (controls.throttle >= 0) {
				to_set_controls_.brake = 0;
			}
			else {
				to_set_controls_.brake = controls.throttle * -1.0f;
			}
		}
		if (move_started_) {
			move_started_ = false;
			to_set_controls_.steering = 0;
			to_set_controls_.throttle = 0;
			set_throttle = true;
			set_steering = true;
		}
		if (turn_started_) {
			if (controls.throttle == 0) {
				// For some reason ChaosVehicles when setting Yaw input with negative value requires a velocity on the vehicle
				if((FMath::Abs(movement_->GetForwardSpeed() / 100)) < 0.02 && controls.steering > 0) {					
					to_set_controls_.throttle = pawn_->fixed_turn_rate_;	
					set_throttle = true;
				}
			}
		}			
		if (controls.steering != 0 && pawn_->stop_turn_) {
			to_set_controls_.steering = FMath::Lerp(controls.steering, 0, pawn_->fixed_turn_rate_);
			set_steering = true;
		}
		if (turn_completed_) {
			to_set_controls_.steering = 0;
			to_set_controls_.throttle = 0;
			turn_completed_ = false;
			set_throttle = true;
			set_steering = true;
			turn_started_ = false;
		}
		if (!set_steering)
		{
			to_set_controls_.steering = controls.steering;
			set_steering = true;
		}
		if (!set_throttle)
		{
			to_set_controls_.throttle = controls.throttle;
			set_throttle = true;
		}
	}
	if(set_throttle)
		movement_->SetThrottleInput(to_set_controls_.throttle);
	if(set_steering)
		movement_->SetYawInput(to_set_controls_.steering);
	movement_->SetBrakeInput(to_set_controls_.brake);
	movement_->SetHandbrakeInput(to_set_controls_.handbrake);
	//UE_LOG(LogTemp, Warning, TEXT("throttle: %f | steering:  %f | brake: %f | handbrake: %s"), to_set_controls_.throttle, to_set_controls_.steering, to_set_controls_.brake, to_set_controls_.handbrake ? TEXT("true") : TEXT("false"));
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
        vehicle_api_->setCarControls(msr::airlib::CarApiBase::CarControls());
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
