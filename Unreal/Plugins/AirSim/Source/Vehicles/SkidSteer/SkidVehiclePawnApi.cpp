// Developed by Cosys-Lab, University of Antwerp

#include "SkidVehiclePawnApi.h"
#include "AirBlueprintLib.h"

#include "PhysXVehicleManager.h"

SkidVehiclePawnApi::SkidVehiclePawnApi(ASkidVehiclePawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
	const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<msr::airlib::SensorFactory> sensor_factory,
	const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment)
	: msr::airlib::CarApiBase(vehicle_setting, sensor_factory, state, environment),
	pawn_(pawn), pawn_kinematics_(pawn_kinematics), home_geopoint_(home_geopoint)
{
	movement_ = pawn->GetSkidVehicleMovement();
}

bool SkidVehiclePawnApi::armDisarm(bool arm)
{
	//TODO: implement arming for car
	unused(arm);
	return true;
}

void SkidVehiclePawnApi::setCarControls(const CarApiBase::CarControls& controls)
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
	movement_->SetUseAutoGears(!controls.is_manual_gear);
}

const msr::airlib::CarApiBase::CarControls& SkidVehiclePawnApi::getCarControls() const
{
	return last_controls_;
}

msr::airlib::CarApiBase::CarState SkidVehiclePawnApi::getCarState() const
{
	CarApiBase::CarState state(
		movement_->GetForwardSpeed() / 100, //cm/s -> m/s
		movement_->GetCurrentGear(),
		movement_->GetEngineRotationSpeed(),
		movement_->GetEngineMaxRotationSpeed(),
		false,
		*pawn_kinematics_,
		msr::airlib::ClockFactory::get()->nowNanos()
	);
	return state;
}

void SkidVehiclePawnApi::reset()
{
	msr::airlib::CarApiBase::reset();

	last_controls_ = CarControls();
	auto phys_comps = UAirBlueprintLib::getPhysicsComponents(pawn_);
	UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
		for (auto* phys_comp : phys_comps) {
			phys_comp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
			phys_comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
			phys_comp->SetSimulatePhysics(false);
		}
		movement_->ResetMoveState();
		movement_->SetActive(false);
		movement_->SetActive(true, true);
		setCarControls(CarControls());

		auto pv = movement_->PVehicle;
		if (pv) {
			pv->mWheelsDynData.setToRestState();
		}
		auto pvd = movement_->PVehicleDrive;
		if (pvd) {
			pvd->mDriveDynData.setToRestState();
		}
	}, true);

	UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
		for (auto* phys_comp : phys_comps)
			phys_comp->SetSimulatePhysics(true);
	}, true);
}

void SkidVehiclePawnApi::update(float delta)
{
	msr::airlib::CarApiBase::update(delta);
}

msr::airlib::GeoPoint SkidVehiclePawnApi::getHomeGeoPoint() const
{
	return home_geopoint_;
}

void SkidVehiclePawnApi::enableApiControl(bool is_enabled)
{
	if (api_control_enabled_ != is_enabled) {
		last_controls_ = CarControls();
		api_control_enabled_ = is_enabled;
	}
}

bool SkidVehiclePawnApi::isApiControlEnabled() const
{
	return api_control_enabled_;
}

SkidVehiclePawnApi::~SkidVehiclePawnApi() = default;
