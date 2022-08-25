// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#include "SkidVehicleMovementComponent.h"
#include "physics/Kinematics.hpp"
#include "SkidVehiclePawn.h"


class SkidVehiclePawnApi : public msr::airlib::CarApiBase {
public:
	typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

	SkidVehiclePawnApi(ASkidVehiclePawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
		const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<msr::airlib::SensorFactory> sensor_factory,
		const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment);

	virtual void setCarControls(const CarApiBase::CarControls& controls) override;

	virtual CarApiBase::CarState getCarState() const override;

	virtual void reset() override;
	virtual void update(float delta = 0) override;

	virtual msr::airlib::GeoPoint getHomeGeoPoint() const override;

	virtual void enableApiControl(bool is_enabled) override;
	virtual bool isApiControlEnabled() const override;
	virtual bool armDisarm(bool arm) override;

	virtual const CarApiBase::CarControls& getCarControls() const override;

	virtual ~SkidVehiclePawnApi();

private:
	USkidVehicleMovementComponent* movement_;
	bool api_control_enabled_ = false;
	CarControls last_controls_;
	ASkidVehiclePawn* pawn_;
	const msr::airlib::Kinematics::State* pawn_kinematics_;
	msr::airlib::GeoPoint  home_geopoint_;
};