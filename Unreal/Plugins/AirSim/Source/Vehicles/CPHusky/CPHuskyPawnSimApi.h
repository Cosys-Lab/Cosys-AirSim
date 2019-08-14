#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "CPHuskyPawn.h"
#include "CPHuskyPawnApi.h"
#include "PawnEvents.h"
#include "PawnSimApi.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

class CPHuskyPawnSimApi : public PawnSimApi
{
public:
	typedef msr::airlib::Utils Utils;
	typedef msr::airlib::StateReporter StateReporter;
	typedef msr::airlib::UpdatableObject UpdatableObject;
	typedef msr::airlib::Pose Pose;

public:
	virtual void initialize() override;
	virtual ~CPHuskyPawnSimApi() = default;

	//VehicleSimApiBase interface
	//implements game interface to update pawn
	CPHuskyPawnSimApi(const Params& params,
		const CPHuskyPawnApi::CarControls&  keyboard_controls, UWheeledVehicleMovementComponent* movement);

	virtual void reset() override;
	virtual void update(float delta = 0) override;

	virtual std::string getRecordFileLine(bool is_header_line) const override;

	virtual void updateRenderedState(float dt) override;
	virtual void updateRendering(float dt) override;

	msr::airlib::CarApiBase* getVehicleApi() const
	{
		return vehicle_api_.get();
	}

	virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
	{
		return vehicle_api_.get();
	}

private:
	void createVehicleApi(ACPHuskyPawn* pawn, const msr::airlib::GeoPoint& home_geopoint);
	void updateCarControls();

private:
	Params params_;

	std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
	std::vector<std::string> vehicle_api_messages_;

	//storing reference from pawn
	const CPHuskyPawnApi::CarControls& keyboard_controls_;

	CPHuskyPawnApi::CarControls joystick_controls_;
	CPHuskyPawnApi::CarControls current_controls_;
};
