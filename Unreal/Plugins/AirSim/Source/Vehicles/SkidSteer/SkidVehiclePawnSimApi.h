// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "SkidVehiclePawn.h"
#include "SkidVehiclePawnApi.h"
#include "PawnEvents.h"
#include "PawnSimApi.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/car/CarApiFactory.hpp"

class SkidVehiclePawnSimApi : public PawnSimApi
{
public:
	typedef msr::airlib::Utils Utils;
	typedef msr::airlib::StateReporter StateReporter;
	typedef msr::airlib::UpdatableObject UpdatableObject;
	typedef msr::airlib::Pose Pose;

public:
	virtual void initialize() override;
	virtual ~SkidVehiclePawnSimApi() = default;

	//VehicleSimApiBase interface
	//implements game interface to update pawn
	SkidVehiclePawnSimApi(const Params& params,
		                  const msr::airlib::CarApiBase::CarControls& keyboard_controls);

	virtual void update(float delta = 0) override;

	virtual void reportState(StateReporter& reporter) override;

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


protected:
	virtual void resetImplementation() override;

private:
	void updateCarControls();

private:
	std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
	std::vector<std::string> vehicle_api_messages_;
	std::unique_ptr<SkidVehiclePawnApi> pawn_api_;

	//storing reference from pawn
	const msr::airlib::CarApiBase::CarControls& keyboard_controls_;

	msr::airlib::CarApiBase::CarControls joystick_controls_;
	msr::airlib::CarApiBase::CarControls current_controls_;
};
