#pragma once

#include <functional>

#include "PawnSimApi.h"
#include "UrdfBotPawn.h"
#include "UrdfBotApi.h"
#include "AirBlueprintLib.h"
#include "Airlib/include/physics/Kinematics.hpp"
#include "AirLib/include/vehicles/urdfbot/api/UrdfBotApiBase.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"

class UrdfBotSimApi : public PawnSimApi
{
	public:
		typedef msr::airlib::Utils Utils;
		typedef msr::airlib::StateReporter StateReporter;
		typedef msr::airlib::UpdatableObject UpdatableObject;
		typedef msr::airlib::Pose Pose;

	public:
		virtual void initialize() override;
		virtual ~UrdfBotSimApi() = default;

		UrdfBotSimApi(const Params& params);

		virtual void reset() override;
		virtual void update(float delta = 0) override;
		virtual void reportState(msr::airlib::StateReporter& reporter) override;
		
		virtual std::string getRecordFileLine(bool is_header_line) const override;

		virtual void updateRenderedState(float dt) override;
		virtual void updateRendering(float dt) override;

		msr::airlib::UrdfBotApiBase* getVehicleApi() const
		{
			return vehicle_api_.get();
		}

		virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
		{
			return vehicle_api_.get();
		}

	private:
		Params params_;
		std::unique_ptr<msr::airlib::UrdfBotApiBase> vehicle_api_;
		std::vector<std::string> vehicle_api_messages_;

		void createVehicleApi(AUrdfBotPawn* pawn, const msr::airlib::GeoPoint& home_geopoint);
};