// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "SimModeComputerVision.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/World.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "PawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "common/AirSimSettings.hpp"
#include "physics/Kinematics.hpp"
#include "api/RpcLibServerBase.hpp"
#include "vehicles/computervision/api/ComputerVisionRpcLibServer.hpp"
#include "ComputerVisionPawnSimApi.h"


void ASimModeComputerVision::BeginPlay()
{
	Super::BeginPlay();

	initializePauseState();
}

void ASimModeComputerVision::initializePauseState()
{
	pause_period_ = 0;
	pause_period_start_ = 0;
	pause(false);
}

bool ASimModeComputerVision::isPaused() const
{
	return current_clockspeed_ == 0;
}

void ASimModeComputerVision::pause(bool is_paused)
{
	if (is_paused)
		current_clockspeed_ = 0;
	else
		current_clockspeed_ = getSettings().clock_speed;

	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
}

void ASimModeComputerVision::continueForTime(double seconds)
{
	pause_period_start_ = ClockFactory::get()->nowNanos();
	pause_period_ = seconds;
	pause(false);
}

void ASimModeComputerVision::setupClockSpeed()
{
	current_clockspeed_ = getSettings().clock_speed;

	//setup clock in PhysX
	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
	UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeComputerVision::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (pause_period_start_ > 0) {
		if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
			if (!isPaused())
				pause(true);

			pause_period_start_ = 0;
		}
	}
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeComputerVision::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::ComputerVisionRpcLibServer(
        getApiProvider(), getSettings().api_server_address));
#endif
}

void ASimModeComputerVision::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
}

bool ASimModeComputerVision::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    return vehicle_type == msr::airlib::AirSimSettings::kVehicleTypeComputerVision;
}

std::string ASimModeComputerVision::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //decide which derived BP to use
    std::string pawn_path = vehicle_setting.pawn_path;
    if (pawn_path == "")
        pawn_path = "DefaultComputerVision";

    return pawn_path;
}

PawnEvents* ASimModeComputerVision::getVehiclePawnEvents(APawn* pawn) const
{
    return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeComputerVision::getVehiclePawnCameras(
    APawn* pawn) const
{
    return static_cast<const TVehiclePawn*>(pawn)->getCameras();
}
void ASimModeComputerVision::initializeVehiclePawn(APawn* pawn)
{
    static_cast<TVehiclePawn*>(pawn)->initializeForBeginPlay();
}

std::unique_ptr<PawnSimApi> ASimModeComputerVision::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
	auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new ComputerVisionPawnSimApi(pawn_sim_api_params));
	vehicle_sim_api->initialize();
	vehicle_sim_api->reset();
	return vehicle_sim_api;
}

msr::airlib::VehicleApiBase* ASimModeComputerVision::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
    const PawnSimApi* sim_api) const
{
    const auto computer_vision_sim_api = static_cast<const ComputerVisionPawnSimApi*>(sim_api);
    return computer_vision_sim_api->getVehicleApi();
}