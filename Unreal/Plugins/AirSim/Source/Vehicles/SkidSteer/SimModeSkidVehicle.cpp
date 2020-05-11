#include "SimModeSkidVehicle.h"
#include "ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "SkidVehiclePawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "vehicles/car/api/CarRpcLibServer.hpp"


void ASimModeSkidVehicle::BeginPlay()
{
	Super::BeginPlay();

	initializePauseState();
}

void ASimModeSkidVehicle::initializePauseState()
{
	pause_period_ = 0;
	pause_period_start_ = 0;
	pause(false);
}

bool ASimModeSkidVehicle::isPaused() const
{
	return current_clockspeed_ == 0;
}

void ASimModeSkidVehicle::pause(bool is_paused)
{
	if (is_paused)
		current_clockspeed_ = 0;
	else
		current_clockspeed_ = getSettings().clock_speed;

	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
}

void ASimModeSkidVehicle::continueForTime(double seconds)
{
	pause_period_start_ = ClockFactory::get()->nowNanos();
	pause_period_ = seconds;
	pause(false);
}

void ASimModeSkidVehicle::setupClockSpeed()
{
	current_clockspeed_ = getSettings().clock_speed;

	//setup clock in PhysX
	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
	UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeSkidVehicle::Tick(float DeltaSeconds)
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

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeSkidVehicle::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
	return ASimModeBase::createApiServer();
#else
	return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::CarRpcLibServer(
		getApiProvider(), getSettings().api_server_address));
#endif
}

void ASimModeSkidVehicle::getExistingVehiclePawns(TArray<AirsimVehicle*>& pawns) const
{
	for (TActorIterator<TVehiclePawn> it(this->GetWorld()); it; ++it)
	{
		pawns.Add(static_cast<AirsimVehicle*>(*it));
	}
}

bool ASimModeSkidVehicle::isVehicleTypeSupported(const std::string& vehicle_type) const
{
	return ((vehicle_type == AirSimSettings::kVehicleTypeCPHusky) ||
		(vehicle_type == AirSimSettings::kVehicleTypePioneer));
}

std::string ASimModeSkidVehicle::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
	//decide which derived BP to use
	std::string pawn_path = vehicle_setting.pawn_path;
	if (pawn_path == "") {
		if (vehicle_setting.vehicle_type == "cphusky") {
			pawn_path = "CPHusky";
		}
		else if (vehicle_setting.vehicle_type == "pioneer") {
			pawn_path = "Pioneer";
		}
		else {
			pawn_path = "CPHusky";
		}
	}


	return pawn_path;
}

PawnEvents* ASimModeSkidVehicle::getVehiclePawnEvents(APawn* pawn) const
{
	return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeSkidVehicle::getVehiclePawnCameras(
	APawn* pawn) const
{
	return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
}
void ASimModeSkidVehicle::initializeVehiclePawn(APawn* pawn)
{
	auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
	vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);
}
std::unique_ptr<PawnSimApi> ASimModeSkidVehicle::createVehicleSimApi(
	const PawnSimApi::Params& pawn_sim_api_params) const
{
	auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn_sim_api_params.vehicle->GetPawn());
	auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new SkidVehiclePawnSimApi(pawn_sim_api_params,
		vehicle_pawn->getKeyBoardControls(), vehicle_pawn->getVehicleMovementComponent()));
	vehicle_sim_api->initialize();
	vehicle_sim_api->reset();
	return vehicle_sim_api;
}
msr::airlib::VehicleApiBase* ASimModeSkidVehicle::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
	const PawnSimApi* sim_api) const
{
	const auto car_sim_api = static_cast<const SkidVehiclePawnSimApi*>(sim_api);
	return car_sim_api->getVehicleApi();
}
