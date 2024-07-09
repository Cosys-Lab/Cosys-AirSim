// Developed by Cosys-Lab, University of Antwerp

#include "SimModeSkidVehicle.h"
#include "UObject/ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "SkidVehiclePawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "vehicles/car/api/CarRpcLibServer.hpp"

extern CORE_API uint32 GFrameNumber;

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

void ASimModeSkidVehicle::continueForTime(double seconds)
{
	pause_period_start_ = ClockFactory::get()->nowNanos();
	pause_period_ = seconds * current_clockspeed_;
	pause(false);
}

void ASimModeSkidVehicle::continueForFrames(uint32_t frames)
{
	targetFrameNumber_ = GFrameNumber + frames;
	frame_countdown_enabled_ = true;
	pause(false);
}

void ASimModeSkidVehicle::setupClockSpeed()
{
	Super::setupClockSpeed();

	current_clockspeed_ = getSettings().clock_speed;

	//setup clock in PhysX
	UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
	UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeSkidVehicle::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!isPaused())
		ClockFactory::get()->stepBy(DeltaSeconds);

	if (pause_period_start_ > 0) {
		if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
			if (!isPaused())
				pause(true);

			pause_period_start_ = 0;
		}
	}

	if (frame_countdown_enabled_) {
		if (targetFrameNumber_ <= GFrameNumber) {
			if (!isPaused())
				pause(true);

			frame_countdown_enabled_ = false;
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
		getApiProvider(), getSettings().api_server_address, getSettings().api_port));
#endif
}

void ASimModeSkidVehicle::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
	UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
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
		if (vehicle_setting.vehicle_type == "pioneer") {
			pawn_path = "Pioneer";
		}
		else {
			pawn_path = "DefaultSkidVehicle";
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
	auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn_sim_api_params.pawn);
	auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new SkidVehiclePawnSimApi(pawn_sim_api_params,
		vehicle_pawn->getKeyBoardControls()));
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
