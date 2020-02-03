#include "SimModeUrdfBot.h"

ASimModeUrdfBot::ASimModeUrdfBot()
{

}

void ASimModeUrdfBot::BeginPlay()
{
    // Will call setupVehiclesAndCamera()
    Super::BeginPlay();

    for (auto api : getApiProvider()->getVehicleSimApis()) {
        api->reset();
    }

    this->checkVehicleReady();
    this->initializePauseState();
}

void ASimModeUrdfBot::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    if (this->pause_period_start_.load(std::memory_order_relaxed) > 0) {
        if (ClockFactory::get()->elapsedSince(pause_period_start_.load(std::memory_order_relaxed)) > this->pause_period_.load(std::memory_order_relaxed)) {
            if (!this->isPaused()) {
                this->pause(true);
            }

            this->pause_period_start_.store(0, std::memory_order_relaxed);
        }
    }
}

void ASimModeUrdfBot::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    this->spawned_actors_.Empty();
    this->vehicle_sim_apis_.clear();
    Super::EndPlay(EndPlayReason);
}

bool ASimModeUrdfBot::isPaused() const
{
    return msr::airlib::Utils::isApproximatelyZero(this->current_clockspeed_.load(std::memory_order_relaxed));
}

void ASimModeUrdfBot::pause(bool is_paused)
{
    this->current_clockspeed_ = (is_paused ? 0.0 : this->getSettings().clock_speed);

    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
}

void ASimModeUrdfBot::continueForTime(double seconds)
{
    this->pause_period_start_ = ClockFactory::get()->nowNanos();
    this->pause_period_ = seconds;
    this->pause(false);
}

void ASimModeUrdfBot::setupClockSpeed()
{
    this->current_clockspeed_ = this->getSettings().clock_speed;

    //setup clock in PhysX
    UAirBlueprintLib::setUnrealClockSpeed(this, this->current_clockspeed_);
    UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeUrdfBot::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::UrdfBotRpcLibServer(
        this->getApiProvider(), this->getSettings().api_server_address
    ));
#endif
}

void ASimModeUrdfBot::getExistingVehiclePawns(TArray<AirsimVehicle*>& pawns) const
{
    for (TActorIterator<TVehiclePawn> it(this->GetWorld()); it; ++it)
    {
        pawns.Add(static_cast<AirsimVehicle*>(*it));
    }
}

void ASimModeUrdfBot::initializePauseState()
{
    this->pause_period_ = 0.0f;
    this->pause_period_start_ = 0;
    this->pause(false);
}

void ASimModeUrdfBot::cycleVisibleCameraForward()
{
    this->cycleVisibleCamera(true);
}

void ASimModeUrdfBot::cycleVisibleCameraBackward()
{
    this->cycleVisibleCamera(false);
}

void ASimModeUrdfBot::cycleVisibleCamera(bool forward)
{
    int currentIndex = this->camera_index_;

    this->camera_index_ += (forward ? 1 : -1);

    if (this->camera_index_ < 0) {
        this->camera_index_ = this->cameras_.Num() - 1;
    } 
    else if (this->camera_index_ >= this->cameras_.Num()) {
        this->camera_index_ = 0;
    }

    this->cameras_[currentIndex]->disableMain();
    this->cameras_[this->camera_index_]->showToScreen();
}

void ASimModeUrdfBot::setupVehiclesAndCamera()
{
    FTransform uu_origin = this->getGlobalNedTransform().getGlobalTransform();

    TArray<AirsimVehicle*> pawns;
    getExistingVehiclePawns(pawns);

    AUrdfBotPawn* fpv_pawn = nullptr;

    int cnt = this->getSettings().vehicles.size();
    for (auto const& vehicle_setting_pair : this->getSettings().vehicles) {
        std::string vehicle_name = vehicle_setting_pair.first;
        const auto& vehicle_setting = *vehicle_setting_pair.second;

        if (vehicle_setting.auto_create && vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeUrdfBot) {

            // Compute initial pose
            FVector spawn_position = uu_origin.GetLocation();
            FRotator spawn_rotation = uu_origin.Rotator();
            msr::airlib::Vector3r settings_position = vehicle_setting.position;
            if (!msr::airlib::VectorMath::hasNan(settings_position))
                spawn_position = getGlobalNedTransform().fromLocalNed(settings_position);
            const auto& rotation = vehicle_setting.rotation;
            if (!std::isnan(rotation.yaw))
                spawn_rotation.Yaw = rotation.yaw;
            if (!std::isnan(rotation.pitch))
                spawn_rotation.Pitch = rotation.pitch;
            if (!std::isnan(rotation.roll))
                spawn_rotation.Roll = rotation.roll;

            // Spawn vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.Name = FName(vehicle_setting.vehicle_name.c_str());
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            AUrdfBotPawn* spawned_pawn = this->GetWorld()->SpawnActor<AUrdfBotPawn>(
                spawn_position, spawn_rotation, pawn_spawn_params);

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);

            if (vehicle_setting.is_fpv_vehicle)
                fpv_pawn = spawned_pawn;
        }
    }

    //create API objects for each pawn we have
    this->cameras_.Empty();
    int camera_offset = 0;
    for (AirsimVehicle* pawn : pawns) {
        //initialize each vehicle pawn we found
        AUrdfBotPawn* vehicle_pawn = static_cast<AUrdfBotPawn*>(pawn);
        vehicle_pawn->InitializeForBeginPlay();

        //create vehicle sim api
        const auto& ned_transform = getGlobalNedTransform();
        const auto& pawn_ned_pos = ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
        const auto& home_geopoint = msr::airlib::EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);

        PawnSimApi::Params params;
        params.vehicle = vehicle_pawn;
        params.global_transform = &getGlobalNedTransform();
        params.pawn_events = vehicle_pawn->GetPawnEvents();
        params.cameras = vehicle_pawn->GetCameras();
        params.pip_camera_class = pip_camera_class;
        params.collision_display_template = collision_display_template;
        params.home_geopoint = home_geopoint;
        params.vehicle_name = "UrdfBot"; // TODO: This may need to be changed for multiple vehicles.

        auto vehicle_sim_api = std::unique_ptr<UrdfBotSimApi>(new UrdfBotSimApi(params));

        for (APIPCamera* camera : vehicle_sim_api->getAllCameras()) {
            int add_index = camera_offset + camera->getIndex();
            while (this->cameras_.Num() <= add_index) {
                this->cameras_.Emplace(nullptr);
            }
            this->cameras_[add_index] = camera;
        }
        camera_offset += vehicle_sim_api->getCameraCount();

        std::string vehicle_name = vehicle_sim_api->getVehicleName();

        auto vehicle_api = vehicle_sim_api->getVehicleApi();
        auto vehicle_sim_api_p = vehicle_sim_api.get();
        getApiProvider()->insert_or_assign(vehicle_name, vehicle_api, vehicle_sim_api_p);
        if ((fpv_pawn == vehicle_pawn || !getApiProvider()->hasDefaultVehicle()) && vehicle_name != "")
            getApiProvider()->makeDefaultVehicle(vehicle_name);

        vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));
    }

    if (getApiProvider()->hasDefaultVehicle()) {
        getVehicleSimApi()->possess();
    }

    // Set up camera cycling
    if (this->cameras_.Num() < 1) {
        throw std::runtime_error("UrdfBotPawn has no cameras defined. Please define at least one camera in settings.json");
    }

    UAirBlueprintLib::EnableInput(this);
    UAirBlueprintLib::BindActionToKey("inputEventCycleCameraForward", EKeys::N, this, &ASimModeUrdfBot::cycleVisibleCameraForward);
    UAirBlueprintLib::BindActionToKey("inputEventCycleCameraBackward", EKeys::P, this, &ASimModeUrdfBot::cycleVisibleCameraBackward);

    this->cameras_[0]->showToScreen();
}

