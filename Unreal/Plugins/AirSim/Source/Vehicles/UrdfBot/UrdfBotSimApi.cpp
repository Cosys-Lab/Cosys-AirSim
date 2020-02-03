#include "UrdfBotSimApi.h"

UrdfBotSimApi::UrdfBotSimApi(Params params)
    : PawnSimApi(params)
{
    this->createVehicleApi(static_cast<AUrdfBotPawn*>(params.vehicle->GetPawn()), params.home_geopoint);
}

void UrdfBotSimApi::updateRenderedState(float dt)
{
    PawnSimApi::updateRenderedState(dt);

    // TODO: What does this do?
    this->vehicle_api_->getStatusMessages(this->vehicle_api_messages_);
}

void UrdfBotSimApi::updateRendering(float dt)
{
    PawnSimApi::updateRendering(dt);

    for (auto message : this->vehicle_api_messages_) {
        UAirBlueprintLib::LogMessage(FString(message.c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

void UrdfBotSimApi::reset()
{
    PawnSimApi::reset();
    this->vehicle_api_->reset();
}

void UrdfBotSimApi::update()
{
    this->vehicle_api_->update();
    PawnSimApi::update();
}

void UrdfBotSimApi::reportState(msr::airlib::StateReporter& reporter)
{
    // TODO: Implementation copied from car. We should probably report more stuff?

    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    FVector unrealPosition = this->getUUPosition();
    reporter.writeValue("unreal pos", Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
}

std::string UrdfBotSimApi::getRecordFileLine(bool is_header_line) const
{
    // TODO: Do something acutally useful here.
    return "Not implemented yet. Whee!\n";
}

void UrdfBotSimApi::createVehicleApi(AUrdfBotPawn* pawn, const msr::airlib::GeoPoint& home_geopoint)
{
    TMap<FString, AActor*> components;
    for (auto const &kvp : pawn->GetLinkComponents())
    {
        components.Add(kvp.Key, static_cast<AActor*>(kvp.Value));
    }

    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(components);

    std::function<const msr::airlib::Kinematics::State*(std::string)> state_provider_fxn = 
        [&](std::string linkName)
    {
        FString linkNameFstr(linkName.c_str());
        return &(static_cast<AUrdfLink*>(pawn->GetLink(linkNameFstr))->GetKinematics());
    };

    this->vehicle_api_ = std::unique_ptr<msr::airlib::UrdfBotApiBase>(new UrdfBotApi(pawn, getPawnKinematics(), home_geopoint, state_provider_fxn, 
        getVehicleSetting(), sensor_factory, (*getGroundTruthEnvironment())));
}
