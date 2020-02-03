#include "ComputerVisionPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "ComputerVisionPawnApi.h"
#include <exception>

using namespace msr::airlib;

ComputerVisionPawnSimApi::ComputerVisionPawnSimApi(const Params& params)
    : PawnSimApi(params), params_(params)
{
}

void ComputerVisionPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    createVehicleApi(static_cast<AComputerVisionPawn*>(params_.vehicle->GetPawn()), params_.home_geopoint);

    //TODO: should do reset() here?
}

void ComputerVisionPawnSimApi::createVehicleApi(AComputerVisionPawn* pawn, const msr::airlib::GeoPoint& home_geopoint)
{
    //create vehicle params
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_api_ = std::unique_ptr<ComputerVisionApiBase>(new ComputerVisionPawnApi(pawn, getGroundTruthKinematics(), home_geopoint,
        getVehicleSetting(), sensor_factory, 
        (*getGroundTruthKinematics()), (*getGroundTruthEnvironment())));
}

//these are called on render ticks
void ComputerVisionPawnSimApi::updateRenderedState(float dt)
{
    PawnSimApi::updateRenderedState(dt);
    
    vehicle_api_->getStatusMessages(vehicle_api_messages_);

}
void ComputerVisionPawnSimApi::updateRendering(float dt)
{
    PawnSimApi::updateRendering(dt);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

//*** Start: UpdatableState implementation ***//
void ComputerVisionPawnSimApi::reset()
{
    PawnSimApi::reset();

    vehicle_api_->reset();
}

//physics tick
void ComputerVisionPawnSimApi::update(float delta)
{
    vehicle_api_->update(delta);

    PawnSimApi::update(delta);
}

//*** End: UpdatableState implementation ***//

