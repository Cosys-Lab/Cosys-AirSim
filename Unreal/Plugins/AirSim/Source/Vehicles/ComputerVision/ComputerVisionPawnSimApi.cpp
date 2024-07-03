#include "ComputerVisionPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "ComputerVisionPawnApi.h"
#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"
#include "vehicles/computervision/api/ComputerVisionApi.hpp"
#include <exception>

using namespace msr::airlib;

ComputerVisionPawnSimApi::ComputerVisionPawnSimApi(const Params& params)
    : PawnSimApi(params)
{
}

void ComputerVisionPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle params
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());   
    vehicle_api_ = std::unique_ptr<ComputerVisionApiBase>(new ComputerVisionApi(getVehicleSetting(), sensor_factory, *getGroundTruthKinematics(), *getGroundTruthEnvironment()));
    pawn_api_ = std::unique_ptr<ComputerVisionPawnApi>(new ComputerVisionPawnApi(static_cast<AComputerVisionPawn*>(getPawn()), getGroundTruthKinematics(), vehicle_api_.get()));

    //TODO: should do reset() here?
}

std::string ComputerVisionPawnSimApi::getRecordFileLine(bool is_header_line) const
{
    std::string common_line = PawnSimApi::getRecordFileLine(is_header_line);
    if (is_header_line) {
        return common_line +
            "Throttle\tSteering\tBrake\tGear\tHandbrake\tRPM\tSpeed\t";
    }

    const auto& state = pawn_api_->getComputerVisionState();

    std::ostringstream ss;
    ss << common_line;
    return ss.str();
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
    catch (std::exception& e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

//*** Start: UpdatableState implementation ***//
void ComputerVisionPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    pawn_api_->reset();
}

//physics tick
void ComputerVisionPawnSimApi::update(float delta)
{
    pawn_api_->update(delta);

    PawnSimApi::update(delta);
}

void ComputerVisionPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    vehicle_api_->reportState(reporter);
}

//*** End: UpdatableState implementation ***//

