#include "ComputerVisionPawnApi.h"
#include "AirBlueprintLib.h"

ComputerVisionPawnApi::ComputerVisionPawnApi(AComputerVisionPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
    const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<msr::airlib::SensorFactory> sensor_factory, 
    const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment)
    : msr::airlib::ComputerVisionApiBase(vehicle_setting, sensor_factory, state, environment),
    pawn_(pawn), pawn_kinematics_(pawn_kinematics), home_geopoint_(home_geopoint)
{
}

bool ComputerVisionPawnApi::armDisarm(bool arm)
{
    //TODO: implement arming for ComputerVision
    unused(arm);
    return true;
}

msr::airlib::ComputerVisionApiBase::ComputerVisionState ComputerVisionPawnApi::getComputerVisionState() const
{
    ComputerVisionApiBase::ComputerVisionState state(
        *pawn_kinematics_,
        msr::airlib::ClockFactory::get()->nowNanos()
    );
    return state;
}

void ComputerVisionPawnApi::reset()
{
    msr::airlib::ComputerVisionApiBase::reset();
}

void ComputerVisionPawnApi::update(float delta)
{
    msr::airlib::ComputerVisionApiBase::update(delta);
}

msr::airlib::GeoPoint ComputerVisionPawnApi::getHomeGeoPoint() const
{
    return home_geopoint_;
}

void ComputerVisionPawnApi::enableApiControl(bool is_enabled)
{
    if (api_control_enabled_ != is_enabled) {
        api_control_enabled_ = is_enabled;
    }
}

bool ComputerVisionPawnApi::isApiControlEnabled() const
{
    return api_control_enabled_;
}

ComputerVisionPawnApi::~ComputerVisionPawnApi() = default;
