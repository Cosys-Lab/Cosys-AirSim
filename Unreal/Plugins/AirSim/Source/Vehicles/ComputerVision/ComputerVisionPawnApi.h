#pragma once

#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "ComputerVisionPawn.h"


class ComputerVisionPawnApi : public msr::airlib::ComputerVisionApiBase {
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    ComputerVisionPawnApi(AComputerVisionPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
        const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<msr::airlib::SensorFactory> sensor_factory,
        const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment);

    virtual ComputerVisionApiBase::ComputerVisionState getComputerVisionState() const override;

    virtual void resetImplementation() override;
    virtual void update(float delta = 0) override;

    virtual msr::airlib::GeoPoint getHomeGeoPoint() const override;

    virtual void enableApiControl(bool is_enabled) override;
    virtual bool isApiControlEnabled() const override;
    virtual bool armDisarm(bool arm) override;

    virtual ~ComputerVisionPawnApi();

private:
    bool api_control_enabled_ = false;
    AComputerVisionPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::GeoPoint  home_geopoint_;
};