#pragma once

#include "AirLib/include/common/VectorMath.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorCollection.hpp"
#include "ControlledMotionComponent/ControlledMotionComponent.h"
#include "UrdfBotPawn.h"

class UrdfBotApi : public msr::airlib::UrdfBotApiBase {
public:

    UrdfBotApi(AUrdfBotPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
        std::function<const msr::airlib::Kinematics::State*(std::string)> state_provider_fxn, const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting,
        std::shared_ptr<msr::airlib::SensorFactory> sensor_factory, const msr::airlib::Environment& environment);
    virtual ~UrdfBotApi();

    virtual void addLinearForce(const AddLinearForce& added_force, const std::string& vehicle_name) override;
    virtual void addAngularForce(const AddAngularForce& added_force, const std::string& vehicle_name) override;
    virtual void updateForceMagnitude(const UpdateForceMagnitude& updated_force, const std::string& vehicle_name) override;
    virtual void updateControlledMotionComponentControlSignal(const UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name) override;
    virtual const UrdfBotState getBotState(const std::string& vehicle_name) override;

    virtual void reset() override;
    virtual void update(float delta = 0) override;

    virtual msr::airlib::GeoPoint getHomeGeoPoint() const override;

    virtual void enableApiControl(bool is_enabled) override;
    virtual bool isApiControlEnabled() const override;
    virtual bool armDisarm(bool arm) override;

private:
    bool api_control_enabled_ = false;
    AUrdfBotPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::GeoPoint home_geopoint_;

    TMap<FString, AUrdfLink*> force_link_mapping_;

};