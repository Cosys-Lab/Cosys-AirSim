// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ComputerVisionController_hpp
#define msr_airlib_ComputerVisionController_hpp

#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"

namespace msr
{
namespace airlib
{

    class ComputerVisionApi : public ComputerVisionApiBase
    {
    public:
        ComputerVisionApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory,
            const Kinematics::State& state, const Environment& environment)
            : ComputerVisionApiBase(vehicle_setting, sensor_factory, state, environment), home_geopoint_(environment.getHomeGeoPoint())
        {
        }

        ~ComputerVisionApi()
        {
        }

    protected:
        virtual void resetImplementation() override
        {
            ComputerVisionApiBase::resetImplementation();
        }

    public:
        virtual void update(float delta) override
        {
            ComputerVisionApiBase::update(delta);
        }

        virtual const SensorCollection& getSensors() const override
        {
            return ComputerVisionApiBase::getSensors();
        }

        // VehicleApiBase Implementation
        virtual void enableApiControl(bool is_enabled) override
        {
            if (api_control_enabled_ != is_enabled) {
                api_control_enabled_ = is_enabled;
            }
        }

        virtual bool isApiControlEnabled() const override
        {
            return api_control_enabled_;
        }

        virtual GeoPoint getHomeGeoPoint() const override
        {
            return home_geopoint_;
        }

        virtual bool armDisarm(bool arm) override
        {
            //TODO: implement arming for car
            unused(arm);
            return true;
        }

    public:

        virtual void updateComputerVisionState(const ComputerVisionState& car_state) override
        {
            last_car_state_ = car_state;
        }

        virtual const ComputerVisionState& getComputerVisionState() const override
        {
            return last_car_state_;
        }

    private:
        bool api_control_enabled_ = false;
        GeoPoint home_geopoint_;
        ComputerVisionState last_car_state_;
    };
}
}

#endif
