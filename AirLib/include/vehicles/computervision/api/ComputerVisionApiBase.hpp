// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ComputerVisionApiBase_hpp
#define air_ComputerVisionApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr { namespace airlib {

class ComputerVisionApiBase : public VehicleApiBase  {
public:

    struct ComputerVisionState {
        Kinematics::State kinematics_estimated;
        uint64_t timestamp;

        ComputerVisionState(const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
            : kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
        {
        }
    };

public:

    // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
	//ComputerVisionApiBase() {}

    ComputerVisionApiBase(const AirSimSettings::VehicleSetting* vehicle_setting, 
        std::shared_ptr<SensorFactory> sensor_factory, 
        const Kinematics::State& state, const Environment& environment)
    {
        initialize(vehicle_setting, sensor_factory, state, environment);
    }

    //default implementation so derived class doesn't have to call on VehicleApiBase
    virtual void reset() override
    {
        VehicleApiBase::reset();

        //reset sensors last after their ground truth has been reset
        getSensors().reset();
    }

    virtual void update(float delta = 0) override
    {
        VehicleApiBase::update(delta);

        getSensors().update(delta);
    }
	
    void reportState(StateReporter& reporter) override
    {
        getSensors().reportState(reporter);
    }

    // sensor helpers
    virtual const SensorCollection& getSensors() const override
    {
        return sensors_;
    }

    SensorCollection& getSensors()
    {
        return sensors_;
    }

    void initialize(const AirSimSettings::VehicleSetting* vehicle_setting, 
        std::shared_ptr<SensorFactory> sensor_factory, 
        const Kinematics::State& state, const Environment& environment)
    {
        sensor_factory_ = sensor_factory;

        sensor_storage_.clear();
        sensors_.clear();
        
        addSensorsFromSettings(vehicle_setting);

        getSensors().initialize(&state, &environment);
    }

    void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
    {
        // use sensors from vehicle settings; if empty list, use default sensors.
        // note that the vehicle settings completely override the default sensor "list";
        // there is no piecemeal add/remove/update per sensor.
        const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>>& sensor_settings
            = vehicle_setting->sensors.size() > 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;

        sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
    }

    virtual ComputerVisionState getComputerVisionState() const = 0;

    virtual ~ComputerVisionApiBase() = default;

    std::shared_ptr<const SensorFactory> sensor_factory_;
    SensorCollection sensors_; //maintains sensor type indexed collection of sensors
    vector<unique_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
};


}} //namespace
#endif