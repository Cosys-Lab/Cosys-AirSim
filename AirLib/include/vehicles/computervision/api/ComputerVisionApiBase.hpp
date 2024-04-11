// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ComputerVisionApiBase_hpp
#define air_ComputerVisionApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr { namespace airlib {

class ComputerVisionApiBase : public VehicleApiBase
{
public:

    struct ComputerVisionState {
        Kinematics::State kinematics_estimated;
        uint64_t timestamp;

        ComputerVisionState()
        {
        }

        ComputerVisionState(const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
            : kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
        {
        }

        //shortcuts
        const Vector3r& getPosition() const
        {
            return kinematics_estimated.pose.position;
        }
        const Quaternionr& getOrientation() const
        {
            return kinematics_estimated.pose.orientation;
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
        const auto& sensor_settings = vehicle_setting->sensors;
        sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
    }

    virtual const ComputerVisionState& getComputerVisionState() const = 0;
    virtual void updateComputerVisionState(const ComputerVisionState& state) = 0;

    virtual ~ComputerVisionApiBase() = default;

    std::shared_ptr<const SensorFactory> sensor_factory_;
    SensorCollection sensors_; //maintains sensor type indexed collection of sensors
    vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
    
protected:
    //default implementation so derived class doesn't have to call on VehicleApiBase
    virtual void resetImplementation() override
    {

        //reset sensors last after their ground truth has been reset
        getSensors().reset();
    }
};


}} //namespace
#endif