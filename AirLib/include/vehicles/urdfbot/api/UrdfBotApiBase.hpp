#ifndef air_UrdfBotApiBase_hpp
#define air_UrdfBotApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

#include <vector>

namespace msr { namespace airlib {

class UrdfBotApiBase : public VehicleApiBase {
public:
    struct AddLinearForce {
        std::string force_name;
        std::string link_name;
        msr::airlib::Vector3r application_point;
        msr::airlib::Vector3r axis;

        AddLinearForce() {};

        AddLinearForce(std::string force_name, std::string link_name, msr::airlib::Vector3r application_point, msr::airlib::Vector3r axis)
            : force_name(force_name), link_name(link_name), application_point(application_point), axis(axis) {};
    };

    struct AddAngularForce {
        std::string force_name;
        std::string link_name;
        msr::airlib::Vector3r axis;

        AddAngularForce() {};

        AddAngularForce(std::string force_name, std::string link_name, msr::airlib::Vector3r axis)
            : force_name(force_name), link_name(link_name), axis(axis) {};
    };

    struct UpdateForceMagnitude {
        std::string force_name;
        float magnitude;

        UpdateForceMagnitude() {};

        UpdateForceMagnitude(std::string force_name, float magnitude)
            : force_name(force_name), magnitude(magnitude) {};
    };

    struct LinkInformation {
        std::string link_name;
        msr::airlib::Pose link_relative_pose;
        msr::airlib::Twist link_relative_twist;

        LinkInformation() {};

        LinkInformation(std::string link_name, msr::airlib::Pose link_relative_pose, msr::airlib::Twist link_relative_twist)
            : link_name(link_name), link_relative_pose(link_relative_pose), link_relative_twist(link_relative_twist) {};
    };

    struct UrdfBotState {
        std::vector<LinkInformation> link_information;
        Kinematics::State kinematics_estimated;
        std::map<std::string, std::map<std::string, std::string>> controlled_motion_component_states;

        UrdfBotState() {};

        UrdfBotState(std::vector<LinkInformation> &link_information, Kinematics::State kinematics_estimated, std::map<std::string, std::map<std::string, std::string>> controlled_motion_component_states)
            : link_information(link_information), kinematics_estimated(kinematics_estimated), controlled_motion_component_states(controlled_motion_component_states)
        {
        }
    };

    struct UrdfBotControlledMotionComponentControlSignal {
        std::string component_name;
        std::map<std::string, float> control_signal_values;

        UrdfBotControlledMotionComponentControlSignal() {};

        UrdfBotControlledMotionComponentControlSignal(std::string component_name, std::map<std::string, float> control_signal_values)
            : component_name(component_name), control_signal_values(control_signal_values)
        {
        }
    };

public:
    UrdfBotApiBase(const AirSimSettings::VehicleSetting* vehicle_setting,
        std::shared_ptr<SensorFactory> sensor_factory,
        std::function<const msr::airlib::Kinematics::State*(std::string)> state_provider_fxn, const Environment& environment)
    {
        sensor_factory_ = sensor_factory;

        sensor_storage_.clear();
        sensors_.clear();
        
        addSensorsFromSettings(vehicle_setting);

        getSensors().initialize(state_provider_fxn, &environment);
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

    // sensor helpers
    virtual const SensorCollection& getSensors() const override
    {
        return sensors_;
    }

    SensorCollection& getSensors()
    {
        return sensors_;
    }

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

    virtual ~UrdfBotApiBase() = default;

    virtual void addLinearForce(const AddLinearForce& addedForce, const std::string& vehicle_name) = 0;
    virtual void addAngularForce(const AddAngularForce& addedForce, const std::string& vehicle_name) = 0;
    virtual void updateForceMagnitude(const UpdateForceMagnitude& updatedForce, const std::string& vehicle_name) = 0;
    virtual void updateControlledMotionComponentControlSignal(const UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name) = 0;
    virtual const UrdfBotState getBotState(const std::string& vehicle_name) = 0;

    std::shared_ptr<const SensorFactory> sensor_factory_;
    SensorCollection sensors_; //maintains sensor type indexed collection of sensors
    vector<unique_ptr<SensorBase>> sensor_storage_; //RAII for created sensor

};
}}

#endif