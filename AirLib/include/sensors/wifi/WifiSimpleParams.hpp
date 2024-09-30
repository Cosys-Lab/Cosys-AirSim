// Developed by Cosys-Lab, University of Antwerp

#ifndef msr_airlib_WifiSimpleParams_hpp
#define msr_airlib_WifiSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr { namespace airlib {

struct WifiSimpleParams {
	int number_of_traces;					// Amount of traces (rays) being cast 
	float sensor_opening_angle;				// The opening angle in which rays will be cast from the sensor
	real_T measurement_frequency;			// The frequency of the sensor (measurements/s)
	bool pause_after_measurement ;			// Pause the simulation after each measurement. Useful for API interaction to be synced
    bool external = false;                  // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
    bool external_ned = true;               // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates
    bool draw_sensor = false;						// Draw the physical sensor in the world on the vehicle


    Pose relative_pose {
        Vector3r(0,0,-1),                   // position - a little above vehicle (especially for cars) or Vector3r::Zero()
        Quaternionr::Identity()             // orientation - by default Quaternionr(1, 0, 0, 0) 
        };                       


											// If true, the time passed in-engine will be used (when performance doesn't allow real-time operation)
    std::string data_frame = AirSimSettings::kVehicleInertialFrame;

    real_T update_frequency;				// polling rate of update function, in Hz
    real_T startup_delay;               // startup delay of sensor, in sec

    void initializeFromSettings(const AirSimSettings::WifiSetting& settings)
    {
        std::string simmode_name = AirSimSettings::singleton().simmode_name;

        const auto& settings_json = settings.settings;
        measurement_frequency = settings_json.getFloat("MeasurementFrequency", measurement_frequency);
        pause_after_measurement = settings_json.getBool("PauseAfterMeasurement", pause_after_measurement);
        number_of_traces = settings_json.getInt("NumberOfTraces", number_of_traces);
        sensor_opening_angle = settings_json.getFloat("SensorOpeningAngle", sensor_opening_angle);
        external = settings_json.getBool("External", external);
        external_ned = settings_json.getBool("ExternalLocal", external_ned);
        draw_sensor = settings_json.getBool("DrawSensor", draw_sensor);
        data_frame = settings_json.getString("DataFrame", data_frame);


        relative_pose.position = AirSimSettings::createVectorSetting(settings_json, VectorMath::nanVector());
        auto rotation = AirSimSettings::createRotationSetting(settings_json, AirSimSettings::Rotation::nanRotation());

        if (std::isnan(relative_pose.position.x()))
            relative_pose.position.x() = 0;
        if (std::isnan(relative_pose.position.y()))
            relative_pose.position.y() = 0;
        if (std::isnan(relative_pose.position.z())) {
            relative_pose.position.z() = 0;
        }

        float pitch, roll, yaw;
        pitch = !std::isnan(rotation.pitch) ? rotation.pitch : 0;
        roll = !std::isnan(rotation.roll) ? rotation.roll : 0;
        yaw = !std::isnan(rotation.yaw) ? rotation.yaw : 0;
        relative_pose.orientation = VectorMath::toQuaternion(
            Utils::degreesToRadians(pitch), // pitch - rotation around Y axis
            Utils::degreesToRadians(roll), // roll  - rotation around X axis
            Utils::degreesToRadians(yaw)); // yaw   - rotation around Z axis

        update_frequency = settings_json.getFloat("UpdateFrequency", update_frequency);
        startup_delay = 0;
    }
};

}} //namespace
#endif
