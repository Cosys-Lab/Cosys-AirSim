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
	std::string name = "Wifi";
    bool external = false;                  // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
    bool external_ned = true;               // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates
    bool draw_sensor;						// Draw the physical sensor in the world on the vehicle


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
		number_of_traces = settings.number_of_traces;
		sensor_opening_angle = settings.sensor_opening_angle;
		pause_after_measurement = settings.pause_after_measurement;
		measurement_frequency = settings.measurement_frequency;
        external = settings.external;
        external_ned = settings.external_ned;
        draw_sensor = settings.draw_sensor;
		name = settings.sensor_name;

        relative_pose.position = settings.position;
        if (std::isnan(relative_pose.position.x()))
            relative_pose.position.x() = 0;
        if (std::isnan(relative_pose.position.y()))
            relative_pose.position.y() = 0;
        if (std::isnan(relative_pose.position.z())) {
            if (simmode_name == "Multirotor")
                relative_pose.position.z() = 0;
            else
                relative_pose.position.z() = -1;  // a little bit above for cars
        }

        float pitch, roll, yaw;
        pitch = !std::isnan(settings.rotation.pitch) ? settings.rotation.pitch : 0;
        roll = !std::isnan(settings.rotation.roll) ? settings.rotation.roll : 0;
        yaw = !std::isnan(settings.rotation.yaw) ? settings.rotation.yaw : 0;
        relative_pose.orientation = VectorMath::toQuaternion(
            Utils::degreesToRadians(pitch),   //pitch - rotation around Y axis
            Utils::degreesToRadians(roll),    //roll  - rotation around X axis
            Utils::degreesToRadians(yaw)	  //yaw   - rotation around Z axis
		);  

        data_frame = settings.data_frame;
		update_frequency = settings.measurement_frequency;

		startup_delay = 0;
    }
};

}} //namespace
#endif
