// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_EchoSimpleParams_hpp
#define msr_airlib_EchoSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr { namespace airlib {

struct EchoSimpleParams {

    // default settings
    // TODO: enable reading of these params from AirSim settings

    int number_of_traces;					// Amount of traces (rays) being cast 
	float reflection_opening_angle;			// Beam width of the scattered traces
	real_T attenuation_per_distance;		// Attenuation of signal wrt distance traveled (dB/m)
	real_T attenuation_per_reflection;		// Attenuation of signal wrt reflections (dB)
	real_T attenuation_limit;				// Attenuation at which the signal is considered dissipated (dB)
	real_T distance_limit;					// Maximum distance the signal can travel.
	int reflection_limit;					// Maximum times the signal can reflect.
	real_T reflection_distance_limit;		// Maximum distance between reflection locations.
	real_T measurement_frequency;			// The frequency of the sensor (measurements/s)
	real_T sensor_diameter;					// The diameter of the sensor plane used to capture the reflecting traces (meter)
	float sensor_opening_angle;				// The opening angle in which rays will be cast from the sensor
	bool pause_after_measurement ;			// Pause the simulation after each measurement. Useful for API interaction to be synced
	bool ignore_marked = false;
	int testParam = 1;

	std::string name = "EchoSensor";

    Pose relative_pose {
        Vector3r(0,0,-1),                   // position - a little above vehicle (especially for cars) or Vector3r::Zero()
        Quaternionr::Identity()             // orientation - by default Quaternionr(1, 0, 0, 0) 
        };                       

	bool external = false;                  // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
	bool external_ned = true;               // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates


	bool draw_reflected_points;				// Draw debug points in world where reflected points are captured by the echo sensor
	bool draw_reflected_lines;				// Draw debug lines in world from reflected points to the echo sensor
	bool draw_reflected_paths;				// Draw debug lines for the full path of reflected points to the sensor
	bool draw_initial_points;				// Draw the points of the initial half sphere where the traces (rays) are cast
	bool draw_bounce_lines;					// Draw lines of all bouncing reflections of the traces with their color depending on attenuation
	bool draw_sensor;						// Draw the physical sensor in the world on the vehicle
	bool draw_external_points;				// Draw points from an external source (e.g. MATLAB clustered pointcloud)

    real_T update_frequency;				// polling rate of update function, in Hz
    real_T startup_delay;               // startup delay of sensor, in sec

    void initializeFromSettings(const AirSimSettings::EchoSetting& settings)
    {
        std::string simmode_name = AirSimSettings::singleton().simmode_name;

		number_of_traces = settings.number_of_traces;
		reflection_opening_angle = settings.reflection_opening_angle;
		sensor_opening_angle = settings.sensor_opening_angle;
		attenuation_per_distance = settings.attenuation_per_distance;
		attenuation_per_reflection = settings.attenuation_per_reflection;
		attenuation_limit = settings.attenuation_limit;
		distance_limit = settings.distance_limit;
		reflection_limit = settings.reflection_limit;
		reflection_distance_limit = settings.reflection_distance_limit;
		measurement_frequency = settings.measurement_frequency;
		sensor_diameter = settings.sensor_diameter;
		pause_after_measurement = settings.pause_after_measurement;
		ignore_marked = settings.ignore_marked;

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
           
        draw_reflected_points = settings.draw_reflected_points;
		draw_reflected_lines = settings.draw_reflected_lines;
		draw_reflected_paths = settings.draw_reflected_paths;
		draw_initial_points = settings.draw_initial_points;
		draw_bounce_lines = settings.draw_bounce_lines;
		draw_sensor = settings.draw_sensor;
		draw_external_points = settings.draw_external_points;

		update_frequency = settings.measurement_frequency;

		external = settings.external;
		external_ned = settings.external_ned;

		startup_delay = 0;
    }
};

}} //namespace
#endif
