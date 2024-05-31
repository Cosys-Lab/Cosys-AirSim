// Developed by Cosys-Lab, University of Antwerp

#ifndef msr_airlib_EchoSimpleParams_hpp
#define msr_airlib_EchoSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr { namespace airlib {

struct EchoSimpleParams {

    int number_of_traces = 1000;					// Amount of traces (rays) being cast 
	float reflection_opening_angle = 10;			// Beam width of the scattered traces
	real_T attenuation_per_distance = 0;		// Attenuation of signal wrt distance traveled (dB/m)
	real_T attenuation_per_reflection = 0;		// Attenuation of signal wrt reflections (dB)
	real_T attenuation_limit = -100;				// Attenuation at which the signal is considered dissipated (dB)
	real_T distance_limit = 10;					// Maximum distance the signal can travel.
	int reflection_limit = 3;					// Maximum times the signal can reflect.
	real_T reflection_distance_limit = 0.4;		// Maximum distance between reflection locations.
	real_T measurement_frequency = 10;			// The frequency of the sensor (measurements/s)
	real_T sensor_diameter = 0.5;					// The diameter of the sensor plane used to capture the reflecting traces (meter)
	float sensor_lower_azimuth_limit = -90;		// The lower azimuth limit of the sensor opening angle in degrees.
	float sensor_upper_azimuth_limit = 90;		// The upper azimuth limit of the sensor opening angle in degrees.
	float sensor_lower_elevation_limit = -90;		// The lower elevation limit of the sensor opening angle in degrees.
	float sensor_upper_elevation_limit = 90;		// The upper elevation limit of the sensor opening angle in degrees.
	float sensor_passive_radius = 10;            // The radius in meters in which the sensor will receive signals from passive sources if that mode is enabled. 

	bool pause_after_measurement = false;			// Pause the simulation after each measurement. Useful for API interaction to be synced
	bool ignore_marked = false;
	int testParam = 1;

    Pose relative_pose {
        Vector3r(0,0,-1),                   // position - a little above vehicle (especially for cars) or Vector3r::Zero()
        Quaternionr::Identity()             // orientation - by default Quaternionr(1, 0, 0, 0) 
        };                       

	bool external = false;                  // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
	bool external_ned = true;               // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates
	bool passive = false;                   // Sense and capture passive echo beacon data
	bool active = true;                     // Sense and capture active echo beacon data (enable emission)
	bool parallel = true;                   // Use ParallelFor for speeding up sampling. This disables all debug drawing except for the final reflected points if enabled.

	bool draw_reflected_points = false;				// Draw debug points in world where reflected points are captured by the echo sensor
	bool draw_reflected_lines = false;				// Draw debug lines in world from reflected points to the echo sensor
	bool draw_reflected_paths = false;				// Draw debug lines for the full path of reflected points to the sensor
	bool draw_initial_points = false;				// Draw the points of the initial half sphere where the traces (rays) are cast
	bool draw_bounce_lines = false;					// Draw lines of all bouncing reflections of the traces with their color depending on attenuation
	bool draw_sensor = false;						// Draw the physical sensor in the world on the vehicle
	bool draw_external_points = false;				// Draw points from an external source (e.g. MATLAB clustered pointcloud)
	bool draw_passive_sources = false;		        // Draw debug points and reflection lines for all detected passive echo sources (original sources and their reflection echos against objects).
	bool draw_passive_lines = false;	        	// Draw debug lines of the sensor to the passive echo sources that are detected with line of sight. 

    real_T update_frequency = 10;				// polling rate of update function, in Hz
    real_T startup_delay = 1;                   // startup delay of sensor, in sec

    void initializeFromSettings(const AirSimSettings::EchoSetting& settings)
    {
        std::string simmode_name = AirSimSettings::singleton().simmode_name;

        const auto& settings_json = settings.settings;

		number_of_traces = settings_json.getInt("NumberOfTraces", number_of_traces);
		reflection_opening_angle = settings_json.getFloat("ReflectionOpeningAngle", reflection_opening_angle);
		sensor_lower_azimuth_limit = settings_json.getFloat("SensorLowerAzimuthLimit", sensor_lower_azimuth_limit);
		sensor_upper_azimuth_limit = settings_json.getFloat("SensorUpperAzimuthLimit", sensor_upper_azimuth_limit);
		sensor_lower_elevation_limit = settings_json.getFloat("SensorLowerElevationLimit", sensor_lower_elevation_limit);
		sensor_upper_elevation_limit = settings_json.getFloat("SensorUpperElevationLimit", sensor_upper_elevation_limit);
		sensor_passive_radius = settings_json.getFloat("PassiveRadius", sensor_passive_radius);
		attenuation_per_distance = settings_json.getFloat("AttenuationPerDistance", attenuation_per_distance);
		attenuation_per_reflection = settings_json.getFloat("AttenuationPerReflection", attenuation_per_reflection);
		attenuation_limit = settings_json.getFloat("AttenuationLimit", attenuation_limit);
		distance_limit = settings_json.getFloat("DistanceLimit", distance_limit);
		reflection_limit = settings_json.getInt("ReflectionLimit", reflection_limit);
		reflection_distance_limit = settings_json.getFloat("ReflectionDistanceLimit", reflection_distance_limit);
		measurement_frequency = settings_json.getFloat("MeasurementFrequency", measurement_frequency);
		sensor_diameter = settings_json.getFloat("SensorDiameter", sensor_diameter);
		pause_after_measurement = settings_json.getBool("PauseAfterMeasurement", pause_after_measurement);
		ignore_marked = settings_json.getBool("IgnoreMarked", ignore_marked);
		parallel = settings_json.getBool("RunParallel", parallel);

        relative_pose.position = AirSimSettings::createVectorSetting(settings_json, VectorMath::nanVector());
        auto rotation = AirSimSettings::createRotationSetting(settings_json, AirSimSettings::Rotation::nanRotation());


        if (std::isnan(relative_pose.position.x()))
            relative_pose.position.x() = 0;
        if (std::isnan(relative_pose.position.y()))
            relative_pose.position.y() = 0;
        if (std::isnan(relative_pose.position.z())) {
            if (simmode_name == "Multirotor")
                relative_pose.position.z() = 0;
            else
                relative_pose.position.z() = -1;
        }

        float pitch, roll, yaw;
        pitch = !std::isnan(rotation.pitch) ? rotation.pitch : 0;
        roll = !std::isnan(rotation.roll) ? rotation.roll : 0;
        yaw = !std::isnan(rotation.yaw) ? rotation.yaw : 0;
        relative_pose.orientation = VectorMath::toQuaternion(
            Utils::degreesToRadians(pitch), // pitch - rotation around Y axis
            Utils::degreesToRadians(roll), // roll  - rotation around X axis
            Utils::degreesToRadians(yaw)); // yaw   - rotation around Z axis


        draw_reflected_points = settings_json.getBool("DrawReflectedPoints", draw_reflected_points);
		draw_reflected_lines = settings_json.getBool("DrawReflectedLines", draw_reflected_lines);
		draw_reflected_paths = settings_json.getBool("DrawReflectedPaths", draw_reflected_paths);
		draw_initial_points = settings_json.getBool("DrawInitialPoints", draw_initial_points);
		draw_bounce_lines = settings_json.getBool("DrawBounceLines", draw_bounce_lines);
		draw_sensor = settings_json.getBool("DrawSensor", draw_sensor);
		draw_external_points = settings_json.getBool("DrawExternalPoints", draw_external_points);
		draw_passive_sources = settings_json.getBool("DrawPassiveSources", draw_passive_sources);
		draw_passive_lines = settings_json.getBool("DrawPassiveLines", draw_passive_lines);

		update_frequency = settings_json.getFloat("MeasurementFrequency", update_frequency);

		external = settings_json.getBool("External", external);
		external_ned = settings_json.getBool("ExternalLocal", external_ned);
		passive = settings_json.getBool("SensePassive", passive);
		active = settings_json.getBool("SenseActive", active);

		startup_delay = 0;
    }
};

}} //namespace
#endif
