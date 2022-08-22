// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GPULidarSimpleParams_hpp
#define msr_airlib_GPULidarSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr {
	namespace airlib {

		struct GPULidarSimpleParams {

			// Velodyne VLP-16 Puck config
			// https://velodynelidar.com/vlp-16.html

			// default settings
			// TODO: enable reading of these params from AirSim settings

			uint number_of_channels = 64;
			real_T range = 5000.0f / 100;                // meters
			float range_max_lambertian_percentage = 80;  // Lambertian reflectivity percentage to max out on. Will act linear to 0% for below.
			float rain_max_intensity = 70;               // Rain intensity maximum to scale from in mm/hour.
			float rain_constant_a = 0.01;                // Two constants to calculate the extinction coefficient in rain
			float rain_constant_b = 0.6;
			uint measurement_per_cycle = 2048;
			real_T horizontal_rotation_frequency = 10;   // rotations/sec
			real_T horizontal_FOV_start = 0;
			real_T horizontal_FOV_end = 359;
			real_T vertical_FOV_upper = -15;             // drones -15, car +10
			real_T vertical_FOV_lower = -45;             // drones -45, car -10
			uint resolution = 512;
			bool ground_truth = false;                   // Generate ground truth segmentation color values

			bool ignore_marked = false;
			bool generate_noise = false;			     // Toggle range based noise
			real_T min_noise_standard_deviation = 0;     // Minimum noise standard deviation
			real_T noise_distance_scale = 1;		     // Factor to scale noise based on distance
			real_T update_frequency = 10;                // Hz

			Pose relative_pose{
				Vector3r(0,0,-1),                        // position - a little above vehicle (especially for cars) or Vector3r::Zero()
				Quaternionr::Identity()                  // orientation - by default Quaternionr(1, 0, 0, 0)
			};

			bool generate_intensity = false;             // Toggle intensity calculation on or off

			bool external = false;                       // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
			bool external_ned = true;                    // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates

			bool draw_debug_points = false;
			uint draw_mode = 0;							 // 0 = no coloring, 1 = instance segmentation, 2 = material, 3 = impact angle, 4 = intensity
			bool draw_sensor;						     // Draw the physical sensor in the world on the vehicle

			real_T startup_delay = 1;                    // sec

			std::string material_list_file = "";         // String holding all material data


			void initializeFromSettings(const AirSimSettings::GPULidarSetting& settings)
			{
				std::string simmode_name = AirSimSettings::singleton().simmode_name;

				number_of_channels = settings.number_of_channels;
				range = settings.range;
				measurement_per_cycle = settings.measurement_per_cycle;
				horizontal_rotation_frequency = settings.horizontal_rotation_frequency;
				resolution = settings.resolution;

				horizontal_FOV_start = settings.horizontal_FOV_start;
				horizontal_FOV_end = settings.horizontal_FOV_end;

				ground_truth = settings.ground_truth;

				update_frequency = settings.update_frequency;
				ignore_marked = settings.ignore_marked;
				generate_noise = settings.generate_noise;
				min_noise_standard_deviation = settings.min_noise_standard_deviation;
				noise_distance_scale = settings.noise_distance_scale;

				range_max_lambertian_percentage = settings.range_max_lambertian_percentage;
				rain_max_intensity = settings.rain_max_intensity;
				rain_constant_a = settings.rain_constant_a;
				rain_constant_b = settings.rain_constant_b;

				// By default, for multirotors the lidars FOV point downwards;
				// for cars, the lidars FOV is more forward facing.
				vertical_FOV_upper = settings.vertical_FOV_upper;
				if (std::isnan(vertical_FOV_upper)) {
					if (simmode_name == "Multirotor")
						vertical_FOV_upper = -15;
					else
						vertical_FOV_upper = +10;
				}

				vertical_FOV_lower = settings.vertical_FOV_lower;
				if (std::isnan(vertical_FOV_lower)) {
					if (simmode_name == "Multirotor")
						vertical_FOV_lower = -45;
					else
						vertical_FOV_lower = -10;
				}

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
					Utils::degreesToRadians(yaw));    //yaw   - rotation around Z axis

				generate_intensity = settings.generate_intensity;
				material_list_file = settings.material_list_file;

				draw_debug_points = settings.draw_debug_points;
				draw_mode = settings.draw_mode;
				draw_sensor = settings.draw_sensor;

				external = settings.external;
				external_ned = settings.external_ned;
			}
		};

	}
} //namespace
#endif
