// Developed by Cosys-Lab, University of Antwerp

#ifndef msr_airlib_GPULidarSimpleParams_hpp
#define msr_airlib_GPULidarSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr {
	namespace airlib {

		struct GPULidarSimpleParams {

			uint number_of_channels = 64;				 // Amount of lasers of the sensor
			real_T range = 50.0f;                        // maximum range a point is detected in meters
			uint measurement_per_cycle = 2048;		     // The horizontal measurement count/frequency
			real_T horizontal_rotation_frequency = 10;   // rotation frequency of the sensor in Hz
			real_T horizontal_FOV_start = 0;			 // starting angle of rotation in degrees
			real_T horizontal_FOV_end = 360;			 // ending angle of rotation in degrees, by default it works as a full horizontal FOV of 360 degrees
			real_T vertical_FOV_upper = -15;             // Upper angle of vertical FOV in degrees
			real_T vertical_FOV_lower = -45;             // lower angle of vertical FOV in degrees
			uint resolution = 512;					     // Resolution of the render texture, influences performance on the GPU
			bool ground_truth = false;                   // Generate ground truth segmentation color values, if false will set to zero
            bool instance_segmentation = true;           // Force the groundtruth labels to be instance segmentation. Set to false to use different annotation layer.
            std::string annotation_name = "";            // If disabling instance_segmentation, this field chooses with annotation layer gets used by the lidar for groundtruth. 
			bool ignore_marked = false;					 // If enabled, it will not detect objects marked to be ignored  (with the 'LidarIgnore' tag)
			bool generate_noise = false;			     // Toggle range based noise
			real_T min_noise_standard_deviation = 0;     // Minimum noise standard deviation
			real_T noise_distance_scale = 1;		     // Factor to scale noise based on distance

			Pose relative_pose{
				Vector3r(0,0,-1),                        // position - a little above vehicle (especially for cars)
				Quaternionr::Identity()                  // orientation - by default Quaternionr(1, 0, 0, 0)
			};

			bool generate_intensity = false;             // Toggle intensity calculation on or off
			float range_max_lambertian_percentage = 80;  // Lambertian reflectivity percentage to max out on. Will act linear to 0% for below
			float rain_max_intensity = 70;               // Rain intensity maximum to scale from in mm/hour
			std::string material_list_file = "";         // String holding all material data

			float rain_constant_a = 0.01;                // Two constants to calculate the extinction coefficient in rain
			float rain_constant_b = 0.6;

			bool external = false;                       // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
			bool external_ned = true;                    // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates

			bool draw_debug_points = false;				 // Enable the drawing of debug cubes of the pointcloud. Disable this for real measurements as it is impacted(detected) by the virtual camera! 
			uint draw_mode = 0;							 // 0 = no coloring, 1 = instance segmentation, 2 = material, 3 = impact angle, 4 = intensity
			bool draw_sensor = false;						     // Draw the physical sensor in the world on the vehicle with a 3d colored axis

			real_T update_frequency = 10;                // Frequency to update the sensor at in Hz
			real_T startup_delay = 1;                    // Delay until sensor is enabled in seconds

            float pitch = 0;                            // Pitch of the sensor in degrees
            float roll = 0;                             // Roll of the sensor in degrees
            float yaw = 0;                              // Yaw of the sensor in degrees
			void initializeFromSettings(const AirSimSettings::GPULidarSetting& settings)
			{
				std::string simmode_name = AirSimSettings::singleton().simmode_name;


                const auto& settings_json = settings.settings;

                number_of_channels = settings_json.getInt("NumberOfChannels", number_of_channels);
                range = settings_json.getFloat("Range", range);
                measurement_per_cycle = settings_json.getInt("MeasurementsPerCycle", measurement_per_cycle);
                horizontal_rotation_frequency = settings_json.getInt("RotationsPerSecond", horizontal_rotation_frequency);
                resolution = settings_json.getInt("Resolution", resolution);
                update_frequency = settings_json.getFloat("UpdateFrequency", update_frequency);
                vertical_FOV_upper = settings_json.getFloat("VerticalFOVUpper", Utils::nan<float>());
                draw_debug_points = settings_json.getBool("DrawDebugPoints", draw_debug_points);
                draw_sensor = settings_json.getBool("DrawSensor", draw_sensor);
                external = settings_json.getBool("External", external);
                external_ned = settings_json.getBool("ExternalLocal", external_ned);
                generate_noise = settings_json.getBool("GenerateNoise", generate_noise);
                min_noise_standard_deviation = settings_json.getFloat("MinNoiseStandardDeviation", min_noise_standard_deviation);
                noise_distance_scale = settings_json.getFloat("NoiseDistanceScale", noise_distance_scale);
                ground_truth = settings_json.getBool("GroundTruth", ground_truth);
                instance_segmentation = settings_json.getBool("InstanceSegmentation", instance_segmentation);
                annotation_name = settings_json.getString("Annotation", annotation_name);
                ignore_marked = settings_json.getBool("IgnoreMarked", ignore_marked);
                range_max_lambertian_percentage = settings_json.getFloat("rangeMaxLambertianPercentage", range_max_lambertian_percentage);
				rain_max_intensity = settings_json.getFloat("rainMaxIntensity", rain_max_intensity);
				rain_constant_a = settings_json.getFloat("rainConstantA", rain_constant_a);
				rain_constant_b = settings_json.getFloat("rainConstantB", rain_constant_b);
				generate_intensity = settings_json.getBool("GenerateIntensity", generate_intensity);
				draw_mode = settings_json.getInt("DrawMode", draw_mode);

                if (FILE* file = fopen(msr::airlib::Settings::getExecutableFullPath("materials.csv").c_str(), "r")) {
                    fclose(file);
                    material_list_file = msr::airlib::Settings::getExecutableFullPath("materials.csv");
                }
                else {
                    material_list_file = msr::airlib::Settings::Settings::getUserDirectoryFullPath("materials.csv");
                }

                // By default, for multirotors the lidars FOV point downwards;
                // for cars, the lidars FOV is more forward facing.
                if (std::isnan(vertical_FOV_upper)) {
                    if (simmode_name == AirSimSettings::kSimModeTypeMultirotor)
                        vertical_FOV_upper = -15;
                    else
                        vertical_FOV_upper = +10;
                }

                vertical_FOV_lower = settings_json.getFloat("VerticalFOVLower", Utils::nan<float>());
                if (std::isnan(vertical_FOV_lower)) {
                    if (simmode_name == AirSimSettings::kSimModeTypeMultirotor)
                        vertical_FOV_lower = -45;
                    else
                        vertical_FOV_lower = -10;
                }

                horizontal_FOV_start = settings_json.getFloat("HorizontalFOVStart", horizontal_FOV_start);
                horizontal_FOV_end = settings_json.getFloat("HorizontalFOVEnd", horizontal_FOV_end);

                relative_pose.position = AirSimSettings::createVectorSetting(settings_json, VectorMath::nanVector());
                auto rotation = AirSimSettings::createRotationSetting(settings_json, AirSimSettings::Rotation::nanRotation());

                if (std::isnan(relative_pose.position.x()))
                    relative_pose.position.x() = 0;
                if (std::isnan(relative_pose.position.y()))
                    relative_pose.position.y() = 0;
                if (std::isnan(relative_pose.position.z())) {
                    relative_pose.position.z() = 0;
                }

                pitch = !std::isnan(rotation.pitch) ? rotation.pitch : 0;
                roll = !std::isnan(rotation.roll) ? rotation.roll : 0;
                yaw = !std::isnan(rotation.yaw) ? rotation.yaw : 0;
                relative_pose.orientation = VectorMath::toQuaternion(
                    Utils::degreesToRadians(pitch), // pitch - rotation around Y axis
                    Utils::degreesToRadians(roll), // roll  - rotation around X axis
                    Utils::degreesToRadians(yaw)); // yaw   - rotation around Z axis
			}
		};

	}
} //namespace
#endif
