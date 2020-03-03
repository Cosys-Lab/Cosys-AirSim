// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_GPULidar_hpp
#define msr_airlib_GPULidar_hpp

#include <random>
#include "common/Common.hpp"
#include "GPULidarSimpleParams.hpp"
#include "GPULidarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr {
	namespace airlib {

		class GPULidarSimple : public GPULidarBase {
		public:
			GPULidarSimple(const AirSimSettings::GPULidarSetting& setting = AirSimSettings::GPULidarSetting())
				: GPULidarBase(setting.sensor_name, setting.attach_link)
			{
				// initialize params
				params_.initializeFromSettings(setting);

				//initialize frequency limiter
				freq_limiter_.initialize(params_.horizontal_rotation_frequency, params_.startup_delay, false);

			}

			//*** Start: UpdatableState implementation ***//
			virtual void reset() override
			{
				GPULidarBase::reset();
				freq_limiter_.reset();

				updateOutput(0.0f);
			}

			virtual void update(float delta = 0) override
			{
				GPULidarBase::update(delta);
				refreshPointCloud(delta);
				freq_limiter_.update(delta);
				if (freq_limiter_.isWaitComplete())
				{
					updateOutput(delta);
				}

			}

			virtual void reportState(StateReporter& reporter) override
			{
				//call base
				GPULidarBase::reportState(reporter);

				reporter.writeValue("Lidar-NumChannels", params_.number_of_channels);
				reporter.writeValue("Lidar-Range", params_.range);
				reporter.writeValue("Lidar-MeasurementsPerCycle", params_.measurement_per_cycle_);
				reporter.writeValue("Lidar-HorizontalRotationFrequency", params_.horizontal_rotation_frequency);
				reporter.writeValue("Lidar-FOV-Upper", params_.vertical_FOV_upper);
				reporter.writeValue("Lidar-FOV-Lower", params_.vertical_FOV_lower);
			}
			//*** End: UpdatableState implementation ***//

			virtual ~GPULidarSimple() = default;

			const GPULidarSimpleParams& getParams() const
			{
				return params_;
			}

		protected:
			virtual void getPointCloud(float delta_time, vector<real_T>& point_cloud) = 0;

			virtual void pause(const bool is_paused) = 0;

		private: //methods
			void refreshPointCloud(float delta_time)
			{

				double start = FPlatformTime::Seconds();
				point_cloud_.clear();
				getPointCloud(delta_time, point_cloud_);
				point_cloud_full_.insert(point_cloud_full_.end(), point_cloud_.begin(), point_cloud_.end());
				double end = FPlatformTime::Seconds();
				UAirBlueprintLib::LogMessageString("GPULidar: ", "Sensor data generation took " + std::to_string(end - start) + " and generated " + std::to_string(point_cloud_.size() / 3) + " points", LogDebugLevel::Informational);
			}

			void updateOutput(float delta_time)
			{
				const GroundTruth& ground_truth = getGroundTruth();
				Pose lidar_pose = params_.relative_pose + ground_truth.kinematics->pose;
				GPULidarData output;
				output.point_cloud = point_cloud_full_;
				output.time_stamp = clock()->nowNanos();
				output.pose = lidar_pose;
				setOutput(output);
				point_cloud_full_.clear();
			}

		private:
			GPULidarSimpleParams params_;
			vector<real_T> point_cloud_;
			vector<real_T> point_cloud_full_;
			FrequencyLimiter freq_limiter_;
		};

	}
} //namespace
#endif
