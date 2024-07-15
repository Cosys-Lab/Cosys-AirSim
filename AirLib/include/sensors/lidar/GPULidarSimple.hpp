// Developed by Cosys-Lab, University of Antwerp

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
				: GPULidarBase(setting.sensor_name)
			{
				// initialize params
				params_.initializeFromSettings(setting);

				last_time_ = clock()->nowNanos();

				//initialize frequency limiter
				freq_limiter_.initialize(params_.update_frequency, params_.startup_delay, false);
			}

			//*** Start: UpdatableState implementation ***//
			virtual void resetImplementation() override
			{
				GPULidarBase::reset();
				freq_limiter_.reset();
				last_time_ = clock()->nowNanos();
				updateOutput();
			}

			virtual void update(float delta = 0) override
			{
				GPULidarBase::update(delta);
				freq_limiter_.update(delta);
				updateOutput();
			}

			virtual void reportState(StateReporter& reporter) override
			{
				//call base
				GPULidarBase::reportState(reporter);

				reporter.writeValue("Lidar-NumChannels", params_.number_of_channels);
				reporter.writeValue("Lidar-Range", params_.range);
				reporter.writeValue("Lidar-MeasurementsPerCycle", params_.measurement_per_cycle);
				reporter.writeValue("Lidar-HorizontalRotationFrequency", params_.horizontal_rotation_frequency);
				reporter.writeValue("Lidar-VFOV-Upper", params_.vertical_FOV_upper);
				reporter.writeValue("Lidar-VFOV-Lower", params_.vertical_FOV_lower);
				reporter.writeValue("Lidar-HFOV-Upper", params_.horizontal_FOV_start);
				reporter.writeValue("Lidar-HFOV-Lower", params_.horizontal_FOV_end);
			}
			//*** End: UpdatableState implementation ***//

			virtual ~GPULidarSimple() = default;

			const GPULidarSimpleParams& getParams() const
			{
				return params_;
			}

		protected:
			virtual bool getPointCloud(float delta_time, vector<real_T>& point_cloud, vector<real_T>& point_cloud_final) = 0;

			virtual void pause(const bool is_paused) = 0;

			virtual void getLocalPose(Pose& sensor_pose) = 0;

		private: //methods
			void updateOutput()
			{

				TTimeDelta delta_time = clock()->updateSince(last_time_);

				bool refresh = getPointCloud(delta_time, point_cloud_temp_, point_cloud_);

				if (refresh) {
					GPULidarData output;
					output.point_cloud = point_cloud_;
					output.time_stamp = clock()->nowNanos();
					const GroundTruth& ground_truth = getGroundTruth();
					Pose lidar_pose = params_.relative_pose;
					if (params_.external && params_.external_ned) {
						getLocalPose(output.pose);
					}
					else {
						output.pose = params_.relative_pose;
					}
					setOutput(output);
					last_time_ = output.time_stamp;
				} else {
					last_time_ = clock()->nowNanos();
				}

			}

		private:
			GPULidarSimpleParams params_;
			vector<real_T> point_cloud_;
			vector<real_T> point_cloud_temp_;
			FrequencyLimiter freq_limiter_;
			TTimePoint last_time_;

		};

	}
} //namespace
#endif
