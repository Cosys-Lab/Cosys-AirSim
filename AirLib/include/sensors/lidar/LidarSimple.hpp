// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Lidar_hpp
#define msr_airlib_Lidar_hpp

#include <random>
#include "common/Common.hpp"
#include "LidarSimpleParams.hpp"
#include "LidarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr
{
namespace airlib
{

    class LidarSimple : public LidarBase
    {
    public:
        LidarSimple(const AirSimSettings::LidarSetting& setting = AirSimSettings::LidarSetting())
            : LidarBase(setting.sensor_name)
        {
            // initialize params
            params_.initializeFromSettings(setting);

            //initialize frequency limiter
            freq_limiter_.initialize(params_.update_frequency, params_.startup_delay, false);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            LidarBase::reset();
            freq_limiter_.reset();
            last_time_ = clock()->nowNanos();

            updateOutput();
        }

        virtual void update(float delta = 0) override
        {
            LidarBase::update(delta);

            freq_limiter_.update(delta);

            if (freq_limiter_.isWaitComplete()) {
                updateOutput();
            }
        }

        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            LidarBase::reportState(reporter);

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

        virtual ~LidarSimple() = default;

        const LidarSimpleParams& getParams() const
        {
            return params_;
        }

    protected:
        virtual bool getPointCloud(const Pose& lidar_pose, const Pose& vehicle_pose,
            TTimeDelta delta_time, vector<real_T>& point_cloud_temp, vector<std::string>& groundtruth_temp, vector<real_T>& point_cloud, vector<std::string>& groundtruth) = 0;

        virtual void pause(const bool is_paused) = 0;

        virtual void updatePose(const Pose& lidar_pose, const Pose& vehicle_pose) = 0;

        virtual void getLocalPose(Pose& sensor_pose) = 0;

    private: //methods
        void updateOutput()
        {
            TTimeDelta delta_time = clock()->updateSince(last_time_);

            point_cloud_.clear();

            const GroundTruth& ground_truth = getGroundTruth();
            Pose const pose_offset = params_.external ? Pose() : ground_truth.kinematics->pose;

            bool refresh = getPointCloud(params_.relative_pose, // relative lidar pose
                pose_offset,   // relative vehicle pose
                delta_time,
                point_cloud_temp_, groundtruth_temp_, point_cloud_, groundtruth_);
            if (refresh) {
                LidarData output;
                output.point_cloud = point_cloud_;
                output.groundtruth = groundtruth_;

                output.time_stamp = clock()->nowNanos();
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
        LidarSimpleParams params_;
        vector<real_T> point_cloud_;
        vector<std::string> groundtruth_;

        vector<real_T> point_cloud_temp_;
        vector<std::string> groundtruth_temp_;

        FrequencyLimiter freq_limiter_;
        FrequencyLimiter freq_limiter_rotation_;
        TTimePoint last_time_;
        bool last_tick_measurement_ = false;
    };
}
} //namespace
#endif
