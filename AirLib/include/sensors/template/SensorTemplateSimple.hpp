// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SensorTemplate_hpp
#define msr_airlib_SensorTemplate_hpp

#include <random>
#include "common/Common.hpp"
#include "SensorTemplateSimpleParams.hpp"
#include "SensorTemplateBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr { namespace airlib {

class SensorTemplateSimple : public SensorTemplateBase {
public:
	SensorTemplateSimple(const AirSimSettings::SensorTemplateSetting& setting = AirSimSettings::SensorTemplateSetting())
        : SensorTemplateBase(setting.sensor_name)
    {
        // initialize params
        params_.initializeFromSettings(setting);

        //initialize frequency limiter
		freq_limiter_.initialize(params_.update_frequency, params_.startup_delay, false);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
		SensorTemplateBase::reset();

		freq_limiter_.reset();
		last_time_ = clock()->nowNanos();

		updateOutput();        

		SensorTemplateData emptyInput;
		setInput(emptyInput);
    }

	virtual void update(float delta = 0) override
	{
		SensorTemplateBase::update(delta);
		freq_limiter_.update(delta);

		if (freq_limiter_.isWaitComplete())
		{
			last_time_ = freq_limiter_.getLastTime();
			if(params_.pause_after_measurement)pause(true);
			updateOutput();
			updateInput();
		}		

    }

    virtual void reportState(StateReporter& reporter) override
    {
        //call base
		SensorTemplateBase::reportState(reporter);

		reporter.writeValue("SensorTemplate-MeasurementFreq", params_.measurement_frequency);
    }
    //*** End: UpdatableState implementation ***//

    virtual ~SensorTemplateSimple() = default;

    const SensorTemplateSimpleParams& getParams() const
    {
        return params_;
    }

protected:
    virtual void getPointCloud(const Pose& sensor_pose, const Pose& vehicle_pose, vector<real_T>& point_cloud) = 0;

	virtual void updatePose(const Pose& sensor_pose, const Pose& vehicle_pose) = 0;

	virtual void getLocalPose(Pose& sensor_pose) = 0;

	virtual void pause(const bool is_paused) = 0;

	virtual void setPointCloud(const Pose& sensor_pose, vector<real_T>& point_cloud, TTimePoint time_stamp) = 0;

private:
	void updateOutput()
	{
		point_cloud_.clear();

		const GroundTruth& ground_truth = getGroundTruth();
		Pose const pose_offset = params_.external ? Pose() : ground_truth.kinematics->pose;

		// calculate the pose before obtaining the point-cloud. Before/after is a bit arbitrary
		// decision here. If the pose can change while obtaining the point-cloud (could happen for drones)
		// then the pose won't be very accurate either way.
		//
		// TODO: Seems like pose is in vehicle inertial-frame (NOT in Global NED frame).
		//    That could be a bit unintuitive but seems consistent with the position/orientation returned as part of 
		//    ImageResponse for cameras and pose returned by getCameraInfo API.
		//    Do we need to convert pose to Global NED frame before returning to clients?

		getPointCloud(params_.relative_pose, // relative sensor pose
			pose_offset,   // relative vehicle pose			
			point_cloud_);
		SensorTemplateData output;
		output.point_cloud = point_cloud_;
		output.time_stamp = last_time_;
		if (params_.external && params_.external_ned) {
			getLocalPose(output.pose);
		}
		else {
			output.pose = params_.relative_pose;
		}
		setOutput(output);
	}
	void updateInput() {
		SensorTemplateData input = getInput();
		setPointCloud(input.pose, input.point_cloud, input.time_stamp);
	}
private:
	SensorTemplateSimpleParams params_;
    vector<real_T> point_cloud_;

    FrequencyLimiter freq_limiter_;
    TTimePoint last_time_;
	bool last_tick_measurement_ = false;
};

}} //namespace
#endif
