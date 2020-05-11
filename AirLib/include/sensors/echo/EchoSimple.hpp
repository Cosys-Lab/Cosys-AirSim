// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Echo_hpp
#define msr_airlib_Echo_hpp

#include <random>
#include "common/Common.hpp"
#include "EchoSimpleParams.hpp"
#include "EchoBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr { namespace airlib {

class EchoSimple : public EchoBase {
public:
    EchoSimple(const AirSimSettings::EchoSetting& setting = AirSimSettings::EchoSetting())
        : EchoBase(setting.sensor_name, setting.attach_link)
    {
        // initialize params
        params_.initializeFromSettings(setting);

        //initialize frequency limiter
		freq_limiter_.initialize(params_.update_frequency, params_.startup_delay, params_.engine_time);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        EchoBase::reset();

		freq_limiter_.reset();
		last_time_ = clock()->nowNanos();

		updateOutput();        
    }

	virtual void update(float delta = 0) override
	{
		EchoBase::update(delta);
		freq_limiter_.update(delta);

		if (last_tick_measurement_ && params_.pause_after_measurement == false && params_.engine_time) {
			pause(false);
			last_tick_measurement_ = false;
		}
		if (freq_limiter_.isWaitComplete())
		{
			last_time_ = freq_limiter_.getLastTime();
			if(params_.engine_time || params_.pause_after_measurement)pause(true);
			updateOutput();
			if (params_.engine_time)last_tick_measurement_ = true;
		}		

    }

    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        EchoBase::reportState(reporter);

        reporter.writeValue("Echo-NumTraces", params_.number_of_traces);
		reporter.writeValue("Echo-AttenuationDist", params_.attenuation_per_distance);
		reporter.writeValue("Echo-AttenuationRefl", params_.attenuation_per_reflection);
		reporter.writeValue("Echo-AttenuationLimit", params_.attenuation_limit);
		reporter.writeValue("Echo-MeasurementFreq", params_.measurement_frequency);
    }
    //*** End: UpdatableState implementation ***//

    virtual ~EchoSimple() = default;

    const EchoSimpleParams& getParams() const
    {
        return params_;
    }

protected:
    virtual void getPointCloud(const Pose& echo_pose, const Pose& vehicle_pose, vector<real_T>& point_cloud) = 0;

	virtual void updatePose(const Pose& echo_pose, const Pose& vehicle_pose) = 0;

	virtual void pause(const bool is_paused) = 0;

private:
	void updateOutput()
	{
		point_cloud_.clear();

		const GroundTruth& ground_truth = getGroundTruth();

		// calculate the pose before obtaining the point-cloud. Before/after is a bit arbitrary
		// decision here. If the pose can change while obtaining the point-cloud (could happen for drones)
		// then the pose won't be very accurate either way.
		//
		// TODO: Seems like pose is in vehicle inertial-frame (NOT in Global NED frame).
		//    That could be a bit unintuitive but seems consistent with the position/orientation returned as part of 
		//    ImageResponse for cameras and pose returned by getCameraInfo API.
		//    Do we need to convert pose to Global NED frame before returning to clients?

		Pose echo_pose = params_.relative_pose + ground_truth.kinematics->pose;
		double start = FPlatformTime::Seconds();
		getPointCloud(params_.relative_pose, // relative echo pose
			ground_truth.kinematics->pose,   // relative vehicle pose			
			point_cloud_);
		double end = FPlatformTime::Seconds();
		UAirBlueprintLib::LogMessageString("Echo: ", "Sensor data generation took " + std::to_string(end - start), LogDebugLevel::Informational);

		EchoData output;
		output.point_cloud = point_cloud_;
		output.time_stamp = last_time_;
		output.pose = echo_pose;
		setOutput(output);
	}
private:
    EchoSimpleParams params_;
    vector<real_T> point_cloud_;

    FrequencyLimiter freq_limiter_;
    TTimePoint last_time_;
	bool last_tick_measurement_ = false;
};

}} //namespace
#endif
