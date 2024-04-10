// Developed by Cosys-Lab, University of Antwerp

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
        : EchoBase(setting.sensor_name)
    {
        // initialize params
        params_.initializeFromSettings(setting);

        //initialize frequency limiter
		freq_limiter_.initialize(params_.update_frequency, params_.startup_delay, false);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
        EchoBase::reset();

		freq_limiter_.reset();
		last_time_ = clock()->nowNanos();

		updateOutput();        

		EchoData emptyInput;
		setInput(emptyInput);
    }

	virtual void update(float delta = 0) override
	{
		EchoBase::update(delta);
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
    virtual void getPointCloud(const Pose& echo_pose, const Pose& vehicle_pose, vector<real_T>& point_cloud, vector<std::string>& groundtruth,
		                       vector<real_T>& passive_beacons_point_cloud, vector<std::string>& passive_beacons_groundtruth) = 0;

	virtual void updatePose(const Pose& echo_pose, const Pose& vehicle_pose) = 0;

	virtual void getLocalPose(Pose& sensor_pose) = 0;

	virtual void pause(const bool is_paused) = 0;

	virtual void setPointCloud(const Pose& echo_pose, vector<real_T>& point_cloud, TTimePoint time_stamp) = 0;

private:
	void updateOutput()
	{
		point_cloud_.clear();
		groundtruth_.clear();
		passive_beacons_point_cloud_.clear();
		passive_beacons_groundtruth_.clear();

		const GroundTruth& ground_truth = getGroundTruth();
		Pose const pose_offset = params_.external ? Pose() : ground_truth.kinematics->pose;

		getPointCloud(params_.relative_pose, // relative echo pose
			pose_offset,   // relative vehicle pose			
			point_cloud_, groundtruth_, passive_beacons_point_cloud_, passive_beacons_groundtruth_);
		EchoData output;
		output.point_cloud = point_cloud_;
		output.time_stamp = last_time_;
		output.groundtruth = groundtruth_;
		output.passive_beacons_point_cloud = passive_beacons_point_cloud_;
		output.passive_beacons_groundtruth = passive_beacons_groundtruth_;
		if (params_.external && params_.external_ned) {
			getLocalPose(output.pose);
		}
		else {
			output.pose = params_.relative_pose;
		}
		setOutput(output);
	}
	void updateInput() {
		EchoData input = getInput();
		setPointCloud(input.pose, input.point_cloud, input.time_stamp);
	}
private:
    EchoSimpleParams params_;
    vector<real_T> point_cloud_;
	vector<std::string> groundtruth_;
	vector<real_T> passive_beacons_point_cloud_;
	vector<std::string> passive_beacons_groundtruth_;
    FrequencyLimiter freq_limiter_;
    TTimePoint last_time_;
	bool last_tick_measurement_ = false;
};

}} //namespace
#endif
