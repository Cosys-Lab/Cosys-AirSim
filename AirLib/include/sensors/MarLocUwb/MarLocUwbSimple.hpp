// Developed by Cosys-Lab, University of Antwerp

#ifndef msr_airlib_MarLocUwb_hpp
#define msr_airlib_MarLocUwb_hpp

#include <random>
#include "common/Common.hpp"
#include "MarLocUwbSimpleParams.hpp"
#include "MarLocUwbBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr { namespace airlib {

class MarLocUwbSimple : public MarLocUwbBase {
public:
	/*MarLocUwbSimple(const AirSimSettings::MarLocUwbSetting& setting = AirSimSettings::MarLocUwbSetting())
        : MarLocUwbBase(setting.sensor_name, setting.attach_link)
    {*/
	MarLocUwbSimple(const AirSimSettings::MarLocUwbSetting& setting = AirSimSettings::MarLocUwbSetting())
		: MarLocUwbBase(setting.sensor_name)
	{
        // initialize params
        params_.initializeFromSettings(setting);

        //initialize frequency limiter
		freq_limiter_.initialize(params_.update_frequency, params_.startup_delay, false);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
		MarLocUwbBase::reset();

		freq_limiter_.reset();
		last_time_ = clock()->nowNanos();

		updateOutput();        

		MarLocUwbSensorData emptyInput;
		setInput(emptyInput);
    }

	virtual void update(float delta = 0) override
	{
		MarLocUwbBase::update(delta);
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
		MarLocUwbBase::reportState(reporter);

		reporter.writeValue("MarLocUwb-MeasurementFreq", params_.measurement_frequency);
    }
    //*** End: UpdatableState implementation ***//

    virtual ~MarLocUwbSimple() = default;

    const MarLocUwbSimpleParams& getParams() const
    {
        return params_;
    }

protected:
    //virtual void getPointCloud(const Pose& sensor_pose, const Pose& vehicle_pose, vector<real_T>& point_cloud) = 0;

	virtual void updatePose(const Pose& sensor_pose, const Pose& vehicle_pose) = 0;

	virtual void getLocalPose(Pose& sensor_pose) = 0;

	virtual void pause(const bool is_paused) = 0;

	//virtual void setPointCloud(const Pose& sensor_pose, vector<real_T>& point_cloud, TTimePoint time_stamp) = 0;

	virtual void updateUWBRays() = 0;
	virtual TArray<msr::airlib::Pose> getBeaconActors() = 0;

private:
	void updateOutput()
	{
		//point_cloud_.clear();

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

		/*getPointCloud(params_.relative_pose, // relative sensor pose
			ground_truth.kinematics->pose,   // relative vehicle pose			
			point_cloud_);*/
		updatePose(params_.relative_pose, pose_offset);
		updateUWBRays();
		MarLocUwbSensorData output;

		//output.point_cloud = point_cloud_;
		output.time_stamp = last_time_;
		if (params_.external && params_.external_ned) {
			getLocalPose(output.pose);
		}
		else {
			output.pose = params_.relative_pose;
		}

		std::vector<float> beaconsActiveRSSI;
		std::vector<std::string> beaconsActiveID;
		std::vector<float> beaconsActivePosX;
		std::vector<float> beaconsActivePosY;
		std::vector<float> beaconsActivePosZ;
		std::vector<float> beaconsActiveDistance;
		for (int i = 0; i < beaconsActive_.Num(); i++) {
			for (int ii = 0; ii < beaconsActive_[i].Num(); ii++) {
				output.beaconsActiveID.push_back(beaconsActive_[i][ii].beaconID);
				output.beaconsActivePosX.push_back(beaconsActive_[i][ii].beaconPosX);
				output.beaconsActivePosY.push_back(beaconsActive_[i][ii].beaconPosY);
				output.beaconsActivePosZ.push_back(beaconsActive_[i][ii].beaconPosZ);
				output.beaconsActiveDistance.push_back(beaconsActive_[i][ii].distance);
				output.beaconsActiveRssi.push_back(beaconsActive_[i][ii].rssi);
			}
		}
		
		TArray<msr::airlib::Pose> beacon_poses = getBeaconActors();
		msr::airlib::Pose thisPose;

		for (int i = 0; i < beacon_poses.Num(); i++) {
			thisPose = beacon_poses[i];
			output.allBeaconsId.push_back(thisPose.orientation.w());
			output.allBeaconsX.push_back(thisPose.position.x()/100);
			output.allBeaconsY.push_back(thisPose.position.y()/100);
			output.allBeaconsZ.push_back(-thisPose.position.z()/100);
		}

		setOutput(output);
	}
	void updateInput() {
		MarLocUwbSensorData input = getInput();
		//setPointCloud(input.pose, input.point_cloud, input.time_stamp);
	}
private:
    //vector<real_T> point_cloud_;

    FrequencyLimiter freq_limiter_;
    TTimePoint last_time_;
	bool last_tick_measurement_ = false;
protected:
	MarLocUwbSimpleParams params_;
	TArray<TArray<UWBHit>> beaconsActive_;
};

}} //namespace
#endif
