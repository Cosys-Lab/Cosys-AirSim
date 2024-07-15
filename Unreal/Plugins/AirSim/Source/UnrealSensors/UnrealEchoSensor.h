// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "UnrealEchoCommon.h"
#include "Beacons/PassiveEchoBeacon.h"
#include "sensors/echo/EchoSimple.hpp"
#include "Components/StaticMeshComponent.h"
#include "NedTransform.h"
#include "AirBlueprintLib.h"

// UnrealEchoSensor implementation that uses Ray Tracing in Unreal.
class UnrealEchoSensor : public msr::airlib::EchoSimple {

public:
	typedef msr::airlib::AirSimSettings AirSimSettings;

public:
	UnrealEchoSensor(const AirSimSettings::EchoSetting& setting,
		AActor* actor, const NedTransform* ned_transform);

	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;
	

protected:
	virtual void getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose,
		msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth,
		msr::airlib::vector<msr::airlib::real_T>& passive_beacons_point_cloud, msr::airlib::vector<std::string>& passive_beacons_groundtruth) override;

	virtual void updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose);

	virtual void pause(const bool is_paused);

	virtual void getLocalPose(msr::airlib::Pose& sensor_pose);

	virtual void setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp) override;

private:
	void generateSampleDirectionPoints();

private:
	AActor* actor_;
	const NedTransform* ned_transform_;
	float saved_clockspeed_;
	msr::airlib::vector<msr::airlib::Vector3r> sample_direction_points_;
	msr::airlib::Pose sensor_reference_frame_;
	TArray<AActor*> ignore_actors_;

	const msr::airlib::EchoSimpleParams sensor_params_;
	const float attenuation_per_distance_;
	const float attenuation_per_reflection_;
	const float attenuation_limit_;
	const float distance_limit_;
	const int reflection_limit_;
	const float reflection_distance_limit_;
	const float reflection_opening_angle_;
	const float sensor_passive_radius_;
	const float draw_time_;
	const float line_thickness_;
	const bool external_;
	TArray<UnrealEchoCommon::EchoPoint> passive_points_;

	msr::airlib::vector<FVector> point_cloud_draw_reflected_points_;
};