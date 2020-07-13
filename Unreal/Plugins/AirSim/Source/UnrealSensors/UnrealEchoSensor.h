// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
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

protected:
	virtual void getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose,
		msr::airlib::vector<msr::airlib::real_T>& point_cloud) override;

	virtual void updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose);

	virtual void pause(const bool is_paused);

	virtual void setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp) override;

private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;

	void generateSampleDirections();
	void sampleHorizontalSlice(int num_points, float opening_angle, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud);
	void sampleSphereCap(int num_points, float opening_angle, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud);
	void applyFreeSpaceLoss(float &signal_attenuation, float previous_distance, float added_distance);
	float remainingDistance(float signal_attenuation, float total_distance);
	void traceDirection(FVector trace_start_position, FVector trace_end_position, msr::airlib::vector<msr::airlib::real_T> &points);
    void bounceTrace(FVector &trace_start_position, FVector &trace_direction, float &trace_length,
		const FHitResult &trace_hit_result, float &total_distance, float &signal_attenuation);
    FVector Vector3rToFVector(const Vector3r &input_vector);
	Vector3r FVectorToVector3r(const FVector &input_vector);
	float angleBetweenVectors(FVector vector1, FVector vector2);
	float receptionAttenuation(float reception_angle);

private:
	AActor* actor_;
	const NedTransform* ned_transform_;
	float saved_clockspeed_;
	msr::airlib::vector<msr::airlib::Vector3r> sample_directions_;
	msr::airlib::vector<msr::airlib::Vector3r> spread_directions_;
	msr::airlib::Pose sensor_reference_frame_;
	TArray<AActor*> ignore_actors_;

	const msr::airlib::EchoSimpleParams sensor_params_;
	const float attenuation_per_distance_;
	const float attenuation_per_reflection_;
	const float attenuation_limit_;
	const float distance_limit_;
	const int reflection_limit_;
	const float reflection_distance_limit_;
	const float opening_angle_;
	const float draw_time_;
	const float line_thinkness_;
};