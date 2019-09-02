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

private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;

	void generateSampleDirections();
	void generateSpreadDirections();
	void sampleSphereCap(int num_points, float opening_angle, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud);
	bool traceDirection(FVector trace_start_position, FVector trace_direction, int bounce_depth, float signal_attenuation,
		const msr::airlib::EchoSimpleParams &sensor_params, TArray<AActor*> ignore_actors, msr::airlib::vector<msr::airlib::real_T> &points);
    float bounceTrace(FVector &trace_start_position, FVector &trace_direction, const FHitResult &trace_hit_result,
        float &signal_attenuation, const msr::airlib::EchoSimpleParams &sensor_params);
    FVector Vector3rToFVector(const Vector3r &input_vector);
	Vector3r FVectorToVector3r(const FVector &input_vector);


private:
	AActor* actor_;
	const NedTransform* ned_transform_;
	float saved_clockspeed_;
	msr::airlib::vector<msr::airlib::Vector3r> sample_directions_;
	msr::airlib::vector<msr::airlib::Vector3r> spread_directions_;
	AActor* physical_sensor_obj_;
	msr::airlib::Pose sensor_reference_frame_;

	const float attenuation_per_distance_;
	const float attenuation_per_reflection_;
	const float attenuation_limit_;
	const FString sensor_name_;
};