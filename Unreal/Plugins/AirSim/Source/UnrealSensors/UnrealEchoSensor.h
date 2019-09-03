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
	virtual void getPointCloud(const msr::airlib::Pose& echo_pose, const msr::airlib::Pose& vehicle_pose, 
		msr::airlib::vector<msr::airlib::real_T>& point_cloud) override;

	virtual void updatePose(const msr::airlib::Pose& echo_pose, const msr::airlib::Pose& vehicle_pose);

	virtual void pause(const bool is_paused);

private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;

	void generateSampleDirections();
	void generateSpreadDirections();
	bool traceDirection(const msr::airlib::Pose& echo_pose, const msr::airlib::Pose& vehicle_pose,
		Vector3r direction, const msr::airlib::EchoSimpleParams params, Vector3r &point, float &signal_attenuation_final);
    void bounceTrace(FVector &trace_start_position, FVector &trace_end_position, const FHitResult &trace_hit_result,
        float &signal_attenuation, float max_attenuation, const msr::airlib::EchoSimpleParams &params);
    void bounceTrace2(FVector &trace_start_position, FVector &trace_end_position, const FHitResult &trace_hit_result,
        float &signal_attenuation, float max_attenuation, const msr::airlib::EchoSimpleParams &params);
    FVector Vector3rToFVector(const Vector3r &input_vector);


private:
	AActor* actor_;
	const NedTransform* ned_transform_;
	float saved_clockspeed_ = 1;
	msr::airlib::vector<msr::airlib::Vector3r> sample_directions_;
	AActor* physical_sensor_actor_object_;

};