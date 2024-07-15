// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/template/SensorTemplateSimple.hpp"
#include "Components/StaticMeshComponent.h"
#include "NedTransform.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "AirBlueprintLib.h"
#include "Weather/WeatherLib.h"
#include "map"

// UnrealSensorTemplate implementation that uses Ray Tracing in Unreal.
class UnrealSensorTemplate : public msr::airlib::SensorTemplateSimple {

public:
	typedef msr::airlib::AirSimSettings AirSimSettings;

public:
	UnrealSensorTemplate(const AirSimSettings::SensorTemplateSetting& setting,
		AActor* actor, const NedTransform* ned_transform);

protected:
	virtual void getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose,
		msr::airlib::vector<msr::airlib::real_T>& point_cloud) override;

	virtual void updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose);

	virtual void pause(const bool is_paused);

	virtual void getLocalPose(msr::airlib::Pose& sensor_pose);

	virtual void setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp) override;

private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;

	FVector Vector3rToFVector(const Vector3r& input_vector);
private:
	AActor* actor_;
	const NedTransform* ned_transform_;
	float saved_clockspeed_;
	msr::airlib::Pose sensor_reference_frame_;
	const msr::airlib::SensorTemplateSimpleParams sensor_params_;
	const bool external_;
};