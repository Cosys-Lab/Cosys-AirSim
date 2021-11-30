// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/lidar/GPULidarSimple.hpp"
#include "NedTransform.h"
#include "LidarCamera.h"
#include <random>

// UnrealLidarSensor implementation that uses a rotating depth camera to accelerate the Lidar Simulation
class UnrealGPULidarSensor : public msr::airlib::GPULidarSimple {
public:
	typedef msr::airlib::AirSimSettings AirSimSettings;

public:
	UnrealGPULidarSensor(const AirSimSettings::GPULidarSetting& setting,
		AActor* actor, const NedTransform* ned_transform);

protected:
	virtual bool getPointCloud(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final) override;
	virtual void pause(const bool is_paused);

private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;


private:
	AActor* actor_;
	ALidarCamera* lidar_camera_;

	const NedTransform* ned_transform_;
	float saved_clockspeed_ = 1;
};