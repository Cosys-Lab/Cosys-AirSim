// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealGPULidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "UObject/ConstructorHelpers.h"
#include <random>

// ctor
UnrealGPULidarSensor::UnrealGPULidarSensor(const AirSimSettings::GPULidarSetting& setting,
	AActor* actor, const NedTransform* ned_transform)
	: GPULidarSimple(setting), actor_(actor), ned_transform_(ned_transform)
{
	// Seed and initiate noise
	std::random_device rd;
	gen_ = std::mt19937(rd());
	dist_ = std::normal_distribution<float>(0, getParams().min_noise_standard_deviation);

	FActorSpawnParameters camera_spawn_params;

	camera_spawn_params.Name = FName(*(actor->GetName() + "_gpulidar_" + FString(setting.sensor_name.c_str())));
	lidar_camera_ = actor->GetWorld()->SpawnActor<ALidarCamera>(camera_spawn_params);
	lidar_camera_->AttachToActor(actor, FAttachmentTransformRules::KeepRelativeTransform);
	lidar_camera_->InitializeSettings(setting);
}

// Pause Unreal simulation
void UnrealGPULidarSensor::pause(const bool is_paused) {
	if (is_paused) {
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else {
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
}

// returns a point-cloud for the tick
void UnrealGPULidarSensor::getPointCloud(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{	
	lidar_camera_->Update(delta_time, point_cloud);
}
