// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealSensorTemplate.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "CoreMinimal.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Engine/Engine.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

// ctor
UnrealSensorTemplate::UnrealSensorTemplate(const AirSimSettings::SensorTemplateSetting& setting, AActor* actor, const NedTransform* ned_transform)
	: SensorTemplateSimple(setting), actor_(actor), ned_transform_(ned_transform), saved_clockspeed_(1), sensor_params_(getParams()), external_(getParams().external)
{
}


// Set UnrealSensorTemplate object in correct pose in physical world
void UnrealSensorTemplate::updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose)
{
	sensor_reference_frame_ = VectorMath::add(sensor_pose, vehicle_pose);
	if (sensor_params_.draw_sensor) {
		FVector sensor_position;
		if (external_) {
			sensor_position = ned_transform_->toFVector(sensor_reference_frame_.position, 100, true);
		}
		else {
			sensor_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
		}
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), sensor_position, 5, FColor::Black, false, 0.3);
		FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, 0.3, 10);
	}
}

// Get UnrealSensorTemplate pose in Local NED
void UnrealSensorTemplate::getLocalPose(msr::airlib::Pose& sensor_pose)
{
	FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1)); ;
	sensor_pose = ned_transform_->toLocalNed(FTransform(sensor_direction.Rotation(), ned_transform_->toFVector(sensor_reference_frame_.position, 100, true), FVector(1, 1, 1)));
}

// Pause Unreal simulation
void UnrealSensorTemplate::pause(const bool is_paused) {
	if (is_paused) {
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else {
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
} 
void UnrealSensorTemplate::getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
	// Set the physical SensorTemplate mesh in the correct location in the world
	updatePose(sensor_pose, vehicle_pose);

	point_cloud.clear();
	Vector3r random_vector = ned_transform_->toVector3r(FMath::RandPointInBox(FBox(FVector(-1, -1, -1), FVector(1, 1, 1))), 1.0f, true);
	point_cloud.emplace_back(random_vector.x());
	point_cloud.emplace_back(random_vector.y());
	point_cloud.emplace_back(random_vector.z());
}

void UnrealSensorTemplate::setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp) {
	// TODO consume point cloud (+ draw_time_)?

	const int DATA_PER_POINT = 5;
	for (int point_count = 0; point_count < point_cloud.size(); point_count += DATA_PER_POINT) {
		Vector3r point_local = Vector3r(point_cloud[point_count], point_cloud[point_count + 1], point_cloud[point_count + 2]);
		Vector3r point_global1 = VectorMath::transformToWorldFrame(point_local, sensor_pose, true);
		FVector point_global = ned_transform_->fromLocalNed(point_global1);
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), point_global, 10, FColor::Orange, false, 1.05f / sensor_params_.measurement_frequency);	
	}
}

FVector UnrealSensorTemplate::Vector3rToFVector(const Vector3r& input_vector) {
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}