// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealEchoSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "PhysicalSensorActor.h"
#include "CoreMinimal.h"

// ctor
UnrealEchoSensor::UnrealEchoSensor(const AirSimSettings::EchoSetting& setting, AActor* actor, const NedTransform* ned_transform)
	: EchoSimple(setting), actor_(actor), ned_transform_(ned_transform), saved_clockspeed_(1),
	attenuation_per_distance_(getParams().attenuation_per_distance), 
	attenuation_per_reflection_(getParams().attenuation_per_reflection),
	attenuation_limit_(getParams().attenuation_limit), 
	sensor_name_(getParams().name.c_str())
{
	FActorSpawnParameters spawnParams;
	FRotator rotator = FRotator(0, 0, 0);
	FVector position = FVector(0, 0, 0);
	APhysicalSensorActor* physical_sensor_actor = NewObject<APhysicalSensorActor>(APhysicalSensorActor::StaticClass());
	physical_sensor_obj_ = actor->GetWorld()->SpawnActor<AActor>(physical_sensor_actor->sensor_blueprint_, position, rotator, spawnParams);
	physical_sensor_obj_->SetActorLabel(setting.sensor_name.c_str());

	UStaticMeshComponent* sensorPlaneMesh = Cast<UStaticMeshComponent>(physical_sensor_obj_->GetComponentByClass(UStaticMeshComponent::StaticClass()));
	sensorPlaneMesh->SetRelativeScale3D(FVector(1, setting.sensor_diameter, setting.sensor_diameter));
	if (!setting.draw_sensor) {
		sensorPlaneMesh->ToggleVisibility();
	}

	generateSampleDirections();
	generateSpreadDirections();
}

// initializes information based on echo configuration
void UnrealEchoSensor::generateSampleDirections()
{
	msr::airlib::EchoSimpleParams sensor_params = getParams();

	int num_traces = sensor_params.number_of_traces;
	float opening_angle = 180;  // deg, full hemisphere
	sampleSphereCap(num_traces, opening_angle, sample_directions_);
}

// initializes information based on echo configuration
void UnrealEchoSensor::generateSpreadDirections()
{
	msr::airlib::EchoSimpleParams sensor_params = getParams();

	int num_spread_traces = sensor_params.number_of_spread_traces;
	float opening_angle = sensor_params.spread_opening_angle;  // deg
	sampleSphereCap(num_spread_traces, opening_angle, spread_directions_);
}

void UnrealEchoSensor::sampleSphereCap(int num_points, float opening_angle, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud) {
	point_cloud.clear();

	// Add point in frontal direction
	point_cloud.emplace(point_cloud.begin(), Vector3r(1, 0, 0));

	//Convert opening angle to plane coordinates
	float x_limit = FMath::Cos(FMath::DegreesToRadians(opening_angle) / 2);

	// Calculate ratio of sphere surface to cap surface.
	// Scale points accordingly, e.g. if nPoints = 10 and ratio = 0.01,
	// generate 1000 points on the sphere
	float h = 1 - x_limit;
	float surface_ratio = h / 2;  // (4 * pi * R^2) / (2 * pi * R * h)
	int num_sphere_points = FMath::CeilToInt(num_points * 1 / surface_ratio);

	// Generate points on the sphere, retain those within the opening angle
	float offset = 2.0f / num_sphere_points;
	float increment = PI * (3.0f - FMath::Sqrt(5.0f));
	for (auto i = 1; i <= num_sphere_points; ++i)
	{
		float y = ((i * offset) - 1) + (offset / 2.0f);
		float r = FMath::Sqrt(1 - FMath::Pow(y, 2));
		float phi = ((i + 1) % num_sphere_points) * increment;
		float x = FMath::Cos(phi) * r;
		if (point_cloud.size() == num_points)
		{
			return;
		}
		else if (x >= x_limit)
		{
			float z = FMath::Sin(phi) * r;
			point_cloud.emplace_back(Vector3r(x, y, z));
		}
	}
}

// Set echo object in correct pose in physical world
void UnrealEchoSensor::updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose)
{
	physical_sensor_obj_->SetActorLocation(ned_transform_->fromLocalNed(VectorMath::add(sensor_pose, vehicle_pose).position));
	physical_sensor_obj_->SetActorRotation(ned_transform_->fromNed(vehicle_pose.orientation).Rotator());

	sensor_reference_frame_ = VectorMath::add(sensor_pose, vehicle_pose);
}

// Pause Unreal simulation
void UnrealEchoSensor::pause(const bool is_paused) {
	if (is_paused) {
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else {
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
}

// returns a point-cloud for the tick
void UnrealEchoSensor::getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
	msr::airlib::EchoSimpleParams sensor_params = getParams();

	// set the physical echo mesh in the correct location in the world
	updatePose(sensor_pose, vehicle_pose);
	FVector trace_start_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);

	// shoot traces (rays)
	point_cloud.clear();
	for (auto direction_count = 0u; direction_count < sample_directions_.size(); ++direction_count)
	{
		Vector3r sample_direction = sample_directions_[direction_count];
		FVector trace_direction = Vector3rToFVector(VectorMath::rotateVector(sample_direction, VectorMath::rotateQuaternion(vehicle_pose.orientation, sensor_pose.orientation, 1), 1));

		int bounce_depth = 0;
		float signal_attenuation = 0.0f;

		// shoot trace and get the impact point and remaining attenuation, if any returns
		TArray<AActor*> ignore_actors = { physical_sensor_obj_ };
		traceDirection(trace_start_position, trace_direction, bounce_depth, signal_attenuation, sensor_params, ignore_actors, point_cloud);		
	}

	return;
}

bool UnrealEchoSensor::traceDirection(FVector trace_start_position, FVector trace_direction, int bounce_depth, float signal_attenuation,
	const msr::airlib::EchoSimpleParams &sensor_params, TArray<AActor*> ignore_actors, msr::airlib::vector<msr::airlib::real_T>& point_cloud) {
	// TODO find solution for passing ignore_actors and sensor_params objects
	// TODO implement minimum distance before applying beam spread
	// Trace initial emission
	float trace_length = ned_transform_->fromNed((attenuation_limit_ - signal_attenuation) / attenuation_per_distance_);
	FVector trace_end_position = trace_start_position + trace_direction * trace_length;

	// DRAW DEBUG
	if (sensor_params.draw_bounce_lines) {
		uint8 color_scale = static_cast<uint8>(255 * signal_attenuation / attenuation_limit_);
		FColor debug_line_color = FColor(color_scale, color_scale, color_scale);
		DrawDebugLine(actor_->GetWorld(), trace_start_position, trace_end_position, debug_line_color, false, 0.2f);
	}

	FHitResult trace_hit_result = FHitResult(ForceInit);
	bool trace_hit = UAirBlueprintLib::GetObstacle(actor_, trace_start_position, trace_end_position, trace_hit_result, ignore_actors, ECC_Visibility, true);
	if (!trace_hit) {
		return false;
	}

	// DRAW DEBUG
	if (sensor_params.draw_initial_points && bounce_depth == 0) DrawDebugPoint(actor_->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Green, false, 1 / sensor_params.measurement_frequency);

	// Check for end conditions -> signal hits sensor || signal dissipates
	if (trace_hit_result.GetActor()->GetActorLabel().Equals(sensor_name_)) 
	{
		// DRAW DEBUG
		if (sensor_params.draw_reflected_lines) DrawDebugLine(actor_->GetWorld(), trace_hit_result.TraceStart, trace_hit_result.ImpactPoint, FColor::Red, false, 1 / sensor_params.measurement_frequency);
		if (sensor_params.draw_reflected_points) DrawDebugPoint(actor_->GetWorld(), trace_hit_result.TraceStart, 5, FColor::Red, false, 1 / sensor_params.measurement_frequency);

		Vector3r point_sensor_frame = ned_transform_->toLocalNed(trace_hit_result.ImpactPoint);
		point_sensor_frame = VectorMath::transformToBodyFrame(point_sensor_frame, sensor_reference_frame_, true);

		point_cloud.emplace_back(point_sensor_frame.x());
		point_cloud.emplace_back(point_sensor_frame.y());
		point_cloud.emplace_back(point_sensor_frame.z());
		point_cloud.emplace_back(signal_attenuation);

		return true;
	}

	float distance_traveled = bounceTrace(trace_start_position, trace_direction, trace_hit_result, signal_attenuation, sensor_params);
	bounce_depth++;

	if (signal_attenuation >= attenuation_limit_) {
		return false;
	}

	// Recursively cast (scattered) reflections
	VectorMath::Quaternionf trace_rotation_quat = VectorMath::toQuaternion(VectorMath::front(), FVectorToVector3r(trace_direction));
	trace_rotation_quat.norm();

	bool beam_hit = false;
	float min_spread_distance = 1; // TODO make class property or user setting
	int num_spread_traces = (distance_traveled >= min_spread_distance) ? spread_directions_.size() : 1;
	for (int spread_count = 0; spread_count < num_spread_traces; ++spread_count) {
		// Rotate spread traces with respect to the current bounced trace
		Vector3r spread_trace = spread_directions_[spread_count];
		FVector spread_trace_direction = Vector3rToFVector(VectorMath::rotateVector(spread_trace, trace_rotation_quat, true));

		beam_hit |= traceDirection(trace_start_position, spread_trace_direction, bounce_depth, signal_attenuation, sensor_params, TArray<AActor*>{}, point_cloud);
	}

	// DRAW DEBUG
	if (beam_hit && sensor_params.draw_reflected_paths) DrawDebugLine(actor_->GetWorld(), trace_hit_result.TraceStart, trace_hit_result.ImpactPoint, FColor::Red, false, 1 / sensor_params.measurement_frequency);

	return beam_hit;
}

float UnrealEchoSensor::bounceTrace(FVector &trace_start_position, FVector &trace_direction,	const FHitResult &trace_hit_result,
    float &signal_attenuation, const msr::airlib::EchoSimpleParams &sensor_params) {

	// Attenuate signal
	float distance_traveled = ned_transform_->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));
	signal_attenuation += distance_traveled * sensor_params.attenuation_per_distance;
	signal_attenuation += sensor_params.attenuation_per_reflection;

	// Reflect signal 
	trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;

	return distance_traveled;

}

FVector UnrealEchoSensor::Vector3rToFVector(const Vector3r &input_vector){
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

msr::airlib::Vector3r UnrealEchoSensor::FVectorToVector3r(const FVector &input_vector) {
	return Vector3r(input_vector.X, input_vector.Y, -input_vector.Z);
}
