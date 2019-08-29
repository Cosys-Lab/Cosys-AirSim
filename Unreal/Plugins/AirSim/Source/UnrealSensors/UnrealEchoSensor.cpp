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
UnrealEchoSensor::UnrealEchoSensor(const AirSimSettings::EchoSetting& setting,
	AActor* actor, const NedTransform* ned_transform)
	: EchoSimple(setting), actor_(actor), ned_transform_(ned_transform), attenuation_per_distance(), attenuation_per_reflection()
{
	FActorSpawnParameters spawnParams;
	FRotator rotator = FRotator(0, 0, 0);
	FVector position = FVector(0, 0, 0);
	APhysicalSensorActor* physical_sensor_actor = NewObject<APhysicalSensorActor>(APhysicalSensorActor::StaticClass());
	physical_sensor_actor_object_ = actor->GetWorld()->SpawnActor<AActor>(physical_sensor_actor->sensor_blueprint_, position, rotator, spawnParams);
	physical_sensor_actor_object_->SetActorLabel(setting.sensor_name.c_str());

	UStaticMeshComponent* sensorPlaneMesh = Cast<UStaticMeshComponent>(physical_sensor_actor_object_->GetComponentByClass(UStaticMeshComponent::StaticClass()));
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

	float opening_angle = 180;  // deg, full hemisphere
	sampleSphereCap(sensor_params.number_of_traces, opening_angle, sample_directions_);
}

// initializes information based on echo configuration
void UnrealEchoSensor::generateSpreadDirections()
{
	float opening_angle = 10;  // deg
	int num_spread_traces = 10;
	sampleSphereCap(num_spread_traces, opening_angle, spread_directions_);
}

void UnrealEchoSensor::sampleSphereCap(int num_points, float opening_angle, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud) {

	//Convert opening angle to plane coordinates
	float x_limit = FMath::Cos(FMath::DegreesToRadians(opening_angle) / 2);

	// Calculate ratio of sphere surface to cap surface.
	// Scale points accordingly, e.g. if nPoints = 10 and ratio = 0.01,
	// generate 1000 points on the sphere
	float h = 1 - x_limit;
	float surface_ratio = h / 2;  // 4 * pi*R ^ 2 / 2 * pi*R*h
	int num_sphere_points = FMath::CeilToInt(num_points * 1 / surface_ratio);

	// Generate points on the sphere, retain those within the opening angle
	point_cloud.clear();
	float offset = 2.0f / num_sphere_points;
	float increment = PI * (3.0f - FMath::Sqrt(5.0f));
	for (auto i = 1; i <= num_sphere_points; ++i)
	{
		float y = ((i * offset) - 1) + (offset / 2.0f);
		float r = FMath::Sqrt(1 - FMath::Pow(y, 2));
		float phi = ((i + 1) % num_sphere_points) * increment;
		float x = FMath::Cos(phi) * r;
		if (x >= x_limit)
		{
			float z = FMath::Sin(phi) * r;
			point_cloud.emplace_back(Vector3r(x, y, z));
		}
	}
}

// Set echo object in correct pose in physical world
void UnrealEchoSensor::updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose)
{
	physical_sensor_actor_object_->SetActorLocation(ned_transform_->fromLocalNed(VectorMath::add(sensor_pose, vehicle_pose).position));
	physical_sensor_actor_object_->SetActorRotation(ned_transform_->fromNed(vehicle_pose.orientation).Rotator());
}

// Pause Unreal simulation
void UnrealEchoSensor::pause() {
	UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
}

// simulate signal propagation via Unreal ray-tracing.
bool UnrealEchoSensor::traceDirection(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose,
	Vector3r direction,	const msr::airlib::EchoSimpleParams &sensor_params, Vector3r &point, float &signal_attenuation_final)
{
	const float attenuation_per_distance = sensor_params.attenuation_per_distance;
	const float attenuation_per_reflection = sensor_params.attenuation_per_reflection;
	const float max_attenuation = sensor_params.attenuation_limit;
	const FString sensor_name = sensor_params.name.c_str();
	float signal_attenuation = 0.0f;
	TArray<AActor*> ignore_actors = { physical_sensor_actor_object_ };

	// Trace initial emission
	FVector trace_start_position = ned_transform_->fromLocalNed(VectorMath::add(sensor_pose, vehicle_pose).position);
	FVector trace_direction = Vector3rToFVector(VectorMath::rotateVector(direction, VectorMath::rotateQuaternion(vehicle_pose.orientation, sensor_pose.orientation, 1), 1));
	float trace_length = ned_transform_->fromNed((max_attenuation - signal_attenuation) / attenuation_per_distance);
	FVector trace_end_position = trace_start_position + trace_direction * trace_length;

	FHitResult trace_hit_result = FHitResult(ForceInit);
	// The physical sensor object itself has to be ignored as otherwise the trace would pottentially instantly collide with the sensor
	bool trace_hit = UAirBlueprintLib::GetObstacle(actor_, trace_start_position, trace_end_position, trace_hit_result, ignore_actors, ECC_Visibility, true);
	if(!trace_hit) return false;

	if (sensor_params.draw_initial_points) DrawDebugPoint(actor_->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Green, false, 1 / sensor_params.measurement_frequency);

	bounceTrace(trace_start_position, trace_end_position, trace_hit_result,  signal_attenuation, max_attenuation, sensor_params);

	// Trace reflections until signal fades
	FColor debug_line_color;
	while (signal_attenuation < max_attenuation)
	{
		trace_hit_result = FHitResult(ForceInit);
		trace_hit = UAirBlueprintLib::GetObstacle(actor_, trace_start_position, trace_end_position, trace_hit_result, TArray<AActor*>{}, ECC_Visibility, true);
		
	    if(!trace_hit)
		{
			return false;
		}
        // If the ray/trace hits the physical sensor object, the impact becomes a valid point for the reflector pointcloud. This also stops the cycle
		else if (trace_hit_result.GetActor()->GetActorLabel().Equals(sensor_name))  // Check if the trace intersects the sensor
		{

			signal_attenuation_final = signal_attenuation; // Store the final signal attenuation to be stored in the data

			if (sensor_params.draw_reflected_lines)
				DrawDebugLine(actor_->GetWorld(), trace_hit_result.TraceStart, trace_hit_result.ImpactPoint, FColor::Red, false, 1 / sensor_params.measurement_frequency);
			if (sensor_params.draw_reflected_points)
				DrawDebugPoint(actor_->GetWorld(), trace_hit_result.TraceStart, 6, FColor::Red, false, 1 / sensor_params.measurement_frequency);

			// decide the frame for the point-cloud
			if (sensor_params.data_frame == AirSimSettings::kVehicleInertialFrame) {
				point = ned_transform_->toLocalNed(trace_hit_result.TraceStart);
			}
			else if (sensor_params.data_frame == AirSimSettings::kSensorLocalFrame) {
				Vector3r point_v_i = ned_transform_->toLocalNed(trace_hit_result.ImpactPoint);
				point = VectorMath::transformToBodyFrame(point_v_i, sensor_pose + vehicle_pose, true);
			}
			else {
				throw std::runtime_error("Unknown requested data frame");
			}

			return true;
		}

		if(sensor_params.draw_bounce_lines){
			uint8 color_scale = static_cast<uint8>(255 * signal_attenuation / max_attenuation);
			debug_line_color = FColor(color_scale, color_scale, color_scale);
			DrawDebugLine(actor_->GetWorld(), trace_hit_result.TraceStart, trace_hit_result.ImpactPoint, debug_line_color, false, 0.2f);
		}

		bounceTrace(trace_start_position, trace_end_position, trace_hit_result,  signal_attenuation, max_attenuation, sensor_params);
	}

	return false;
}

// returns a point-cloud for the tick
void UnrealEchoSensor::getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
	msr::airlib::EchoSimpleParams sensor_params = getParams();

	// set the physical echo mesh in the correct location in the world
	updatePose(sensor_pose, vehicle_pose);
	Vector3r trace_start = VectorMath::add(sensor_pose, vehicle_pose).position;

	// shoot traces (rays)
	point_cloud.clear();
	for (auto direction_count = 0u; direction_count < sample_directions_.size(); ++direction_count)
	{
		Vector3r trace_direction = sample_directions_[direction_count];
		trace_direction = VectorMath::rotateVector(trace_direction, VectorMath::rotateQuaternion(vehicle_pose.orientation, sensor_pose.orientation, 1), 1);

		float signal_attenuation = 0.0f;
		msr::airlib::vector<Vector3r> points;

		// shoot trace and get the impact point and remaining attenuation, if any returns
		if (traceDirection2(trace_start, trace_direction, signal_attenuation, sensor_params, points))
		{
			for (auto point_count = 0u; point_count < points.size(); ++point_count) {
				point_cloud.emplace_back(points[point_count].x());
				point_cloud.emplace_back(points[point_count].y());
				point_cloud.emplace_back(points[point_count].z());
				point_cloud.emplace_back(signal_attenuation);
			}
		}
	}
	// TODO to improve performance, fill the "point_cloud" list directly instead of first filling the "point" list and then moving the data
	// or atleast convert "point" to "point_cloud" in one go

	return;
}

bool UnrealEchoSensor::traceDirection2(Vector3r trace_start, Vector3r trace_direction, float &signal_attenuation, const msr::airlib::EchoSimpleParams &sensor_params,
	msr::airlib::vector<Vector3r> &points) {

	const float attenuation_per_distance = sensor_params.attenuation_per_distance;
	const float attenuation_per_reflection = sensor_params.attenuation_per_reflection;
	const float max_attenuation = sensor_params.attenuation_limit;
	const FString sensor_name = sensor_params.name.c_str();
	const TArray<AActor*> ignore_actors = { physical_sensor_actor_object_ };

	float signal_attenuation = 0.0f;


	// Trace initial emission
	FVector trace_start_position = ned_transform_->fromLocalNed(trace_start);
	FVector trace_cast_direction = Vector3rToFVector(trace_direction);
	float trace_length = ned_transform_->fromNed((max_attenuation - signal_attenuation) / attenuation_per_distance);
	FVector trace_end_position = trace_start_position + trace_cast_direction * trace_length;

	// Recursively cast scattered rays

	// Repeat previous step until end condition is reached (attenuation too high)

	if (signal_attenuation >= max_attenuation) {

	}
	return true;

}

void UnrealEchoSensor::bounceTrace(FVector &trace_start_position, FVector &trace_end_position,	const FHitResult &trace_hit_result,
    float &signal_attenuation, float max_attenuation, const msr::airlib::EchoSimpleParams &sensor_params) {

	float trace_length = ned_transform_->fromNed((max_attenuation - signal_attenuation) / sensor_params.attenuation_per_distance);
	FVector trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;
	trace_end_position = trace_start_position + (trace_direction * trace_length);

	// Beam spread attenuation
	float distance_traveled = ned_transform_->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));
	// Atmospheric attenuation
	signal_attenuation += distance_traveled * sensor_params.attenuation_per_distance;
	signal_attenuation += sensor_params.attenuation_per_reflection;
}

FVector UnrealEchoSensor::Vector3rToFVector(const Vector3r &input_vector){
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

