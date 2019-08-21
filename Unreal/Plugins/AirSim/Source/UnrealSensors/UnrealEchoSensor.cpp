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
	: EchoSimple(setting), actor_(actor), ned_transform_(ned_transform)
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
}

// initializes information based on echo configuration
void UnrealEchoSensor::generateSampleDirections()
{
	msr::airlib::EchoSimpleParams params = getParams();

	const auto number_of_points = params.number_of_traces * 2;

	if (number_of_points <= 0)
		return;

	// calculate end points for all lasers using Fibonacci sphere algorithm
	sample_directions_.clear();
	float offset = 2.0f / number_of_points;
	float increment = PI * (3.0f - FMath::Sqrt(5.0f));
	for (auto i = 1u; i <= number_of_points; ++i)
	{
		float y = ((i * offset) - 1) + (offset / 2.0f);
		float r = FMath::Sqrt(1 - FMath::Pow(y, 2));
		float phi = ((i + 1) % number_of_points) * increment;
		float x = FMath::Cos(phi) * r;
		if (x > 0)
		{
			float z = FMath::Sin(phi) * r;
			sample_directions_.emplace_back(Vector3r(x, y, z));
		}
	}	
}

// returns a point-cloud for the tick
void UnrealEchoSensor::getPointCloud(const msr::airlib::Pose& echo_pose, const msr::airlib::Pose& vehicle_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{

	msr::airlib::EchoSimpleParams params = getParams();

	// set the physical echo mesh in the correct location in the world
	updatePose(echo_pose, vehicle_pose);

	const int number_of_traces = sample_directions_.size();

	uint32 total_directions_to_scan = number_of_traces;

	// shoot traces (rays)
	point_cloud.clear();
	for (auto direction_count = 0u; direction_count < total_directions_to_scan; ++direction_count)
	{
		Vector3r sample_direction = sample_directions_[direction_count];

		Vector3r point;
		float signal_attenuation_final;

		// shoot trace and get the impact point and remaining attenuation, if any returns
		if (traceDirection(echo_pose, vehicle_pose, sample_direction, params, point, signal_attenuation_final))
		{
			point_cloud.emplace_back(point.x());
			point_cloud.emplace_back(point.y());
			point_cloud.emplace_back(point.z());
			point_cloud.emplace_back(signal_attenuation_final);
		}
	}
		
	return;
}

// Set echo object in correct pose in physical world
void UnrealEchoSensor::updatePose(const msr::airlib::Pose& echo_pose, const msr::airlib::Pose& vehicle_pose) 
{
	physical_sensor_actor_object_->SetActorLocation(ned_transform_->fromLocalNed(VectorMath::add(echo_pose, vehicle_pose).position));
	physical_sensor_actor_object_->SetActorRotation(ned_transform_->fromNed(vehicle_pose.orientation).Rotator());
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

// simulate signal propagation via Unreal ray-tracing.
bool UnrealEchoSensor::traceDirection(	const msr::airlib::Pose& echo_pose, const msr::airlib::Pose& vehicle_pose,
	Vector3r direction,	const msr::airlib::EchoSimpleParams params, Vector3r &point, float &signal_attenuation_final)
{
	const float attenuation_per_distance = params.attenuation_per_distance;
	const float attenuation_per_reflection = params.attenuation_per_reflection;
	const float max_attenuation = params.attenuation_limit;
	const FString sensor_name = params.name.c_str();
	float signal_attenuation = 0.0f;

	// Trace initial emission
	FVector trace_start_position = ned_transform_->fromLocalNed(VectorMath::add(echo_pose, vehicle_pose).position);
	FVector trace_direction = Vector3rToFVector(VectorMath::rotateVector(direction, VectorMath::rotateQuaternion(vehicle_pose.orientation, echo_pose.orientation, 1), 1));
	float trace_length = ned_transform_->fromNed((max_attenuation - signal_attenuation) / attenuation_per_distance);
	FVector trace_end_position = trace_start_position + trace_direction * trace_length;

	FHitResult trace_hit_result = FHitResult(ForceInit);
	// The physical sensor object itself has to be ignored as otherwise the trace would pottentially instantly collide with the sensor
	bool trace_hit = UAirBlueprintLib::GetObstacle(actor_, trace_start_position, trace_end_position, trace_hit_result, { physical_sensor_actor_object_ }, ECC_Visibility, true);
	if(!trace_hit) return false;

	if (params.draw_initial_points) DrawDebugPoint(actor_->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Green, false, 1 / params.measurement_frequency);

	bounceTrace(trace_start_position, trace_end_position, trace_hit_result,  signal_attenuation, max_attenuation, params);

	// Trace reflections until signal fades
	FColor debug_line_color;
	while (signal_attenuation < max_attenuation)
	{
		trace_hit_result = FHitResult(ForceInit);
		trace_hit = UAirBlueprintLib::GetObstacle(actor_, trace_start_position, trace_end_position, trace_hit_result, {}, ECC_Visibility, true);
		
	    if(!trace_hit)
		{
			return false;
		}
        // If the ray/trace hits the physical sensor object, the impact becomes a valid point for the reflector pointcloud. This also stops the cycle
		else if (trace_hit_result.GetActor()->GetActorLabel().Equals(sensor_name))  // Check if the trace intersects the sensor
		{

			signal_attenuation_final = signal_attenuation; // Store the final signal attenuation to be stored in the data

			if (params.draw_reflected_lines)
				DrawDebugLine(actor_->GetWorld(), trace_hit_result.TraceStart, trace_hit_result.ImpactPoint, FColor::Red, false, 1 / params.measurement_frequency);
			if (params.draw_reflected_points)
				DrawDebugPoint(actor_->GetWorld(), trace_hit_result.TraceStart, 6, FColor::Red, false, 1 / params.measurement_frequency);

			// decide the frame for the point-cloud
			if (params.data_frame == AirSimSettings::kVehicleInertialFrame) {
				point = ned_transform_->toLocalNed(trace_hit_result.TraceStart);
			}
			else if (params.data_frame == AirSimSettings::kSensorLocalFrame) {
				Vector3r point_v_i = ned_transform_->toLocalNed(trace_hit_result.ImpactPoint);
				point = VectorMath::transformToBodyFrame(point_v_i, echo_pose + vehicle_pose, true);
			}
			else {
				throw std::runtime_error("Unknown requested data frame");
			}

			return true;
		}

		if(params.draw_bounce_lines){
			uint8 color_scale = static_cast<uint8>(255 * signal_attenuation / max_attenuation);
			debug_line_color = FColor(color_scale, color_scale, color_scale);
			DrawDebugLine(actor_->GetWorld(), trace_hit_result.TraceStart, trace_hit_result.ImpactPoint, debug_line_color, false, 0.2f);
		}

		bounceTrace(trace_start_position, trace_end_position, trace_hit_result,  signal_attenuation, max_attenuation, params);
	}

	return false;
}

void UnrealEchoSensor::bounceTrace(FVector &trace_start_position, FVector &trace_end_position,	const FHitResult &trace_hit_result,
    float &signal_attenuation, float max_attenuation, const msr::airlib::EchoSimpleParams &params) {

	float trace_length = ned_transform_->fromNed((max_attenuation - signal_attenuation) / params.attenuation_per_distance);
	FVector trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;
	trace_end_position = trace_start_position + (trace_direction * trace_length);

	// Beam spread attenuation
	float distance_traveled = ned_transform_->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));
	// Atmospheric attenuation
	signal_attenuation += distance_traveled * params.attenuation_per_distance;
	signal_attenuation += params.attenuation_per_reflection;
}

void UnrealEchoSensor::bounceTrace2( FVector &trace_start_position, FVector &trace_end_position, const FHitResult &trace_hit_result,
							float &signal_attenuation, float max_attenuation, const msr::airlib::EchoSimpleParams &params) {

	float near_field_boundary = 4 * M_PI;  // TODO where is the boundary -> depends on the frequency
	float trace_length = ned_transform_->fromNed((max_attenuation - signal_attenuation) / params.attenuation_per_distance);
	FVector trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;
	trace_end_position = trace_start_position + (trace_direction * trace_length);

	// Beam spread attenuation
	float distance_traveled = ned_transform_->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));
	if (distance_traveled <= near_field_boundary){
		signal_attenuation += 0;  // Near field (TODO which formula???)
	}
	else {
		signal_attenuation += 20 * std::log10(4 * PI * distance_traveled);  // Far field (Friis)
	}

	// Atmospheric attenuation
	signal_attenuation += distance_traveled * params.attenuation_per_distance;  // TODO rename to "atmospheric attenuation"?

	// Reflective attenuation
	float refractive_index_air = 1;
	float refractive_index_object = 0;
	float reflectance = std::pow((refractive_index_air - refractive_index_object)/(refractive_index_air + refractive_index_object), 2);
	signal_attenuation += params.attenuation_per_reflection * reflectance;  // TODO IRL reflectance depends on angle of incidence (Fresnell coefficients)
}

FVector UnrealEchoSensor::Vector3rToFVector(const Vector3r &input_vector){
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

