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
	sensor_params_(getParams()),
	attenuation_per_distance_(getParams().attenuation_per_distance),
	attenuation_per_reflection_(getParams().attenuation_per_reflection),
	attenuation_limit_(getParams().attenuation_limit),
	opening_angle_half_(FMath::DegreesToRadians(getParams().spread_opening_angle/2))
{
	generateSampleDirections();	
	ignore_actors_ = TArray<AActor*>{};
}

// initializes information based on echo configuration
void UnrealEchoSensor::generateSampleDirections()
{
	int num_traces = sensor_params_.number_of_traces;
	float opening_angle = 180;  // deg, = full hemisphere
	sampleSphereCap(num_traces, opening_angle, sample_directions_);
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
	sensor_reference_frame_ = VectorMath::add(sensor_pose, vehicle_pose);
	
	// DRAW DEBUG
	if(sensor_params_.draw_sensor) DrawDebugPoint(actor_->GetWorld(), ned_transform_->fromLocalNed(sensor_reference_frame_.position), 5, FColor::Blue, false, 1 / sensor_params_.measurement_frequency);
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
	// set the physical echo mesh in the correct location in the world
	updatePose(sensor_pose, vehicle_pose);
	FVector trace_start_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);

	// shoot traces (rays)
	point_cloud.clear();
	for (auto direction_count = 0u; direction_count < sample_directions_.size(); ++direction_count)
	{
		Vector3r sample_direction = sample_directions_[direction_count];

		int bounce_depth = 0;
		float signal_attenuation = 0.0f;

		FVector trace_direction = Vector3rToFVector(VectorMath::rotateVector(sample_direction, VectorMath::rotateQuaternion(vehicle_pose.orientation, sensor_pose.orientation, 1), 1)); // sensor_reference_frame_.orientation
		float trace_length = ned_transform_->fromNed((attenuation_limit_ - signal_attenuation) / attenuation_per_distance_);  // Maximum possible distance given the current attenuation
		trace_length /= 2 ;  // For the initial emission, make sure points have a return path (= distance/2)
		FVector trace_end_position = trace_start_position + trace_direction * trace_length;

		// shoot trace and get the impact point and remaining attenuation, if any returns
		traceDirection(trace_start_position, trace_end_position, signal_attenuation, point_cloud);
	}

	return;
}

void UnrealEchoSensor::traceDirection(FVector trace_start_position, FVector trace_end_position, float signal_attenuation,
	msr::airlib::vector<msr::airlib::real_T>& point_cloud) {

	TArray<FVector> trace_path = TArray<FVector>{};

	while(signal_attenuation < attenuation_limit_) {
		FHitResult trace_hit_result = FHitResult(ForceInit);
		bool trace_hit = UAirBlueprintLib::GetObstacle(actor_, trace_start_position, trace_end_position, trace_hit_result, ignore_actors_, ECC_Visibility, true);

		// DRAW DEBUG
		if (sensor_params_.draw_bounce_lines) {
			uint8 color_scale = static_cast<uint8>(255 * signal_attenuation / attenuation_limit_);
			FColor debug_line_color = FColor(color_scale, color_scale, color_scale);
			DrawDebugLine(actor_->GetWorld(), trace_start_position, trace_hit ? trace_hit_result.ImpactPoint : trace_end_position, debug_line_color, false, 0.2f);
		}

		if (!trace_hit) {
			return;
		}

		// DRAW DEBUG
		if (sensor_params_.draw_initial_points && signal_attenuation == 0) DrawDebugPoint(actor_->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Green, false, 1 / sensor_params_.measurement_frequency);

		// Bounce trace
		FVector trace_direction;
		float trace_length;
		bounceTrace(trace_start_position, trace_direction, trace_length, trace_hit_result, signal_attenuation);
		trace_end_position = trace_start_position + trace_direction * trace_length;

		FVector sensor_location = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
		float distance_to_sensor = FVector::Distance(trace_start_position, sensor_location);

		if (distance_to_sensor > trace_length) {
			return;
		}

		if (sensor_params_.draw_reflected_paths) trace_path.Emplace(trace_start_position);

		bool sensorInCone = locationInCone(sensor_location - trace_start_position, trace_direction, opening_angle_half_);
		if (sensorInCone)
		{
			trace_hit = UAirBlueprintLib::GetObstacle(actor_, trace_start_position, sensor_location, FHitResult(ForceInit), ignore_actors_, ECC_Visibility, true);

			if (!trace_hit) {  // No hit = clear LOS to sensor
				// DRAW DEBUG
				if (sensor_params_.draw_reflected_lines) {
					FVector draw_location = trace_start_position + trace_direction * distance_to_sensor;

					float radius = 100;
					FVector y_axis = FVector(0, 1, 0);
					FVector z_axis = FVector(0, 0, 1);
					DrawDebugCircle(actor_->GetWorld(), draw_location, radius, 32, FColor::Blue, false, 1 / sensor_params_.measurement_frequency, 0u, 0.0f, y_axis, z_axis, false);

					DrawDebugLine(actor_->GetWorld(), trace_start_position, draw_location, FColor::Red, false, 1 / sensor_params_.measurement_frequency);
				}

				/*if (sensor_params_.draw_reflected_lines) DrawDebugCone(actor_->GetWorld(), trace_start_position, trace_direction, distance_to_sensor, opening_angle_half_,
					opening_angle_half_, 16, FColor::Blue, false, 1 / sensor_params_.measurement_frequency);*/

				if (sensor_params_.draw_reflected_points) DrawDebugPoint(actor_->GetWorld(), trace_start_position, 5, FColor::Red, false, 1 / sensor_params_.measurement_frequency);

				if (sensor_params_.draw_reflected_paths) {
					DrawDebugLine(actor_->GetWorld(), sensor_location, trace_path[0], FColor::Red, false, 1 / sensor_params_.measurement_frequency);
					for (int trace_count = 0; trace_count < trace_path.Num()-1; trace_count++)
					{
						DrawDebugLine(actor_->GetWorld(), trace_path[trace_count], trace_path[trace_count+1], FColor::Red, false, 1 / sensor_params_.measurement_frequency);
					}
					DrawDebugLine(actor_->GetWorld(), trace_path.Last(), sensor_location, FColor::Red, false, 1 / sensor_params_.measurement_frequency);
				}

				Vector3r point_sensor_frame = ned_transform_->toLocalNed(trace_hit_result.ImpactPoint);
				point_sensor_frame = VectorMath::transformToBodyFrame(point_sensor_frame, sensor_reference_frame_, true);

				point_cloud.emplace_back(point_sensor_frame.x());
				point_cloud.emplace_back(point_sensor_frame.y());
				point_cloud.emplace_back(point_sensor_frame.z());
				point_cloud.emplace_back(signal_attenuation);

				return;
			}
		}
	}
}

float UnrealEchoSensor::bounceTrace(FVector &trace_start_position, FVector &trace_direction, float &trace_length, const FHitResult &trace_hit_result,
	float &signal_attenuation) {

	// Attenuate signal
	float distance_traveled = ned_transform_->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));
	signal_attenuation += distance_traveled * sensor_params_.attenuation_per_distance;
	signal_attenuation += sensor_params_.attenuation_per_reflection;

	// Reflect signal 
	trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;

	trace_length = ned_transform_->fromNed((attenuation_limit_ - signal_attenuation) / attenuation_per_distance_);  // Maximum possible distance given the current attenuation

	//TODO determine attenuation based on angle

	return distance_traveled;
}

FVector UnrealEchoSensor::Vector3rToFVector(const Vector3r &input_vector) {
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

msr::airlib::Vector3r UnrealEchoSensor::FVectorToVector3r(const FVector &input_vector) {
	return Vector3r(input_vector.X, input_vector.Y, -input_vector.Z);
}

bool UnrealEchoSensor::locationInCone(FVector location, FVector pointing_vector, const float opening_angle) {
	// Location relative to origin
	location.Normalize();
	float angle_between_vectors = FMath::Acos(FVector::DotProduct(location, pointing_vector));

	return angle_between_vectors <= opening_angle;
}