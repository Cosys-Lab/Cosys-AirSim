// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealEchoSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "CoreMinimal.h"

// ctor
UnrealEchoSensor::UnrealEchoSensor(const AirSimSettings::EchoSetting& setting, AActor* actor, const NedTransform* ned_transform)
	: EchoSimple(setting), actor_(actor), ned_transform_(ned_transform), saved_clockspeed_(1),
	sensor_params_(getParams()),
	attenuation_per_distance_(getParams().attenuation_per_distance),
	attenuation_per_reflection_(getParams().attenuation_per_reflection),
	attenuation_limit_(getParams().attenuation_limit),
	distance_limit_(getParams().distance_limit),
	reflection_limit_(getParams().reflection_limit),
	reflection_distance_limit_(ned_transform_->fromNed(getParams().reflection_distance_limit)),
	opening_angle_(FMath::DegreesToRadians(getParams().spread_opening_angle)),
	draw_time_(1.05f / sensor_params_.measurement_frequency),
	line_thinkness_(1.0f)
{
	generateSampleDirections();	

	if (sensor_params_.ignore_marked) {
		static const FName lidar_ignore_tag = TEXT("MarkedIgnore");
		for (TActorIterator<AActor> ActorIterator(actor->GetWorld()); ActorIterator; ++ActorIterator)
		{
			AActor* Actor = *ActorIterator;
			if (Actor && Actor != actor && Actor->Tags.Contains(lidar_ignore_tag))ignore_actors_.Add(Actor);
		}
	}
}

// initializes information based on echo configuration
void UnrealEchoSensor::generateSampleDirections()
{
	int num_traces = sensor_params_.number_of_traces;
	float opening_angle = 180;  // deg, = full hemisphere

	if (num_traces < 0)
	{
		sampleHorizontalSlice(num_traces, opening_angle, sample_directions_);
	}
	else
	{
		sampleSphereCap(num_traces, opening_angle, sample_directions_);
	}
}

void UnrealEchoSensor::sampleHorizontalSlice(int num_points, float opening_angle, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud) {
	num_points = -num_points;
	
	point_cloud.clear();

	float angle_step = FMath::DegreesToRadians(opening_angle / (num_points-1));
	float angle_offset = FMath::DegreesToRadians(opening_angle / 2);
	for (auto i = 0; i < num_points; ++i)
	{
		float angle = angle_step * i - angle_offset + PI/2;

		float x = FMath::Sin(angle);
		float y = FMath::Cos(angle);
		float z = 0;

		point_cloud.emplace_back(Vector3r(x, y, z));
	}
}

void UnrealEchoSensor::sampleSphereCap(int num_points, float opening_angle, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud) {
	point_cloud.clear();

	// Add point in frontal directionl
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
	if (sensor_params_.draw_sensor) {
		FVector sensor_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
		DrawDebugPoint(actor_->GetWorld(), sensor_position, 5, FColor::Blue, false, draw_time_);
		//FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		//DrawDebugCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, draw_time_);
	}
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

void UnrealEchoSensor::getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
	// Set the physical echo mesh in the correct location in the world
	updatePose(sensor_pose, vehicle_pose);
	FVector trace_start_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);

	// Shoot traces (rays)
	point_cloud.clear();
	for (auto direction_count = 0u; direction_count < sample_directions_.size(); ++direction_count)
	{
		Vector3r sample_direction = sample_directions_[direction_count];

		FVector trace_direction = Vector3rToFVector(VectorMath::rotateVector(sample_direction, sensor_reference_frame_.orientation, 1)); // sensor_reference_frame_.orientation

		float trace_length = ned_transform_->fromNed(remainingDistance(0, 0));  // Maximum possible distance for emitted signal (0 distance, 0 attenuation)
		FVector trace_end_position = trace_start_position + trace_direction * trace_length;

		// Shoot trace and get the impact point and remaining attenuation, if any returns
		traceDirection(trace_start_position, trace_end_position, point_cloud);
	}

	return;
}

void UnrealEchoSensor::applyFreeSpaceLoss(float &signal_attenuation, float previous_distance, float added_distance) {
	float spread_attenuation;

	if (previous_distance + added_distance < 1) {
		spread_attenuation = 0;
	}
	else{
		if (previous_distance < 1){
			added_distance = added_distance - (1 - previous_distance);
			previous_distance = 1;
		}
		spread_attenuation = 20 * FMath::LogX(10, previous_distance / (previous_distance + added_distance));
	}

	signal_attenuation += spread_attenuation;
}

float UnrealEchoSensor::remainingDistance(float signal_attenuation, float total_distance) {
	float attenuationDistance;

	if (total_distance < 1)
	{
		attenuationDistance = FMath::Pow(10, -attenuation_limit_ / 20) - total_distance;
	}
	else
	{
		attenuationDistance = total_distance * (FMath::Pow(10, (signal_attenuation - attenuation_limit_) / 20) - 1);
	}

	return FMath::Min(attenuationDistance, distance_limit_ - total_distance);
}

void UnrealEchoSensor::traceDirection(FVector trace_start_position, FVector trace_end_position, msr::airlib::vector<msr::airlib::real_T>& point_cloud) {
	float total_distance = 0.0f;
	float signal_attenuation = 0.0f;
	int reflection_count = 0;
	TArray<FVector> trace_path = TArray<FVector>{};

	FHitResult trace_hit_result, hit_result_temp;
	bool trace_hit;

	while(total_distance < distance_limit_ && reflection_count < reflection_limit_ && signal_attenuation > attenuation_limit_) {
		trace_hit_result = FHitResult(ForceInit);
		trace_hit = UAirBlueprintLib::GetObstacleAdv(actor_, trace_start_position, trace_end_position, trace_hit_result, ignore_actors_, ECC_Visibility, true);

		// DRAW DEBUG
		if (sensor_params_.draw_bounce_lines) {
			FColor line_color = FColor::MakeRedToGreenColorFromScalar(1 - (signal_attenuation / attenuation_limit_));
			DrawDebugLine
			(actor_->GetWorld(), trace_start_position, trace_hit ? trace_hit_result.ImpactPoint : trace_end_position, line_color, false, draw_time_, 0, line_thinkness_);
		}

		// Stop if nothing was hit to reflect off, or 
		// if distance between reflections is above distance limit (after emission)
		if (!trace_hit || (reflection_count > 0 && FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint) > reflection_distance_limit_)) {
			return;
		}

		// DRAW DEBUG
		if (sensor_params_.draw_initial_points && signal_attenuation == 0) DrawDebugPoint(actor_->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Green, false, draw_time_);

		// Bounce trace
		FVector trace_direction;
		float trace_length;
		bounceTrace(trace_start_position, trace_direction, trace_length, trace_hit_result, total_distance, signal_attenuation);
		trace_end_position = trace_start_position + trace_direction * trace_length;
		reflection_count += 1;

		FVector sensor_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
		float distance_to_sensor = FVector::Distance(trace_start_position, sensor_position);

		// Stop if signal can't travel far enough to return
		if (distance_to_sensor > trace_length) {
			return;
		}

		if (sensor_params_.draw_reflected_paths) trace_path.Emplace(trace_start_position);

		FVector direction_to_sensor = sensor_position - trace_start_position;
		float receiving_angle = angleBetweenVectors(direction_to_sensor, trace_direction);
		if (receiving_angle < opening_angle_)  // Check if sensor lies in opening angle
		{
			float received_attenuation = signal_attenuation + receptionAttenuation(receiving_angle);
			if (received_attenuation < attenuation_limit_) {
				continue;
			}

			hit_result_temp = FHitResult(ForceInit);
			trace_hit = UAirBlueprintLib::GetObstacleAdv(actor_, trace_start_position, sensor_position, hit_result_temp, ignore_actors_, ECC_Visibility, true);
			if (trace_hit) {  // Hit = no clear LoS to sensor
				continue;
			}

			Vector3r point_sensor_frame = ned_transform_->toLocalNed(trace_hit_result.ImpactPoint);
			point_sensor_frame = VectorMath::transformToBodyFrame(point_sensor_frame, sensor_reference_frame_, true);

			point_cloud.emplace_back(point_sensor_frame.x());
			point_cloud.emplace_back(point_sensor_frame.y());
			point_cloud.emplace_back(point_sensor_frame.z());
			point_cloud.emplace_back(received_attenuation);
			point_cloud.emplace_back(total_distance);

			// DRAW DEBUG
			if (sensor_params_.draw_reflected_points) DrawDebugPoint(actor_->GetWorld(), trace_start_position, 5, FColor::Red, false, draw_time_);
			if (sensor_params_.draw_reflected_paths) {
				DrawDebugLine(actor_->GetWorld(), sensor_position, trace_path[0], FColor::Red, false, draw_time_, 0, line_thinkness_);
				for (int trace_count = 0; trace_count < trace_path.Num() - 1; trace_count++)
				{
					DrawDebugLine(actor_->GetWorld(), trace_path[trace_count], trace_path[trace_count + 1], FColor::Red, false, draw_time_, 0, line_thinkness_);
				}
				DrawDebugLine(actor_->GetWorld(), trace_path.Last(), sensor_position, FColor::Red, false, draw_time_, 0, line_thinkness_);
			}
			if (sensor_params_.draw_reflected_lines) {
				FVector draw_location = trace_start_position + trace_direction * distance_to_sensor;

				DrawDebugLine(actor_->GetWorld(), trace_start_position, draw_location, FColor::Red, false, draw_time_, 0, line_thinkness_);
				DrawDebugPoint(actor_->GetWorld(), draw_location, 5, FColor::Red, false, draw_time_);

				float radius = distance_to_sensor * FMath::Tan(opening_angle_);
				VectorMath::Quaternionf trace_rotation_quat = VectorMath::toQuaternion(VectorMath::front(), FVectorToVector3r(trace_direction));
				Vector3r y_axis = VectorMath::rotateVector(VectorMath::right(), trace_rotation_quat, true);
				Vector3r z_axis = VectorMath::rotateVector(VectorMath::down(), trace_rotation_quat, true);
				DrawDebugCircle(actor_->GetWorld(), draw_location, radius, 128, FColor::Blue, false, draw_time_, 0u, 1.0f, Vector3rToFVector(y_axis), Vector3rToFVector(z_axis), false);
			}
		}
	}
}

void UnrealEchoSensor::bounceTrace(FVector &trace_start_position, FVector &trace_direction, float &trace_length, const FHitResult &trace_hit_result,
	float &total_distance, float &signal_attenuation) {

	// Attenuate signal
	float distance_traveled = ned_transform_->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));
	applyFreeSpaceLoss(signal_attenuation, total_distance, distance_traveled);
	signal_attenuation += distance_traveled * attenuation_per_distance_;
	signal_attenuation += attenuation_per_reflection_;

	total_distance += distance_traveled;

	// Reflect signal 
	trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;
	trace_length = ned_transform_->fromNed(remainingDistance(signal_attenuation, total_distance));
}

void UnrealEchoSensor::setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp){
	// TODO draw_time_

	const int DATA_PER_POINT = 5;
	for (int point_count = 0; point_count < point_cloud.size(); point_count+=DATA_PER_POINT){
		Vector3r point_local = Vector3r(point_cloud[point_count], point_cloud[point_count+1], -point_cloud[point_count+2]);
		Vector3r point_global1 = VectorMath::transformToWorldFrame(point_local, sensor_reference_frame_, true);
		FVector point_global = ned_transform_->fromLocalNed(point_global1);

		DrawDebugPoint(actor_->GetWorld(), point_global, 10, FColor::Blue, false, draw_time_);
	}
}

FVector UnrealEchoSensor::Vector3rToFVector(const Vector3r &input_vector) {
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

msr::airlib::Vector3r UnrealEchoSensor::FVectorToVector3r(const FVector &input_vector) {
	return Vector3r(input_vector.X, input_vector.Y, -input_vector.Z);
}

float UnrealEchoSensor::angleBetweenVectors(FVector vector1, FVector vector2) {
	// Location relative to origin
	vector1.Normalize();
	vector2.Normalize();

	return FMath::Acos(FVector::DotProduct(vector1, vector2));
}

float UnrealEchoSensor::receptionAttenuation(float reception_angle) {
	//float sigma = opening_angle_ / 10 * 3;

	//return -(FMath::Exp(-FMath::Pow(reception_angle, 2) / (2*FMath::Pow(sigma, 2))) - 1) * attenuation_limit_;
	// TODO
	return 0;
}