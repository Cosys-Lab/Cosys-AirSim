// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealLidarSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "Async/ParallelFor.h"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include <random>

// ctor
UnrealLidarSensor::UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
	AActor* actor, const NedTransform* ned_transform)
	: LidarSimple(setting), actor_(actor), ned_transform_(ned_transform),
	sensor_params_(getParams()),
	draw_time_(1.05f / sensor_params_.horizontal_rotation_frequency),
	external_(getParams().external)
{
	// Seed and initiate noise
	std::random_device rd;
	gen_ = std::mt19937(rd());
	dist_ = std::normal_distribution<float>(0, getParams().min_noise_standard_deviation);
	point_cloud_draw_.clear();
	createLasers();
}

// initializes information based on lidar configuration
void UnrealLidarSensor::createLasers()
{


	msr::airlib::LidarSimpleParams params = getParams();

	const auto number_of_lasers = params.number_of_channels;

	float horizontal_delta = (params.horizontal_FOV_end - params.horizontal_FOV_start) / float(params.measurement_per_cycle - 1);
	for (uint32 i = 0; i < params.measurement_per_cycle; i++) {
		horizontal_angles_.Add(params.horizontal_FOV_start + i * horizontal_delta);
	}

	if (number_of_lasers <= 0)
		return;

	// calculate verticle angle distance between each laser
	float delta_angle = 0;
	if (number_of_lasers > 1)
		delta_angle = (params.vertical_FOV_upper - (params.vertical_FOV_lower)) /
		static_cast<float>(number_of_lasers - 1);

	// store vertical angles for each laser
	laser_angles_.clear();
	for (auto i = 0u; i < number_of_lasers; ++i)
	{
		const float vertical_angle = params.vertical_FOV_upper - static_cast<float>(i) * delta_angle;
		laser_angles_.emplace_back(vertical_angle);
	}

	current_horizontal_angle_index_ = horizontal_angles_.Num()-1;
}

// Set echo object in correct pose in physical world
void UnrealLidarSensor::updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose)
{
	sensor_reference_frame_ = VectorMath::add(sensor_pose, vehicle_pose);
	// DRAW DEBUG
	if (sensor_params_.draw_sensor) {
		FVector sensor_position;
		if (external_) {
			sensor_position = ned_transform_->toFVector(sensor_reference_frame_.position, 100, true);
		}
		else {
			sensor_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
		}
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), sensor_position, 5, FColor::Black, false, draw_time_);
		FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, draw_time_, 10);
	}
}

// Get echo pose in Local NED
void UnrealLidarSensor::getLocalPose(msr::airlib::Pose& sensor_pose)
{
	FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1)); ;
	sensor_pose = ned_transform_->toLocalNed(FTransform(sensor_direction.Rotation(), ned_transform_->toFVector(sensor_reference_frame_.position, 100, true), FVector(1, 1, 1)));
}

// Pause Unreal simulation
void UnrealLidarSensor::pause(const bool is_paused) {
	if (is_paused) {
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else {
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
}

// returns a point-cloud for the tick
bool UnrealLidarSensor::getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
	const msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final)
{

	updatePose(lidar_pose, vehicle_pose);

	bool refresh = false;
	msr::airlib::LidarSimpleParams params = getParams();
	const auto number_of_lasers = params.number_of_channels;
	uint32 total_points = number_of_lasers * params.measurement_per_cycle;

	if (point_cloud.size() == 0)
	{
		point_cloud.assign(total_points * 3, 0);
		groundtruth.assign(total_points, "out_of_range");
	}

	// calculate needed angle/distance between each point
	const float angle_distance_of_tick = params.horizontal_rotation_frequency * 360.0f * delta_time;
	const double angle_distance_of_laser_measure = 360.0f / params.measurement_per_cycle;

	// calculate number of points needed for each laser/channel
	uint32 points_to_scan_with_one_laser_temp = FMath::RoundHalfFromZero(angle_distance_of_tick / angle_distance_of_laser_measure);
	if (points_to_scan_with_one_laser_temp <= 0)
	{
		//UAirBlueprintLib::LogMessageString("Lidar: ", "No points requested this frame", LogDebugLevel::Failure);
		return refresh;
	}
	constexpr float MAX_POINTS_IN_SCAN = 5000;
	if (params.limit_points && points_to_scan_with_one_laser_temp * number_of_lasers > MAX_POINTS_IN_SCAN)
	{
		//UAirBlueprintLib::LogMessageString("Lidar Error: ", "Capping number of points to scan " + std::to_string(points_to_scan_with_one_laser_temp * number_of_lasers), LogDebugLevel::Failure);
		points_to_scan_with_one_laser_temp = MAX_POINTS_IN_SCAN / number_of_lasers;
	}
	const uint32 points_to_scan_with_one_laser = points_to_scan_with_one_laser_temp;

	// normalize FOV start/end
	float laser_start = std::fmod(360.0f + params.horizontal_FOV_start, 360.0f);
	float laser_end = std::fmod(360.0f + params.horizontal_FOV_end, 360.0f);

	float previous_horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];

	if (sensor_params_.draw_debug_points) {
		point_cloud_draw_.clear();
		point_cloud_draw_.assign(points_to_scan_with_one_laser * number_of_lasers, FVector());
	}

	// shoot lasers
	for (uint32 i = 1; i <= points_to_scan_with_one_laser; ++i)
	{
		if (current_horizontal_angle_index_ == horizontal_angles_.Num() - 1) {
			current_horizontal_angle_index_ = 0;
		}
		else {
			current_horizontal_angle_index_ += 1;
		}

		float horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];
		//UE_LOG(LogTemp, Display, TEXT("horizontal_angle: %f "), horizontal_angle);


		if ((previous_horizontal_angle > horizontal_angle) && (point_cloud.size() != 0)) {
			if ((((int)point_cloud.size() / 3) != params.measurement_per_cycle * number_of_lasers) || (groundtruth.size() != params.measurement_per_cycle * number_of_lasers))
			{
				UE_LOG(LogTemp, Warning, TEXT("Pointcloud or labels incorrect size! points:%i labels:%i"), (int)(point_cloud.size() / 3), groundtruth.size());
			}
			//UE_LOG(LogTemp, Display, TEXT("Pointcloud completed! points:%i labels:%i"), (int)(point_cloud.size() / 3), groundtruth.size());
			point_cloud_final = point_cloud;
			groundtruth_final = groundtruth;
			point_cloud.clear();
			groundtruth.clear();
			point_cloud.assign(total_points * 3, 0);
			groundtruth.assign(total_points, "out_of_range");
			refresh = true;
		}

		// check if horizontal angle is a duplicate
		if ((horizontal_angle - previous_horizontal_angle) <= 0.00005f && (horizontal_angle - 0) >= 0.00005f) {
			UE_LOG(LogTemp, Display, TEXT("duplicate horizontal angle! angle! previous:%f current:%f"), previous_horizontal_angle, horizontal_angle);
			continue;
		}

		// check if the laser is outside the requested horizontal FOV
		if (!VectorMath::isAngleBetweenAngles(horizontal_angle, laser_start, laser_end)) {
			UE_LOG(LogTemp, Display, TEXT("outside of FOV: %f "), horizontal_angle);
			continue;
		}

		ParallelFor(number_of_lasers, [&](uint32 laser) {
			float vertical_angle = laser_angles_[laser];
			uint32 current_point_index = number_of_lasers * current_horizontal_angle_index_ + laser;
			uint32 draw_index = number_of_lasers * i + laser;
			Vector3r point;
			FVector draw_point;
			std::string label;

			// shoot laser and get the impact point, if any
			if (shootLaser(lidar_pose, vehicle_pose, laser, horizontal_angle, vertical_angle, params, point, label, draw_point))
			{
				point_cloud[current_point_index * 3] = point.x();
				point_cloud[current_point_index * 3 + 1] = point.y();
				point_cloud[current_point_index * 3 + 2] = point.z();
				groundtruth[current_point_index] = label;
				if (sensor_params_.draw_debug_points)
					point_cloud_draw_[draw_index] = draw_point;
			}
			});


		
		previous_horizontal_angle = horizontal_angles_[current_horizontal_angle_index_];
	}

	if (sensor_params_.draw_debug_points) {
		for (uint32 j = 0; j < point_cloud_draw_.size(); j++)
		{
			UAirBlueprintLib::DrawPoint(
				actor_->GetWorld(),
				point_cloud_draw_[j],
				5,                       //size
				FColor::Green,
				false,                    //persistent (never goes away)
				(1 / (sensor_params_.horizontal_rotation_frequency * 2))                //point leaves a trail on moving object
			);
		}
	}
	return refresh;
}

FVector UnrealLidarSensor::Vector3rToFVector(const Vector3r& input_vector) {
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

// simulate shooting a laser via Unreal ray-tracing.
bool UnrealLidarSensor::shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
	const uint32 laser, const float horizontal_angle, const float vertical_angle,
	const msr::airlib::LidarSimpleParams params, Vector3r &point, std::string &label, FVector& raw_point)
{
	// start position
	Vector3r start = VectorMath::add(lidar_pose, vehicle_pose).position;

	// We need to compose rotations here rather than rotate a vector by a quaternion
	// Hence using coordOrientationAdd(..) rather than rotateQuaternion(..)

	// get ray quaternion in lidar frame (angles must be in radians)
	msr::airlib::Quaternionr ray_q_l = msr::airlib::VectorMath::toQuaternion(
		msr::airlib::Utils::degreesToRadians(vertical_angle),   //pitch - rotation around Y axis
		0,                                                      //roll  - rotation around X axis
		msr::airlib::Utils::degreesToRadians(horizontal_angle));//yaw   - rotation around Z axis

	// get ray quaternion in body frame
	msr::airlib::Quaternionr ray_q_b = VectorMath::coordOrientationAdd(ray_q_l, lidar_pose.orientation);

	// get ray quaternion in world frame
	msr::airlib::Quaternionr ray_q_w = VectorMath::coordOrientationAdd(ray_q_b, vehicle_pose.orientation);

	// get ray vector (end position)
	Vector3r end = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * params.range + start;

	FHitResult hit_result = FHitResult(ForceInit);
	TArray<AActor*> actorArray;
	//actorArray.Add(actor_);
	bool is_hit;
	if (params.external) {
		is_hit = UAirBlueprintLib::GetObstacleAdv(actor_, ned_transform_->toFVector(start, 100, true), ned_transform_->toFVector(end, 100, true), hit_result, actorArray, ECC_Visibility, true, true);
	}
	else {
		is_hit = UAirBlueprintLib::GetObstacleAdv(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), hit_result, actorArray, ECC_Visibility, true, true);
	}
	bool ignoreMaterial = false;
	if (hit_result.PhysMaterial != nullptr) {
		if (hit_result.PhysMaterial.Get()->GetFName().ToString().Contains("Lidar_Ignore_PhysicalMaterial"))
			ignoreMaterial = true;
	}
	if (is_hit && !ignoreMaterial)
	{

		FVector impact_point = hit_result.ImpactPoint;

		//Store the name the hit object.
		auto hitActor = hit_result.GetActor();
		if (hitActor != nullptr)
		{
			label = TCHAR_TO_UTF8(*hitActor->GetName());
		}

		raw_point = impact_point;

		//if (label.empty())
		//{
		//	UE_LOG(LogTemp, Warning, TEXT("Empty label!"));
		//}
		// If enabled add range noise
		if (params.generate_noise) {
			// Add noise based on normal distribution taking into account scaling of noise with distance
			float distance_noise = dist_(gen_) * (1 + ((hit_result.Distance / 100) / params.range) * (params.noise_distance_scale - 1));

			Vector3r impact_point_local = VectorMath::rotateVector(VectorMath::front(), ray_q_w, true) * ((hit_result.Distance / 100) + distance_noise) + start;
			if (params.external) {
				impact_point = ned_transform_->fromRelativeNed(impact_point_local);
			} else {
				impact_point = ned_transform_->fromLocalNed(impact_point_local);
			}			
		}

		raw_point = impact_point;

		Vector3r point_v_i;
		if (params.external) {
			point_v_i = ned_transform_->toVector3r(impact_point, 0.01, true);
		}
		else {
			point_v_i = ned_transform_->toLocalNed(impact_point);
		}

		// tranform to lidar frame
		point = VectorMath::transformToBodyFrame(point_v_i, lidar_pose + vehicle_pose, true);

		return true;
	}
	else
	{
		return false;
	}
}
