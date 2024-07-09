// Developed by Cosys-Lab, University of Antwerp

#include "UnrealEchoSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "Async/ParallelFor.h"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "Math/GenericOctree.h"
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
	reflection_opening_angle_(FMath::DegreesToRadians(getParams().reflection_opening_angle)),
	sensor_passive_radius_(ned_transform_->fromNed(getParams().sensor_passive_radius)),
	draw_time_(1.05f / sensor_params_.measurement_frequency),
	line_thickness_(1.0f),
	external_(getParams().external)
{
	generateSampleDirectionPoints();

	point_cloud_draw_reflected_points_.clear();

	if (sensor_params_.ignore_marked) {
		static const FName lidar_ignore_tag = TEXT("MarkedIgnore");
		for (TActorIterator<AActor> ActorIterator(actor->GetWorld()); ActorIterator; ++ActorIterator)
		{
			AActor* Actor = *ActorIterator;
			if (Actor && Actor != actor && Actor->Tags.Contains(lidar_ignore_tag))ignore_actors_.Add(Actor);
		}
	}

	TSubclassOf<APassiveEchoBeacon> passive_beacon_class = APassiveEchoBeacon::StaticClass();
	TArray<AActor*> found_beacons;
	UGameplayStatics::GetAllActorsOfClass(actor_->GetWorld(), passive_beacon_class, found_beacons);
	for (AActor* beacon_actor : found_beacons) {
		APassiveEchoBeacon* beacon = Cast<APassiveEchoBeacon>(beacon_actor);
		if (!beacon->IsStarted()) {
			beacon->StartSampling();
		}
		passive_points_.Append(beacon->getPoints());
	}
	passive_points_.Sort([](const UnrealEchoCommon::EchoPoint& ip1, const UnrealEchoCommon::EchoPoint& ip2) {
		if (ip1.point.X != ip2.point.X)
			return ip1.point.X < ip2.point.X;
		if (ip1.point.Y != ip2.point.Y)
			return ip1.point.Y < ip2.point.Y;
		return ip1.point.Z < ip2.point.Z;
		});
}


// initializes information based on echo configuration
void UnrealEchoSensor::generateSampleDirectionPoints()
{
	int num_traces = sensor_params_.number_of_traces;
	float lower_azimuth_limit = sensor_params_.sensor_lower_azimuth_limit;
	float upper_azimuth_limit = sensor_params_.sensor_upper_azimuth_limit;
	float lower_elevation_limit = -sensor_params_.sensor_upper_elevation_limit; // yes I know this is backwards. might fix one day
	float upper_elevation_limit = -sensor_params_.sensor_lower_elevation_limit;
	if (num_traces < 0)
	{
		UnrealEchoCommon::sampleHorizontalSlice(num_traces, lower_azimuth_limit, upper_azimuth_limit, sample_direction_points_);
	}
	else
	{
		UnrealEchoCommon::sampleSphereCap(num_traces, lower_azimuth_limit, upper_azimuth_limit, lower_elevation_limit, upper_elevation_limit, sample_direction_points_);
	}
}

void UnrealEchoSensor::getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose,
	msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth,
	msr::airlib::vector<msr::airlib::real_T>& passive_beacons_point_cloud, msr::airlib::vector<std::string>& passive_beacons_groundtruth)
{
	// Set the physical echo mesh in the correct location in the world
	FVector trace_start_position;
	updatePose(sensor_pose, vehicle_pose);
	if (external_) {
		trace_start_position = ned_transform_->toFVector(sensor_reference_frame_.position, 100, true);
	}
	else {
		trace_start_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
	}

	point_cloud.clear();
	groundtruth.clear();
	passive_beacons_point_cloud.clear();
	passive_beacons_groundtruth.clear();

	if (point_cloud.size() == 0 && sensor_params_.parallel)
	{
		point_cloud.assign(sample_direction_points_.size() * 6, 0);
		groundtruth.assign(sample_direction_points_.size(), "label_not_set");
	}

	if (sensor_params_.active) {
		if (sensor_params_.parallel) {

			if (sensor_params_.draw_reflected_points) {
				point_cloud_draw_reflected_points_.clear();
				point_cloud_draw_reflected_points_.assign(sample_direction_points_.size(), FVector());
			}

			ParallelFor(sample_direction_points_.size(), [&](uint32 current_sample_index)
				{
					Vector3r sample_direction_point = sample_direction_points_[current_sample_index];

					FVector trace_direction = UnrealEchoCommon::Vector3rToFVector(VectorMath::rotateVector(sample_direction_point, sensor_reference_frame_.orientation, 1)); // sensor_reference_frame_.orientation

					float trace_length = ned_transform_->fromNed(UnrealEchoCommon::remainingDistance(0, 0, attenuation_limit_, distance_limit_));  // Maximum possible distance for emitted signal (0 distance, 0 attenuation)
					FVector trace_end_position = trace_start_position + trace_direction * trace_length;


					// Shoot trace and get the impact point and remaining attenuation, if any returns
					UnrealEchoCommon::traceDirection(current_sample_index, true, trace_start_position, trace_end_position, point_cloud, groundtruth, point_cloud_draw_reflected_points_, ned_transform_, sensor_reference_frame_,
						distance_limit_, reflection_limit_, attenuation_limit_, reflection_distance_limit_, reflection_opening_angle_, attenuation_per_distance_, attenuation_per_reflection_, ignore_actors_, actor_, external_, false,
						draw_time_, line_thickness_, false, false, false, sensor_params_.draw_reflected_points, false);
				}
			);

			bool persistent_lines = false;
			if (draw_time_ == -1)persistent_lines = true;
			if (sensor_params_.draw_reflected_points) {
				for (uint32 j = 0; j < point_cloud_draw_reflected_points_.size(); j++)
				{
					UAirBlueprintLib::DrawPoint(
						actor_->GetWorld(),
						point_cloud_draw_reflected_points_[j],
						5,                       //size
						FColor::Red,
						persistent_lines,
						draw_time_
					);
				}
			}


		}
		else {
			for (auto current_sample_index = 0u; current_sample_index < sample_direction_points_.size(); ++current_sample_index)
			{
				Vector3r sample_direction_point = sample_direction_points_[current_sample_index];

				FVector trace_direction = UnrealEchoCommon::Vector3rToFVector(VectorMath::rotateVector(sample_direction_point, sensor_reference_frame_.orientation, 1)); // sensor_reference_frame_.orientation

				float trace_length = ned_transform_->fromNed(UnrealEchoCommon::remainingDistance(0, 0, attenuation_limit_, distance_limit_));  // Maximum possible distance for emitted signal (0 distance, 0 attenuation)
				FVector trace_end_position = trace_start_position + trace_direction * trace_length;


				// Shoot trace and get the impact point and remaining attenuation, if any returns
				UnrealEchoCommon::traceDirection(current_sample_index, false, trace_start_position, trace_end_position, point_cloud, groundtruth, point_cloud_draw_reflected_points_, ned_transform_, sensor_reference_frame_,
					distance_limit_, reflection_limit_, attenuation_limit_, reflection_distance_limit_, reflection_opening_angle_, attenuation_per_distance_, attenuation_per_reflection_, ignore_actors_, actor_, external_, false,
					draw_time_, line_thickness_, sensor_params_.draw_reflected_paths, sensor_params_.draw_bounce_lines, sensor_params_.draw_initial_points, sensor_params_.draw_reflected_points, sensor_params_.draw_reflected_lines);
			}
		}
	}

	if(sensor_params_.parallel){
		for (int i = groundtruth.size() - 1; i >= 0; --i) {
			if (groundtruth[i] == "label_not_set") {
				groundtruth.erase(groundtruth.begin() + i);
				if (i * 6 < point_cloud.size()) {
					point_cloud.erase(point_cloud.begin() + i * 6, point_cloud.begin() + i * 6 + 6);
				}
			}
  	    }
	}

	if (sensor_params_.passive) {

		bool persistent_lines = false;
		if (draw_time_ == -1)persistent_lines = true;

		float min_x, max_x, min_y, max_y, min_z, max_z = 0;
		min_x = trace_start_position.X - sensor_passive_radius_;
		max_x = trace_start_position.X + sensor_passive_radius_;
		min_y = trace_start_position.Y - sensor_passive_radius_;
		max_y = trace_start_position.Y + sensor_passive_radius_;
		min_z = trace_start_position.Z - sensor_passive_radius_;
		max_z = trace_start_position.Z + sensor_passive_radius_;

		TArray<UnrealEchoCommon::EchoPoint> passive_points_allowed;
		passive_points_allowed = passive_points_.FilterByPredicate([min_x, max_x, min_y, max_y, min_z, max_z](const UnrealEchoCommon::EchoPoint& cur_point) {
			return (cur_point.point.X >= min_x &&
				cur_point.point.X <= max_x &&
				cur_point.point.Y >= min_y &&
				cur_point.point.Y <= max_y &&
				cur_point.point.Z >= min_z &&
				cur_point.point.Z <= max_z); });
		for (UnrealEchoCommon::EchoPoint passive_point_allowed : passive_points_allowed) {

			Vector3r local_point_position = UnrealEchoCommon::FVectorToVector3r(passive_point_allowed.point - trace_start_position);
			Vector3r rotated_local_point_position = VectorMath::rotateVector(local_point_position, sensor_reference_frame_.orientation.conjugate(), 1);

			float az = FMath::RadiansToDegrees(FMath::Atan2(rotated_local_point_position.y(), rotated_local_point_position.x()));
			float el = FMath::RadiansToDegrees(FMath::Atan2(rotated_local_point_position.z(), FMath::Sqrt(FMath::Pow(rotated_local_point_position.x(), 2) + FMath::Pow(rotated_local_point_position.y(), 2))));
			if (az > sensor_params_.sensor_lower_azimuth_limit && az < sensor_params_.sensor_upper_azimuth_limit && el > -sensor_params_.sensor_upper_elevation_limit && el < -sensor_params_.sensor_lower_elevation_limit) {
				FHitResult trace_hit_result = FHitResult(ForceInit);
				bool trace_hit = UAirBlueprintLib::GetObstacleAdv(actor_, trace_start_position, passive_point_allowed.point, trace_hit_result, ignore_actors_, ECC_Visibility, true);

				if (!trace_hit || FVector::Dist(trace_hit_result.ImpactPoint, passive_point_allowed.point) <= 0.1) {

					if (sensor_params_.draw_passive_sources) {
						UAirBlueprintLib::DrawPoint(actor_->GetWorld(), passive_point_allowed.point, 5, FColor::Red, persistent_lines, draw_time_);
						FVector draw_line_end_point = passive_point_allowed.point + passive_point_allowed.direction * 100;
						FColor line_color = FColor::MakeRedToGreenColorFromScalar(1 - (passive_point_allowed.total_attenuation / attenuation_limit_));
						UAirBlueprintLib::DrawLine(actor_->GetWorld(), passive_point_allowed.point, draw_line_end_point, line_color, persistent_lines, draw_time_, 0, line_thickness_);
					}
					if (sensor_params_.draw_passive_lines) {
						UAirBlueprintLib::DrawLine(actor_->GetWorld(), trace_start_position, passive_point_allowed.point, FColor(177, 0, 151), persistent_lines, draw_time_, 0, line_thickness_);
					}

					Vector3r point_sensor_frame;
					if (external_) {
						point_sensor_frame = ned_transform_->toVector3r(passive_point_allowed.point, 0.01, true);
					}
					else {
						point_sensor_frame = ned_transform_->toLocalNed(passive_point_allowed.point);
					}
					point_sensor_frame = VectorMath::transformToBodyFrame(point_sensor_frame, sensor_reference_frame_, true);

					passive_beacons_point_cloud.emplace_back(point_sensor_frame.x());
					passive_beacons_point_cloud.emplace_back(point_sensor_frame.y());
					passive_beacons_point_cloud.emplace_back(point_sensor_frame.z());

					passive_beacons_point_cloud.emplace_back(passive_point_allowed.total_attenuation);
					passive_beacons_point_cloud.emplace_back(passive_point_allowed.total_distance);
					passive_beacons_point_cloud.emplace_back(passive_point_allowed.reflections);

					FVector point_direction = UnrealEchoCommon::Vector3rToFVector(VectorMath::rotateVectorReverse(UnrealEchoCommon::FVectorToVector3r(passive_point_allowed.direction), sensor_reference_frame_.orientation, 1));
					passive_beacons_point_cloud.emplace_back(point_direction.X);
					passive_beacons_point_cloud.emplace_back(point_direction.Y);
					passive_beacons_point_cloud.emplace_back(-point_direction.Z);

					passive_beacons_groundtruth.emplace_back(passive_point_allowed.reflection_object);
					passive_beacons_groundtruth.emplace_back(passive_point_allowed.source_object);
				}
			}
		}
	}
}



void UnrealEchoSensor::setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp){
	// TODO consume point cloud (+ draw_time_)?

	if (sensor_params_.draw_external_points) {
		const int DATA_PER_POINT = 5;
		for (int point_count = 0; point_count < point_cloud.size(); point_count += DATA_PER_POINT) {
			Vector3r point_local = Vector3r(point_cloud[point_count], point_cloud[point_count + 1], point_cloud[point_count + 2]);
			Vector3r point_global1 = VectorMath::transformToWorldFrame(point_local, sensor_pose, true);
			FVector point_global;
			if (external_) {
				point_global = ned_transform_->toFVector(point_global1, 100, true);
			}
			else {
				point_global = ned_transform_->fromLocalNed(point_global1);
			}

			UAirBlueprintLib::DrawPoint(actor_->GetWorld(),	point_global, 10, FColor::Orange, false, draw_time_);
		}
	}
}


// Set echo object in correct pose in physical world
void UnrealEchoSensor::updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose)
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
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), sensor_position, 5, FColor::Black, false, draw_time_);
		FVector sensor_direction = UnrealEchoCommon::Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, draw_time_, 10);
	}
}

// Get echo pose in Local NED
void UnrealEchoSensor::getLocalPose(msr::airlib::Pose& sensor_pose)
{
	FVector sensor_direction = UnrealEchoCommon::Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1)); ;
	sensor_pose = ned_transform_->toLocalNed(FTransform(sensor_direction.Rotation(), ned_transform_->toFVector(sensor_reference_frame_.position, 100, true), FVector(1, 1, 1)));
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