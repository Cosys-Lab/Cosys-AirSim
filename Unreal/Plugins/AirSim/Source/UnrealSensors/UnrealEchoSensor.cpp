// Developed by Cosys-Lab, University of Antwerp

#include "UnrealEchoSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
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
		passive_points_.Append(beacon->getPoints());
	}
	passive_points_.Sort([](const EchoPoint& ip1, const EchoPoint& ip2) {
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
		sampleHorizontalSlice(num_traces, lower_azimuth_limit, upper_azimuth_limit, sample_direction_points_);
	}
	else
	{
		sampleSphereCap(num_traces, lower_azimuth_limit, upper_azimuth_limit, lower_elevation_limit, upper_elevation_limit, sample_direction_points_);
	}
}

void UnrealEchoSensor::sampleHorizontalSlice(int num_points, float lower_azimuth_limit, float upper_azimuth_limit, msr::airlib::vector<msr::airlib::Vector3r>& direction_points) {
	num_points = -num_points;
	
	direction_points.clear();

	float angle_step = FMath::DegreesToRadians((upper_azimuth_limit - lower_azimuth_limit) / (num_points-1));
	for (auto i = 0; i < num_points; ++i)
	{
		float angle = FMath::DegreesToRadians(lower_azimuth_limit) + i * angle_step;

		float y = FMath::Sin(angle);
		float x = FMath::Cos(angle);
		float z = 0;

		direction_points.emplace_back(Vector3r(x, y, z));
	}
}

void UnrealEchoSensor::sampleSphereCap(int num_points, float lower_azimuth_limit, float upper_azimuth_limit, float lower_elevation_limit, float upper_elevation_limit, msr::airlib::vector<msr::airlib::Vector3r>& direction_points) {
	direction_points.clear();

	// Add point in frontal directionl
	direction_points.emplace(direction_points.begin(), Vector3r(1, 0, 0));

	float surface_area = (FMath::DegreesToRadians(upper_azimuth_limit) - FMath::DegreesToRadians(lower_azimuth_limit)) * (FMath::Sin(FMath::DegreesToRadians(upper_elevation_limit)) - FMath::Sin(FMath::DegreesToRadians(lower_elevation_limit)));
	float surface_scaler = 4 * PI / surface_area;
	int num_sphere_points = FMath::CeilToInt(num_points * surface_scaler);

	// Generate points on the sphere, retain those within the opening angle
	float offset = 2.0f / num_sphere_points;
	float increment = PI * (3.0f - FMath::Sqrt(5.0f));
	for (auto i = 1; i <= num_sphere_points; ++i)
	{
		float y = ((i * offset) - 1) + (offset / 2.0f);
		float r = FMath::Sqrt(1 - FMath::Pow(y, 2));
		float phi = ((i + 1) % num_sphere_points) * increment;
		float x = FMath::Cos(phi) * r;
		float z = FMath::Sin(phi) * r;
		if (direction_points.size() == num_points)
		{
			return;
		}
		else
		{
			float az = FMath::RadiansToDegrees(FMath::Atan2(y, x));
			float el = FMath::RadiansToDegrees(FMath::Atan2(z, FMath::Sqrt(FMath::Pow(x, 2) + FMath::Pow(y, 2))));
			if(az > lower_azimuth_limit && az < upper_azimuth_limit && el > lower_elevation_limit && el < upper_elevation_limit)
				direction_points.emplace_back(Vector3r(x, y, z));
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
		FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, draw_time_, 10);
	}
}

// Get echo pose in Local NED
void UnrealEchoSensor::getLocalPose(msr::airlib::Pose& sensor_pose)
{
	FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));	;		
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

	if (sensor_params_.active) {
		for (auto sample_direction_point_count = 0u; sample_direction_point_count < sample_direction_points_.size(); ++sample_direction_point_count)
		{
			Vector3r sample_direction_point = sample_direction_points_[sample_direction_point_count];

			FVector trace_direction = Vector3rToFVector(VectorMath::rotateVector(sample_direction_point, sensor_reference_frame_.orientation, 1)); // sensor_reference_frame_.orientation

			float trace_length = ned_transform_->fromNed(remainingDistance(0, 0, attenuation_limit_, distance_limit_));  // Maximum possible distance for emitted signal (0 distance, 0 attenuation)
			FVector trace_end_position = trace_start_position + trace_direction * trace_length;


			// Shoot trace and get the impact point and remaining attenuation, if any returns
			traceDirection(trace_start_position, trace_end_position, point_cloud, groundtruth, ned_transform_, sensor_reference_frame_,
				distance_limit_, reflection_limit_, attenuation_limit_, reflection_distance_limit_, reflection_opening_angle_, attenuation_per_distance_, attenuation_per_reflection_, ignore_actors_, actor_, external_, false,
				draw_time_, line_thickness_, sensor_params_.draw_reflected_paths, sensor_params_.draw_bounce_lines, sensor_params_.draw_initial_points, sensor_params_.draw_reflected_points, sensor_params_.draw_reflected_lines);
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

		TArray<UnrealEchoSensor::EchoPoint> passive_points_allowed;
		passive_points_allowed = passive_points_.FilterByPredicate([min_x, max_x, min_y, max_y, min_z, max_z](const EchoPoint& cur_point) {
																													return (cur_point.point.X >= min_x &&
																															cur_point.point.X <= max_x &&
																															cur_point.point.Y >= min_y &&
																															cur_point.point.Y <= max_y &&
																															cur_point.point.Z >= min_z &&
																															cur_point.point.Z <= max_z);});
		for (EchoPoint passive_point_allowed : passive_points_allowed) {

			Vector3r local_point_position = FVectorToVector3r(passive_point_allowed.point - trace_start_position);
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

					FVector point_direction = Vector3rToFVector(VectorMath::rotateVectorReverse(FVectorToVector3r(passive_point_allowed.direction), sensor_reference_frame_.orientation, 1));
					passive_beacons_point_cloud.emplace_back(point_direction.X);
					passive_beacons_point_cloud.emplace_back(point_direction.Y);
					passive_beacons_point_cloud.emplace_back(point_direction.Z);

					passive_beacons_groundtruth.emplace_back(passive_point_allowed.reflection_object);
					passive_beacons_groundtruth.emplace_back(passive_point_allowed.source_object);
				}
			}			
		}
	}
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

float UnrealEchoSensor::remainingDistance(float signal_attenuation, float total_distance, float attenuation_limit, float distance_limit) {
	float attenuationDistance;

	if (total_distance < 1)
	{
		attenuationDistance = FMath::Pow(10, -attenuation_limit / 20) - total_distance;
	}
	else
	{
		attenuationDistance = total_distance * (FMath::Pow(10, (signal_attenuation - attenuation_limit) / 20) - 1);
	}

	if (attenuationDistance < (distance_limit - total_distance)) {
		return attenuationDistance;
	}
	else {
		return (distance_limit - total_distance);
	}
	return FMath::Min(attenuationDistance, distance_limit - total_distance);

}

void UnrealEchoSensor::traceDirection(FVector trace_start_position, FVector trace_end_position, msr::airlib::vector<msr::airlib::real_T>& points, msr::airlib::vector<std::string>& groundtruth, 
	                                  const NedTransform* ned_transform, const msr::airlib::Pose& pose,	float distance_limit, int reflection_limit, float attenuation_limit, float reflection_distance_limit,
	                                  float reflection_opening_angle, float attenuation_per_distance, float attenuation_per_reflection, TArray<AActor*> ignore_actors, AActor* cur_actor, bool external, bool result_uu,
	                                  float draw_time, float line_thickness, bool debug_draw_reflected_paths, bool debug_draw_bounce_lines, bool debug_draw_initial_points, bool debug_draw_reflected_points,
	                                  bool debug_draw_reflected_lines, bool check_return, bool save_normal, bool save_source, bool only_final_reflection, std::string source_label) {
	float total_distance = 0.0f;
	float signal_attenuation = 0.0f;
	int reflection_count = 0;
	TArray<FVector> trace_path = TArray<FVector>{};
	FHitResult trace_hit_result, hit_result_temp, trace_hit_previous;
	bool trace_hit;
	std::string label;
	AActor* hitActor;
	FVector previous_direction; 
	bool persistent_lines = false;

	if (draw_time == -1)persistent_lines = true;
	while(total_distance < distance_limit && reflection_count < reflection_limit && signal_attenuation > attenuation_limit) {
		trace_hit_result = FHitResult(ForceInit);
		trace_hit = UAirBlueprintLib::GetObstacleAdv(cur_actor, trace_start_position, trace_end_position, trace_hit_result, ignore_actors, ECC_Visibility, true);

		if (debug_draw_bounce_lines) {
			FColor line_color = FColor::MakeRedToGreenColorFromScalar(1 - (signal_attenuation / attenuation_limit));
			UAirBlueprintLib::DrawLine(cur_actor->GetWorld(), trace_start_position, trace_hit ? trace_hit_result.ImpactPoint : trace_end_position, line_color, persistent_lines, draw_time, 0, line_thickness);
		}

		// Stop if nothing was hit to reflect off, or 
		// if distance between reflections is above distance limit (after emission)
		if (!trace_hit || (reflection_count > 0 && FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint) > reflection_distance_limit)) {

			if (!check_return && only_final_reflection && reflection_count > 0) {
				hitActor = trace_hit_previous.GetActor();
				if (hitActor != nullptr)
				{
					label = TCHAR_TO_UTF8(*hitActor->GetName());
				}
				SavePoint(trace_hit_previous, previous_direction, signal_attenuation, total_distance, reflection_count, label, ned_transform, pose, points, groundtruth, external, result_uu, save_normal, save_source, source_label);
			}
			return;
		}

		if (debug_draw_initial_points && signal_attenuation == 0) {
			UAirBlueprintLib::DrawPoint(cur_actor->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Green, persistent_lines, draw_time);
		}

		// Bounce trace
		FVector trace_direction;
		float trace_length;
		bounceTrace(trace_start_position, trace_direction, trace_length, trace_hit_result, total_distance, signal_attenuation,
			        attenuation_per_distance, attenuation_per_reflection, distance_limit, attenuation_limit, ned_transform);
		trace_end_position = trace_start_position + trace_direction * trace_length;
		reflection_count += 1;

		FVector sensor_position;
		if (external) {
			sensor_position = ned_transform->toFVector(pose.position, 100, true);
		}
		else {
			sensor_position = ned_transform->fromLocalNed(pose.position);
		}

		hitActor = trace_hit_result.GetActor();
		if (hitActor != nullptr)
		{
			label = TCHAR_TO_UTF8(*hitActor->GetName());
		}

		if (check_return) {

			float distance_to_sensor = FVector::Distance(trace_start_position, sensor_position);

			// Stop if signal can't travel far enough to return
			if (distance_to_sensor > trace_length) {
				return;
			}

			if (debug_draw_reflected_paths) trace_path.Emplace(trace_start_position);

			FVector direction_to_sensor = sensor_position - trace_start_position;
			float receiving_angle = angleBetweenVectors(direction_to_sensor, trace_direction);
			if (receiving_angle < reflection_opening_angle)  // Check if sensor lies in opening angle
			{
				float received_attenuation = signal_attenuation + receptionAttenuation(receiving_angle);
				if (received_attenuation < attenuation_limit) {
					continue;
				}

				hit_result_temp = FHitResult(ForceInit);
				trace_hit = UAirBlueprintLib::GetObstacleAdv(cur_actor, trace_start_position, sensor_position, hit_result_temp, ignore_actors, ECC_Visibility, true);
				if (trace_hit) {  // Hit = no clear LoS to sensor
					continue;
				}				

				SavePoint(trace_hit_result, trace_direction, signal_attenuation, total_distance, reflection_count, label, ned_transform, pose, points, groundtruth, external, result_uu, save_normal, save_source, source_label);
	
				if (debug_draw_reflected_points) {
					UAirBlueprintLib::DrawPoint(cur_actor->GetWorld(), trace_start_position, 5, FColor::Red, persistent_lines, draw_time);
				}
				if (debug_draw_reflected_paths) {
					UAirBlueprintLib::DrawLine(cur_actor->GetWorld(), sensor_position, trace_path[0], FColor::Red, persistent_lines, draw_time, 0, line_thickness);
					for (int trace_count = 0; trace_count < trace_path.Num() - 1; trace_count++)
					{
						UAirBlueprintLib::DrawLine(cur_actor->GetWorld(), trace_path[trace_count], trace_path[trace_count + 1], FColor::Red, persistent_lines, draw_time, 0, line_thickness);
					}
					UAirBlueprintLib::DrawLine(cur_actor->GetWorld(), trace_path.Last(), sensor_position, FColor::Red, persistent_lines, draw_time, 0, line_thickness);
				}
				if (debug_draw_reflected_lines) {
					FVector draw_location = trace_start_position + trace_direction * distance_to_sensor;

					UAirBlueprintLib::DrawLine(cur_actor->GetWorld(), trace_start_position, draw_location, FColor::Red, persistent_lines, draw_time, 0, line_thickness);
					UAirBlueprintLib::DrawPoint(cur_actor->GetWorld(), draw_location, 5, FColor::Red, false, draw_time);

					float radius = distance_to_sensor * FMath::Tan(reflection_opening_angle);
					VectorMath::Quaternionf trace_rotation_quat = VectorMath::toQuaternion(VectorMath::front(), FVectorToVector3r(trace_direction));
					Vector3r y_axis = VectorMath::rotateVector(VectorMath::right(), trace_rotation_quat, true);
					Vector3r z_axis = VectorMath::rotateVector(VectorMath::down(), trace_rotation_quat, true);
					DrawDebugCircle(cur_actor->GetWorld(), draw_location, radius, 128, FColor::Blue, persistent_lines, draw_time, 0u, 1.0f, Vector3rToFVector(y_axis), Vector3rToFVector(z_axis), false);
				}
			}
		}
		else {	

			if (!only_final_reflection) {

				SavePoint(trace_hit_result, trace_direction, signal_attenuation, total_distance, reflection_count, label, ned_transform, pose, points, groundtruth, external, result_uu, save_normal, save_source, source_label);
	
				if (debug_draw_reflected_points) {
					UAirBlueprintLib::DrawPoint(cur_actor->GetWorld(), trace_start_position, 5, FColor::Red, persistent_lines, draw_time);
				}
			}
			else {
				trace_hit_previous = trace_hit_result;
				previous_direction = trace_direction;
			}
		}
	}
}

void UnrealEchoSensor::SavePoint(FHitResult trace_hit_result, FVector direction, float signal_attenuation, float total_distance, float reflection_count, std::string label,
							     const NedTransform* ned_transform, const msr::airlib::Pose& pose, msr::airlib::vector<msr::airlib::real_T>& points, msr::airlib::vector<std::string>& groundtruth,
	                             bool external, bool result_uu, bool save_normal, bool save_source, std::string source_label) {

	if (result_uu) {
		points.emplace_back(trace_hit_result.ImpactPoint.X);
		points.emplace_back(trace_hit_result.ImpactPoint.Y);
		points.emplace_back(trace_hit_result.ImpactPoint.Z);
	}
	else {
		Vector3r point_sensor_frame;
		if (external) {
			point_sensor_frame = ned_transform->toVector3r(trace_hit_result.ImpactPoint, 0.01, true);
		}
		else {
			point_sensor_frame = ned_transform->toLocalNed(trace_hit_result.ImpactPoint);
		}
		point_sensor_frame = VectorMath::transformToBodyFrame(point_sensor_frame, pose, true);

		points.emplace_back(point_sensor_frame.x());
		points.emplace_back(point_sensor_frame.y());
		points.emplace_back(point_sensor_frame.z());
	}

	points.emplace_back(signal_attenuation);
	points.emplace_back(total_distance);
	points.emplace_back(reflection_count);

	if (save_normal) {
		points.emplace_back(direction.X);
		points.emplace_back(direction.Y);
		points.emplace_back(direction.Z);
	}
	groundtruth.emplace_back(label);
	if (save_source)groundtruth.emplace_back(source_label);
}

void UnrealEchoSensor::bounceTrace(FVector &trace_start_position, FVector &trace_direction, float &trace_length, const FHitResult &trace_hit_result,
	float &total_distance, float &signal_attenuation, float attenuation_per_distance, float attenuation_per_reflection, float distance_limit, float attenuation_limit,
	const NedTransform* ned_transform) {

	// Attenuate signal
	float distance_traveled = ned_transform->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));
	applyFreeSpaceLoss(signal_attenuation, total_distance, distance_traveled);
	signal_attenuation += distance_traveled * attenuation_per_distance;
	signal_attenuation += attenuation_per_reflection;

	total_distance += distance_traveled;

	// Reflect signal 
	trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;
	trace_length = ned_transform->fromNed(remainingDistance(signal_attenuation, total_distance, attenuation_limit, distance_limit));
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
	//float sigma = reflection_opening_angle_ / 10 * 3;

	//return -(FMath::Exp(-FMath::Pow(reception_angle, 2) / (2*FMath::Pow(sigma, 2))) - 1) * attenuation_limit_;
	// TODO
	return 0;
}