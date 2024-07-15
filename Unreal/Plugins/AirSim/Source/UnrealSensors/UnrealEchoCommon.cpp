#include "UnrealEchoCommon.h"
#include "AirBlueprintLib.h"

UnrealEchoCommon::UnrealEchoCommon()
{
}

void UnrealEchoCommon::sampleHorizontalSlice(int num_points, float lower_azimuth_limit, float upper_azimuth_limit, msr::airlib::vector<msr::airlib::Vector3r>& direction_points) {
	num_points = -num_points;

	direction_points.clear();

	float angle_step = FMath::DegreesToRadians((upper_azimuth_limit - lower_azimuth_limit) / (num_points - 1));
	for (auto i = 0; i < num_points; ++i)
	{
		float angle = FMath::DegreesToRadians(lower_azimuth_limit) + i * angle_step;

		float y = FMath::Sin(angle);
		float x = FMath::Cos(angle);
		float z = 0;

		direction_points.emplace_back(Vector3r(x, y, z));
	}
}

void UnrealEchoCommon::sampleSphereCap(int num_points, float lower_azimuth_limit, float upper_azimuth_limit, float lower_elevation_limit, float upper_elevation_limit, msr::airlib::vector<msr::airlib::Vector3r>& direction_points) {
	direction_points.clear();

	// Add point in frontal direction
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
			if (az > lower_azimuth_limit && az < upper_azimuth_limit && el > lower_elevation_limit && el < upper_elevation_limit)
				direction_points.emplace_back(Vector3r(x, y, z));
		}
	}
}

void UnrealEchoCommon::applyFreeSpaceLoss(float& signal_attenuation, float previous_distance, float added_distance) {
	float spread_attenuation;

	if (previous_distance + added_distance < 1) {
		spread_attenuation = 0;
	}
	else {
		if (previous_distance < 1) {
			added_distance = added_distance - (1 - previous_distance);
			previous_distance = 1;
		}
		spread_attenuation = 20 * FMath::LogX(10, previous_distance / (previous_distance + added_distance));
	}

	signal_attenuation += spread_attenuation;
}

float UnrealEchoCommon::remainingDistance(float signal_attenuation, float total_distance, float attenuation_limit, float distance_limit) {
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
}

void UnrealEchoCommon::traceDirection(uint32 current_sample_index, bool use_indexing, FVector trace_start_position, FVector trace_end_position, msr::airlib::vector<msr::airlib::real_T>& points, msr::airlib::vector<std::string>& groundtruth,
	msr::airlib::vector<FVector>& draw_points, const NedTransform* ned_transform, const msr::airlib::Pose& pose, float distance_limit, int reflection_limit, float attenuation_limit, float reflection_distance_limit,
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
	while (total_distance < distance_limit && reflection_count < reflection_limit && signal_attenuation > attenuation_limit) {
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
				SavePoint(current_sample_index, use_indexing, trace_hit_previous, previous_direction, signal_attenuation, total_distance, reflection_count, label, ned_transform, pose, points, groundtruth, external, result_uu, save_normal, save_source, source_label);
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

				SavePoint(current_sample_index, use_indexing, trace_hit_result, trace_direction, signal_attenuation, total_distance, reflection_count, label, ned_transform, pose, points, groundtruth, external, result_uu, save_normal, save_source, source_label);

				if (debug_draw_reflected_points) {
					if (use_indexing)
						draw_points[current_sample_index] = trace_start_position;
					else
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

				SavePoint(current_sample_index, use_indexing, trace_hit_result, trace_direction, signal_attenuation, total_distance, reflection_count, label, ned_transform, pose, points, groundtruth, external, result_uu, save_normal, save_source, source_label);

				if (debug_draw_reflected_points) {
					if (use_indexing)
						draw_points[current_sample_index] = trace_start_position;
					else
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

void UnrealEchoCommon::SavePoint(uint32 current_sample_index, bool use_indexing, FHitResult trace_hit_result, FVector direction, float signal_attenuation, float total_distance, float reflection_count, std::string label,
	const NedTransform* ned_transform, const msr::airlib::Pose& pose, msr::airlib::vector<msr::airlib::real_T>& points, msr::airlib::vector<std::string>& groundtruth,
	bool external, bool result_uu, bool save_normal, bool save_source, std::string source_label) {
	uint32 step_size = 6;
	if (save_normal)
		step_size += 3;
	if (result_uu) {
		if (use_indexing) {
			points[current_sample_index * step_size] = trace_hit_result.ImpactPoint.X;
			points[current_sample_index * step_size + 1] = trace_hit_result.ImpactPoint.Y;
			points[current_sample_index * step_size + 2] = trace_hit_result.ImpactPoint.Z;
		} {
			points.emplace_back(trace_hit_result.ImpactPoint.X);
			points.emplace_back(trace_hit_result.ImpactPoint.Y);
			points.emplace_back(trace_hit_result.ImpactPoint.Z);
		}
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

		if (use_indexing) {
			points[current_sample_index * step_size] = point_sensor_frame.x();
			points[current_sample_index * step_size + 1] = point_sensor_frame.y();
			points[current_sample_index * step_size + 2] = point_sensor_frame.z();
		}
		else {
			points.emplace_back(point_sensor_frame.x());
			points.emplace_back(point_sensor_frame.y());
			points.emplace_back(point_sensor_frame.z());
		}
	}
	if (use_indexing) {
		points[current_sample_index * step_size + 3] = signal_attenuation;
		points[current_sample_index * step_size + 4] = total_distance;
		points[current_sample_index * step_size + 5] = reflection_count;
	}
	else {
		points.emplace_back(signal_attenuation);
		points.emplace_back(total_distance);
		points.emplace_back(reflection_count);
	}

	if (save_normal) {
		if (use_indexing) {
			points[current_sample_index * step_size + 6] = direction.X;
			points[current_sample_index * step_size + 7] = direction.Y;
			points[current_sample_index * step_size + 8] = direction.Z;
		}
		else {
			points.emplace_back(direction.X);
			points.emplace_back(direction.Y);
			points.emplace_back(direction.Z);
		}
	}

	if (save_source) {
		if (use_indexing) {
			groundtruth[current_sample_index * 2] = label;
			groundtruth[current_sample_index * 2 + 1] = source_label;
		}
		else {
			groundtruth.emplace_back(label);
			groundtruth.emplace_back(source_label);
		}
	}
	else {
		if (use_indexing)
			groundtruth[current_sample_index] = label;
		else
			groundtruth.emplace_back(label);
	}
}

void UnrealEchoCommon::bounceTrace(FVector& trace_start_position, FVector& trace_direction, float& trace_length, const FHitResult& trace_hit_result,
	float& total_distance, float& signal_attenuation, float attenuation_per_distance, float attenuation_per_reflection, float distance_limit, float attenuation_limit,
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

FVector UnrealEchoCommon::Vector3rToFVector(const Vector3r& input_vector) {
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

msr::airlib::Vector3r UnrealEchoCommon::FVectorToVector3r(const FVector& input_vector) {
	return Vector3r(input_vector.X, input_vector.Y, -input_vector.Z);
}

float UnrealEchoCommon::angleBetweenVectors(FVector vector1, FVector vector2) {
	// Location relative to origin
	vector1.Normalize();
	vector2.Normalize();

	return FMath::Acos(FVector::DotProduct(vector1, vector2));
}

float UnrealEchoCommon::receptionAttenuation(float reception_angle) {
	//float sigma = reflection_opening_angle_ / 10 * 3;

	//return -(FMath::Exp(-FMath::Pow(reception_angle, 2) / (2*FMath::Pow(sigma, 2))) - 1) * attenuation_limit_;
	// TODO
	return 0;
}