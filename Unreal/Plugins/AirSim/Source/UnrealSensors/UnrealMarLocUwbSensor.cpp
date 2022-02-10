// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#define DRAWDEBUGLINES

#include "UnrealMarLocUwbSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "CoreMinimal.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include <PxScene.h>
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Beacons/UWBBeacon.h"
#include <typeinfo>
#include <mutex>
#include "common/CommonStructs.hpp"

using std::fill_n;
std::mutex mtx;
// ctor
UnrealMarLocUwbSensor::UnrealMarLocUwbSensor(const AirSimSettings::MarLocUwbSetting& setting, AActor* actor, const NedTransform* ned_transform)
	: MarLocUwbSimple(setting), actor_(actor), ned_transform_(ned_transform), saved_clockspeed_(1), sensor_params_(getParams()), external_(getParams().external)
{
	// Initialize UWB properties
	//configureUwbProperties();

	// Initialize the trace directions
	sampleSphereCap(sensor_params_.number_of_traces, sensor_params_.sensor_opening_angle);

	traceRayMaxDistance = 200;
	traceRayMaxBounces = 5;
	traceRayMinSignalStrength = 0.001;

	for (TActorIterator<AUWBBeacon> It(actor_->GetWorld()); It; ++It)
	{
		beacon_actors.Add(*It);
	}
}

// Set MarLocUwbSensor object in correct pose in physical world
void UnrealMarLocUwbSensor::updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose)
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
		DrawDebugPoint(actor_->GetWorld(), sensor_position, 5, FColor::Black, false, 0.3);
		FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		DrawDebugCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, 0.3, 10);
	}
}

// Pause Unreal simulation
void UnrealMarLocUwbSensor::pause(const bool is_paused) {
	if (is_paused) {
		saved_clockspeed_ = UAirBlueprintLib::getUnrealClockSpeed(actor_);
		UAirBlueprintLib::setUnrealClockSpeed(actor_, 0);
	}
	else {
		UAirBlueprintLib::setUnrealClockSpeed(actor_, saved_clockspeed_);
	}
} 


// Get MarLocUwbSensor pose in Local NED
void UnrealMarLocUwbSensor::getLocalPose(msr::airlib::Pose& sensor_pose)
{
	FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1)); ;
	sensor_pose = ned_transform_->toLocalNed(FTransform(sensor_direction.Rotation(), ned_transform_->toFVector(sensor_reference_frame_.position, 100, true), FVector(1, 1, 1)));
}

/*void UnrealMarLocUwbSensor::getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
	// Set the physical MarLocUwbSensor mesh in the correct location in the world
	updatePose(sensor_pose, vehicle_pose);

	point_cloud.clear();
	Vector3r random_vector = ned_transform_->toVector3r(FMath::RandPointInBox(FBox(FVector(-1, -1, -1), FVector(1, 1, 1))), 1.0f, true);
	point_cloud.emplace_back(random_vector.x());
	point_cloud.emplace_back(random_vector.y());
	point_cloud.emplace_back(random_vector.z());
}*/

/*void UnrealMarLocUwbSensor::setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp) {
	// TODO consume point cloud (+ draw_time_)?

	const int DATA_PER_POINT = 5;
	for (int point_count = 0; point_count < point_cloud.size(); point_count += DATA_PER_POINT) {
		Vector3r point_local = Vector3r(point_cloud[point_count], point_cloud[point_count + 1], point_cloud[point_count + 2]);
		Vector3r point_global1 = VectorMath::transformToWorldFrame(point_local, sensor_pose, true);
		FVector point_global = ned_transform_->fromLocalNed(point_global1);

		DrawDebugPoint(actor_->GetWorld(), point_global, 10, FColor::Orange, false, 1.05f / sensor_params_.measurement_frequency);
	}
}*/

void UnrealMarLocUwbSensor::updateUWBRays() {
	const GroundTruth& ground_truth = getGroundTruth();
	
	Vector3r sensorBase_local = Vector3r(sensor_reference_frame_.position);
	FVector sensorBase_global = ned_transform_->fromLocalNed(sensorBase_local);

	//actor_->GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();

	// Clear oldest UWB Hits 
	while (beaconsActive_.IsValidIndex(maxUWBHits)) {
		beaconsActive_.RemoveAt(0);
	}
	// Create new log record for newest UWB measurements
	TArray<UWBHit> UWBHitLog;

	ParallelFor(sample_directions_.size(), [&](int32 direction_count) {
	//for (int32 direction_count = 0; direction_count< sample_directions_.size(); direction_count++){
		Vector3r sample_direction = sample_directions_[direction_count];

		//FVector trace_direction = ned_transform_->toFVector(VectorMath::rotateVector(sample_direction, sensor_reference_frame_.orientation, 1), 1.0f, true); 

		msr::airlib::Quaternionr sensorOrientationQuat = sensor_reference_frame_.orientation;
		float roll, pitch, yaw;

		msr::airlib::VectorMath::toEulerianAngle(sensorOrientationQuat, roll, pitch, yaw);
		FRotator sensorOrientationEulerDegree = FRotator(FMath::RadiansToDegrees(pitch), FMath::RadiansToDegrees(yaw), FMath::RadiansToDegrees(roll));

		FVector lineEnd = uwbTraceMaxDistances[direction_count] * FVector(sample_direction[0], sample_direction[1], sample_direction[2]);
		lineEnd = sensorOrientationEulerDegree.RotateVector(lineEnd);
		lineEnd += sensorBase_global;
		//DrawDebugLine(actor_->GetWorld(), sensorBase_global, lineEnd, FColor::Red, false, 1);

		traceDirection(sensorBase_global, lineEnd, &UWBHitLog, 0, 0, 1, 0);
		//int traceDir = 0;
		//if (direction_count <= 10) {
		//traceDir = traceDirection(sensorBase_global, lineEnd, &UWBHitLog, 0, 0, 1, 0);
			/*if (traceDir) {
				UAirBlueprintLib::LogMessageString("Drawing ", "a line", LogDebugLevel::Informational);
			}*/
		//}
	});
	//}
	//actor_->GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();
	//UWBHits.Add(UWBHitLog);
	beaconsActive_.Add(UWBHitLog);
}

// Thanks Girmi
void UnrealMarLocUwbSensor::sampleSphereCap(int num_points, float opening_angle) {
	sample_directions_.clear();

	uwbTraceMaxDistances.clear();

	// Add point in frontal directionl
	sample_directions_.emplace(sample_directions_.begin(), Vector3r(1, 0, 0));
	uwbTraceMaxDistances.push_back(10000);

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
		if (sample_directions_.size() == num_points)
		{
			return;
		}
		else if (x >= x_limit)
		{
			float z = FMath::Sin(phi) * r;
			sample_directions_.emplace_back(Vector3r(x, y, z));

			// Calculate max distance (and probably attentuation and such)
			Vector3r center = Vector3r(1, 0, 0);
			float angleToCenter = FMath::RadiansToDegrees(VectorMath::angleBetween(center, Vector3r(x, y, z), true));
			angleToCenter += 10000;
			uwbTraceMaxDistances.push_back(angleToCenter);
		}
	}
}

int UnrealMarLocUwbSensor::traceDirection(FVector trace_start_position, FVector trace_end_position, TArray<UWBHit> *UWBHitLog, float traceRayCurrentDistance, float traceRayCurrentbounces, float traceRayCurrentSignalStrength, bool drawDebug) {
	FHitResult trace_hit_result;
	bool trace_hit;
	TArray<AActor*> ignore_actors_;

	if (traceRayCurrentDistance < traceRayMaxDistance) {
		if (traceRayCurrentbounces < traceRayMaxBounces) {
			if (traceRayCurrentSignalStrength > traceRayMinSignalStrength) {
				trace_hit_result = FHitResult(ForceInit);
				trace_hit = UAirBlueprintLib::GetObstacleAdv(actor_, trace_start_position, trace_end_position, trace_hit_result, ignore_actors_, ECC_Visibility, true, true);

				// Stop if nothing was hit to reflect off
				if (!trace_hit) {
					if (drawDebug) {
						DrawDebugLine(actor_->GetWorld(), trace_start_position, trace_end_position, FColor::Red, false, 0.1);
					}
					return 1;
				}

				// Bounce trace
				FVector trace_direction;
				float trace_length;
				FVector trace_start_original = trace_start_position;
				bounceTrace(trace_start_position, trace_direction, trace_length, trace_hit_result, traceRayCurrentDistance, traceRayCurrentSignalStrength);
				trace_end_position = trace_start_position + trace_direction * trace_length;
				traceRayCurrentbounces += 1;

				// If beacon was hit
				if (trace_hit_result.Actor != nullptr) {
					if (trace_hit_result.Actor->IsA(AUWBBeacon::StaticClass())) {
						mtx.lock();
						FString tmpName, tmpId;
						//UAirBlueprintLib::LogMessageString("Calculating the name", "as BEACON", LogDebugLevel::Informational, 1);

						//std::string aaa (TCHAR_TO_UTF8(*trace_hit_result.Actor->GetFName().ToString().Split(TEXT("_"), &tmpName, &tmpId)));
						trace_hit_result.Actor->GetFName().ToString().Split(TEXT("_"), &tmpName, &tmpId);
						int tmpRssi = (int)traceRayCurrentSignalStrength;
						FVector beaconPos = trace_hit_result.Actor->GetActorLocation();
						UWBHit thisHit = { FCString::Atoi(*tmpId),  tmpRssi, beaconPos[0], beaconPos[1] , beaconPos[2] };
						UWBHitLog->Add(thisHit);
						mtx.unlock();
					}
				}

				if (drawDebug) {
					DrawDebugLine(actor_->GetWorld(), trace_start_original, trace_start_position, FColor::Red, false, 0.1);
				}

				/*if (isnan(traceRayCurrentDistance)) {
					traceRayCurrentDistance = -1;
				}
				if (isnan(traceRayCurrentbounces)) {
					traceRayCurrentbounces = -1;
				}
				if (isnan(traceRayCurrentSignalStrength)) {
					traceRayCurrentbounces = -1;
				}*/
					
				/*try {
					UAirBlueprintLib::LogMessageString("Calculating the current distance", std::to_string(traceRayCurrentDistance), LogDebugLevel::Informational);
					UAirBlueprintLib::LogMessageString("Calculating the current bounces", std::to_string(traceRayCurrentbounces), LogDebugLevel::Informational);
					UAirBlueprintLib::LogMessageString("Calculating the current signal strength", std::to_string(traceRayCurrentSignalStrength), LogDebugLevel::Informational);
				}
				catch (const std::exception& e) {
					UAirBlueprintLib::LogMessageString("Calculating the current distance", (e.what()), LogDebugLevel::Informational);
				}*/

				return(traceDirection(trace_start_position, trace_end_position, UWBHitLog, traceRayCurrentDistance, traceRayCurrentbounces, traceRayCurrentSignalStrength, drawDebug));

				/*if (drawDebug) {
					DrawDebugLine(actor_->GetWorld(), trace_start_position, trace_end_position, FColor::Green, false, 0.1);
				}*/

	//			FVector sensor_position = ned_transform_->fromLocalNed(sensor_reference_frame_.position);
	//			float distance_to_sensor = FVector::Distance(trace_hit_result.ImpactPoint, sensor_position);
	//			FVector direction_to_sensor = sensor_position - trace_hit_result.ImpactPoint;
	//			float receiving_angle = angleBetweenVectors(direction_to_sensor, trace_direction);

	//			received_attenuation = signal_attenuation + getFreeSpaceLoss(total_distance, ned_transform_->toNed(distance_to_sensor)) + (ned_transform_->toNed(distance_to_sensor) * attenuation_atmospheric_);
	//			 Stop if signal can't travel far enough to return (distance or attenuation)
	//			/*if (distance_to_sensor > trace_length || received_attenuation < attenuation_limit_) {
	//				return;
	//			}*/

	//			if (sensor_params_.draw_reflected_paths) trace_path.Emplace(trace_hit_result.ImpactPoint);

	//			 If impact point is in near field, register as reflection
	//			/*bool reflectionInNearField = (distance_to_sensor <= near_field_ && reflection_count > 0);
	//			bool reflectionFromVehicle = (trace_hit_result.Actor.Get()->GetName().Compare(FString(TEXT("airsimvehicle"))) == 0);  // Compare returns 0 if match
	//			if (reflectionInNearField && !reflectionFromVehicle)
	//			{
	//				echo_received = true;

	//				 DRAW DEBUG
	//				/*if (sensor_params_.draw_near_field_reflections) {
	//					point_cloud_mutex_.Lock();
	//					DrawDebugPoint(actor_->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Purple, false, draw_time_);
	//					point_cloud_mutex_.Unlock();
	//				}
	//			}
	//			else */
	//			/*if (receiving_angle < reflection_opening_angle_)  // Check if sensor lies in opening angle
	//			{
	//				hit_result_temp = FHitResult(ForceInit);
	//				trace_hit = UAirBlueprintLib::GetObstacleAdv(actor_, trace_hit_result.ImpactPoint, sensor_position, hit_result_temp, ignore_actors_, ECC_Visibility, true);
	//				if (trace_hit) {  // Hit = no clear LoS to sensor
	//					continue;
	//				}

	//				echo_received = true;

	//				 DRAW DEBUG
	//				/*if (sensor_params_.draw_reflected_points) {
	//					point_cloud_mutex_.Lock();
	//					DrawDebugPoint(actor_->GetWorld(), trace_hit_result.ImpactPoint, 5, FColor::Red, false, draw_time_);
	//					point_cloud_mutex_.Unlock();
	//				}*/
	//				/*if (sensor_params_.draw_reflected_paths) {
	//					point_cloud_mutex_.Lock();
	//					DrawDebugLine(actor_->GetWorld(), sensor_position, trace_path[0], FColor::Red, false, draw_time_, 0, line_thinkness_);
	//					for (int trace_count = 0; trace_count < trace_path.Num() - 1; trace_count++)
	//					{
	//						DrawDebugLine(actor_->GetWorld(), trace_path[trace_count], trace_path[trace_count + 1], FColor::Red, false, draw_time_, 0, line_thinkness_);
	//					}
	//					DrawDebugLine(actor_->GetWorld(), trace_path.Last(), sensor_position, FColor::Red, false, draw_time_, 0, line_thinkness_);
	//					point_cloud_mutex_.Unlock();
	//				}*/
	//				/*if (sensor_params_.draw_reflected_lines) {
	//					point_cloud_mutex_.Lock();
	//					FVector draw_location = trace_hit_result.ImpactPoint + trace_direction * distance_to_sensor;

	//					DrawDebugLine(actor_->GetWorld(), trace_hit_result.ImpactPoint, draw_location, FColor::Red, false, draw_time_, 0, line_thinkness_);
	//					DrawDebugPoint(actor_->GetWorld(), draw_location, 5, FColor::Red, false, draw_time_);

	//					float radius = distance_to_sensor * FMath::Tan(reflection_opening_angle_);
	//					VectorMath::Quaternionf trace_rotation_quat = VectorMath::toQuaternion(VectorMath::front(), ned_transform_->toVector3r(trace_direction, 1.0f, true));
	//					Vector3r y_axis = VectorMath::rotateVector(VectorMath::right(), trace_rotation_quat, true);
	//					Vector3r z_axis = VectorMath::rotateVector(VectorMath::down(), trace_rotation_quat, true);
	//					DrawDebugCircle(actor_->GetWorld(), draw_location, radius, 128, FColor::Blue, false, draw_time_, 0u, 1.0f, ned_transform_->toFVector(y_axis, 1.0f, true), ned_transform_->toFVector(z_axis, 1.0f, true), false);
	//					point_cloud_mutex_.Unlock();
	//				}
	//			}*/
	//		}
	//	}

	//	/*if (echo_received)
	//	{
	//		Vector3r point_sensor_frame = ned_transform_->toLocalNed(trace_hit_result.ImpactPoint);
	//		point_sensor_frame = VectorMath::transformToBodyFrame(point_sensor_frame, sensor_reference_frame_, true);

	//		/*point_cloud_mutex_.Lock();
	//		point_cloud.emplace_back(point_sensor_frame.x());
	//		point_cloud.emplace_back(point_sensor_frame.y());
	//		point_cloud.emplace_back(point_sensor_frame.z());
	//		point_cloud.emplace_back(received_attenuation);
	//		point_cloud.emplace_back(total_distance);
	//		point_cloud_mutex_.Unlock();
	//	}*/
	//}
			}
		}
	}
	return 1;
}

void UnrealMarLocUwbSensor::bounceTrace(FVector &trace_start_position, FVector &trace_direction, float &trace_length, const FHitResult &trace_hit_result, float &total_distance, float &signal_attenuation) {

	float distance_traveled = ned_transform_->toNed(FVector::Distance(trace_start_position, trace_hit_result.ImpactPoint));

	//TMP
	float attenuation_atmospheric_ = 50;

	// Attenuate signal
	signal_attenuation += getFreeSpaceLoss(total_distance, distance_traveled);
	signal_attenuation += (distance_traveled * attenuation_atmospheric_);
	signal_attenuation += getReflectionAttenuation(trace_hit_result);

	total_distance += distance_traveled;

	// Reflect signal 
	trace_direction = (trace_hit_result.ImpactPoint - trace_start_position).MirrorByVector(trace_hit_result.ImpactNormal);
	trace_direction.Normalize();
	trace_start_position = trace_hit_result.ImpactPoint;
	trace_length = ned_transform_->fromNed(remainingDistance(signal_attenuation, total_distance));
}

float UnrealMarLocUwbSensor::angleBetweenVectors(FVector vector1, FVector vector2) {
	// Location relative to origin
	vector1.Normalize();
	vector2.Normalize();

	return FMath::Acos(FVector::DotProduct(vector1, vector2));
}

float UnrealMarLocUwbSensor::getFreeSpaceLoss(float previous_distance, float added_distance) {
	float spread_attenuation;
	float total_distance = previous_distance + added_distance;

	//TMP
	float wavelength_ = 0.1;

	float attenuation_previous = -20 * FMath::LogX(10, 4 * M_PI * previous_distance / wavelength_);
	attenuation_previous = (attenuation_previous > 0) ? 0 : attenuation_previous;

	float attenuation_total = -20 * FMath::LogX(10, 4 * M_PI * total_distance / wavelength_);
	attenuation_total = (attenuation_total > 0) ? 0 : attenuation_total;

	spread_attenuation = attenuation_total - attenuation_previous;

	return spread_attenuation;
}

float UnrealMarLocUwbSensor::getReflectionAttenuation(const FHitResult &trace_hit_result) {
	float materialAttenuation = 0.0f;

	if (trace_hit_result.PhysMaterial.IsValid()) {
		EPhysicalSurface currentSurfaceType = UPhysicalMaterial::DetermineSurfaceType(trace_hit_result.PhysMaterial.Get());

		for (std::map<EPhysicalSurface, float>::const_iterator it = material_attenuations_.begin(); it != material_attenuations_.end(); ++it) {
			if (currentSurfaceType == it->first)
			{
				materialAttenuation = it->second;
			}
		}
	}

	return materialAttenuation;
}

float UnrealMarLocUwbSensor::remainingDistance(float signal_attenuation, float total_distance) {
	//	float distanceFSPL;
	//	float distanceAtmospheric = FMath::Pow(10, (signal_attenuation - attenuation_limit_) / 20) - 1;

	//TMP
	float distance_limit_ = 500;

	float distanceLength = distance_limit_ - total_distance;

	return FMath::Max(distanceLength, 0.0f);
}

FVector UnrealMarLocUwbSensor::Vector3rToFVector(const Vector3r& input_vector) {
	return FVector(input_vector.x(), input_vector.y(), -input_vector.z());
}

void UnrealMarLocUwbSensor::updateActiveBeacons() {
	beaconsActive_.Empty();
}