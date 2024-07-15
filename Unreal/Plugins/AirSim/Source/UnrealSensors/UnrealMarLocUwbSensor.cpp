// Developed by Cosys-Lab, University of Antwerp

#define DRAWDEBUGLINES

#include "UnrealMarLocUwbSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "CoreMinimal.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Beacons/UWBBeacon.h"
#include <typeinfo>
#include <mutex>
#include "Engine/Engine.h"
#include "common/CommonStructs.hpp"

using std::fill_n;
std::mutex mtx;
// ctor
UnrealMarLocUwbSensor::UnrealMarLocUwbSensor(const AirSimSettings::MarLocUwbSetting& setting, AActor* actor, const NedTransform* ned_transform)
	: MarLocUwbSimple(setting), actor_(actor), ned_transform_(ned_transform), saved_clockspeed_(1), sensor_params_(getParams()), external_(getParams().external)
{
	// Initialize the trace directions
	sampleSphereCap(sensor_params_.number_of_traces, sensor_params_.sensor_opening_angle);

	traceRayMaxDistance = 200;
	traceRayMaxBounces = 5;
	traceRayMinSignalStrength = 0.001;

	for (TActorIterator<AUWBBeacon> It(actor_->GetWorld()); It; ++It)
	{
		//FVector startPos = ned_transform->getLocalOffset();
		FVector startPos = ned_transform->getGlobalOffset();

		msr::airlib::Pose posi;
		FVector actorLoc = It->GetActorLocation();
		actorLoc -= startPos;
		posi.position = Eigen::Vector3f(actorLoc.X, actorLoc.Y, actorLoc.Z);
		//posi.orientation = It->GetActorRotation();
		beacon_poses.Add(posi);
		//beacon_actors.Add(*It);
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
		UAirBlueprintLib::DrawPoint(actor_->GetWorld(), sensor_position, 5, FColor::Black, false, 0.3);
		FVector sensor_direction = Vector3rToFVector(VectorMath::rotateVector(VectorMath::front(), sensor_reference_frame_.orientation, 1));
		UAirBlueprintLib::DrawCoordinateSystem(actor_->GetWorld(), sensor_position, sensor_direction.Rotation(), 25, false, 0.3, 10);
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

TArray<msr::airlib::Pose> UnrealMarLocUwbSensor::getBeaconActors() {
	return beacon_poses;
}

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
	TArray<msr::airlib::UWBHit> UWBHitLog;

	//FVector lineEnd = uwbTraceMaxDistances[direction_count] * FVector(sample_direction[0], sample_direction[1], sample_direction[2]);
	//UAirBlueprintLib::DrawLine(actor_->GetWorld(), sensorBase_global, lineEnd, FColor::Red, false, 1);

	//ParallelFor(sample_directions_.size(), [&](int32 direction_count) {
	for (int32 direction_count = 0; direction_count< sample_directions_.size(); direction_count++){
		Vector3r sample_direction = sample_directions_[direction_count];

		//FVector trace_direction = ned_transform_->toFVector(VectorMath::rotateVector(sample_direction, sensor_reference_frame_.orientation, 1), 1.0f, true); 

		msr::airlib::Quaternionr sensorOrientationQuat = sensor_reference_frame_.orientation;
		float roll, pitch, yaw;

		msr::airlib::VectorMath::toEulerianAngle(sensorOrientationQuat, roll, pitch, yaw);
		FRotator sensorOrientationEulerDegree = FRotator(FMath::RadiansToDegrees(pitch), FMath::RadiansToDegrees(yaw), FMath::RadiansToDegrees(roll));

		FVector lineEnd = uwbTraceMaxDistances[direction_count] * FVector(sample_direction[0], sample_direction[1], sample_direction[2]);
		lineEnd = sensorOrientationEulerDegree.RotateVector(lineEnd);
		lineEnd += sensorBase_global;
		//UAirBlueprintLib::DrawLine(actor_->GetWorld(), sensorBase_global, lineEnd, FColor::Red, false, 1);

		// Trace ray, bounce and add to hitlog if beacon was hit
		traceDirection(sensorBase_global, lineEnd, &UWBHitLog, 0, 0, 1, 0, sensorBase_global);
	//});
	}
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

int UnrealMarLocUwbSensor::traceDirection(FVector trace_start_position, FVector trace_end_position, TArray<msr::airlib::UWBHit> *UWBHitLog, float traceRayCurrentDistance, float traceRayCurrentbounces, float traceRayCurrentSignalStrength, bool drawDebug, FVector trace_origin) {
	FHitResult trace_hit_result;
	bool trace_hit;
	TArray<AActor*> ignore_actors_;

	FVector startPos = this->ned_transform_->getGlobalOffset();

	if (traceRayCurrentDistance < traceRayMaxDistance) {
		if (traceRayCurrentbounces < traceRayMaxBounces) {
			if (traceRayCurrentSignalStrength > traceRayMinSignalStrength) {
				trace_hit_result = FHitResult(ForceInit);
				trace_hit = UAirBlueprintLib::GetObstacleAdv(actor_, trace_start_position, trace_end_position, trace_hit_result, ignore_actors_, ECC_Visibility, true, true);

				// Stop if nothing was hit to reflect off
				if (!trace_hit) {
					if (drawDebug) {
						UAirBlueprintLib::DrawLine(actor_->GetWorld(), trace_start_position, trace_end_position, FColor::Red, false, 0.1);
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
				if (trace_hit_result.GetActor() != nullptr) {
					//if ((trace_hit_result.Actor->GetName().Len() >= 9) && (trace_hit_result.Actor->GetName().Left(9) == "uwbBeacon")) {
					if (trace_hit_result.GetActor()->IsA(AUWBBeacon::StaticClass())) {
						mtx.lock();
						auto hitActor = trace_hit_result.GetActor();
						//FString tmpName, tmpId;
						//FString tmpName;
						//UAirBlueprintLib::LogMessageString("Calculating the name", "as BEACON", LogDebugLevel::Informational, 1);

						//std::string aaa (TCHAR_TO_UTF8(*trace_hit_result.Actor->GetFName().ToString().Split(TEXT("_"), &tmpName, &tmpId)));
						//trace_hit_result.Actor->GetFName().ToString().Split(TEXT("_"), &tmpName, &tmpId);
						//tmpName = trace_hit_result.Actor->GetFName().ToString();
						int tmpRssi = (int)traceRayCurrentSignalStrength;
						FVector beaconPos = trace_hit_result.GetActor()->GetActorLocation() - startPos;

						//FVector beaconPos = trace_hit_result.Actor->
						//UWBHit thisHit = { FCString::Atoi(*tmpId),  tmpRssi, beaconPos[0], beaconPos[1] , beaconPos[2] };
						msr::airlib::UWBHit thisHit;
						
						thisHit.beaconID = TCHAR_TO_UTF8(*hitActor->GetName());
						
						thisHit.rssi = tmpRssi;
						thisHit.beaconPosX = beaconPos[0];
						thisHit.beaconPosY = beaconPos[1];
						thisHit.beaconPosZ = beaconPos[2];

						thisHit.distance = sqrt(pow(beaconPos[0] - trace_origin.X, 2) + pow(beaconPos[1] - trace_origin.Y, 2) + pow(beaconPos[2] - trace_origin.Z, 2)) / 100;
						UWBHitLog->Add(thisHit);
						mtx.unlock();
					}
				}

				if (drawDebug) {
					UAirBlueprintLib::DrawLine(actor_->GetWorld(), trace_start_original, trace_start_position, FColor::Red, false, 0.1);
				}

				return(traceDirection(trace_start_position, trace_end_position, UWBHitLog, traceRayCurrentDistance, traceRayCurrentbounces, traceRayCurrentSignalStrength, drawDebug, trace_origin));
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