// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "sensors/MarLocUwb/MarLocUwbSimple.hpp"
#include "Components/StaticMeshComponent.h"
#include "NedTransform.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "AirBlueprintLib.h"
#include "Weather/WeatherLib.h"
#include "map"

// UnrealMarLocUwbSensor implementation that uses Ray Tracing in Unreal.
class UnrealMarLocUwbSensor : public msr::airlib::MarLocUwbSimple {

public:
	typedef msr::airlib::AirSimSettings AirSimSettings;

public:
	UnrealMarLocUwbSensor(const AirSimSettings::MarLocUwbSetting& setting,
		AActor* actor, const NedTransform* ned_transform);

protected:
	//virtual void getPointCloud(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud) override;

	virtual void updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose);

	virtual void pause(const bool is_paused);

	virtual void getLocalPose(msr::airlib::Pose& sensor_pose);

	//virtual void setPointCloud(const msr::airlib::Pose& sensor_pose, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::TTimePoint time_stamp) override;

	void updateUWBRays();
	TArray<msr::airlib::Pose> getBeaconActors();
private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;
	
private:
	AActor* actor_;
	const NedTransform* ned_transform_;
	float saved_clockspeed_;
	msr::airlib::Pose sensor_reference_frame_;
	const msr::airlib::MarLocUwbSimpleParams sensor_params_;
	msr::airlib::vector<msr::airlib::Vector3r> sample_directions_;
	const bool external_;
	std::vector<float> uwbTraceMaxDistances; // in meter

	void sampleSphereCap(int num_points, float opening_angle);
	int traceDirection(FVector trace_start_position, FVector trace_end_position, TArray<msr::airlib::UWBHit> *UWBHitLogfloat, float traceRayCurrentDistance, float traceRayCurrentbounces, float traceRayCurrentSignalStrength, bool drawDebug, FVector trace_origin);
	void bounceTrace(FVector &trace_start_position, FVector &trace_direction, float &trace_length, const FHitResult &trace_hit_result, float &total_distance, float &signal_attenuation);
	float angleBetweenVectors(FVector vector1, FVector vector2);
	float getFreeSpaceLoss(float previous_distance, float added_distance);
	float getReflectionAttenuation(const FHitResult &trace_hit_result);
	float remainingDistance(float signal_attenuation, float total_distance);
	FVector Vector3rToFVector(const Vector3r& input_vector);
	const std::map<EPhysicalSurface, float> material_attenuations_ = {
		{ EPhysicalSurface::SurfaceType_Default,		0.0f },
		{ EPhysicalSurface::SurfaceType1,				0.0f }
	};

	float traceRayMaxDistance; // Meter
	float traceRayMaxBounces;
	float traceRayMinSignalStrength;

	//TArray<AActor*> beacon_actors;
	TArray<msr::airlib::Pose> beacon_poses;

	//TArray<TArray<UWBHit>> UWBHits;
	int maxUWBHits = 5;

	void updateActiveBeacons();
};