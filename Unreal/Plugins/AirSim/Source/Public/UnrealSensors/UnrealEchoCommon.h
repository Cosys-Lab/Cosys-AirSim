#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "common/Common.hpp"
#include "Components/StaticMeshComponent.h"
#include "NedTransform.h"
#include "AirBlueprintLib.h"

class AIRSIM_API UnrealEchoCommon
{
public:
    struct EchoPoint {
        FVector point;
        FVector direction;
        std::string reflection_object;
        std::string source_object;
        float total_distance;
        float total_attenuation;
        float reflections;
    };

	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;

public:
	UnrealEchoCommon();

	static void sampleHorizontalSlice(int num_points, float lower_azimuth_limit, float upper_azimuth_limit, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud);
	static void sampleSphereCap(int num_points, float lower_azimuth_limit, float upper_azimuth_limit, float lower_elevation_limit, float upper_elevation_limit, msr::airlib::vector<msr::airlib::Vector3r>& point_cloud);
	static void applyFreeSpaceLoss(float& signal_attenuation, float previous_distance, float added_distance);
	static float remainingDistance(float signal_attenuation, float total_distance, float attenuation_limit, float distance_limit);
	static void traceDirection(uint32 current_sample_index, bool use_indexing, FVector trace_start_position, FVector trace_end_position, msr::airlib::vector<msr::airlib::real_T>& points, msr::airlib::vector<std::string>& groundtruth, msr::airlib::vector<FVector>& draw_points, const NedTransform* ned_transform, const msr::airlib::Pose& pose,
		float distance_limit, int reflection_limit, float attenuation_limit, float reflection_distance_limit, float reflection_opening_angle,
		float attenuation_per_distance, float attenuation_per_reflection, TArray<AActor*> ignore_actors, AActor* cur_actor, bool external, bool result_uu,
		float draw_time, float line_thickness, bool debug_draw_reflected_paths = false, bool debug_draw_bounce_lines = false, bool debug_draw_initial_points = false,
		bool debug_draw_reflected_points = false, bool debug_draw_reflected_lines = false, bool check_return = true, bool save_normal = false, bool save_source = false, bool only_final_reflection = false, std::string source_label = "");
	static void SavePoint(uint32 current_sample_index, bool use_indexing, FHitResult trace_hit_result, FVector direction, float signal_attenuation, float total_distance, float reflection_count, std::string label,
		const NedTransform* ned_transform, const msr::airlib::Pose& pose, msr::airlib::vector<msr::airlib::real_T>& points, msr::airlib::vector<std::string>& groundtruth,
		bool external, bool result_uu, bool save_normal, bool save_source, std::string source_label = "");
	static void bounceTrace(FVector& trace_start_position, FVector& trace_direction, float& trace_length, const FHitResult& trace_hit_result, float& total_distance,
		float& signal_attenuation, float attenuation_per_distance, float attenuation_per_reflection, float distance_limit, float attenuation_limit, const NedTransform* ned_transform);
	static FVector Vector3rToFVector(const Vector3r& input_vector);
	static Vector3r FVectorToVector3r(const FVector& input_vector);
	static float angleBetweenVectors(FVector vector1, FVector vector2);
	static float receptionAttenuation(float reception_angle);
};
