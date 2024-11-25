// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "RHIGPUReadback.h"
#include "common/WorkerThread.hpp"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/ArrowComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkinnedMeshComponent.h"
#include "Camera/CameraActor.h"
#include "Materials/Material.h"
#include "Runtime/Core/Public/PixelFormat.h"
#include "TextureResource.h"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "sensors/lidar/GPULidarSimple.hpp"
#include "common/Common.hpp"
#include <random>

#include "LidarCamera.generated.h"


//struct FLidarCameraRenderRequest {
//	FIntPoint ImageSize;
//	FRHIGPUTextureReadback Readback;
//	FRenderCommandFence RenderFence;
//
//	FLidarCameraRenderRequest(
//		const FIntPoint& ImageSize,
//		const FRHIGPUTextureReadback& Readback) :
//		ImageSize(ImageSize),
//		Readback(Readback) {}
//};

UCLASS()
class AIRSIM_API ALidarCamera : public AActor
{
	GENERATED_BODY()

public:

	ALidarCamera();

	virtual void PostInitializeComponents() override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	void updateInstanceSegmentationAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList);
	void updateAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList);

	void InitializeSettingsFromAirSim(const msr::airlib::GPULidarSimpleParams& settings);
	void InitializeSensor();
	bool Update(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Render")
		int32 resolution_ = 1024;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float sensor_rotation_frequency_ = 10;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		int32 horizontal_samples_ = 1024;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float horizontal_fov_min_ = 0;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float horizontal_fov_max_ = 360;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		int32 num_lasers_ = 16;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float vertical_fov_min_ = -17;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float vertical_fov_max_ = 17;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float max_range_ = 100;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		bool generate_groundtruth_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		bool instance_segmentation_ = true;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		FString annotation_name_ = "";

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		bool generate_intensity_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float max_range_lambertian_percentage_ = 80;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float rain_max_intensity_ = 70;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float rain_constant_a_ = 0.01;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float rain_constant_b_ = 0.6;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		bool ignore_marked_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		bool generate_distance_noise_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Sensor")
		float distance_noise_scale_ = 0;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Debug")
		bool draw_debug_ = true;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "LidarCamera|Debug")
		int32 debug_draw_mode_ = 0;

private: 

	void GenerateLidarCoordinates();
	void RotateCamera(float sensor_rotation_angle);
	bool SampleRenders(float sensor_rotation_angle, float fov, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final);
	//void ExecuteScanTask();
	std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;


	UPROPERTY()
		USceneCaptureComponent2D* capture_2D_depth_;

	UPROPERTY()
		USceneCaptureComponent2D* capture_2D_segmentation_;

	UPROPERTY()
		USceneCaptureComponent2D* capture_2D_intensity_;

	UPROPERTY()
		UTextureRenderTarget2D* render_target_2D_depth_;

	UPROPERTY()
		UTextureRenderTarget2D* render_target_2D_segmentation_;

	UPROPERTY()
		UTextureRenderTarget2D* render_target_2D_intensity_;

	UPROPERTY()
		UArrowComponent* arrow_;

	TArray<float> h_angles_;
	TArray<float> h_angles_atan2_;
	TArray<float> v_angles_;
	float h_delta_angle_ = 0;
	float v_delta_angle_ = 0;
	TArray<FVector> polar_to_cartesian_lut_;
	int32 h_cur_atan2_index_ = -1;
	float sensor_sum_rotation_angle_ = 0;
	float sensor_cur_angle_ = 0;
	float sensor_prev_rotation_angle_ = 0;
	int32 target_fov_ = 0;
	std::string material_list_file_ = "";
	std::map<uint8, float> material_map_;
	std::mt19937 gen_;
	std::normal_distribution<float> dist_;
	static int32 unique_colors_[765];
	bool used_by_airsim_ = false;
	bool initialized = false;
	int32 wait_frames_ = 100;
	int32 waited_frames_ = 0;
	float hfov_ = 0;
	float completed_hfov_ = 0;
	bool reset_hfov_ = false;

	//bool saved_DisableWorldRendering_ = false;
	//UGameViewportClient* game_viewport_;
	//FDelegateHandle end_draw_handle_;
	//TArray<FColor> buffer_2D_depth_;
	//TArray<FColor> buffer_2D_segmentation_;
	//TArray<FColor> buffer_2D_intensity_;
	//bool first_frame_ = true;
};

