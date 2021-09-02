#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/ArrowComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraActor.h"
#include "Materials/Material.h"
#include "Runtime/Core/Public/PixelFormat.h"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"

#include "LidarCamera.generated.h"


UCLASS()
class AIRSIM_API ALidarCamera : public AActor
{
	GENERATED_BODY()


public:

	typedef msr::airlib::AirSimSettings AirSimSettings;

	ALidarCamera();

	virtual void PostInitializeComponents() override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	void GenerateLidarCoordinates();
	bool SampleRenders(float rotation, float fov, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth,
		               msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final);
	void RotateCamera(float rotation);
	void InitializeSettings(const AirSimSettings::GPULidarSetting& settings);
	bool Update(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth,
		        msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final);

	TArray<float> horizontal_angles_;
	TArray<float> vertical_angles_;
	TArray<FVector> angle_to_xyz_lut_;
	uint32 current_horizontal_angle_index_ = 0;
	float current_angle_ = 0;
	float previous_rotation_ = 0;
	int32 target_fov_ = 0;
	float horizontal_delta_ = 0;
	float vertical_delta_ = 0;
	float sum_rotation_ = 0;
	std::string material_list_file_ = "";
	std::map<uint8, float> material_map_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		UArrowComponent* arrow_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		USceneCaptureComponent2D* capture_2D_depth_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		USceneCaptureComponent2D* capture_2D_segmentation_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		USceneCaptureComponent2D* capture_2D_intensity_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		UTextureRenderTarget2D* render_target_2D_depth_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		UTextureRenderTarget2D* render_target_2D_segmentation_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		UTextureRenderTarget2D* render_target_2D_intensity_;

	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		uint32 num_of_lasers_ = 16;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		float frequency_ = 10;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		uint32 measurement_per_cycle_ = 1024;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		float horizontal_min_ = 0;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		float horizontal_max_ = 360;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		float vertical_min_ = -17;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		float vertical_max_ = 17;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		uint32 resolution_ = 1024;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		bool draw_debug_ = true;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		uint32 draw_mode_ = 0;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		bool ground_truth_ = false;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		float max_range_ = 100;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		bool ignore_marked_ = false;
	UPROPERTY(EditAnywhere, Category = "Lidar Camera")
		bool generate_intensity_ = false;

private: //members
 
private: //methods

};

