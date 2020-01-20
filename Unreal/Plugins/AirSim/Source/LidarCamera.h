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

	int counter = 0;

	void GenerateLidarCoordinates();
	void SampleRender(float rotation, float fov, msr::airlib::vector<msr::airlib::real_T>& point_cloud);
	void RotateCamera(float rotation);
	void InitializeSettings(const AirSimSettings::GPULidarSetting& settings);
	void Update(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud);

	TArray<float> horizontal_angles_;
	TArray<float> vertical_angles_;
	TArray<FVector> angle_to_xyz_lut_;

	float current_angle_ = 0;
	float previous_rotation_ = 0;
	float target_fov_ = 0;
	float horizontal_delta_ = 0;
	float vertical_delta_ = 0;
	float sum_rotation_ = 0;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		UArrowComponent* arrow_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		USceneCaptureComponent2D* capture2D_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		UTextureRenderTarget2D* render_target2D_;

	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		uint32 num_of_lasers_ = 64;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		float frequency_ = 10;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		uint32 measurement_per_cycle_ = 2048;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		float horizontal_min_ = 0;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		float horizontal_max_ = 360;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		float vertical_min_ = -16.6;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		float vertical_max_ = 16.6;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		uint32 resolution_ = 1800;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		bool draw_debug_ = true;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		float max_range_ = 100;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		bool ignore_marked_ = false;

private: //members
 
private: //methods

};

