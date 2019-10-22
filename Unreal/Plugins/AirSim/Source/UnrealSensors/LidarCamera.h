// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneCaptureComponentCube.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraActor.h"
#include "Materials/Material.h"
#include "Runtime/Core/Public/PixelFormat.h"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/AirSimSettings.hpp"
#include "NedTransform.h"

#include "LidarCamera.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API ALidarCamera : public ACameraActor
{
	GENERATED_BODY()
	
public:
	typedef msr::airlib::AirSimSettings AirSimSettings;
	typedef AirSimSettings::CameraSetting CameraSetting;


	ALidarCamera();

	virtual void PostInitializeComponents() override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	void setupCameraFromSettings();
	void setCameraOrientation(const FRotator& rotator);


	msr::airlib::Pose getPose() const;

	int counter = 0;

	void GenerateLidarCoordinates();
	void CaptureAndSample();

	std::vector<float> horizontal_angles_;
	std::vector<float> vertical_angles_;
	std::vector<FVector> angle_to_xyz_lut_;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		USceneCaptureComponentCube* capture_;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		UTextureRenderTargetCube* render_target_;

	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		uint32 num_of_lasers_ = 64;
	UPROPERTY(EditAnywhere, Category = "LIDAR Setup")
		float rpm_ = 600;
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

	UPROPERTY(BlueprintReadWrite, EditAnywhere) TArray<USceneCaptureComponent2D*> captures_;
	UPROPERTY(BlueprintReadWrite, EditAnywhere) TArray<UTextureRenderTarget2D*> render_targets_;

private: //members


	FRotator gimbald_rotator_;
	float gimbal_stabilization_;
	const NedTransform* ned_transform_;

private: //methods
	typedef common_utils::Utils Utils;
	typedef AirSimSettings::CaptureSetting CaptureSetting;
	typedef AirSimSettings::NoiseSetting NoiseSetting;

	static void updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target);

};

