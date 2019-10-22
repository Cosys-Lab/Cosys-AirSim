#include "LidarCamera.h"
#include "ConstructorHelpers.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneCaptureComponentCube.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ImageUtils.h"

#include <string>
#include <exception>
#include "AirBlueprintLib.h"


std::vector<float> LinearSpacedArray(float a, float b, std::size_t N)
{
	double h = (b - a) / static_cast<float>(N - 1);
	std::vector<float> xs(N);
	std::vector<float>::iterator x;
	double val;
	for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
		*x = val;
	}
	return xs;
}


ALidarCamera::ALidarCamera()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALidarCamera::PostInitializeComponents()
{
	Super::PostInitializeComponents();

	capture_ = UAirBlueprintLib::GetActorComponent<USceneCaptureComponentCube>(this, TEXT("LidarCaptureComponent"));

	//captures_.Init(nullptr, 3);
	//render_targets_.Init(nullptr, 3);

	//captures_[0] =
	//	UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("LidarOneComponent"));
	//captures_[1] =
	//	UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("LidarTwoComponent"));
	//captures_[2] =
	//	UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("LidarThreeComponent"));
}

void ALidarCamera::GenerateLidarCoordinates() {
	horizontal_angles_ = LinearSpacedArray(horizontal_min_, horizontal_max_, measurement_per_cycle_);
	vertical_angles_ = LinearSpacedArray(vertical_min_, vertical_max_, num_of_lasers_);

	for (uint32 icol = 0; icol < measurement_per_cycle_; icol++)
	{
		float h_angle_0 = horizontal_angles_[icol];
		for (uint32 ipx = 0; ipx < num_of_lasers_; ipx++)
		{
			angle_to_xyz_lut_.push_back(FVector(FMath::Cos(msr::airlib::Utils::degreesToRadians(vertical_angles_[ipx])) * FMath::Sin(msr::airlib::Utils::degreesToRadians(h_angle_0)),
												FMath::Cos(msr::airlib::Utils::degreesToRadians(vertical_angles_[ipx])) * FMath::Cos(msr::airlib::Utils::degreesToRadians(h_angle_0)),
												FMath::Sin(msr::airlib::Utils::degreesToRadians(vertical_angles_[ipx]))));
		}
	}
}

void ALidarCamera::BeginPlay()
{
	Super::BeginPlay();

	GenerateLidarCoordinates();
	render_target_ = NewObject<UTextureRenderTargetCube>();

	render_target_->InitAutoFormat(1800);
	//capture_->TextureTarget = render_target_;
	capture_->bAlwaysPersistRenderingState = true;
	capture_->bCaptureEveryFrame = false;


	//for (unsigned int camera_index = 0; camera_index < 3; ++camera_index) {
	//	//use final color for all calculations
	//	captures_[camera_index]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	//	render_targets_[camera_index] = NewObject<UTextureRenderTarget2D>();
	//}
	//setupCameraFromSettings();

	gimbal_stabilization_ = 0;
	gimbald_rotator_ = this->GetActorRotation();
	this->SetActorTickEnabled(true);
	UAirBlueprintLib::LogMessageString("lidartest: ", "made it here", LogDebugLevel::Informational);
}

void ALidarCamera::Tick(float DeltaTime)
{
	if (gimbal_stabilization_ > 0) {
		FRotator rotator = this->GetActorRotation();
		if (!std::isnan(gimbald_rotator_.Pitch))
			rotator.Pitch = gimbald_rotator_.Pitch * gimbal_stabilization_ +
			rotator.Pitch * (1 - gimbal_stabilization_);
		if (!std::isnan(gimbald_rotator_.Roll))
			rotator.Roll = gimbald_rotator_.Roll * gimbal_stabilization_ +
			rotator.Roll * (1 - gimbal_stabilization_);
		if (!std::isnan(gimbald_rotator_.Yaw))
			rotator.Yaw = gimbald_rotator_.Yaw * gimbal_stabilization_ +
			rotator.Yaw * (1 - gimbal_stabilization_);

		this->SetActorRotation(rotator);
	}

	if (counter == 10) {
		CaptureAndSample();
		counter = 0;
	}
	else {
		counter = counter + 1;
	}
	Super::Tick(DeltaTime);
}

void ALidarCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{

	capture_ = nullptr;
	render_target_ = nullptr;

	//for (unsigned int camera_index = 0; camera_index < 3; ++camera_index) {
	//	//use final color for all calculations
	//	captures_[camera_index] = nullptr;
	//	render_targets_[camera_index] = nullptr;
	//}
}

void ALidarCamera::setCameraOrientation(const FRotator& rotator)
{
	if (gimbal_stabilization_ > 0) {
		gimbald_rotator_.Pitch = rotator.Pitch;
		gimbald_rotator_.Roll = rotator.Roll;
		gimbald_rotator_.Yaw = rotator.Yaw;
	}
	this->SetActorRelativeRotation(rotator);
}

void ALidarCamera::setupCameraFromSettings()
{
	for (int camera_index = 0; camera_index < 3; ++camera_index) {
		updateCaptureComponentSetting(captures_[camera_index], render_targets_[camera_index]);				
	}
}

void ALidarCamera::updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target)
{
	render_target->InitAutoFormat(1800, 1800);
	capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	capture->FOVAngle = 120.0f;
	capture->TextureTarget = render_target;
	capture->bAlwaysPersistRenderingState = true;
	capture->bCaptureEveryFrame = false;
}

msr::airlib::Pose ALidarCamera::getPose() const
{
	return ned_transform_->toLocalNed(this->GetActorTransform());
}

void ALidarCamera::CaptureAndSample() {
	capture_->CaptureScene();
	TArray<FColor> buffer;
	FTextureRenderTargetCubeResource* RenderTarget = (FTextureRenderTargetCubeResource*)capture_->TextureTarget->GameThread_GetRenderTargetResource();
	FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
	flags.SetLinearToGamma(false);
	RenderTarget->ReadPixels(buffer, flags, FIntRect(0, 0, 0, 0));
	for (uint32 icol = 0; icol < measurement_per_cycle_; icol++)
	{
		for (uint32 ipx = 0; ipx < num_of_lasers_; ipx++)
		{
			int Vpixel = FMath::FloorToInt((vertical_angles_[ipx] * (1800 / 2)) / 90) + 900;
			int Hpixel = FMath::FloorToInt((horizontal_angles_[ipx] *1800) / 360) + 1350;
			if (Hpixel >= 1800)Hpixel = Hpixel - 1800;
			int value = buffer[Hpixel + (Vpixel * 1800)].R;
			float distance = ((value / 255.0) * 1000.0);
			FVector point = (distance * angle_to_xyz_lut_[ipx + (icol*num_of_lasers_)]) + this->GetActorLocation();
			UAirBlueprintLib::DrawPoint(
				this->GetWorld(),
				point,
				5,                       //size
				FColor::Red,
				true,                    //persistent (never goes away)
				10                      //point leaves a trail on moving object
			);
		}
	}

}