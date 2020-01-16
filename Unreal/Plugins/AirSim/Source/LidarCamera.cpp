#include "LidarCamera.h"
#include "ConstructorHelpers.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/ArrowComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ImageUtils.h"
#include "common/AirSimSettings.hpp"
#include "EngineUtils.h"

#include "DrawDebugHelpers.h"

#include "AirBlueprintLib.h"
#include <string>
#include <exception>

TArray<float> LinearSpacedArray(float min, float max, size_t N) {
	TArray<float> range;
	float delta = (max - min) / float(N - 1);
	for (int i = 0; i < N; i++) {
		range.Add(min + i * delta);
	}
	return range;
}

ALidarCamera::ALidarCamera()
{
	arrow_ = CreateDefaultSubobject<UArrowComponent>(TEXT("Arrow"));
	capture2D_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2D"));

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/AirSim/HUDAssets/LidarDepthMaterial.LidarDepthMaterial'"));
	if (mat_finder.Succeeded())
	{
		UMaterialInstanceDynamic* noise_material_ = UMaterialInstanceDynamic::Create(mat_finder.Object, capture2D_);
		capture2D_->PostProcessSettings.AddBlendable(noise_material_, 1.0f);
	}
	else
		UAirBlueprintLib::LogMessageString("Cannot create lidar depth material for the LidarCamera",
			"", LogDebugLevel::Failure);

	PrimaryActorTick.bCanEverTick = true;
}

void ALidarCamera::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	render_target2D_ = NewObject<UTextureRenderTarget2D>();
	capture2D_->SetRelativeRotation(FRotator(0, 0, 0));
	capture2D_->SetRelativeLocation(FVector(0, 0, 0));
	capture2D_->AttachTo(this->RootComponent);
	capture2D_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
}

void ALidarCamera::InitializeSettings(const AirSimSettings::GPULidarSetting& settings) {

	resolution_ = settings.resolution;
	num_of_lasers_ = settings.number_of_channels;
	frequency_ = settings.horizontal_rotation_frequency;
	measurement_per_cycle_ = settings.measurement_per_cycle_;
	horizontal_min_ = settings.horizontal_FOV_start;
	horizontal_max_ = settings.horizontal_FOV_end;
	vertical_min_ = settings.vertical_FOV_lower;
	vertical_max_ = settings.vertical_FOV_upper;
	draw_debug_ = settings.draw_debug_points;
	max_range_ = settings.range;

	render_target2D_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, true);
	this->SetActorRelativeLocation(FVector(settings.position.x() * 100, settings.position.y() * 100, -settings.position.z() * 100));
	this->SetActorRelativeRotation(FRotator(settings.rotation.pitch, settings.rotation.yaw, settings.rotation.roll));

	capture2D_->TextureTarget = render_target2D_;
	capture2D_->bAlwaysPersistRenderingState = true;
	capture2D_->bCaptureEveryFrame = false;
	capture2D_->bUseCustomProjectionMatrix = false;

	GenerateLidarCoordinates();
	horizontal_delta_ = (horizontal_max_ - horizontal_min_) / float(measurement_per_cycle_ - 1);
	vertical_delta_ = (FMath::Abs(vertical_min_) + vertical_max_) / num_of_lasers_;
	target_fov_ = FMath::Abs(vertical_min_) + vertical_max_ + (2 * vertical_delta_);
}

void ALidarCamera::GenerateLidarCoordinates() {
	horizontal_angles_ = LinearSpacedArray(horizontal_min_, horizontal_max_, measurement_per_cycle_);
	vertical_angles_ = LinearSpacedArray(vertical_min_, vertical_max_, num_of_lasers_);
	for (uint32 icol = 0; icol < measurement_per_cycle_; icol++)
	{
		float h_angle_0 = horizontal_angles_[icol];
		for (uint32 ipx = 0; ipx < num_of_lasers_; ipx++)
		{
			angle_to_xyz_lut_.Add(FVector(FMath::Cos(FMath::DegreesToRadians(vertical_angles_[ipx])) * FMath::Cos(FMath::DegreesToRadians(h_angle_0)),
				FMath::Cos(FMath::DegreesToRadians(vertical_angles_[ipx])) * FMath::Sin(FMath::DegreesToRadians(h_angle_0)),
				FMath::Sin(FMath::DegreesToRadians(vertical_angles_[ipx]))));
		}
	}
}

void ALidarCamera::BeginPlay()
{
	Super::BeginPlay();	
	static const FName lidar_ignore_tag = TEXT("LidarIgnore");
	TArray<AActor*> actors;
	for (TActorIterator<AActor> ActorIterator(GetWorld()); ActorIterator; ++ActorIterator)
	{
		AActor* Actor = *ActorIterator;
		if (Actor && Actor != this && Actor->Tags.Contains(lidar_ignore_tag))actors.Add(Actor);
	}

	capture2D_->HiddenActors = actors;
}

void ALidarCamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}


void ALidarCamera::Update(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud)
{
	float rotation = 0;
	float fov = target_fov_;
	rotation = 360 * delta_time * frequency_;
	sum_rotation_ += rotation;
	if (sum_rotation_ > horizontal_delta_) {
		if (sum_rotation_ > target_fov_) fov = FMath::Min(sum_rotation_ + (2 * vertical_delta_), 150.0f);
		fov = 90;
		capture2D_->FOVAngle = fov;
		RotateCamera(current_angle_ + previous_rotation_ + (fov / 2));
		current_angle_ = current_angle_ + previous_rotation_;
		current_angle_ = FMath::Fmod(current_angle_, 360);
		capture2D_->CaptureScene();
		SampleRender(sum_rotation_, fov, point_cloud);
		previous_rotation_ = sum_rotation_;
		sum_rotation_ = 0;
	}
}

void ALidarCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	capture2D_ = nullptr;
	render_target2D_ = nullptr;
}

void ALidarCamera::RotateCamera(float rotation)
{
	capture2D_->SetRelativeRotation(FRotator(0, rotation, 0));
}

int32 getIndexOfMatchOrUpperClosest(TArray<float> range, float value) {
	for (int i = 0; i < GetNum(range) - 1; i++) {
		if (range[i] == value)return i;
		if (range[i] < value && range[i + 1] > value)return i + 1;
	}
	return 0;
}

int32 getIndexLowerClosest(TArray<float> range, float value) {
	if (range[0] == value)return GetNum(range) - 1;
	for (int i = 0; i < GetNum(range) - 1; i++) {

		if (range[i] == value) return i - 1;
		if (range[i] < value && range[i + 1] > value)return i;
	}
	if (range.Last() == value) return GetNum(range) - 2;
	else {
		return GetNum(range) - 1;
	}
	return 0;
}

void ALidarCamera::SampleRender(float rotation, float fov, msr::airlib::vector<msr::airlib::real_T>& point_cloud) {

	TArray<FColor> buffer2D;
	FTextureRenderTarget2DResource* RenderTarget2D = (FTextureRenderTarget2DResource*)capture2D_->TextureTarget->Resource;
	RenderTarget2D->ReadPixels(buffer2D);

	float c_x = resolution_ / 2.0f;
	float c_y = resolution_ / 2.0f;
	float f_x = (resolution_ / 2.0f) / FMath::Tan(FMath::DegreesToRadians(fov / 2.0f));
	float f_y = (resolution_ / 2.0f) / FMath::Tan(FMath::DegreesToRadians(fov / 2.0f));

	if (rotation > fov)rotation = fov;
	float max_angle = FMath::Fmod(current_angle_ + rotation, 360);
	int32 first_horizontal_idx = getIndexOfMatchOrUpperClosest(horizontal_angles_, current_angle_);
	int32 last_horizontal_idx = getIndexLowerClosest(horizontal_angles_, max_angle);
	int32 pointCount = (FMath::Abs(last_horizontal_idx - first_horizontal_idx))*num_of_lasers_;
	bool withinRange = true;
	int32 icol = first_horizontal_idx;

	while (withinRange) {
		int32 icol_circle = (icol) % measurement_per_cycle_;
		if (last_horizontal_idx == icol_circle)withinRange = false;

		float horizontal_angle = horizontal_angles_[icol_circle];
		float horizontal_angle_converted = FMath::Fmod(horizontal_angle - current_angle_, 360) - (fov / 2);

		float cos_hor = FMath::Cos(FMath::DegreesToRadians(horizontal_angle_converted));
		float sin_hor = FMath::Sin(FMath::DegreesToRadians(horizontal_angle_converted));

		int32 hPixel = FMath::FloorToInt(((sin_hor * f_x) / cos_hor) + c_x);
		for (uint32 ipx = 0; ipx < num_of_lasers_; ipx++)
		{
			float verticle_angle = vertical_angles_[ipx];

			float cos_ver = FMath::Cos(FMath::DegreesToRadians(verticle_angle));
			float sin_ver = FMath::Sin(FMath::DegreesToRadians(verticle_angle));

			int32 vPixel = FMath::FloorToInt((sin_ver * -f_y) / (cos_ver*cos_hor) + c_y);

			FColor value = buffer2D[hPixel + (vPixel * resolution_)];
			float depth = 100000 * ((value.R + value.G * 256 + value.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));
			float distance = depth / (cos_ver*cos_hor);

			FVector point = (distance * angle_to_xyz_lut_[ipx + (icol_circle * num_of_lasers_)]);	
			point_cloud.emplace_back(point.X / 100);
			point_cloud.emplace_back(point.Y / 100);
			point_cloud.emplace_back(-point.Z / 100);
			point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
			if (draw_debug_ && depth < (max_range_* 100) )DrawDebugPoint(this->GetWorld(), point, 5, FColor::Blue, false, (1 / (frequency_ * 4)));
		}
		icol += 1;
	}
}