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
#include "ObjectPainter.h"
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
	capture_2D_depth_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DDepth"));
	capture_2D_segmentation_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DSegmentation"));


	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/AirSim/HUDAssets/LidarDepthMaterial.LidarDepthMaterial'"));
	if (mat_finder.Succeeded())
	{
		UMaterialInstanceDynamic* depth_material = UMaterialInstanceDynamic::Create(mat_finder.Object, capture_2D_depth_);
		capture_2D_depth_->PostProcessSettings.AddBlendable(depth_material, 1.0f);
	}
	else
		UAirBlueprintLib::LogMessageString("Cannot create depth material for the LidarCamera", "", LogDebugLevel::Failure);

	PrimaryActorTick.bCanEverTick = true;
}

void ALidarCamera::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	render_target_2D_depth_ = NewObject<UTextureRenderTarget2D>();
	render_target_2D_segmentation_ = NewObject<UTextureRenderTarget2D>();
	capture_2D_depth_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_depth_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_depth_->AttachTo(this->RootComponent);
	capture_2D_depth_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	capture_2D_segmentation_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_segmentation_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_segmentation_->AttachTo(this->RootComponent);
	capture_2D_segmentation_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
}

void ALidarCamera::InitializeSettings(const AirSimSettings::GPULidarSetting& settings) {

	resolution_ = settings.resolution;
	num_of_lasers_ = settings.number_of_channels;
	frequency_ = settings.horizontal_rotation_frequency;
	measurement_per_cycle_ = settings.measurement_per_cycle;
	horizontal_min_ = settings.horizontal_FOV_start;
	horizontal_max_ = settings.horizontal_FOV_end;
	vertical_min_ = settings.vertical_FOV_lower;
	vertical_max_ = settings.vertical_FOV_upper;
	draw_debug_ = settings.draw_debug_points;
	draw_mode_ = settings.draw_mode;
	max_range_ = settings.range;
	ignore_marked_ = settings.ignore_marked;
	ground_truth_ = settings.ground_truth;
	generate_intensity_ = settings.generate_intensity;

	render_target_2D_depth_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, true);
	render_target_2D_segmentation_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, true);

	this->SetActorRelativeLocation(FVector(settings.position.x() * 100, settings.position.y() * 100, -settings.position.z() * 100));
	this->SetActorRelativeRotation(FRotator(settings.rotation.pitch, settings.rotation.yaw, settings.rotation.roll));

	capture_2D_depth_->TextureTarget = render_target_2D_depth_;
	capture_2D_depth_->bAlwaysPersistRenderingState = true;
	capture_2D_depth_->bCaptureEveryFrame = false;
	capture_2D_depth_->bCaptureOnMovement = false;
	capture_2D_depth_->bUseCustomProjectionMatrix = false;

	UObjectPainter::SetViewForVertexColor(capture_2D_segmentation_->ShowFlags);
	render_target_2D_segmentation_->TargetGamma = 1;

	capture_2D_segmentation_->TextureTarget = render_target_2D_segmentation_;
	capture_2D_segmentation_->bAlwaysPersistRenderingState = true;
	capture_2D_segmentation_->bCaptureEveryFrame = false;
	capture_2D_segmentation_->bCaptureOnMovement = false;
	capture_2D_segmentation_->bUseCustomProjectionMatrix = false;

	GenerateLidarCoordinates();
	horizontal_delta_ = (horizontal_max_ - horizontal_min_) / float(measurement_per_cycle_ - 1);
	vertical_delta_ = (FMath::Abs(vertical_min_) + vertical_max_) / num_of_lasers_;
	target_fov_ = FMath::CeilToInt(FMath::Abs(vertical_min_) + vertical_max_ + (10 * vertical_delta_));
	UE_LOG(LogTemp, Warning, TEXT("Target FOV: %f"), (int)target_fov_);
	if (target_fov_ % 2 != 0) {
		target_fov_ = target_fov_ + 1;
	}
	//current_horizontal_angle_index_ = horizontal_angles_.Num() - 1;
	current_horizontal_angle_index_ = 0;

	static const FName lidar_ignore_tag = TEXT("LidarIgnore");
	TArray<AActor*> actors;
	for (TActorIterator<AActor> ActorIterator(GetWorld()); ActorIterator; ++ActorIterator)
	{
		AActor* Actor = *ActorIterator;
		if (Actor && Actor != this && Actor->Tags.Contains(lidar_ignore_tag))actors.Add(Actor);
	}

	if (ignore_marked_) {
		static const FName lidar_ignore_tag = TEXT("MarkedIgnore");
		for (TActorIterator<AActor> ActorIterator(GetWorld()); ActorIterator; ++ActorIterator)
		{
			AActor* Actor = *ActorIterator;
			if (Actor && Actor != this && Actor->Tags.Contains(lidar_ignore_tag))actors.Add(Actor);
		}
	}

	capture_2D_depth_->HiddenActors = actors;
	capture_2D_segmentation_->HiddenActors = actors;
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
}

void ALidarCamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}


bool ALidarCamera::Update(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth,
	                      msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final)
{
	int32 fov = target_fov_;
	float rotation = 360 * delta_time * frequency_;
	sum_rotation_ += rotation;
	bool refresh = false;
	if (sum_rotation_ > horizontal_delta_) {
		if (sum_rotation_ > target_fov_) fov = FMath::CeilToInt(FMath::Min(sum_rotation_ + (2 * vertical_delta_), 150.0f));
		UE_LOG(LogTemp, Warning, TEXT("Chosen FOV: %i"), (int)fov);
		capture_2D_depth_->FOVAngle = fov;
		capture_2D_segmentation_->FOVAngle = fov;
		RotateCamera(current_angle_ + previous_rotation_ + (fov / 2));
		current_angle_ = current_angle_ + previous_rotation_;
		current_angle_ = FMath::Fmod(current_angle_, 360);
		capture_2D_depth_->CaptureScene();
		capture_2D_segmentation_->CaptureScene();
		refresh = SampleRenders(sum_rotation_, fov, point_cloud, groundtruth, point_cloud_final, groundtruth_final);
		previous_rotation_ = sum_rotation_;
		sum_rotation_ = 0;
	}
	return refresh;
}

void ALidarCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	capture_2D_depth_ = nullptr;
	render_target_2D_depth_ = nullptr;
}

void ALidarCamera::RotateCamera(float rotation)
{
	capture_2D_depth_->SetRelativeRotation(FRotator(0, rotation, 0));
	capture_2D_segmentation_->SetRelativeRotation(FRotator(0, rotation, 0));
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

bool ALidarCamera::SampleRenders(float rotation, float fov, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth,
	                             msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final) {

	bool refresh = false;

	TArray<FColor> buffer_2D_depth;
	FTextureRenderTarget2DResource* render_target_2D_depth = (FTextureRenderTarget2DResource*)capture_2D_depth_->TextureTarget->Resource;
	render_target_2D_depth->ReadPixels(buffer_2D_depth);

	TArray<FColor> buffer_2D_segmentation;
	FTextureRenderTarget2DResource* render_target_2D_segmentation;
	if (ground_truth_) {
		render_target_2D_segmentation = (FTextureRenderTarget2DResource*)capture_2D_segmentation_->TextureTarget->Resource;
		render_target_2D_segmentation->ReadPixels(buffer_2D_segmentation);
	}
	float c_x = resolution_ / 2.0f;
	float c_y = resolution_ / 2.0f;
	float f_x = (resolution_ / 2.0f) / FMath::Tan(FMath::DegreesToRadians(fov / 2.0f));
	float f_y = (resolution_ / 2.0f) / FMath::Tan(FMath::DegreesToRadians(fov / 2.0f));

	if (rotation > fov)rotation = fov;
	float max_angle = FMath::Fmod(current_angle_ + rotation, 360);
	int32 first_horizontal_idx = getIndexOfMatchOrUpperClosest(horizontal_angles_, current_angle_);
	int32 last_horizontal_idx = getIndexLowerClosest(horizontal_angles_, max_angle);
	bool within_range = true;
	int32 icol = first_horizontal_idx;

	float previous_horizontal_angle = FMath::Fmod(horizontal_angles_[current_horizontal_angle_index_], 360);

	while (within_range) {
		int32 icol_circle = (icol) % measurement_per_cycle_;
		if (last_horizontal_idx == icol_circle)within_range = false;
		
		current_horizontal_angle_index_ = icol_circle;
		float horizontal_angle = horizontal_angles_[icol_circle];
		float horizontal_angle_converted = FMath::Fmod(horizontal_angle - current_angle_, 360) - (fov / 2);

		float cos_hor = FMath::Cos(FMath::DegreesToRadians(horizontal_angle_converted));
		float sin_hor = FMath::Sin(FMath::DegreesToRadians(horizontal_angle_converted));

		int32 h_pixel = FMath::FloorToInt(((sin_hor * f_x) / cos_hor) + c_x);
		for (uint32 ipx = 0; ipx < num_of_lasers_; ipx++)
		{
			if ((previous_horizontal_angle > FMath::Fmod(horizontal_angle, 360)) && (point_cloud.size() != 0)) {
				if (ipx == 0) {
					if ((((int)point_cloud.size() / 3) != measurement_per_cycle_ * num_of_lasers_))
					{
						UE_LOG(LogTemp, Warning, TEXT("Pointcloud or labels incorrect size! points:%i labels:%i %f %f"), (int)(point_cloud.size() / 3), groundtruth.size(), previous_horizontal_angle, FMath::Fmod(horizontal_angle, 360));
						point_cloud.clear();
						groundtruth.clear();
						refresh = false;
					}
					else {
						point_cloud_final = point_cloud;
						groundtruth_final = groundtruth;
						point_cloud.clear();
						groundtruth.clear();
						refresh = true;
					}					
				}
			}

			float verticle_angle = vertical_angles_[ipx];

			float cos_ver = FMath::Cos(FMath::DegreesToRadians(verticle_angle));
			float sin_ver = FMath::Sin(FMath::DegreesToRadians(verticle_angle));

			int32 v_pixel = FMath::FloorToInt((sin_ver * -f_y) / (cos_ver*cos_hor) + c_y);

			FColor value_depth = buffer_2D_depth[h_pixel + (v_pixel * resolution_)];
			float depth = 100000 * ((value_depth.R + value_depth.G * 256 + value_depth.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));			
			if (depth < (max_range_ * 100)) {
				float distance = depth / (cos_ver*cos_hor);
				FVector point = (distance * angle_to_xyz_lut_[ipx + (icol_circle * num_of_lasers_)]);
				point_cloud.emplace_back(point.X / 100);
				point_cloud.emplace_back(point.Y / 100);
				point_cloud.emplace_back(-point.Z / 100);
				if (ground_truth_) {
					FColor value_segmentation = buffer_2D_segmentation[h_pixel + (v_pixel * resolution_)];
					std::string color_string = std::to_string((int)value_segmentation.R) + "," + std::to_string((int)value_segmentation.G) + "," + std::to_string((int)value_segmentation.B);
					groundtruth.emplace_back(color_string);
					if (draw_debug_ && draw_mode_ == 1) {
						point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point, 5, FColor(value_segmentation.R, value_segmentation.G, value_segmentation.B, 1), false, (1 / (frequency_ * 4)));
					}
				}
				if (draw_debug_ && draw_mode_ == 0) {
					point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
					DrawDebugPoint(this->GetWorld(), point, 5, FColor::Blue, false, (1 / (frequency_ * 4)));
				}
				
			}
			else {
				point_cloud.emplace_back(0);
				point_cloud.emplace_back(0);
				point_cloud.emplace_back(0);
				if (ground_truth_) {
					groundtruth.emplace_back("-1,-1,-1");
				}
			}
		}
		previous_horizontal_angle = FMath::Fmod(horizontal_angle, 360);
		icol += 1;
	}
	return refresh;
}