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
#include "api/WorldSimApiBase.hpp"
#include "EngineUtils.h"
#include "ObjectPainter.h"
#include "DrawDebugHelpers.h"
#include "Weather/WeatherLib.h"
#include <random>
#include "AirBlueprintLib.h"
#include <string>
#include <exception>
#include "Misc/FileHelper.h"




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

	// Seed and initiate noise
	std::random_device rd;
	gen_ = std::mt19937(rd());
	dist_ = std::normal_distribution<float>(0, 1);
	arrow_ = CreateDefaultSubobject<UArrowComponent>(TEXT("Arrow"));
	capture_2D_depth_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DDepth"));
	capture_2D_segmentation_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DSegmentation"));
	capture_2D_intensity_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DIntensity"));


	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/AirSim/HUDAssets/LidarDepthMaterial.LidarDepthMaterial'"));
	if (mat_finder.Succeeded())
	{
		UMaterialInstanceDynamic* depth_material = UMaterialInstanceDynamic::Create(mat_finder.Object, capture_2D_depth_);
		capture_2D_depth_->PostProcessSettings.AddBlendable(depth_material, 1.0f);
	}
	else
		UAirBlueprintLib::LogMessageString("Cannot create depth material for the LidarCamera", "", LogDebugLevel::Failure);

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder_intensity(TEXT("Material'/AirSim/HUDAssets/LidarImpactNormalMaterial.LidarImpactNormalMaterial'"));
	if (mat_finder_intensity.Succeeded())
	{
		UMaterialInstanceDynamic* intensity_material = UMaterialInstanceDynamic::Create(mat_finder_intensity.Object, capture_2D_intensity_);
		capture_2D_intensity_->PostProcessSettings.AddBlendable(intensity_material, 1.0f);
	}
	else
		UAirBlueprintLib::LogMessageString("Cannot create intensity material for the LidarCamera", "", LogDebugLevel::Failure);


	PrimaryActorTick.bCanEverTick = true;
}

void ALidarCamera::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	render_target_2D_depth_ = NewObject<UTextureRenderTarget2D>();
	render_target_2D_segmentation_ = NewObject<UTextureRenderTarget2D>();
	render_target_2D_intensity_ = NewObject<UTextureRenderTarget2D>();
	capture_2D_depth_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_depth_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_depth_->AttachTo(this->RootComponent);
	capture_2D_depth_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	capture_2D_segmentation_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_segmentation_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_segmentation_->AttachTo(this->RootComponent);
	capture_2D_segmentation_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	capture_2D_intensity_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_intensity_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_intensity_->AttachTo(this->RootComponent);
	capture_2D_intensity_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
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
	noise_distance_scale_ = settings.noise_distance_scale;
	ignore_marked_ = settings.ignore_marked;
	ground_truth_ = settings.ground_truth;
	generate_intensity_ = settings.generate_intensity;
	material_list_file_ = settings.material_list_file;
	range_max_lambertian_percentage_ = settings.range_max_lambertian_percentage;
	rain_max_intensity_ = settings.rain_max_intensity;
	rain_constant_a_ = settings.rain_constant_a;
	rain_constant_b_ = settings.rain_constant_b;
	generate_noise_ = settings.generate_noise;
	std::string material_List_content;
	FString materialListContent;
	bool found = FPaths::FileExists(FString(msr::airlib::Settings::getExecutableFullPath("materials.csv").c_str()));

	if (FFileHelper::LoadFileToString(materialListContent, UTF8_TO_TCHAR(settings.material_list_file.c_str()))) {
		material_List_content = std::string(TCHAR_TO_UTF8(*materialListContent));
	}


	std::istringstream iss(material_List_content);
	std::string line, word;

	material_map_.emplace(0, 1);
	int stencil_index = 1;
	while (std::getline(iss, line))
	{
		std::stringstream ss(line);
		std::vector<std::string> row;
		while (std::getline(ss, word, ',')) {
			row.push_back(word);
		}
		material_map_.emplace(stencil_index, std::stof(row[1]));
		stencil_index = stencil_index + 1;
	}

	render_target_2D_depth_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, true);
	render_target_2D_segmentation_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, false);
	render_target_2D_intensity_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, true);

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

	capture_2D_intensity_->TextureTarget = render_target_2D_intensity_;
	capture_2D_intensity_->bAlwaysPersistRenderingState = true;
	capture_2D_intensity_->bCaptureEveryFrame = false;
	capture_2D_intensity_->bCaptureOnMovement = false;
	capture_2D_intensity_->bUseCustomProjectionMatrix = false;

	GenerateLidarCoordinates();
	horizontal_delta_ = (horizontal_max_ - horizontal_min_) / float(measurement_per_cycle_ - 1);
	vertical_delta_ = (FMath::Abs(vertical_min_) + vertical_max_) / float(num_of_lasers_ - 1);
	target_fov_ = FMath::CeilToInt(FMath::Abs(vertical_min_) + vertical_max_ + ((vertical_max_ - vertical_min_) * 0.7f * vertical_delta_));
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
	capture_2D_intensity_->HiddenActors = actors;

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


bool ALidarCamera::Update(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud,
                          msr::airlib::vector<msr::airlib::real_T>& point_cloud_final)
{
	int32 fov = target_fov_;
	float rotation = 360 * delta_time * frequency_;
	sum_rotation_ += rotation;
	bool refresh = false;
	if (sum_rotation_ > horizontal_delta_) {
		//if (sum_rotation_ > target_fov_) fov = FMath::CeilToInt(FMath::Max(FMath::Min(sum_rotation_ + (2 * vertical_delta_), 179.0f), 179.0f));
		fov = 179.0f;
		//UAirBlueprintLib::LogMessageString("GPULidar2: ", "Chosen FOV: " + std::to_string((int)fov) + ".", LogDebugLevel::Informational);
		capture_2D_depth_->FOVAngle = fov;
		capture_2D_segmentation_->FOVAngle = fov;
		capture_2D_intensity_->FOVAngle = fov;
		RotateCamera(current_angle_ + previous_rotation_ + (fov / 2));
		current_angle_ = current_angle_ + previous_rotation_;
		current_angle_ = FMath::Fmod(current_angle_, 360);
		capture_2D_depth_->CaptureScene();
		capture_2D_segmentation_->CaptureScene();
		capture_2D_intensity_->CaptureScene();
		refresh = SampleRenders(sum_rotation_, fov, point_cloud, point_cloud_final);
		previous_rotation_ = sum_rotation_;
		sum_rotation_ = 0;
	}
	return refresh;
}

void ALidarCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	capture_2D_depth_ = nullptr;
	render_target_2D_depth_ = nullptr;
	capture_2D_segmentation_ = nullptr;
	render_target_2D_segmentation_ = nullptr;
	capture_2D_intensity_ = nullptr;
	render_target_2D_intensity_ = nullptr;

}

void ALidarCamera::RotateCamera(float rotation)
{
	capture_2D_depth_->SetRelativeRotation(FRotator(0, rotation, 0));
	capture_2D_segmentation_->SetRelativeRotation(FRotator(0, rotation, 0));
	capture_2D_intensity_->SetRelativeRotation(FRotator(0, rotation, 0));
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

bool ALidarCamera::SampleRenders(float rotation, float fov, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final) {

	bool refresh = false;

	TArray<FColor> buffer_2D_depth;
	FTextureRenderTarget2DResource* render_target_2D_depth = (FTextureRenderTarget2DResource*)capture_2D_depth_->TextureTarget->Resource;
	render_target_2D_depth->ReadPixels(buffer_2D_depth);

	TArray<FColor> buffer_2D_segmentation;
	FTextureRenderTarget2DResource* render_target_2D_segmentation;

	TArray<FColor> buffer_2D_intensity;
	FTextureRenderTarget2DResource* render_target_2D_intensity;
	if (ground_truth_) {
		render_target_2D_segmentation = (FTextureRenderTarget2DResource*)capture_2D_segmentation_->TextureTarget->Resource;
		FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
		flags.SetLinearToGamma(false);
		render_target_2D_segmentation->ReadPixels(buffer_2D_segmentation);
	}
	if (generate_intensity_) {
		render_target_2D_intensity = (FTextureRenderTarget2DResource*)capture_2D_intensity_->TextureTarget->Resource;
		render_target_2D_intensity->ReadPixels(buffer_2D_intensity);
	}
	float c_x = (resolution_-1) / 2.0f;
	float c_y = (resolution_-1) / 2.0f;
	float f_x = (resolution_ / 2.0f) / FMath::Tan(FMath::DegreesToRadians(fov / 2.0f));
	float f_y = (resolution_ / 2.0f) / FMath::Tan(FMath::DegreesToRadians(fov / 2.0f));

	if (rotation > fov)rotation = fov;
	float max_angle = FMath::Fmod(current_angle_ + rotation, 360);
	int32 first_horizontal_idx = current_horizontal_angle_index_+1;
	int32 last_horizontal_idx = getIndexLowerClosest(horizontal_angles_, max_angle);
	bool within_range = true;
	int32 icol = first_horizontal_idx;

	float previous_horizontal_angle = FMath::Fmod(horizontal_angles_[current_horizontal_angle_index_], 360);
	float rain_value;

	if (generate_intensity_) {
		rain_value = UWeatherLib::getWeatherParamScalar(this->GetWorld(), msr::airlib::Utils::toEnum<EWeatherParamScalar>(0));
	}
	while (within_range) {
		int32 icol_circle = (icol) % measurement_per_cycle_;
		if (last_horizontal_idx == icol_circle)within_range = false;

		current_horizontal_angle_index_ = icol_circle;
		float horizontal_angle = horizontal_angles_[icol_circle];
		float horizontal_angle_converted = FMath::Fmod(horizontal_angle - current_angle_, 360) - (fov / 2);

		float cos_hor = FMath::Cos(FMath::DegreesToRadians(horizontal_angle_converted));
		float sin_hor = FMath::Sin(FMath::DegreesToRadians(horizontal_angle_converted));

		int32 h_pixel = FMath::Max(FMath::FloorToInt(((sin_hor * f_x) / cos_hor) + c_x), 0);

		
		for (uint32 ipx = 0; ipx < num_of_lasers_; ipx++)
		{
			if ((previous_horizontal_angle > FMath::Fmod(horizontal_angle, 360)) && (point_cloud.size() != 0)) {
				if (ipx == 0) {
					if ((((int)point_cloud.size() / 5) != measurement_per_cycle_ * num_of_lasers_))
					{
						UE_LOG(LogTemp, Warning, TEXT("Pointcloud or labels incorrect size! points:%i %f %f"), (int)(point_cloud.size() / 5), previous_horizontal_angle, FMath::Fmod(horizontal_angle, 360));
						point_cloud.clear();
						refresh = false;
					}
					else {
						point_cloud_final = point_cloud;
						point_cloud.clear();
						refresh = true;
					}					
				}
			}

			float verticle_angle = vertical_angles_[ipx];

			float cos_ver = FMath::Cos(FMath::DegreesToRadians(verticle_angle));
			float sin_ver = FMath::Sin(FMath::DegreesToRadians(verticle_angle));

			int32 v_pixel = FMath::Max(FMath::FloorToInt((sin_ver * -f_y) / (cos_ver*cos_hor) + c_y), 0);

			FColor value_depth = buffer_2D_depth[h_pixel + (v_pixel * resolution_)];
			float depth = 100000 * ((value_depth.R + value_depth.G * 256 + value_depth.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));

			if(generate_noise_){
				float distance_noise = dist_(gen_) * (1 + ((depth / 100) / max_range_) * (noise_distance_scale_ - 1));
				depth = depth + distance_noise;
			}
			if (depth < (max_range_ * 100)) {
				if (generate_intensity_) {
					float noise = dist_(gen_) * 0.02 * depth * FMath::Pow(1 - FMath::Exp(-rain_max_intensity_ * rain_value), 2);
					depth = depth + noise;
				}
				float distance = depth / (cos_ver*cos_hor);
				FVector point = (distance * angle_to_xyz_lut_[ipx + (icol_circle * num_of_lasers_)]);
				bool threshold_enable = true;
				FColor value_segmentation(0,0,0);
				float final_intensity = 1;

				if (generate_intensity_) {
					FColor value_intensity = buffer_2D_intensity[h_pixel + (v_pixel * resolution_)];
					float impact_angle = ((value_intensity.R + value_intensity.G * 256 + value_intensity.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));
					//float impact_angle = value_intensity.R / 255.0f;
					final_intensity = impact_angle * material_map_.at(value_intensity.A) * FMath::Exp(-2.0f * rain_constant_a_ * FMath::Pow(rain_max_intensity_ * rain_value, rain_constant_b_) * (depth / 100.0f));

					if (draw_debug_ && draw_mode_ == 2) {
						FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point_draw, 5, FColor(unique_colors_[value_intensity.A*3], unique_colors_[(value_intensity.A * 3) + 1], unique_colors_[(value_intensity.A * 3) + 2], 1), false, (1 / (frequency_ * 4)));
					}
					if (draw_debug_ && draw_mode_ == 3) {
						FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point_draw, 5, FColor(0, FMath::FloorToInt(impact_angle*254), 0, 1), false, (1 / (frequency_ * 4)));
					}


					if((impact_angle * material_map_.at(value_intensity.A)) < (max_range_ / range_max_lambertian_percentage_ / 100) * depth / 100.0)threshold_enable = false;
					if (draw_debug_ && draw_mode_ == 4 && threshold_enable) {
						FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point_draw, 5, FColor(0, 0, FMath::FloorToInt(final_intensity * 254), 1), false, 2);
					}
	
				}

				if (ground_truth_) {
					value_segmentation = buffer_2D_segmentation[h_pixel + (v_pixel * resolution_)];
					if (draw_debug_ && draw_mode_ == 1 && threshold_enable) {
						FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point_draw, 5, FColor(value_segmentation.R, value_segmentation.G, value_segmentation.B, 1), false, 2);
					}
				}

				if (threshold_enable) {
					point_cloud.emplace_back(point.X / 100);
					point_cloud.emplace_back(-point.Y / 100);
					point_cloud.emplace_back(point.Z / 100);
					std::uint32_t rgb = ((std::uint32_t)value_segmentation.R << 16 | (std::uint32_t)value_segmentation.G << 8 | (std::uint32_t)value_segmentation.B);
					//UE_LOG(LogTemp, Warning, TEXT("RGB FCOLOR:%i %i %i"), (int)(value_segmentation.R), (int)(value_segmentation.G), (int)(value_segmentation.B));
					point_cloud.emplace_back(rgb);
					point_cloud.emplace_back(final_intensity);
				}
				else {
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
				}

				if (draw_debug_ && draw_mode_ == 0 && threshold_enable) {
					FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
					DrawDebugPoint(this->GetWorld(), point_draw, 5, FColor::Blue, false, (1 / (frequency_ * 4)));
				}				
			}
			else {
				point_cloud.emplace_back(0);
				point_cloud.emplace_back(0);
				point_cloud.emplace_back(0);
				point_cloud.emplace_back(0);
				point_cloud.emplace_back(0);
			}
		}
		previous_horizontal_angle = FMath::Fmod(horizontal_angle, 360);
		icol += 1;
	}
	return refresh;
}

int32 ALidarCamera::unique_colors_[765] = {
	0, 0, 255,
	255, 0, 0,
	0, 255, 0,
	0, 0, 43,
	255, 26, 184,
	255, 211, 0,
	0, 87, 0,
	131, 131, 255,
	158, 79, 70,
	0, 255, 193,
	0, 131, 149,
	0, 0, 123,
	149, 211, 79,
	246, 158, 219,
	211, 17, 255,
	123, 26, 105,
	246, 17, 96,
	255, 193, 131,
	35, 35, 8,
	140, 167, 123,
	246, 131, 8,
	131, 114, 0,
	114, 246, 255,
	158, 193, 255,
	114, 96, 123,
	158, 0, 0,
	0, 79, 255,
	0, 70, 149,
	211, 255, 0,
	184, 79, 211,
	61, 0, 26,
	237, 255, 175,
	255, 123, 96,
	70, 255, 123,
	17, 167, 96,
	211, 167, 167,
	211, 79, 131,
	105, 0, 193,
	43, 96, 70,
	0, 149, 246,
	8, 61, 79,
	167, 87, 8,
	114, 96, 61,
	8, 149, 0,
	158, 105, 184,
	255, 255, 114,
	167, 246, 202,
	149, 175, 184,
	175, 175, 8,
	43, 0, 79,
	0, 202, 255,
	79, 35, 0,
	0, 184, 167,
	158, 0, 52,
	79, 123, 175,
	26, 70, 193,
	87, 211, 0,
	114, 149, 52,
	228, 167, 52,
	246, 140, 149,
	105, 17, 140,
	228, 96, 193,
	237, 211, 255,
	158, 255, 158,
	255, 79, 246,
	149, 96, 255,
	193, 184, 96,
	237, 43, 52,
	175, 105, 131,
	70, 43, 87,
	184, 131, 87,
	140, 8, 255,
	70, 79, 0,
	219, 211, 175,
	228, 149, 255,
	0, 0, 8,
	96, 61, 61,
	237, 0, 131,
	175, 0, 149,
	105, 0, 0,
	184, 167, 255,
	167, 149, 193,
	131, 43, 79,
	114, 123, 114,
	219, 70, 0,
	8, 52, 0,
	149, 202, 123,
	0, 17, 211,
	184, 237, 255,
	87, 61, 149,
	70, 131, 70,
	175, 255, 79,
	70, 202, 87,
	228, 87, 105,
	237, 140, 70,
	123, 79, 17,
	96, 114, 202,
	0, 255, 237,
	79, 149, 131,
	184, 0, 202,
	255, 219, 114,
	17, 35, 184,
	167, 0, 105,
	158, 114, 105,
	26, 184, 202,
	0, 43, 35,
	149, 140, 79,
	175, 211, 8,
	184, 70, 52,
	0, 114, 255,
	255, 246, 0,
	96, 167, 202,
	87, 211, 158,
	114, 70, 193,
	140, 202, 184,
	17, 52, 96,
	131, 255, 0,
	70, 105, 123,
	0, 35, 131,
	70, 0, 61,
	175, 79, 149,
	193, 79, 255,
	211, 255, 140,
	8, 26, 87,
	61, 70, 61,
	96, 87, 149,
	246, 167, 140,
	246, 123, 255,
	228, 255, 219,
	255, 175, 202,
	123, 70, 114,
	184, 123, 255,
	96, 158, 0,
	255, 175, 0,
	43, 17, 8,
	87, 255, 79,
	211, 202, 211,
	175, 123, 17,
	131, 52, 8,
	43, 131, 35,
	105, 17, 35,
	114, 61, 255,
	43, 35, 52,
	70, 175, 246,
	52, 228, 140,
	255, 228, 158,
	246, 131, 175,
	96, 114, 61,
	149, 61, 158,
	193, 211, 87,
	175, 158, 131,
	211, 114, 96,
	140, 246, 114,
	202, 158, 79,
	0, 211, 61,
	184, 123, 175,
	255, 96, 175,
	202, 35, 61,
	131, 149, 219,
	35, 0, 35,
	158, 140, 149,
	70, 105, 0,
	202, 17, 96,
	0, 96, 96,
	0, 131, 96,
	79, 61, 8,
	96, 175, 79,
	255, 26, 255,
	0, 87, 131,
	0, 96, 202,
	211, 96, 43,
	211, 35, 149,
	140, 43, 193,
	184, 219, 167,
	87, 35, 105,
	228, 219, 43,
	255, 202, 184,
	131, 140, 0,
	255, 87, 140,
	237, 184, 255,
	211, 175, 0,
	131, 211, 255,
	114, 255, 211,
	79, 8, 158,
	79, 87, 237,
	96, 175, 131,
	140, 105, 211,
	193, 211, 246,
	61, 70, 35,
	219, 70, 202,
	175, 255, 246,
	219, 255, 87,
	158, 17, 228,
	193, 87, 105,
	202, 175, 123,
	123, 140, 167,
	17, 114, 193,
	255, 96, 61,
	202, 114, 0,
	114, 70, 43,
	123, 193, 0,
	193, 219, 211,
	175, 149, 17,
	96, 43, 70,
	211, 123, 202,
	255, 79, 96,
	79, 140, 255,
	158, 17, 79,
	70, 70, 87,
	96, 228, 211,
	246, 246, 140,
	167, 96, 52,
	0, 140, 193,
	255, 0, 211,
	140, 79, 87,
	184, 105, 211,
	211, 158, 193,
	0, 96, 43,
	255, 211, 228,
	149, 17, 26,
	105, 211, 123,
	52, 8, 114,
	123, 211, 219,
	0, 26, 43,
	219, 0, 8,
	158, 123, 52,
	114, 105, 35,
	202, 131, 123,
	158, 8, 167,
	255, 105, 0,
	255, 140, 237,
	193, 255, 184,
	193, 211, 131,
	228, 140, 96,
	61, 43, 167,
	96, 167, 175,
	158, 175, 158,
	131, 131, 96,
	255, 123, 52,
	87, 87, 193,
	79, 193, 43,
	140, 175, 0,
	193, 184, 246,
	255, 219, 79,
	114, 96, 96,
	114, 43, 26,
	70, 8, 0,
	131, 70, 140,
	105, 61, 211,
	131, 158, 255,
	96, 0, 105,
	158, 52, 0,
	123, 246, 175,
	52, 87, 35,
	0, 167, 123
};