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
	ignore_marked_ = settings.ignore_marked;
	ground_truth_ = settings.ground_truth;
	generate_intensity_ = settings.generate_intensity;
	material_list_file_ = settings.material_list_file;
	range_max_lambertian_percentage_ = settings.range_max_lambertian_percentage;
	rain_max_intensity_ = settings.rain_max_intensity;
	rain_constant_a_ = settings.rain_constant_a;
	rain_constant_b_ = settings.rain_constant_b;

	std::string::size_type key_pos = 0;
	std::string::size_type key_end;
	std::string::size_type val_pos;
	std::string::size_type val_end;

	while ((key_end = material_list_file_.find(':', key_pos)) != std::string::npos)
	{
		if ((val_pos = material_list_file_.find_first_not_of(": ", key_end)) == std::string::npos)
			break;

		val_end = material_list_file_.find('\n', val_pos);
		material_map_.emplace(std::stoi(material_list_file_.substr(key_pos, key_end - key_pos)), std::stof(material_list_file_.substr(val_pos, val_end - val_pos)));

		key_pos = val_end;
		if (key_pos != std::string::npos)
			++key_pos;
	}

	render_target_2D_depth_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, true);
	render_target_2D_segmentation_->InitCustomFormat(resolution_, resolution_, PF_B8G8R8A8, true);
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
	vertical_delta_ = (FMath::Abs(vertical_min_) + vertical_max_) / num_of_lasers_;
	target_fov_ = FMath::CeilToInt(FMath::Abs(vertical_min_) + vertical_max_ + (10 * vertical_delta_));
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
		if (sum_rotation_ > target_fov_) fov = FMath::CeilToInt(FMath::Min(sum_rotation_ + (2 * vertical_delta_), 150.0f));
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
		render_target_2D_segmentation->ReadPixels(buffer_2D_segmentation);
	}
	if (generate_intensity_) {
		render_target_2D_intensity = (FTextureRenderTarget2DResource*)capture_2D_intensity_->TextureTarget->Resource;
		render_target_2D_intensity->ReadPixels(buffer_2D_intensity);
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
	float rain_value;
	if (generate_intensity_)rain_value = UWeatherLib::getWeatherParamScalar(this->GetWorld(), msr::airlib::Utils::toEnum<EWeatherParamScalar>(0));

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

			int32 v_pixel = FMath::FloorToInt((sin_ver * -f_y) / (cos_ver*cos_hor) + c_y);

			FColor value_depth = buffer_2D_depth[h_pixel + (v_pixel * resolution_)];
			float depth = 100000 * ((value_depth.R + value_depth.G * 256 + value_depth.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));	

			if (depth < (max_range_ * 100)) {
				float distance = depth / (cos_ver*cos_hor);
				FVector point = (distance * angle_to_xyz_lut_[ipx + (icol_circle * num_of_lasers_)]);
				bool threshold_enable = true;
				FColor value_segmentation(0,0,0);
				float final_intensity = 1;

				if (generate_intensity_) {
					FColor value_intensity = buffer_2D_intensity[h_pixel + (v_pixel * resolution_)];
					//float impact_angle = ((value_intensity.R + value_intensity.G * 256 + value_intensity.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));
					float impact_angle = value_intensity.R / 255.0f;
					final_intensity = impact_angle * material_map_.at(value_intensity.A) * FMath::Exp(-2.0f * rain_constant_a_ * FMath::Pow(rain_max_intensity_ * rain_value, rain_constant_b_) * (depth / 100.0f));

					if (draw_debug_ && draw_mode_ == 2) {
						point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point, 5, FColor(value_intensity.A, 0, 0, 1), false, (1 / (frequency_ * 4)));
					}
					if (draw_debug_ && draw_mode_ == 3) {
						point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point, 5, FColor(0, FMath::FloorToInt(impact_angle*254), 0, 1), false, (1 / (frequency_ * 4)));
					}

					if(final_intensity < (max_range_ / range_max_lambertian_percentage_ / 100) * depth / 100.0)threshold_enable = false;
					if (draw_debug_ && draw_mode_ == 4 && threshold_enable) {
						point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point, 5, FColor(0, 0, FMath::FloorToInt(final_intensity * 254), 1), false, (1 / (frequency_ * 4)));
					}
				}

				if (ground_truth_) {
					value_segmentation = buffer_2D_segmentation[h_pixel + (v_pixel * resolution_)];
					if (draw_debug_ && draw_mode_ == 1 && threshold_enable) {
						point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						DrawDebugPoint(this->GetWorld(), point, 5, FColor(value_segmentation.R, value_segmentation.G, value_segmentation.B, 1), false, (1 / (frequency_ * 4)));
					}
				}

				if (threshold_enable) {
					point_cloud.emplace_back(point.X / 100);
					point_cloud.emplace_back(-point.Y / 100);
					point_cloud.emplace_back(point.Z / 100);
					std::uint32_t rgb = ((std::uint32_t)value_segmentation.R << 16 | (std::uint32_t)value_segmentation.G << 8 | (std::uint32_t)value_segmentation.B);
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
					point = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
					DrawDebugPoint(this->GetWorld(), point, 5, FColor::Blue, false, (1 / (frequency_ * 4)));
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