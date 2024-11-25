// Developed by Cosys-Lab, University of Antwerp

#include "LidarCamera.h"
#include "UObject/ConstructorHelpers.h"
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
#include "Annotation/ObjectAnnotator.h"
#include "DrawDebugHelpers.h"
#include "Weather/WeatherLib.h"
#include <random>
#include "AirBlueprintLib.h"
#include "Engine/Engine.h"
#include <string>
#include <exception>
#include "Misc/FileHelper.h"

// Generate linear spaced array for N values between min and max
TArray<float> LinearSpacedArray(float min, float max, size_t N) {
	TArray<float> range;
	float delta = (max - min) / float(N - 1);
	for (int i = 0; i < N; i++) {
		range.Add(min + i * delta);
	}
	return range;
}

// Get the index of a value in an array that matched the searched value or is one higher than the closest found value in the array
int32 getIndexOfMatchOrUpperClosest(TArray<float> range, float value) {
	for (int i = 0; i < GetNum(range) - 1; i++) {
		if (range[i] == value)return i;
		if (range[i] < value && range[i + 1] > value)return i + 1;
	}
	return 0;
}

// Get the index of a value in an array that matched the searched value or is one lower than the closest found value in the array
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

// Constructor
//ALidarCamera::ALidarCamera() : wait_signal_(new msr::airlib::WorkerThreadSignal)
ALidarCamera::ALidarCamera()
{
	// Seed and initiate noise
	std::random_device rd;
	gen_ = std::mt19937(rd());
	dist_ = std::normal_distribution<float>(0, 1);

	// Add components
	arrow_ = CreateDefaultSubobject<UArrowComponent>(TEXT("Arrow"));
	this->SetRootComponent(arrow_);
	capture_2D_depth_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DDepth"));
	capture_2D_segmentation_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DSegmentation"));
	capture_2D_intensity_ = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Lidar2DIntensity"));

	// Find materials in Plugin content and assign to post process settings of the capture components
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

	// Create new render targets that will be used by the capture components
	render_target_2D_depth_ = NewObject<UTextureRenderTarget2D>();
	render_target_2D_segmentation_ = NewObject<UTextureRenderTarget2D>();
	render_target_2D_intensity_ = NewObject<UTextureRenderTarget2D>();

	// Set the position, rotation and source type of each capture component and attach it to the root of the Actor
	capture_2D_depth_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_depth_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_depth_->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	capture_2D_depth_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	capture_2D_segmentation_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_segmentation_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_segmentation_->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	capture_2D_segmentation_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	capture_2D_intensity_->SetRelativeRotation(FRotator(0, 0, 0));
	capture_2D_intensity_->SetRelativeLocation(FVector(0, 0, 0));
	capture_2D_intensity_->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	capture_2D_intensity_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
}

void ALidarCamera::BeginPlay()
{
	Super::BeginPlay();
}

void ALidarCamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (!used_by_airsim_) {
		msr::airlib::vector<msr::airlib::real_T> point_cloud_empty;
		Update(DeltaTime, point_cloud_empty, point_cloud_empty);
	}
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

// Get all the settings from AirSim
void ALidarCamera::InitializeSettingsFromAirSim(const msr::airlib::GPULidarSimpleParams& settings)
{
	// Get all the settings from AirSim
	resolution_ = (int32)settings.resolution;
	num_lasers_ = (int32)settings.number_of_channels;
	sensor_rotation_frequency_ = settings.horizontal_rotation_frequency;
	horizontal_samples_ = (int32)settings.measurement_per_cycle;
	horizontal_fov_min_ = settings.horizontal_FOV_start;
	horizontal_fov_max_ = settings.horizontal_FOV_end;
	vertical_fov_min_ = settings.vertical_FOV_lower;
	vertical_fov_max_ = settings.vertical_FOV_upper;
	draw_debug_ = settings.draw_debug_points;
	debug_draw_mode_ = (int32)settings.draw_mode;
	max_range_ = settings.range;
	distance_noise_scale_ = settings.noise_distance_scale;
	ignore_marked_ = settings.ignore_marked;
	generate_groundtruth_ = settings.ground_truth;
	instance_segmentation_ = settings.instance_segmentation;
	annotation_name_ = FString(settings.annotation_name.c_str());
	generate_intensity_ = settings.generate_intensity;
	material_list_file_ = settings.material_list_file;
	max_range_lambertian_percentage_ = settings.range_max_lambertian_percentage;
	rain_max_intensity_ = settings.rain_max_intensity;
	rain_constant_a_ = settings.rain_constant_a;
	rain_constant_b_ = settings.rain_constant_b;
	generate_distance_noise_ = settings.generate_noise;

	// Load materials.csv file holding the lambertian reflectance coefficients for certain material types and save tem into a map
	std::string material_List_content;
	FString materialListContent;
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
		if (row.size() > 1) {
			material_map_.emplace(stencil_index, std::stof(row[1]));
			stencil_index = stencil_index + 1;
		}
	}

	// Set the sensor in the Unreal world at the right position and rotation by transforming the coordinate system to that of Unreal from AirSim NED
	this->SetActorRelativeLocation(FVector(settings.relative_pose.position.x() * 100, settings.relative_pose.position.y() * 100, -settings.relative_pose.position.z() * 100));
	this->SetActorRelativeRotation(FRotator(settings.pitch, settings.yaw, settings.roll));

	// Initialize the sensor further
	InitializeSensor();	

	// Toggle the sensor to know it is being used by AirSim
	used_by_airsim_ = true;
}

void ALidarCamera::InitializeSensor()
{
	// Setup the resolution and pixel format of each render target texture
	render_target_2D_depth_->InitCustomFormat((uint32)resolution_, (uint32)resolution_, PF_B8G8R8A8, true);
	render_target_2D_segmentation_->InitCustomFormat((uint32)resolution_, (uint32)resolution_, PF_B8G8R8A8, false); // The instance segmentation image requires normal gamma (none-linear)
	render_target_2D_intensity_->InitCustomFormat((uint32)resolution_, (uint32)resolution_, PF_B8G8R8A8, true);

	// Setup the capture component for the virtual depth camera
	capture_2D_depth_->TextureTarget = render_target_2D_depth_;
	capture_2D_depth_->bAlwaysPersistRenderingState = true;
	capture_2D_depth_->bCaptureEveryFrame = false;
	capture_2D_depth_->bCaptureOnMovement = false;
	capture_2D_depth_->bUseCustomProjectionMatrix = false;

	// Setup the capture component for the virtual instance segmentation camera
	FObjectAnnotator::SetViewForAnnotationRender(capture_2D_segmentation_->ShowFlags);
	capture_2D_segmentation_->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;

	render_target_2D_segmentation_->TargetGamma = 1;
	capture_2D_segmentation_->TextureTarget = render_target_2D_segmentation_;
	capture_2D_segmentation_->bAlwaysPersistRenderingState = true;
	capture_2D_segmentation_->bCaptureEveryFrame = false;
	capture_2D_segmentation_->bCaptureOnMovement = false;
	capture_2D_segmentation_->bUseCustomProjectionMatrix = false;

	// Setup the capture component for the virtual intensity camera
	capture_2D_intensity_->TextureTarget = render_target_2D_intensity_;
	capture_2D_intensity_->bAlwaysPersistRenderingState = true;
	capture_2D_intensity_->bCaptureEveryFrame = false;
	capture_2D_intensity_->bCaptureOnMovement = false;
	capture_2D_intensity_->bUseCustomProjectionMatrix = false;

	// Generate the XYZ-coordinates LUT based on the LiDAR sensor laser configuration
	GenerateLidarCoordinates();

	// Calculate the angular distance delta between two samples for both the horizontal and vertical axis
	h_delta_angle_ = (horizontal_fov_max_ - horizontal_fov_min_) / float(horizontal_samples_ - 1);
	v_delta_angle_ = (FMath::Abs(vertical_fov_min_) + vertical_fov_max_) / float(num_lasers_ - 1);

	// Calculate the ideal virtual camera FOV based on the total vertical FOV of the LiDAR
	target_fov_ = FMath::CeilToInt(FMath::Abs(vertical_fov_min_) + vertical_fov_max_);
	if (target_fov_ % 2 != 0) target_fov_ = target_fov_ + 1;


	// Find all objects that have a tag called 'LidarIgnore', this is best to do for objects like glass or others that would not reflect a real LiDAR sensor
	static const FName lidar_ignore_tag = TEXT("LidarIgnore");
	TArray<AActor*> actors;
	for (TActorIterator<AActor> ActorIterator(GetWorld()); ActorIterator; ++ActorIterator)
	{
		AActor* Actor = *ActorIterator;
		if (Actor && Actor != this && Actor->Tags.Contains(lidar_ignore_tag))actors.Add(Actor);
	}

	// Find all objects that have a tag called 'MarkedIgnore' which is an optional setting to also mask out these objects to the virtual cameras
	if (ignore_marked_) {
		static const FName marked_ignore_tag = TEXT("MarkedIgnore");
		for (TActorIterator<AActor> ActorIterator(GetWorld()); ActorIterator; ++ActorIterator)
		{
			AActor* Actor = *ActorIterator;
			if (Actor && Actor != this && Actor->Tags.Contains(marked_ignore_tag))actors.Add(Actor);
		}
	}
	capture_2D_depth_->HiddenActors = actors;
	capture_2D_segmentation_->HiddenActors = actors;
	capture_2D_intensity_->HiddenActors = actors;

	sensor_cur_angle_ = FMath::Fmod(horizontal_fov_min_, 360);
	hfov_ = abs(horizontal_fov_max_ - horizontal_fov_min_);

	initialized = true;
}

// Method function to call each time the sensor needs to update
bool ALidarCamera::Update(float delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud,
                          msr::airlib::vector<msr::airlib::real_T>& point_cloud_final)
{
	// Initialize the sensor if not done yet (when not setup by AirSim)
	if(!initialized) {
		InitializeSensor();
	}

	// Toggle to indicate to AirSim that the sensor has done a full measurement and that the point_cloud_final holds a new full measurement that can be given to the API
	bool refresh_pointcloud = false;

	// Calculate the added rotation of the sensor by this update based on the time that has pased and the rotational speed of the sensor
	float sensor_rotation_angle_ = hfov_ * delta_time * sensor_rotation_frequency_;
	sensor_sum_rotation_angle_ += sensor_rotation_angle_;

	// If the rotation in this frame is larger than the minimum horizontal FOV delta, a new calculation of points needs to be made
	if (sensor_sum_rotation_angle_ > h_delta_angle_) {

		// If the full horizontal fov was completed last frame, reset the starting angle again
		if (reset_hfov_) {
			sensor_cur_angle_ = FMath::Fmod(horizontal_fov_min_, 360);
			sensor_prev_rotation_angle_ = 0;
			completed_hfov_ = 0;
			reset_hfov_ = false;
		}

		// The ideal virtual camera FOV is set based on the vertical FOV, but if it is a large rotation that has to be calculated, set it to be a even number that is equal to the
		// rotation + 3 horizontal FOV deltas. However, to make the pixel to LiDAR laser coordinates work, the FOV needs to be a minimum of 90 degrees.
		int32 cur_fov = target_fov_;
		if (sensor_sum_rotation_angle_ > target_fov_) cur_fov = FMath::CeilToInt(FMath::Min(sensor_sum_rotation_angle_ + (3 * h_delta_angle_), 178.0f));
		if (cur_fov % 2 != 0) cur_fov += 1;
		if (cur_fov < 90) cur_fov = 90;
		capture_2D_depth_->FOVAngle = cur_fov;
		capture_2D_segmentation_->FOVAngle = cur_fov;
		capture_2D_intensity_->FOVAngle = cur_fov;

		// Rotate the physical cameras in the Unreal world to the new location based on what happened in previous frames
		RotateCamera(FMath::Fmod(sensor_cur_angle_ + sensor_prev_rotation_angle_ + (cur_fov / 2), 360));
		sensor_cur_angle_ = FMath::Fmod(sensor_cur_angle_ + sensor_prev_rotation_angle_, 360);

		// If the rotation is bigger than the allowed FOV, cap the rotation (it will be further completed in the next frame)
		if (sensor_sum_rotation_angle_ > cur_fov)sensor_sum_rotation_angle_ = cur_fov;

		// If the rotation will extend beyond the total horizontal FOV if it is not full 360, cap the rotation to the remaining FOV
		// And also make sure to reset the start rotation next frame
		if (sensor_sum_rotation_angle_ >= hfov_ - completed_hfov_ && hfov_ != 360) {
			sensor_sum_rotation_angle_ = hfov_ - completed_hfov_;
			reset_hfov_ = true;
		}
		completed_hfov_ += sensor_sum_rotation_angle_;

		if (waited_frames_ < wait_frames_) {
			waited_frames_++;
		}
		else {

			capture_2D_depth_->CaptureScene();
			capture_2D_segmentation_->CaptureScene();
			capture_2D_intensity_->CaptureScene();
			refresh_pointcloud = SampleRenders(sensor_sum_rotation_angle_, cur_fov, point_cloud, point_cloud_final);
		}

	

		// Set up the values for the next frame
		sensor_prev_rotation_angle_ = sensor_sum_rotation_angle_;
		sensor_sum_rotation_angle_ = 0;
	}
	return refresh_pointcloud;
}

void ALidarCamera::updateInstanceSegmentationAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList) {
	capture_2D_segmentation_->ShowOnlyComponents = ComponentList;
}

void ALidarCamera::updateAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList) {
	capture_2D_segmentation_->ShowOnlyComponents = ComponentList;
}


// Generate the XYZ-coordinates LUT based on the LiDAR sensor laser configuration
void ALidarCamera::GenerateLidarCoordinates() {
	h_angles_ = LinearSpacedArray(horizontal_fov_min_, horizontal_fov_max_ - h_delta_angle_, horizontal_samples_);
	v_angles_ = LinearSpacedArray(vertical_fov_min_, vertical_fov_max_, num_lasers_);
	for (int32 h_cur_index = 0; h_cur_index < horizontal_samples_; h_cur_index++)
	{
		h_angles_atan2_.Add(FMath::Fmod(h_angles_[h_cur_index], 360));
		float h_angle_0 = h_angles_[h_cur_index];
		for (int32 v_cur_index = 0; v_cur_index < num_lasers_; v_cur_index++)
		{
			polar_to_cartesian_lut_.Add(FVector(FMath::Cos(FMath::DegreesToRadians(v_angles_[v_cur_index])) * FMath::Cos(FMath::DegreesToRadians(h_angle_0)),
				FMath::Cos(FMath::DegreesToRadians(v_angles_[v_cur_index])) * FMath::Sin(FMath::DegreesToRadians(h_angle_0)),
				FMath::Sin(FMath::DegreesToRadians(v_angles_[v_cur_index]))));
		}
	}
}

// Rotate the physical cameras in the Unreal world
void ALidarCamera::RotateCamera(float sensor_rotation_angle)
{
	//UAirBlueprintLib::RunCommandOnGameThread([this, sensor_rotation_angle]() {
	//	capture_2D_depth_->SetRelativeRotation(FRotator(0, sensor_rotation_angle, 0));
	//	capture_2D_segmentation_->SetRelativeRotation(FRotator(0, sensor_rotation_angle, 0));
	//	capture_2D_intensity_->SetRelativeRotation(FRotator(0, sensor_rotation_angle, 0));
	//	}, false);

	capture_2D_depth_->SetRelativeRotation(FRotator(0, sensor_rotation_angle, 0));
	capture_2D_segmentation_->SetRelativeRotation(FRotator(0, sensor_rotation_angle, 0));
	capture_2D_intensity_->SetRelativeRotation(FRotator(0, sensor_rotation_angle, 0));

}

// Perform the camera to LiDAR pointcloud conversion
bool ALidarCamera::SampleRenders(float sensor_rotation_angle, float fov, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final) {

	//CheckNotBlockedOnRenderThread();

	//game_viewport_ = this->GetWorld()->GetGameViewport();


	//AsyncTask(ENamedThreads::GameThread, [this]() {
	//	bool test = IsInGameThread();
	//	check(IsInGameThread());	

	//	saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
	//	game_viewport_->bDisableWorldRendering = 0;
	//	end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this] {
	//		bool test2 = IsInGameThread();
	//		check(IsInGameThread());

	//		// The completion is called immeidately after GameThread sends the
	//		// rendering commands to RenderThread. Hence, our ExecuteTask will
	//		// execute *immediately* after RenderThread renders the scene!
	//		ALidarCamera* This = this;
	//		ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
	//			(
	//				[This](FRHICommandListImmediate& RHICmdList) {
	//					This->ExecuteScanTask();
	//				});

	//		game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;

	//		assert(end_draw_handle_.IsValid());
	//		game_viewport_->OnEndDraw().Remove(end_draw_handle_);
	//	});

	//	// while we're still on GameThread, enqueue request for capture the scene!
	//	capture_2D_depth_->CaptureScene();
	//	capture_2D_intensity_->CaptureScene();
	//	capture_2D_segmentation_->CaptureScene();
	//	
	//});

	//// wait for this task to complete
	//while (!wait_signal_->waitFor(5)) {
	//	// log a message and continue wait
	//	// lamda function still references a few objects for which there is no refcount.
	//	// Walking away will cause memory corruption, which is much more difficult to debug.
	//	UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for lidar data"));
	//}


	// Toggle to indicate to AirSim that the sensor has done a full measurement and that the point_cloud_final holds a new full measurement that can be given to the API
	bool refresh_pointcloud = false;

	// Declare the buffers for holding the RGB data from the cameras
	TArray<FColor> buffer_2D_depth_;
	TArray<FColor> buffer_2D_segmentation_;
	TArray<FColor> buffer_2D_intensity_;

	// Read the RGB data from the cameras and transfer them to the buffers
	FTextureRenderTarget2DResource* render_target_2D_depth = (FTextureRenderTarget2DResource*)capture_2D_depth_->TextureTarget->GetResource();
	render_target_2D_depth->ReadPixels(buffer_2D_depth_);
	if (generate_groundtruth_) {
		FTextureRenderTarget2DResource* render_target_2D_segmentation;
		render_target_2D_segmentation = (FTextureRenderTarget2DResource*)capture_2D_segmentation_->TextureTarget->GetResource();
		FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
		flags.SetLinearToGamma(false);
		render_target_2D_segmentation->ReadPixels(buffer_2D_segmentation_);
	}
	if (generate_intensity_) {
		FTextureRenderTarget2DResource* render_target_2D_intensity;
		render_target_2D_intensity = (FTextureRenderTarget2DResource*)capture_2D_intensity_->TextureTarget->GetResource();
		render_target_2D_intensity->ReadPixels(buffer_2D_intensity_);
	}

	// Calculate the camera intrensic parameters
	float c_x = resolution_ / 2.0f;
	float c_y = resolution_ / 2.0f;
	float f_x = resolution_ / (2.0f * FMath::Tan(FMath::DegreesToRadians(fov / 2.0f)));
	float f_y = resolution_ / (2.0f * FMath::Tan(FMath::DegreesToRadians(fov / 2.0f)));

	// Calculate the first and last horizontal angle of the LiDAR that will be captured in this frame
	int32 h_first_index = h_cur_atan2_index_ + 1;
	float h_max_angle = FMath::Fmod(sensor_cur_angle_ + sensor_rotation_angle, 360);
	int32 h_last_index = getIndexLowerClosest(h_angles_atan2_, h_max_angle);

	// variable that keeps the current horizontal angle index that is being calculated
	int32 h_cur_index = h_first_index;

	// Calculate the last horizontal angle that was measured in the previous frame, used to seeing if the full pointcloud is completed
	float h_prev_angle = 10000; // This is done to avoid an issue with the very first measurement
	if (h_cur_atan2_index_ != -1) {
		h_prev_angle = h_angles_[h_cur_atan2_index_];
	}

	// get the current rain intensity from the AirSim API
	float rain_value;
	if (generate_intensity_) {
		rain_value = UWeatherLib::getWeatherParamScalar(this->GetWorld(), msr::airlib::Utils::toEnum<EWeatherParamScalar>(0));
	}

	// State boolean to check if the loop is still the first and last horizontal angle range that will be captured in this frame
	bool within_range = true;

	while (within_range) {

		// Calculate the index so that it keeps within the Eucledian Plane (between 0 and 360 degrees) by making it circular from 0 to the total horizontal measurement number
		h_cur_atan2_index_ = (h_cur_index) % horizontal_samples_;

		// If the current index is the last to perform during this frame, disable the loop
		if (h_last_index == h_cur_atan2_index_)within_range = false;

		// Get the current horizontal angle, also in Eucledian plane form (between 0 and 360 degrees)
		float h_cur_angle = h_angles_[h_cur_atan2_index_];
		float h_cur_atan2_angle = FMath::Fmod(FMath::Fmod(FMath::Fmod(h_cur_angle, 360) - sensor_cur_angle_, 360) - (fov / 2), 360);

		// Calculate the cosine and sine of the horizontal angle and calculate the pixel index from the render texture target that matches this laser's horizontal angle
		float h_cur_angle_cos = FMath::Cos(FMath::DegreesToRadians(h_cur_atan2_angle));
		float h_cur_angle_sin = FMath::Sin(FMath::DegreesToRadians(h_cur_atan2_angle));
		int32 h_pixel = FMath::FloorToInt(((h_cur_angle_sin * f_x) / h_cur_angle_cos) + c_x);
		if (h_pixel == -1)h_pixel = 0; // for edge case avoiding
		if (h_pixel == resolution_)h_pixel = resolution_ - 1;  // for edge case avoiding
		
		// Loop the vertical lasers
		for (int32 v_cur_index = 0; v_cur_index < num_lasers_; v_cur_index++)
		{
			// if the previous horizontal angle was larger than the current one, it means the sensor has done a full circle and a new pointcloud can be started
			if (used_by_airsim_) {
				if ((h_prev_angle >h_cur_angle) && (point_cloud.size() != 0)) {
					if (v_cur_index == 0) {
						// Check edge cases where the amount of points in the pointcloud doesnt equal the desired full pointcloud size. In this case, throw away the current data
						if ((((int)point_cloud.size() / 5) != horizontal_samples_ * num_lasers_))
						{
							UE_LOG(LogTemp, Warning, TEXT("Pointcloud incorrect size! points:%i %f %f"), (int)(point_cloud.size() / 5), h_prev_angle, h_cur_angle);
							point_cloud.clear();
							refresh_pointcloud = false;
						}
						// Else, save the completed pointcloud into the right array and clear the current one
						else {
							//UE_LOG(LogTemp, Warning, TEXT("REFRESH at angle: %f, with prev angle: %f"), h_cur_angle, h_prev_angle);
							point_cloud_final = point_cloud;
							point_cloud.clear();
							refresh_pointcloud = true;
						}
					}
				}
			}		

			// Calculate the cosine and sine of the verticle angle and calculate the pixel index from the render texture target that matches this laser's verticle angle
			float v_cur_angle = v_angles_[v_cur_index];
			float v_cur_angle_cos = FMath::Cos(FMath::DegreesToRadians(v_cur_angle));
			float v_cur_angle_sin = FMath::Sin(FMath::DegreesToRadians(v_cur_angle));
			int32 v_pixel = FMath::FloorToInt((v_cur_angle_sin * -f_y) / (v_cur_angle_cos * h_cur_angle_cos) + c_y);

			// If the pixel coordinates are within bounds of the render target texture (should always be the case) we can proceed to read from it
			if (h_pixel >= 0 && h_pixel < resolution_ && v_pixel >= 0 && v_pixel < resolution_) {

				// Get the depth value in centimeters, the depth value is spread of the full 3 bytes to achieve three bytes unsigned precision
				FColor value_depth = buffer_2D_depth_[h_pixel + (v_pixel * resolution_)];
				float depth = 100000 * ((value_depth.R + value_depth.G * 256 + value_depth.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));

			    // Added random distance based noise
				if (generate_distance_noise_) {
					float distance_noise = dist_(gen_) * (1 + ((depth / 100) / max_range_) * (distance_noise_scale_ - 1));
					depth = depth + distance_noise;
				}

				// If the depth value is beneath the maximum detected range of the sensor, proceed, otherwise discard this point
				if (depth < (max_range_ * 100)) {

					// Add noise based on the rain intensity, see publication for more information
					if (generate_intensity_) {
						float noise = dist_(gen_) * 0.02 * depth * FMath::Pow(1 - FMath::Exp(-rain_max_intensity_ * rain_value), 2);
						depth = depth + noise;
					}

					// Calculate the true distance based on the projection and get the XYZ coordinates for the 3D pointcloud from the polar angles of the laser
					float distance = depth / (v_cur_angle_cos * h_cur_angle_cos);
					FVector point = (distance * polar_to_cartesian_lut_[v_cur_index + (h_cur_atan2_index_ * num_lasers_)]);

					// State that determines based on the surface material and the angle of impact if the laser signal still gets reflected based on the capability of the sensor,
					// if not the point is dropped. See the paper for more details
					bool threshold_enable = true;

					// Default values for groundtruth segmentation and intensity
					FColor value_segmentation(0, 0, 0);
					float final_intensity = 1;

					if (generate_intensity_) {

						// Get the impact angle in radians, it is spread of the full 3 bytes to achieve three bytes unsigned precision
						FColor value_intensity = buffer_2D_intensity_[h_pixel + (v_pixel * resolution_)];
						float impact_angle = ((value_intensity.R + value_intensity.G * 256 + value_intensity.B * 256 * 256) / static_cast<float>(256 * 256 * 256 - 1));

						// Get the stencil color (saved in the alpha channel of the intensity render target) that defines the surface material 
						// and therefore the Lambertian reflectance coefficient of that material and calculate together with the impact angle on that material the final intensity.
						// Furthermroe, detract the rain-intensity based drop as well. 
						// See the paper for more details
						final_intensity = impact_angle * material_map_.at(value_intensity.A) * FMath::Exp(-2.0f * rain_constant_a_ * FMath::Pow(rain_max_intensity_ * rain_value, rain_constant_b_) * (depth / 100.0f));

						// if the intensity based on the surface material and the impact angle is below the (linear) reflectance limit function the point will be dropped
						// See the paper for more details
						if ((impact_angle * material_map_.at(value_intensity.A)) < (max_range_ / max_range_lambertian_percentage_ / 100) * depth / 100.0)threshold_enable = false;

						// If in the right debug drawing mode, draw the surface material type to screen in the final pointcloud formation
						if (draw_debug_ && debug_draw_mode_ == 2 && threshold_enable) {
							FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
							UAirBlueprintLib::DrawPoint(this->GetWorld(), point_draw, 5, FColor(unique_colors_[value_intensity.A * 3], unique_colors_[(value_intensity.A * 3) + 1], unique_colors_[(value_intensity.A * 3) + 2], 1), false, (1 / (sensor_rotation_frequency_ * 4)));
						}

						// If in the right debug drawing mode, draw the impact angle to screen in the final pointcloud formation
						if (draw_debug_ && debug_draw_mode_ == 3 && threshold_enable) {
							FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
							UAirBlueprintLib::DrawPoint(this->GetWorld(), point_draw, 5, FColor(0, FMath::FloorToInt(impact_angle * 254), 0, 1), false, (1 / (sensor_rotation_frequency_ * 4)));
						}

						// If in the right debug drawing mode, draw the final intensity to screen in the final pointcloud formation
						if (draw_debug_ && debug_draw_mode_ == 4 && threshold_enable) {
							FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
							UAirBlueprintLib::DrawPoint(this->GetWorld(), point_draw, 5, FColor(0, 0, FMath::FloorToInt(final_intensity * 254), 1), false, 2);
						}

					}

					if (generate_groundtruth_) {
						// Get the RGB value associated with the instance segmentation index of the object detected in this point
						value_segmentation = buffer_2D_segmentation_[h_pixel + (v_pixel * resolution_)];

						// If in the right debug drawing mode, draw the instance segmentation color to screen in the final pointcloud formation
						if (draw_debug_ && debug_draw_mode_ == 1 && threshold_enable) {
							FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
							UAirBlueprintLib::DrawPoint(this->GetWorld(), point_draw, 5, FColor(value_segmentation.R, value_segmentation.G, value_segmentation.B, 1), false, 2);
						}
					}

					// If the point is not dropped based on the reflectance limit function of the sensor, add the final point data to the pointcloud, else place an empty point
					if (threshold_enable && used_by_airsim_) {
						point_cloud.emplace_back(point.X / 100);
						point_cloud.emplace_back(point.Y / 100);
						point_cloud.emplace_back(-point.Z / 100);
						std::uint32_t rgb = ((std::uint32_t)value_segmentation.R << 16 | (std::uint32_t)value_segmentation.G << 8 | (std::uint32_t)value_segmentation.B);
						point_cloud.emplace_back(rgb);
						point_cloud.emplace_back(final_intensity);
					}
					else if(used_by_airsim_){
						point_cloud.emplace_back(0);
						point_cloud.emplace_back(0);
						point_cloud.emplace_back(0);
						point_cloud.emplace_back(0);
						point_cloud.emplace_back(0);
					}

					// If in the right debug drawing mode, draw the final pointcloud formation to the screen in a static color
					if (draw_debug_ && debug_draw_mode_ == 0 && threshold_enable) {
						FVector point_draw = this->GetActorRotation().RotateVector(point) + this->GetActorLocation();
						UAirBlueprintLib::DrawPoint(this->GetWorld(), point_draw, 5, FColor::Blue, false, (1 / (sensor_rotation_frequency_ * 4)));
					}
				}
				else if (used_by_airsim_) {
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
				}
			}
			else {
				//if (h_pixel >= resolution_ || h_pixel < 0)UE_LOG(LogTemp, Warning, TEXT("Point H index out of bounds: %i"), h_pixel);
				//if (v_pixel >= resolution_ || v_pixel < 0)UE_LOG(LogTemp, Warning, TEXT("Point V index out of bounds: %i"), v_pixel);
				if (used_by_airsim_) {
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
					point_cloud.emplace_back(0);
				}
			}			
		}

		// Set the variables right for the next horizontal angle
		h_prev_angle = h_cur_angle;
		h_cur_index += 1;
	}
	return refresh_pointcloud;
}
//
//void ALidarCamera::ExecuteScanTask()
//{
//	FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
//	auto render_target_2D_depth = capture_2D_depth_->TextureTarget->GetRenderTargetResource();
//	if (render_target_2D_depth != nullptr) {
//		const FTexture2DRHIRef& rhi_texture = render_target_2D_depth->GetRenderTargetTexture();
//		FIntPoint size = rhi_texture->GetSizeXY();
//		FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
//		flags.SetLinearToGamma(false);
//
//		RHICmdList.ReadSurfaceData(
//			rhi_texture,
//			FIntRect(0, 0, size.X, size.Y),
//			buffer_2D_depth_,
//			flags);
//	}
//
//	if (generate_groundtruth_) {
//		auto render_target_2D_segmentation = capture_2D_segmentation_->TextureTarget->GetRenderTargetResource();			
//		/*FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
//		flags.SetLinearToGamma(false);*/
//		if (render_target_2D_segmentation != nullptr) {
//			const FTexture2DRHIRef& rhi_texture = render_target_2D_segmentation->GetRenderTargetTexture();
//			FIntPoint size = rhi_texture->GetSizeXY();
//			FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
//			flags.SetLinearToGamma(false);
//
//			RHICmdList.ReadSurfaceData(
//				rhi_texture,
//				FIntRect(0, 0, size.X, size.Y),
//				buffer_2D_segmentation_,
//				flags);
//		}
//	}
//	if (generate_intensity_) {
//		auto render_target_2D_intensity = capture_2D_intensity_->TextureTarget->GetRenderTargetResource();
//		if (render_target_2D_intensity != nullptr) {
//			const FTexture2DRHIRef& rhi_texture = render_target_2D_intensity->GetRenderTargetTexture();
//			FIntPoint size = rhi_texture->GetSizeXY();
//			FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
//			flags.SetLinearToGamma(false);
//
//			RHICmdList.ReadSurfaceData(
//				rhi_texture,
//				FIntRect(0, 0, size.X, size.Y),
//				buffer_2D_intensity_,
//				flags);
//		}
//	}
//
//	wait_signal_->signal();
//	
//}

// Pre-calculated array of unique colors that are easy to distinguish from one another 
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