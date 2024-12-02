#include "PIPCamera.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "ImageUtils.h"
#include "Annotation/AnnotationComponent.h"
#include "Annotation/ObjectAnnotator.h"
#include <string>
#include <exception>
#include "AirBlueprintLib.h"

//CinemAirSim
APIPCamera::APIPCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer
                .SetDefaultSubobjectClass<UCineCameraComponent>(TEXT("CameraComponent")))
{
    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/AirSim/HUDAssets/CameraSensorNoise.CameraSensorNoise'"));
    if (mat_finder.Succeeded()) {
        noise_material_static_ = mat_finder.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create noise material for the PIPCamera",
                                           "",
                                           LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> dist_mat_finder(TEXT("Material'/AirSim/HUDAssets/CameraDistortion.CameraDistortion'"));
    if (dist_mat_finder.Succeeded()) {
        distortion_material_static_ = dist_mat_finder.Object;
        distortion_param_collection_ = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, TEXT("'/AirSim/HUDAssets/DistortionParams.DistortionParams'")));
    }
    else{
        UAirBlueprintLib::LogMessageString("Cannot create distortion material for the PIPCamera",
                                           "", LogDebugLevel::Failure);
    }

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder2(TEXT("Material'/AirSim/HUDAssets/CameraSensorLensDistortion.CameraSensorLensDistortion'"));
	if (mat_finder2.Succeeded())
	{
		lens_distortion_material_static_ = mat_finder2.Object;
	}
	else{
		UAirBlueprintLib::LogMessageString("Cannot create lens distortion material for the PIPCamera", "", LogDebugLevel::Failure);
    }

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder3(TEXT("Material'/AirSim/HUDAssets/CameraSensorLensDistortionInvert.CameraSensorLensDistortionInvert'"));
	if (mat_finder3.Succeeded())
	{
		lens_distortion_invert_material_static_ = mat_finder3.Object;
	}
	else
		UAirBlueprintLib::LogMessageString("Cannot create inverted lens distortion material for the PIPCamera",
			"", LogDebugLevel::Failure);


    PrimaryActorTick.bCanEverTick = true;

    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Scene), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthPlanar), EPixelFormat::PF_DepthStencil); // not used. init_auto_format is called in setupCameraFromSettings()
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthPerspective), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthVis), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DisparityNormalized), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Segmentation), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::SurfaceNormals), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Infrared), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::OpticalFlow), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::OpticalFlowVis), EPixelFormat::PF_B8G8R8A8);

    object_filter_ = FObjectFilter();

    static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Models/AnnotationSphere.AnnotationSphere'"));
    if (loadedMesh.Succeeded())
    {
        annotation_sphere_static_ = loadedMesh.Object;
    }
}

void APIPCamera::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    //CinemAirSim
    camera_ = UAirBlueprintLib::GetActorComponent<UCineCameraComponent>(this, TEXT("CameraComponent"));
    captures_.Init(nullptr, imageTypeCount());
    render_targets_.Init(nullptr, imageTypeCount());
    detections_.Init(nullptr, imageTypeCount());

    captures_[Utils::toNumeric(ImageType::Scene)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPlanar)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPlanarCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPerspective)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPerspectiveCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthVis)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthVisCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DisparityNormalized)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DisparityNormalizedCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Segmentation)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SegmentationCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Infrared)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("InfraredCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::SurfaceNormals)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("NormalsCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::OpticalFlow)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("OpticalFlowCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::OpticalFlowVis)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("OpticalFlowVisCaptureComponent"));

    for (unsigned int i = 0; i < imageTypeCount(); ++i) {
        detections_[i] = NewObject<UDetectionComponent>(this);
        if (detections_[i]) {
            detections_[i]->SetupAttachment(captures_[i]);
            detections_[i]->RegisterComponent();
            detections_[i]->Deactivate();
        }
    }
    //set initial focal length
    camera_->CurrentFocalLength = 11.9;

    FObjectAnnotator::SetViewForAnnotationRender(captures_[Utils::toNumeric(ImageType::Segmentation)]->ShowFlags);
    captures_[Utils::toNumeric(ImageType::Segmentation)]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();

    noise_materials_.AddZeroed(imageTypeCount() + 1);
    distortion_materials_.AddZeroed(imageTypeCount() + 1);
	lens_distortion_materials_.AddZeroed(imageTypeCount() + 1);

    //by default all image types are disabled
    camera_type_enabled_.assign(imageTypeCount(), false);

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        if(image_type == Utils::toNumeric(ImageType::Scene)) {
            captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
        }
        else {
            captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        }      
        render_targets_[image_type] = NewObject<UTextureRenderTarget2D>();
    }

    //We set all cameras to start as nodisplay
    //This improves performance because the capture components are no longer updating every frame and only update while requesting an image
    onViewModeChanged(true);

    gimbal_stabilization_ = 0;
    gimbald_rotator_ = this->GetActorRotation();
    this->SetActorTickEnabled(false);

    if (distortion_param_collection_)
        distortion_param_instance_ = this->GetWorld()->GetParameterCollectionInstance(distortion_param_collection_);
}


msr::airlib::AirSimSettings::CameraSetting APIPCamera::getParams() const
{
    return sensor_params_;
}

msr::airlib::ProjectionMatrix APIPCamera::getProjectionMatrix() const
{
    msr::airlib::ProjectionMatrix mat;

    // TODO: This is always the case in current request, might need to change to include annotation if needed
	ImageType image_type = ImageType::Scene;

    //TODO: avoid the need to override const cast here
    const_cast<APIPCamera*>(this)->setCameraTypeEnabled(image_type, true);
    const USceneCaptureComponent2D* capture = const_cast<APIPCamera*>(this)->getCaptureComponent(image_type, false);
    if (capture) {
        FMatrix proj_mat_transpose;

        FIntPoint render_target_size(capture->TextureTarget->GetSurfaceWidth(), capture->TextureTarget->GetSurfaceHeight());
        float x_axis_multiplier = 1.0f;
        float y_axis_multiplier = render_target_size.X / (float)render_target_size.Y;

        if (render_target_size.X < render_target_size.Y) {
            // if the viewport is taller than it is wide
            x_axis_multiplier = render_target_size.Y / static_cast<float>(render_target_size.X);
            y_axis_multiplier = 1.0f;
        }

        if (capture->ProjectionType == ECameraProjectionMode::Orthographic) {
            check((int32)ERHIZBuffer::IsInverted);
            const float OrthoWidth = capture->OrthoWidth / 2.0f;
            const float OrthoHeight = capture->OrthoWidth / 2.0f * x_axis_multiplier / y_axis_multiplier;

            const float NearPlane = 0;
            const float FarPlane = WORLD_MAX / 8.0f;

            const float ZScale = 1.0f / (FarPlane - NearPlane);
            const float ZOffset = -NearPlane;

            proj_mat_transpose = FReversedZOrthoMatrix(
                OrthoWidth,
                OrthoHeight,
                ZScale,
                ZOffset);
        }
        else {
            float halfFov = Utils::degreesToRadians(capture->FOVAngle) / 2;
            if ((int32)ERHIZBuffer::IsInverted) {
                proj_mat_transpose = FReversedZPerspectiveMatrix(
                    halfFov,
                    halfFov,
                    x_axis_multiplier,
                    y_axis_multiplier,
                    GNearClippingPlane,
                    GNearClippingPlane);
            }
            else {
                //The FPerspectiveMatrix() constructor actually returns the transpose of the perspective matrix.
                proj_mat_transpose = FPerspectiveMatrix(
                    halfFov,
                    halfFov,
                    x_axis_multiplier,
                    y_axis_multiplier,
                    GNearClippingPlane,
                    GNearClippingPlane);
            }
        }

        //Takes a vector from NORTH-EAST-DOWN coordinates (AirSim) to EAST-UP-SOUTH coordinates (Unreal). Leaves W coordinate unchanged.
        FMatrix coordinateChangeTranspose = FMatrix(
            FPlane(0, 0, -1, 0),
            FPlane(1, 0, 0, 0),
            FPlane(0, -1, 0, 0),
            FPlane(0, 0, 0, 1));

        FMatrix projMatTransposeInAirSim = coordinateChangeTranspose * proj_mat_transpose;

        //Copy the result to an airlib::ProjectionMatrix while taking transpose.
        for (auto row = 0; row < 4; ++row)
            for (auto col = 0; col < 4; ++col)
                mat.matrix[col][row] = projMatTransposeInAirSim.M[row][col];
    }
    else
        mat.setTo(Utils::nan<float>());

    return mat;
}

void APIPCamera::Tick(float DeltaTime)
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
    if (sensor_params_.draw_sensor) {
        UAirBlueprintLib::DrawPoint(this->GetWorld(), this->GetActorTransform().GetLocation(), 5, FColor::Black, false, 0.3);
        UAirBlueprintLib::DrawCoordinateSystem(this->GetWorld(), this->GetActorLocation(), this->GetActorRotation(), 25, false, 0.3, 10);
    }
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    int image_count_to_delete = static_cast<int>(Utils::toNumeric(ImageType::Count));
    if (noise_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (noise_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(noise_materials_[image_type + 1]);
        }
        if (noise_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(noise_materials_[0]);
    }

	if (lens_distortion_materials_.Num()) {
		for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
			if (lens_distortion_materials_[image_type + 1])
				captures_[image_type]->PostProcessSettings.RemoveBlendable(lens_distortion_materials_[image_type + 1]);
		}
		if (lens_distortion_materials_[0])
			camera_->PostProcessSettings.RemoveBlendable(lens_distortion_materials_[0]);
	}

    noise_material_static_ = nullptr;
	lens_distortion_material_static_ = nullptr;
	lens_distortion_invert_material_static_ = nullptr;
    annotation_sphere_static_ = nullptr;
    noise_materials_.Empty();
	lens_distortion_materials_.Empty();

    if (distortion_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (distortion_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(distortion_materials_[image_type + 1]);
        }
        if (distortion_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(distortion_materials_[0]);
    }

    distortion_material_static_ = nullptr;
    distortion_materials_.Empty();

	annotator_name_to_index_map_.Empty();
    sphere_annotation_component_map_.Empty();


    int camera_full_count = static_cast<int>(cameraCaptureCount());
    for (int current_camera = 0; current_camera < camera_full_count; ++current_camera) {
        //use final color for all calculations
        if (current_camera == Utils::toNumeric(ImageType::Segmentation) || current_camera >= image_count_to_delete - 2) {
            captures_[current_camera]->ShowOnlyComponents.Empty();
        }        
        captures_[current_camera] = nullptr;
        render_targets_[current_camera] = nullptr;
        detections_[current_camera] = nullptr;
    }
    captures_.Empty();
	render_targets_.Empty();
	detections_.Empty();
}

unsigned int APIPCamera::imageTypeCount()
{
    return Utils::toNumeric(ImageType::Count) - 1;
}

unsigned int APIPCamera::cameraCaptureCount()
{
    return static_cast<unsigned int>(captures_.Num());
}

void APIPCamera::showToScreen()
{
    camera_->SetVisibility(true);
    camera_->Activate();
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    controller->SetViewTarget(this);
    UAirBlueprintLib::LogMessage(TEXT("Camera: "), GetName(), LogDebugLevel::Informational);
}

void APIPCamera::disableAll()
{
    disableMain();
    disableAllPIP();
}

bool APIPCamera::getCameraTypeEnabled(ImageType type, std::string annotation_name) const
{
    if (type == ImageType::Annotation) {
        return camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        return camera_type_enabled_[Utils::toNumeric(type)];
    }    
}

bool APIPCamera::GetAnnotationNameExist(std::string annotation_name)
{
    if (annotator_name_to_index_map_.Contains(FString(annotation_name.c_str())))
        return true;
    else
        return false;
}

void APIPCamera::setCameraTypeEnabled(ImageType type, bool enabled, std::string annotation_name)
{
    enableCaptureComponent(type, enabled, annotation_name);
}

void APIPCamera::setCameraOrientation(const FRotator& rotator)
{
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = rotator.Pitch;
        gimbald_rotator_.Roll = rotator.Roll;
        gimbald_rotator_.Yaw = rotator.Yaw;
    }
    this->SetActorRelativeRotation(rotator);
}


void APIPCamera::setCaptureUpdate(USceneCaptureComponent2D* capture, bool nodisplay)
{
    capture->bCaptureEveryFrame = !nodisplay;
    capture->bCaptureOnMovement = !nodisplay;
    capture->bAlwaysPersistRenderingState = true;
}

void APIPCamera::setCameraTypeUpdate(ImageType type, bool nodisplay, std::string annotation_name)
{
    USceneCaptureComponent2D* capture = getCaptureComponent(type, false, annotation_name);
    if (capture != nullptr)
        setCaptureUpdate(capture, nodisplay);
}

void APIPCamera::setCameraPose(const msr::airlib::Pose& relative_pose)
{
    FTransform pose = ned_transform_->fromRelativeNed(relative_pose);

    FVector position = pose.GetLocation();
    this->SetActorRelativeLocation(pose.GetLocation());

    FRotator rotator = pose.GetRotation().Rotator();
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = rotator.Pitch;
        gimbald_rotator_.Roll = rotator.Roll;
        gimbald_rotator_.Yaw = rotator.Yaw;
    }
    else {
        this->SetActorRelativeRotation(rotator);
    }
}

void APIPCamera::setCameraFoV(float fov_degrees)
{
    for (unsigned int image_type = 0; image_type < cameraCaptureCount(); ++image_type) {
        captures_[image_type]->FOVAngle = fov_degrees;
    }
    camera_->SetFieldOfView(fov_degrees);
}

msr::airlib::CameraInfo APIPCamera::getCameraInfo() const
{
    msr::airlib::CameraInfo camera_info;

    camera_info.pose.position = ned_transform_->toLocalNed(this->GetActorLocation());
    camera_info.pose.orientation = ned_transform_->toNed(this->GetActorRotation().Quaternion());
    camera_info.fov = camera_->FieldOfView;
    camera_info.proj_mat = getProjectionMatrix();
    return camera_info;
}

std::vector<float> APIPCamera::getDistortionParams() const
{
    std::vector<float> param_values(5, 0.0);

    auto getParamValue = [this](const auto& name, float& val) {
        distortion_param_instance_->GetScalarParameterValue(FName(name), val);
    };

    getParamValue(TEXT("K1"), param_values[0]);
    getParamValue(TEXT("K2"), param_values[1]);
    getParamValue(TEXT("K3"), param_values[2]);
    getParamValue(TEXT("P1"), param_values[3]);
    getParamValue(TEXT("P2"), param_values[4]);

    return param_values;
}

void APIPCamera::setDistortionParam(const std::string& param_name, float value)
{
    distortion_param_instance_->SetScalarParameterValue(FName(param_name.c_str()), value);
}

void APIPCamera::updateInstanceSegmentationAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, bool only_hide) {
    if(!only_hide)
        captures_[Utils::toNumeric(ImageType::Segmentation)]->ShowOnlyComponents = ComponentList;
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    for(TWeakObjectPtr<UPrimitiveComponent> component : ComponentList) {
        captures_[Utils::toNumeric(ImageType::Scene)]->HiddenComponents.AddUnique(component);
        controller->HiddenPrimitiveComponents.AddUnique(component);
	}
}

void APIPCamera::updateAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, FString annotation_name, bool only_hide) {
    if (!only_hide) {
        captures_[annotator_name_to_index_map_[annotation_name]]->ShowOnlyComponents = ComponentList;
        if (sphere_annotation_component_map_.Contains(annotation_name))
            captures_[annotator_name_to_index_map_[annotation_name]]->ShowOnlyComponents.Add(sphere_annotation_component_map_[annotation_name]);
    }   
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();

    for (TWeakObjectPtr<UPrimitiveComponent> component : ComponentList) {
        captures_[Utils::toNumeric(ImageType::Scene)]->HiddenComponents.AddUnique(component);
        controller->HiddenPrimitiveComponents.AddUnique(component);
    }
}

void APIPCamera::addAnnotationCamera(FString name, FObjectAnnotator::AnnotatorType type, float max_view_distance)
{
    USceneCaptureComponent2D* new_capture = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D ::StaticClass(), *name);

    new_capture->bAutoActivate = false;
    new_capture->bCaptureEveryFrame = true;
    new_capture->bCaptureOnMovement = true;
    new_capture->SetRelativeRotation(FRotator(0, 0, 0));
    new_capture->SetRelativeLocation(FVector(0, 0, 0));
    new_capture->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	new_capture->RegisterComponent();
    new_capture->Deactivate();

    if (max_view_distance > 0) {
        FString sphereName  = name + "_hidden_sphere";
        UStaticMeshComponent* annotation_sphere = NewObject<UStaticMeshComponent>(this, FName(*sphereName));
        annotation_sphere->SetupAttachment(RootComponent);
        annotation_sphere->RegisterComponent();
        annotation_sphere->SetStaticMesh(annotation_sphere_static_);
        annotation_sphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        annotation_sphere->SetRelativeScale3D(FVector(max_view_distance * 2, max_view_distance * 2, max_view_distance * 2));

        FString annotatedSphereName = name + "_annotation_sphere";
        UAnnotationComponent* AnnotationComponent = NewObject<UAnnotationComponent>(annotation_sphere, FName(*annotatedSphereName));
        AnnotationComponent->SetupAttachment(annotation_sphere);
        AnnotationComponent->RegisterComponent();
        AnnotationComponent->MarkRenderStateDirty();
        UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(AnnotationComponent);

        sphere_annotation_component_map_.Add(name, PrimitiveComponent);
	}

    captures_.Add(new_capture);

    render_targets_.Add(NewObject<UTextureRenderTarget2D>());
    int render_index = render_targets_.Num() - 1;
    if (type == FObjectAnnotator::AnnotatorType::RGB || type == FObjectAnnotator::AnnotatorType::InstanceSegmentation) {
        render_targets_[render_index]->TargetGamma = 1;
    }

    FObjectAnnotator::SetViewForAnnotationRender(captures_[render_index]->ShowFlags);
    captures_[render_index]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;

    camera_type_enabled_.push_back(false);   

    annotator_name_to_index_map_.Add(TCHAR_TO_UTF8(*name), render_index);

    detections_.Add(NewObject<UDetectionComponent>(this));
    if (detections_[render_index]) {
        detections_[render_index]->SetupAttachment(captures_[render_index]);
        detections_[render_index]->RegisterComponent();
        detections_[render_index]->Deactivate();
    }

    captures_[render_index]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

    setCaptureUpdate(captures_[render_index], true);    

    if (sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Annotation)).ignore_marked)captures_[render_index]->HiddenActors = ignore_actors_;
    
    captures_[render_index]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;

    updateCaptureComponentSetting(captures_[render_index], render_targets_[render_index],
        false, EPixelFormat::PF_B8G8R8A8,
        sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Annotation)),
        *ned_transform_, false);

    copyCameraSettingsToSceneCapture(camera_, captures_[render_index]);	
}

void APIPCamera::setupCameraFromSettings(const APIPCamera::CameraSetting& camera_setting, const NedTransform& ned_transform)
{
    //TODO: should we be ignoring position and orientation settings here?

    //TODO: can we eliminate storing NedTransform?

    ned_transform_ = &ned_transform;

    sensor_params_ = camera_setting;

    gimbal_stabilization_ = Utils::clip(camera_setting.gimbal.stabilization, 0.0f, 1.0f);
    if (gimbal_stabilization_ > 0) {
        this->SetActorTickEnabled(true);
        gimbald_rotator_.Pitch = camera_setting.gimbal.rotation.pitch;
        gimbald_rotator_.Roll = camera_setting.gimbal.rotation.roll;
        gimbald_rotator_.Yaw = camera_setting.gimbal.rotation.yaw;
    }
    else
        this->SetActorTickEnabled(false);

    if (sensor_params_.draw_sensor) {
        this->SetActorTickEnabled(true);
    }

    if (sensor_params_.external) {
        this->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    }

	static const FName lidar_ignore_tag = TEXT("MarkedIgnore");
    for (TActorIterator<AActor> ActorIterator(this->GetWorld()); ActorIterator; ++ActorIterator)
    {
        AActor* Actor = *ActorIterator;
        if (Actor && Actor != this && Actor->Tags.Contains(lidar_ignore_tag))ignore_actors_.Add(Actor);
    }


    int image_count = static_cast<int>(Utils::toNumeric(ImageType::Count));
    for (int image_type = -1; image_type < image_count - 1; ++image_type) {
        const auto& capture_setting = camera_setting.capture_settings.at(image_type);
        const auto& noise_setting = camera_setting.noise_settings.at(image_type);

        if (image_type >= 0) { //scene capture components
            auto pixel_format_override = camera_setting.ue_setting.pixel_format_override_settings.find(image_type);
            EPixelFormat pixel_format = EPixelFormat::PF_Unknown;
            if (pixel_format_override != camera_setting.ue_setting.pixel_format_override_settings.end()) {
                pixel_format = static_cast<EPixelFormat>(pixel_format_override->second.pixel_format);
            }
            pixel_format = (pixel_format == EPixelFormat::PF_Unknown ? image_type_to_pixel_format_map_[image_type] : pixel_format);

            switch (Utils::toEnum<ImageType>(image_type)) {
            case ImageType::Scene:
            case ImageType::Infrared:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], false, pixel_format, capture_setting, ned_transform, false);
                break;

            case ImageType::Segmentation:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], false, pixel_format, capture_setting, ned_transform, false);
                render_targets_[image_type]->TargetGamma = 1;
            case ImageType::SurfaceNormals:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], true, pixel_format, capture_setting, ned_transform, true);
                break;
            case ImageType::Annotation:
                break;
            default:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], true, pixel_format, capture_setting, ned_transform, false);
                break;
            }
            if(capture_setting.ignore_marked)captures_[image_type]->HiddenActors = ignore_actors_;
            setDistortionMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings);
            setNoiseMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings, noise_setting);
            copyCameraSettingsToSceneCapture(camera_, captures_[image_type]); //CinemAirSim
            if(image_type == Utils::toNumeric(ImageType::Scene)) {
                if (capture_setting.lumen_gi_enabled) {
                    captures_[image_type]->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = 1;
                    captures_[image_type]->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::Lumen;
                }
                else {
                    captures_[image_type]->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = 1;
                    captures_[image_type]->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::None;
                }
                if (capture_setting.lumen_reflections_enabled) {
                    captures_[image_type]->PostProcessSettings.bOverride_ReflectionMethod = 1;
                    captures_[image_type]->PostProcessSettings.ReflectionMethod = EReflectionMethod::Lumen;
                }
                else {
                    captures_[image_type]->PostProcessSettings.bOverride_ReflectionMethod = 1;
					captures_[image_type]->PostProcessSettings.ReflectionMethod = EReflectionMethod::None;
                }

                captures_[image_type]->PostProcessSettings.LumenFinalGatherQuality = capture_setting.lumen_final_quality;
                captures_[image_type]->PostProcessSettings.LumenSceneDetail = capture_setting.lumen_scene_detail;
                captures_[image_type]->PostProcessSettings.LumenSceneLightingQuality = capture_setting.lumen_scene_lightning_quality;
                captures_[image_type]->bUseRayTracingIfEnabled = 1;
			}
        }
        else { //camera component
            updateCameraSetting(camera_, capture_setting, ned_transform);
            setDistortionMaterial(image_type, camera_, camera_->PostProcessSettings);
            setNoiseMaterial(image_type, camera_, camera_->PostProcessSettings, noise_setting);
            copyCameraSettingsToAllSceneCapture(camera_); //CinemAirSim
        }
    }
}

void APIPCamera::updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target,
                                               bool auto_format, const EPixelFormat& pixel_format, const CaptureSetting& setting, const NedTransform& ned_transform,
                                               bool force_linear_gamma)
{
    if (auto_format) {
        render_target->InitAutoFormat(setting.width, setting.height); //256 X 144, X 480
    }
    else {
        render_target->InitCustomFormat(setting.width, setting.height, pixel_format, force_linear_gamma);
    }

    if (!std::isnan(setting.target_gamma))
		render_target->TargetGamma = setting.target_gamma;

    capture->ProjectionType = static_cast<ECameraProjectionMode::Type>(setting.projection_mode);

    if (!std::isnan(setting.fov_degrees))
        capture->FOVAngle = setting.fov_degrees;
    if (capture->ProjectionType == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        capture->OrthoWidth = ned_transform.fromNed(setting.ortho_width);

    updateCameraPostProcessingSetting(capture->PostProcessSettings, setting);
}

//CinemAirSim
void APIPCamera::updateCameraSetting(UCineCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform)
{
    //if (!std::isnan(setting.target_gamma))
    //    camera-> = setting.target_gamma;

    camera->SetProjectionMode(static_cast<ECameraProjectionMode::Type>(setting.projection_mode));

    if (!std::isnan(setting.fov_degrees))
        camera->SetFieldOfView(setting.fov_degrees);
    if (camera->ProjectionMode == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        camera->SetOrthoWidth(ned_transform.fromNed(setting.ortho_width));

    updateCameraPostProcessingSetting(camera->PostProcessSettings, setting);
}

msr::airlib::Pose APIPCamera::getPose() const
{
    return ned_transform_->toLocalNed(this->GetActorTransform());
}

void APIPCamera::updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting)
{
    if (!std::isnan(setting.motion_blur_amount)) {
        obj.bOverride_MotionBlurAmount = 1;
        obj.MotionBlurAmount = setting.motion_blur_amount;
    }
    if (!std::isnan(setting.motion_blur_max))
	{
		obj.bOverride_MotionBlurMax = 1;
		obj.MotionBlurMax = setting.motion_blur_max;
	}
    if (setting.auto_exposure_method >= 0) {
        obj.bOverride_AutoExposureMethod = 1;
        obj.AutoExposureMethod = Utils::toEnum<EAutoExposureMethod>(setting.auto_exposure_method);
    }
    if (!std::isnan(setting.auto_exposure_speed)) {
        obj.bOverride_AutoExposureSpeedDown = 1;
        obj.AutoExposureSpeedDown = obj.AutoExposureSpeedUp = setting.auto_exposure_speed;
    }
    if (!std::isnan(setting.auto_exposure_max_brightness)) {
        obj.bOverride_AutoExposureMaxBrightness = 1;
        obj.AutoExposureMaxBrightness = setting.auto_exposure_max_brightness;
    }
    if (!std::isnan(setting.auto_exposure_min_brightness)) {
        obj.bOverride_AutoExposureMinBrightness = 1;
        obj.AutoExposureMinBrightness = setting.auto_exposure_min_brightness;
    }
    if (!std::isnan(setting.auto_exposure_bias)) {
        obj.bOverride_AutoExposureBias = 1;
        obj.AutoExposureBias = setting.auto_exposure_bias;
    }
    if (!std::isnan(setting.auto_exposure_low_percent)) {
        obj.bOverride_AutoExposureLowPercent = 1;
        obj.AutoExposureLowPercent = setting.auto_exposure_low_percent;
    }
    if (!std::isnan(setting.auto_exposure_high_percent)) {
        obj.bOverride_AutoExposureHighPercent = 1;
        obj.AutoExposureHighPercent = setting.auto_exposure_high_percent;
    }
    if (!std::isnan(setting.auto_exposure_histogram_log_min)) {
        obj.bOverride_HistogramLogMin = 1;
        obj.HistogramLogMin = setting.auto_exposure_histogram_log_min;
    }
    if (!std::isnan(setting.auto_exposure_histogram_log_max)) {
        obj.bOverride_HistogramLogMax = 1;
        obj.HistogramLogMax = setting.auto_exposure_histogram_log_max;
    }
	if (!std::isnan(setting.chromatic_aberration_scale))
	{
		obj.bOverride_SceneFringeIntensity = 1;
		obj.SceneFringeIntensity = setting.chromatic_aberration_scale;
	}
}

void APIPCamera::setDistortionMaterial(int image_type, UObject* outer, FPostProcessSettings& obj)
{
    UMaterialInstanceDynamic* distortion_material = UMaterialInstanceDynamic::Create(distortion_material_static_, outer);
    distortion_materials_[image_type + 1] = distortion_material;
    obj.AddBlendable(distortion_material, 1.0f);
}

void APIPCamera::setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings)
{
    if (!settings.Enabled)
        return;

    UMaterialInstanceDynamic* noise_material = UMaterialInstanceDynamic::Create(noise_material_static_, outer);
    noise_materials_[image_type + 1] = noise_material;

    noise_material->SetScalarParameterValue("HorzWaveStrength", settings.HorzWaveStrength);
    noise_material->SetScalarParameterValue("RandSpeed", settings.RandSpeed);
    noise_material->SetScalarParameterValue("RandSize", settings.RandSize);
    noise_material->SetScalarParameterValue("RandDensity", settings.RandDensity);
    noise_material->SetScalarParameterValue("RandContrib", settings.RandContrib);
    noise_material->SetScalarParameterValue("HorzWaveContrib", settings.HorzWaveContrib);
    noise_material->SetScalarParameterValue("HorzWaveVertSize", settings.HorzWaveVertSize);
    noise_material->SetScalarParameterValue("HorzWaveScreenSize", settings.HorzWaveScreenSize);
    noise_material->SetScalarParameterValue("HorzNoiseLinesContrib", settings.HorzNoiseLinesContrib);
    noise_material->SetScalarParameterValue("HorzNoiseLinesDensityY", settings.HorzNoiseLinesDensityY);
    noise_material->SetScalarParameterValue("HorzNoiseLinesDensityXY", settings.HorzNoiseLinesDensityXY);
    noise_material->SetScalarParameterValue("HorzDistortionStrength", settings.HorzDistortionStrength);
    noise_material->SetScalarParameterValue("HorzDistortionContrib", settings.HorzDistortionContrib);

    obj.AddBlendable(noise_material, 1.0f);

	if (settings.LensDistortionEnable) {

		UMaterialInstanceDynamic* lens_distortion_material_;

		if (settings.LensDistortionInvert) {
			lens_distortion_material_ = UMaterialInstanceDynamic::Create(lens_distortion_invert_material_static_, outer);
		}
		else {
			lens_distortion_material_ = UMaterialInstanceDynamic::Create(lens_distortion_material_static_, outer);
		}

		lens_distortion_materials_[image_type + 1] = lens_distortion_material_;


		lens_distortion_material_->SetScalarParameterValue("AreaFalloff", settings.LensDistortionAreaFalloff);
		lens_distortion_material_->SetScalarParameterValue("AreaRadius", settings.LensDistortionAreaRadius);
		lens_distortion_material_->SetScalarParameterValue("Intensity", settings.LensDistortionIntensity);


		obj.AddBlendable(lens_distortion_material_, 1.0f);
	}
}

void APIPCamera::enableCaptureComponent(const APIPCamera::ImageType type, bool is_enabled, std::string annotation_name)
{
    USceneCaptureComponent2D* capture = getCaptureComponent(type, false, annotation_name);
    if (capture != nullptr) {
        UDetectionComponent* detection = getDetectionComponent(type, false, annotation_name);
        if (is_enabled) {
            //do not make unnecessary calls to Activate() which otherwise causes crash in Unreal
            if (!capture->IsActive() || capture->TextureTarget == nullptr) {
                capture->TextureTarget = getRenderTarget(type, false, annotation_name);
                capture->Activate();
                if (detection != nullptr) {
                    detection->texture_target_ = capture->TextureTarget;
                    detection->Activate();
                }
            }
        }
        else {
            if (capture->IsActive() || capture->TextureTarget != nullptr) {
                capture->Deactivate();
                capture->TextureTarget = nullptr;
                if (detection != nullptr) {
                    detection->Deactivate();
                    detection->texture_target_ = nullptr;
                }
            }
        }
        if (type == ImageType::Annotation)
			camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]] = is_enabled;
        else
            camera_type_enabled_[Utils::toNumeric(type)] = is_enabled;
    }
    //else nothing to enable
}

UTextureRenderTarget2D* APIPCamera::getRenderTarget(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    unsigned int image_type = Utils::toNumeric(type);
	if (type == ImageType::Annotation)
    {        
        if (!if_active || camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]])
            return render_targets_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        if (!if_active || camera_type_enabled_[image_type])
            return render_targets_[image_type];
    }    
    return nullptr;
}

UDetectionComponent* APIPCamera::getDetectionComponent(const ImageType type, bool if_active, std::string annotation_name) const
{
    if (type == ImageType::Annotation) {
        if (!if_active || camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]])
            return detections_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        unsigned int image_type = Utils::toNumeric(type);
        if (!if_active || camera_type_enabled_[image_type])
            return detections_[image_type];
    }
    return nullptr;
}

USceneCaptureComponent2D* APIPCamera::getCaptureComponent(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    if (type == ImageType::Annotation) {
        if (!if_active || camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]])
            return captures_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        unsigned int image_type = Utils::toNumeric(type);
        if (!if_active || camera_type_enabled_[image_type])
            return captures_[image_type];
    }
    return nullptr;
}

void APIPCamera::disableAllPIP()
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
		if (Utils::toEnum<ImageType>(image_type) == ImageType::Annotation)
		{
			for (auto& annotator : annotator_name_to_index_map_)
			{
				enableCaptureComponent(ImageType::Annotation, false, TCHAR_TO_UTF8(*annotator.Key));
			}
		}
        else {
            enableCaptureComponent(Utils::toEnum<ImageType>(image_type), false);
        }
    }
}

void APIPCamera::disableMain()
{
    camera_->Deactivate();
    camera_->SetVisibility(false);
    //APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    //if (controller && controller->GetViewTarget() == this)
    //    controller->SetViewTarget(nullptr);
}

void APIPCamera::onViewModeChanged(bool nodisplay)
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        if (Utils::toEnum<ImageType>(image_type) == ImageType::Annotation)
        {
            for (auto& annotator : annotator_name_to_index_map_)
            {
                USceneCaptureComponent2D* capture = getCaptureComponent(ImageType::Annotation, false, TCHAR_TO_UTF8(*annotator.Key));
                if (capture) {
                    setCaptureUpdate(capture, nodisplay);
                }
            }
        }
        else {
            USceneCaptureComponent2D* capture = getCaptureComponent(static_cast<ImageType>(image_type), false);
            if (capture) {
                setCaptureUpdate(capture, nodisplay);
            }
        }        
    }
}

//CinemAirSim methods
std::vector<std::string> APIPCamera::getPresetLensSettings() const
{
    std::vector<std::string> vector;
    const TArray<FNamedLensPreset> lens_presets = UCineCameraSettings::GetLensPresets();
    for (const FNamedLensPreset& preset : lens_presets) {
        std::ostringstream current_lens_string;
        std::string name = (TCHAR_TO_UTF8(*preset.Name));

        current_lens_string << "Name: " << name << ";\n\t MinFocalLength: " << preset.LensSettings.MinFocalLength << "; \t MaxFocalLength: " << preset.LensSettings.MaxFocalLength;
        current_lens_string << "\n\t Min FStop: " << preset.LensSettings.MinFStop << "; \t Max Fstop: " << preset.LensSettings.MaxFStop;
        vector.push_back(current_lens_string.str());
    }
    return vector;
}

std::string APIPCamera::getLensSettings() const
{
    const FCameraLensSettings current_lens_params = camera_->LensSettings;

    std::ostringstream current_lens_string;

    const FString lens_preset_name = camera_->GetLensPresetName();
    std::string name = (TCHAR_TO_UTF8(*lens_preset_name));

    current_lens_string << "Name: " << name;
    current_lens_string << ";\n\t MinFocalLength: " << current_lens_params.MinFocalLength;
    current_lens_string << "; \t MaxFocalLength: " << current_lens_params.MaxFocalLength;
    current_lens_string << "\n\t Min FStop: " << current_lens_params.MinFStop;
    current_lens_string << "; \t Max Fstop: " << current_lens_params.MaxFStop;
    current_lens_string << "\n\t Diaphragm Blade Count: " << current_lens_params.DiaphragmBladeCount;
    current_lens_string << "\n\t Minimum focus distance: " << current_lens_params.MinimumFocusDistance;

    return current_lens_string.str();
}

void APIPCamera::setPresetLensSettings(std::string preset_string)
{
    const FString preset(preset_string.c_str());
    camera_->SetLensPresetByName(preset);
    copyCameraSettingsToAllSceneCapture(camera_);
}

std::vector<std::string> APIPCamera::getPresetFilmbackSettings() const
{
    std::vector<std::string> vector_all_presets;
    TArray<FNamedFilmbackPreset> lens_presets = UCineCameraSettings::GetFilmbackPresets();
    for (const FNamedFilmbackPreset& preset : lens_presets) {
        std::ostringstream preset_string;
        std::string name = (TCHAR_TO_UTF8(*preset.Name));

        preset_string << "Name: " << name << ";\n\t Sensor Width: " << preset.FilmbackSettings.SensorWidth << "; \t Sensor Height: " << preset.FilmbackSettings.SensorHeight;
        preset_string << "\n\t Sensor Aspect Ratio: " << preset.FilmbackSettings.SensorAspectRatio;
        vector_all_presets.push_back(preset_string.str());
    }
    return vector_all_presets;
}

void APIPCamera::setPresetFilmbackSettings(std::string preset_string)
{
    const FString preset(preset_string.c_str());
    camera_->SetFilmbackPresetByName(preset);
    copyCameraSettingsToAllSceneCapture(camera_);
}

std::string APIPCamera::getFilmbackSettings() const
{
    FCameraFilmbackSettings current_filmback_settings = camera_->Filmback;

    const FString filmback_present_name = camera_->GetFilmbackPresetName();
    std::ostringstream current_filmback_string;
    std::string name = (TCHAR_TO_UTF8(*filmback_present_name));

    current_filmback_string << "Name: " << name << ";\n\t Sensor Width: " << current_filmback_settings.SensorWidth << "; \t Sensor Height: " << current_filmback_settings.SensorHeight;
    current_filmback_string << "\n\t Sensor Aspect Ratio: " << current_filmback_settings.SensorAspectRatio;
    return current_filmback_string.str();
}

float APIPCamera::setFilmbackSettings(float sensor_width, float sensor_height)
{
    camera_->Filmback.SensorWidth = sensor_width;
    camera_->Filmback.SensorHeight = sensor_height;

    copyCameraSettingsToAllSceneCapture(camera_);

    return camera_->Filmback.SensorAspectRatio;
}

float APIPCamera::getFocalLength() const
{
    return camera_->CurrentFocalLength;
}

void APIPCamera::setFocalLength(float focal_length)
{
    camera_->CurrentFocalLength = focal_length;
    copyCameraSettingsToAllSceneCapture(camera_);
}

void APIPCamera::enableManualFocus(bool enable)
{
    if (enable) {
        camera_->FocusSettings.FocusMethod = ECameraFocusMethod::Manual;
    }
    else {
        camera_->FocusSettings.FocusMethod = ECameraFocusMethod::Disable;
    }
    copyCameraSettingsToAllSceneCapture(camera_);
}

float APIPCamera::getFocusDistance() const
{
    return camera_->FocusSettings.ManualFocusDistance;
}

void APIPCamera::setFocusDistance(float focus_distance)
{
    camera_->FocusSettings.ManualFocusDistance = focus_distance;
    copyCameraSettingsToAllSceneCapture(camera_);
}

float APIPCamera::getFocusAperture() const
{
    return camera_->CurrentAperture;
}

void APIPCamera::setFocusAperture(float focus_aperture)
{
    camera_->CurrentAperture = focus_aperture;
    copyCameraSettingsToAllSceneCapture(camera_);
}

void APIPCamera::enableFocusPlane(bool enable)
{
#if WITH_EDITOR
    camera_->FocusSettings.bDrawDebugFocusPlane = enable;
#endif
}

std::string APIPCamera::getCurrentFieldOfView() const
{
    std::ostringstream field_of_view_string;
    field_of_view_string << "Current Field Of View:\n\tHorizontal Field Of View: " << camera_->GetHorizontalFieldOfView() << ";\n\t Vertical Field Of View: " << camera_->GetVerticalFieldOfView();
    return field_of_view_string.str();
}

void APIPCamera::copyCameraSettingsToAllSceneCapture(UCameraComponent* camera)
{
    int image_count = static_cast<int>(cameraCaptureCount());
    for (int image_type = image_count - 1; image_type >= 0; image_type--) {
        copyCameraSettingsToSceneCapture(camera_, captures_[image_type]);
    }
}

void APIPCamera::copyCameraSettingsToSceneCapture(UCameraComponent* src, USceneCaptureComponent2D* dst)
{
    if (src && dst) {
        dst->SetWorldLocationAndRotation(src->GetComponentLocation(), src->GetComponentRotation());

        FMinimalViewInfo camera_view_info;
        src->GetCameraView(/*DeltaTime =*/0.0f, camera_view_info);

        const FPostProcessSettings& src_pp_settings = camera_view_info.PostProcessSettings;
        FPostProcessSettings& dst_pp_settings = dst->PostProcessSettings;

        FWeightedBlendables dst_weighted_blendables = dst_pp_settings.WeightedBlendables;

        // Copy all of the post processing settings
        dst_pp_settings = src_pp_settings;

        // But restore the original blendables
        dst_pp_settings.WeightedBlendables = dst_weighted_blendables;
    }
}

//end CinemAirSim methods
