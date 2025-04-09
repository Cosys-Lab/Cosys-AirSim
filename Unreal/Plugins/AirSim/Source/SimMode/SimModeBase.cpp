#include "SimModeBase.h"
#include "Recording/RecordingThread.h"
#include "Misc/MessageDialog.h"
#include "Misc/EngineVersion.h"
#include "Runtime/Launch/Resources/Version.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/OutputDeviceNull.h"
#include "Engine/World.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Misc/FileHelper.h"
#include <memory>
#include "AirBlueprintLib.h"
#include "Annotation/ObjectAnnotator.h"
#include "LidarCamera.h"
#include "api/VehicleApiBase.hpp"
#include "common/AirSimSettings.hpp"
#include "common/ScalableClock.hpp"
#include "common/SteppableClock.hpp"
#include "SimJoyStick/SimJoyStick.h"
#include "common/EarthCelestial.hpp"
#include "sensors/lidar/LidarSimple.hpp"
#include "sensors/distance/DistanceSimple.hpp"

#include "Weather/WeatherLib.h"

#include "Beacons/TemplateBeacon.h"
#include "Beacons/FiducialBeacon.h"
#include "Beacons/UWBBeacon.h"
#include "Beacons/WifiBeacon.h"
#include "Beacons/DynamicBlockBeacon.h"
#include "Beacons/DynamicRackBeacon.h"
#include "Beacons/PassiveEchoBeacon.h"

#include "DrawDebugHelpers.h"

//TODO: this is going to cause circular references which is fine here but
//in future we should consider moving SimMode not derived from AActor and move
//it to AirLib and directly implement WorldSimApiBase interface
#include "WorldSimApi.h"

ASimModeBase* ASimModeBase::SIMMODE = nullptr;

ASimModeBase* ASimModeBase::getSimMode()
{
    return SIMMODE;
}

ASimModeBase::ASimModeBase()
{
    SIMMODE = this;

    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<AAirSimCameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;

    static ConstructorHelpers::FObjectFinder<UParticleSystem> collision_display(TEXT("ParticleSystem'/AirSim/StarterContent/Particles/P_Explosion.P_Explosion'"));
    if (!collision_display.Succeeded())
        collision_display_template = collision_display.Object;
    else
        collision_display_template = nullptr;

    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class_val(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class = pip_camera_class_val.Succeeded() ? pip_camera_class_val.Class : nullptr;

    PrimaryActorTick.bCanEverTick = true;

    static ConstructorHelpers::FClassFinder<AActor> sky_sphere_class(TEXT("Blueprint'/Engine/EngineSky/BP_Sky_Sphere'"));
    sky_sphere_class_ = sky_sphere_class.Succeeded() ? sky_sphere_class.Class : nullptr;

    static ConstructorHelpers::FClassFinder<UUserWidget> loading_screen_class_find(TEXT("WidgetBlueprint'/AirSim/Blueprints/BP_LoadingScreenWidget'"));
    if (loading_screen_class_find.Succeeded()) {
        auto loading_screen_class = loading_screen_class_find.Class;
        loading_screen_widget_ = CreateWidget<ULoadingScreenWidget>(this->GetWorld(), loading_screen_class);
    }
    else
        loading_screen_widget_ = nullptr;
    static ConstructorHelpers::FObjectFinder<UMaterial> domain_rand_mat_finder(TEXT("Material'/AirSim/HUDAssets/DomainRandomizationMaterial.DomainRandomizationMaterial'"));
    if (domain_rand_mat_finder.Succeeded()) {
        domain_rand_material_ = domain_rand_mat_finder.Object;
    }
}

void ASimModeBase::toggleLoadingScreen(bool is_visible)
{
    if (loading_screen_widget_ == nullptr)
        return;
    else {
        UAirBlueprintLib::RunCommandOnGameThread([this, is_visible]() {
            if (is_visible)
                loading_screen_widget_->SetVisibility(ESlateVisibility::Visible);
            else
                loading_screen_widget_->SetVisibility(ESlateVisibility::Hidden);
        },
                                                 true);
    }
}

void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();

    debug_reporter_.initialize(false);
    debug_reporter_.reset();

    //get player start
    //this must be done from within actor otherwise we don't get player start
    TArray<AActor*> pawns;
    getExistingVehiclePawns(pawns);
    bool have_existing_pawns = pawns.Num() > 0;
    AActor* fpv_pawn = nullptr;
    // Grab player location
    FTransform player_start_transform;
    FVector player_loc;
    if (have_existing_pawns) {
        fpv_pawn = pawns[0];
    }
    else {
        APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
        fpv_pawn = player_controller->GetViewTarget();
    }

    player_start_transform = fpv_pawn->GetActorTransform();
    if (getSettings().move_world_origin) {
        player_loc = player_start_transform.GetLocation();
        // Move the world origin to the player's location (this moves the coordinate system and adds
        // a corresponding offset to all positions to compensate for the shift)
        this->GetWorld()->SetNewWorldOrigin(FIntVector(player_loc) + this->GetWorld()->OriginLocation);
        // Regrab the player's position after the offset has been added (which should be 0,0,0 now)
        player_start_transform = fpv_pawn->GetActorTransform();
    }
    global_ned_transform_.reset(new NedTransform(player_start_transform,
                                                 UAirBlueprintLib::GetWorldToMetersScale(this)));

    UAirBlueprintLib::GenerateAssetRegistryMap(this, asset_map);

    world_sim_api_.reset(new WorldSimApi(this));
    api_provider_.reset(new msr::airlib::ApiProvider(world_sim_api_.get()));

    UAirBlueprintLib::setLogMessagesVisibility(getSettings().log_messages_visible);

    setupPhysicsLoopPeriod();

    setupClockSpeed();

    InitializeMaterialStencils();

    record_tick_count = 0;
    setupInputBindings();

    initializeTimeOfDay();
    AirSimSettings::TimeOfDaySetting tod_setting = getSettings().tod_setting;
    setTimeOfDay(tod_setting.enabled, tod_setting.start_datetime, tod_setting.is_start_datetime_dst, tod_setting.celestial_clock_speed, tod_setting.update_interval_secs, tod_setting.move_sun);

    UAirBlueprintLib::LogMessage(TEXT("Press F1 to see help"), TEXT(""), LogDebugLevel::Informational);

    setupVehiclesAndCamera();
    FRecordingThread::init();

    if (getSettings().recording_setting.enabled)
        startRecording();

    UWorld* World = GetWorld();
    if (World) {
        UWeatherLib::initWeather(World, spawned_actors_);
        //UWeatherLib::showWeatherMenu(World);
    }
    UAirBlueprintLib::GenerateActorMap(this, scene_object_map);

    loading_screen_widget_->AddToViewport();
    loading_screen_widget_->SetVisibility(ESlateVisibility::Hidden);

    InitializeInstanceSegmentation();

    InitializeAnnotation();
}

const NedTransform& ASimModeBase::getGlobalNedTransform()
{
    return *global_ned_transform_;
}

void ASimModeBase::checkVehicleReady()
{
    for (auto& api : api_provider_->getVehicleApis()) {
        if (api) { //sim-only vehicles may have api as null
            std::string message;
            if (!api->isReady(message)) {
                UAirBlueprintLib::LogMessage("Vehicle was not initialized", "", LogDebugLevel::Failure);
                if (message.size() > 0) {
                    UAirBlueprintLib::LogMessage(message.c_str(), "", LogDebugLevel::Failure);
                }
                UAirBlueprintLib::LogMessage("Tip: check connection info in settings.json", "", LogDebugLevel::Informational);
            }
        }
    }
}

void ASimModeBase::RunCommandOnGameThread(TFunction<void()> InFunction, bool wait, const TStatId InStatId)
{
    if (::IsInGameThread())
        InFunction();
    else {
        FGraphEventRef task = FFunctionGraphTask::CreateAndDispatchWhenReady(MoveTemp(InFunction), InStatId, nullptr, ENamedThreads::GameThread);
        if (wait)
            FTaskGraphInterface::Get().WaitUntilTaskCompletes(task);
    }
}

void ASimModeBase::InitializeAnnotation() {
	for (auto& annotator_setting : getSettings().annotator_settings) {
        FString name = FString(annotator_setting.name.c_str());
        FObjectAnnotator::AnnotatorType type = FObjectAnnotator::AnnotatorType(annotator_setting.type);
		bool set_direct = annotator_setting.set_direct;
        FString texture_path = FString(annotator_setting.texture_path.c_str());
        FString texture_prefix = FString(annotator_setting.texture_prefix.c_str());
        float max_view_distance = annotator_setting.max_view_distance;
        annotators_.Emplace(name, FObjectAnnotator(name, type, annotator_setting.show_by_default, set_direct, texture_path, texture_prefix, max_view_distance));
		annotators_[name].Initialize(this->GetLevel());
        AddAnnotatorCamera(name, type, max_view_distance);
        ForceUpdateAnnotation(name);
        updateAnnotation(name);
	}
}

bool ASimModeBase::IsAnnotationRGBValid(FString annotation_name, FColor color) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
	return annotators_[annotation_name].IsRGBColorValid(color);
}

void ASimModeBase::InitializeInstanceSegmentation()
{
    if (getSettings().initial_instance_segmentation) {
        instance_segmentation_annotator_.Initialize(this->GetLevel());
    }
    ForceUpdateInstanceSegmentation();
    updateInstanceSegmentationAnnotation();
}

bool ASimModeBase::AddRGBDirectAnnotationTagToActor(FString annotation_name, AActor* actor, FColor color, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
	FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(color.R) + FString(TEXT("_")) + FString::FromInt(color.G) + FString(TEXT("_")) + FString::FromInt(color.B);
	actor->Tags.Emplace(FName(*tag));
    if(update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::UpdateRGBDirectAnnotationTagToActor(FString annotation_name, AActor* actor, FColor color, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    int32 found_tag = actor->Tags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(color.R) + FString(TEXT("_")) + FString::FromInt(color.G) + FString(TEXT("_")) + FString::FromInt(color.B);
	actor->Tags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::AddRGBIndexAnnotationTagToActor(FString annotation_name, AActor* actor, int32 index, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to index mode."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(index);
    actor->Tags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::UpdateRGBIndexAnnotationTagToActor(FString annotation_name, AActor* actor, int32 index, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to index mode."), *annotation_name);
        return false;
    }
    int32 found_tag = actor->Tags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(index);
    actor->Tags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::AddRGBDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, FColor color, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(color.R) + FString(TEXT("_")) + FString::FromInt(color.G) + FString(TEXT("_")) + FString::FromInt(color.B);
    component->ComponentTags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::UpdateRGBDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, FColor color, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    int32 found_tag = component->ComponentTags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(color.R) + FString(TEXT("_")) + FString::FromInt(color.G) + FString(TEXT("_")) + FString::FromInt(color.B);
    component->ComponentTags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::AddRGBIndexAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, int32 index, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to index mode."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(index);
    component->ComponentTags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::UpdateRGBIndexAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, int32 index, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to index mode."), *annotation_name);
        return false;
    }
    int32 found_tag = component->ComponentTags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::FromInt(index);
    component->ComponentTags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::AddGreyscaleAnnotationTagToActor(FString annotation_name, AActor* actor, float greyscale_value, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Greyscale) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the greyscale type."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::SanitizeFloat(greyscale_value);
    actor->Tags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::UpdateGreyscaleAnnotationTagToActor(FString annotation_name, AActor* actor, float greyscale_value, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Greyscale) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the greyscale type."), *annotation_name);
        return false;
    }
    int32 found_tag = actor->Tags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::SanitizeFloat(greyscale_value);
    actor->Tags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::AddGreyscaleAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, float greyscale_value, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Greyscale) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the greyscale type."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::SanitizeFloat(greyscale_value);
    component->ComponentTags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::UpdateGreyscaleAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, float greyscale_value, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Greyscale) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the greyscale type."), *annotation_name);
        return false;
    }
    int32 found_tag = component->ComponentTags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + FString::SanitizeFloat(greyscale_value);
    component->ComponentTags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::AddTextureDirectAnnotationTagToActorByPath(FString annotation_name, AActor* actor, FString texture_path, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + texture_path;
    actor->Tags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::UpdateTextureDirectAnnotationTagToActorByPath(FString annotation_name, AActor* actor, FString texture_path, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    int32 found_tag = actor->Tags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + texture_path;
    actor->Tags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::AddTextureDirectAnnotationTagToComponentByPath(FString annotation_name, UMeshComponent* component, FString texture_path, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + texture_path;
    component->ComponentTags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::UpdateTextureDirectAnnotationTagToComponentByPath(FString annotation_name, UMeshComponent* component, FString texture_path, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    int32 found_tag = component->ComponentTags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_")) + texture_path;
    component->ComponentTags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::AddTextureDirectAnnotationTagToActor(FString annotation_name, AActor* actor, UTexture* texture, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    TArray<FString> splitPath;
    texture->GetPathName().ParseIntoArray(splitPath, TEXT("."), true);
    FString TextureFilePath = splitPath[0];
    FString tag = annotation_name + FString(TEXT("_")) + TextureFilePath;
    actor->Tags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::UpdateTextureDirectAnnotationTagToActor(FString annotation_name, AActor* actor, UTexture* texture, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    int32 found_tag = actor->Tags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    TArray<FString> splitPath;
    texture->GetPathName().ParseIntoArray(splitPath, TEXT("."), true);
    FString TextureFilePath = splitPath[0];
    FString tag = annotation_name + FString(TEXT("_")) + TextureFilePath;
    actor->Tags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::AddTextureDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, UTexture* texture, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    TArray<FString> splitPath;
    texture->GetPathName().ParseIntoArray(splitPath, TEXT("."), true);
    FString TextureFilePath = splitPath[0];
    FString tag = annotation_name + FString(TEXT("_")) + TextureFilePath;
    component->ComponentTags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::UpdateTextureDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, UTexture* texture, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (!annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *annotation_name);
        return false;
    }
    int32 found_tag = component->ComponentTags.IndexOfByPredicate([annotation_name](const FName& tagFName) {
        FString tag = tagFName.ToString();
        return tag.Contains(annotation_name);
        });
    if (found_tag == INDEX_NONE) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find a tag for this component."), *annotation_name);
        return false;
    }
    TArray<FString> splitPath;
    texture->GetPathName().ParseIntoArray(splitPath, TEXT("."), true);
    FString TextureFilePath = splitPath[0];
    FString tag = annotation_name + FString(TEXT("_")) + TextureFilePath;
    component->ComponentTags[found_tag] = FName(*tag);
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

bool ASimModeBase::EnableTextureByPathAnnotationTagToActor(FString annotation_name, AActor* actor, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to relative path mode."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_enable"));
    actor->Tags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, actor, update_annotation);
    return true;
}

bool ASimModeBase::EnableTextureByPathAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *annotation_name);
        return false;
    }
    if (annotators_[annotation_name].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to relative path mode."), *annotation_name);
        return false;
    }
    FString tag = annotation_name + FString(TEXT("_enable"));
    component->ComponentTags.Emplace(FName(*tag));
    if (update_annotation)
        return AddNewActorToAnnotation(annotation_name, component->GetAttachmentRootActor(), update_annotation);
    return true;
}

void ASimModeBase::InitializeMaterialStencils()
{
	FString materialListContent;
	if (FFileHelper::LoadFileToString(materialListContent, UTF8_TO_TCHAR(getSettings().material_list_file.c_str()))) {
		UAirBlueprintLib::InitializeMeshStencilIDs(true, materialListContent);
	}
	else {
		UAirBlueprintLib::LogMessage("Cannot start stencil initialization. Material list was not found:",
            UTF8_TO_TCHAR(getSettings().material_list_file.c_str()), LogDebugLevel::Failure);
	}
}

std::vector<std::string> ASimModeBase::GetAllInstanceSegmentationMeshIDs() {
    return instance_segmentation_annotator_.GetAllComponentNames();
}

std::vector<msr::airlib::Vector3r> ASimModeBase::GetInstanceSegmentationColorMap() {
    TArray<FColor> color_map = instance_segmentation_annotator_.GetColorMap();
    std::vector<msr::airlib::Vector3r> color_map_vector;
    for (FColor color : color_map) {
        color_map_vector.push_back(msr::airlib::Vector3r(color.R, color.G, color.B));
    }
    return color_map_vector;
}

TMap<UMeshComponent*, FString> ASimModeBase::GetInstanceSegmentationComponentToNameMap() {
    return instance_segmentation_annotator_.GetComponentToNameMap();
}

std::vector<msr::airlib::Pose> ASimModeBase::GetAllInstanceSegmentationMeshPoses(bool ned, bool only_visible) {
    std::vector<msr::airlib::Pose> retval;
    TMap<FString, UMeshComponent*> nameToComponentMapTemp = instance_segmentation_annotator_.GetNameToComponentMap();
    for (auto const& element : nameToComponentMapTemp) {
        UAirBlueprintLib::RunCommandOnGameThread([ned, only_visible, &retval, element, this]() {
            if (element.Value->HasBegunPlay() && element.Value->IsRenderStateCreated()) {
                if (!element.Value->IsBeingDestroyed() && IsValid(element.Value)) {
                    if (!only_visible || element.Value->GetVisibleFlag()) {
                        if (ned) {
                            retval.emplace_back(getGlobalNedTransform().toGlobalNed(FTransform(element.Value->GetComponentRotation(), element.Value->GetComponentLocation())));
                        }
                        else {
                            retval.emplace_back(getGlobalNedTransform().toLocalNed(FTransform(element.Value->GetComponentRotation(), element.Value->GetComponentLocation())));
                        }
                    }
                    else {
                        retval.emplace_back(msr::airlib::Pose::nanPose());
                    }
                }
                else {
                    retval.emplace_back(msr::airlib::Pose::nanPose());
                }
            }
            else{
                retval.emplace_back(msr::airlib::Pose::nanPose());
            }
        }, true);
    }
    return retval;
}

bool ASimModeBase::SetMeshInstanceSegmentationID(const std::string& mesh_name, int object_id, bool is_name_regex, bool update_annotation) {
	if (is_name_regex) {
		std::regex name_regex;
		name_regex.assign(mesh_name, std::regex_constants::icase);
		int changes = 0;
		for (auto It = instance_segmentation_annotator_.GetNameToComponentMap().CreateConstIterator(); It; ++It)
		{
			if (std::regex_match(TCHAR_TO_UTF8(*It.Key()), name_regex)) {
				bool success;
				FString key = It.Key();
				UAirBlueprintLib::RunCommandOnGameThread([this, key, object_id, &success]() {
					success = instance_segmentation_annotator_.SetComponentRGBColorByIndex(key, object_id);
				}, true);
				changes++;
			}
		}
        if(update_annotation && changes > 0)updateInstanceSegmentationAnnotation();
        return changes > 0;
	}
	else if (instance_segmentation_annotator_.GetNameToComponentMap().Contains(mesh_name.c_str())) {
		bool success;
		FString key = mesh_name.c_str();
		UAirBlueprintLib::RunCommandOnGameThread([this, key, object_id, &success]() {
			success = instance_segmentation_annotator_.SetComponentRGBColorByIndex(key, object_id);
		}, true);
        if(update_annotation)updateInstanceSegmentationAnnotation();
        return success;
	}
	else {
		return false;
	}
}

int ASimModeBase::GetMeshInstanceSegmentationID(const std::string& mesh_name) {
	return instance_segmentation_annotator_.GetComponentIndex(mesh_name.c_str());
}

std::vector<std::string> ASimModeBase::GetAllAnnotationMeshIDs(const std::string& annotation_name) {
    std::vector<msr::airlib::string> retval;
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return retval;
    }
    return annotators_[FString(annotation_name.c_str())].GetAllComponentNames();
}

std::vector<msr::airlib::Pose> ASimModeBase::GetAllAnnotationMeshPoses(const std::string& annotation_name, bool ned, bool only_visible) {
    std::vector<msr::airlib::Pose> retval;
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return retval;
    }
    TMap<FString, UMeshComponent*> nameToComponentMapTemp = annotators_[FString(annotation_name.c_str())].GetNameToComponentMap();
    for (auto const& element : nameToComponentMapTemp) {
        UAirBlueprintLib::RunCommandOnGameThread([ned, only_visible, &retval, element, this]() {
            if (element.Value->HasBegunPlay() && element.Value->IsRenderStateCreated()) {
                if (!element.Value->IsBeingDestroyed() && IsValid(element.Value)) {
                    if (!only_visible || element.Value->GetVisibleFlag()) {
                        if (ned) {
                            retval.emplace_back(getGlobalNedTransform().toGlobalNed(FTransform(element.Value->GetComponentRotation(), element.Value->GetComponentLocation())));
                        }
                        else {
                            retval.emplace_back(getGlobalNedTransform().toLocalNed(FTransform(element.Value->GetComponentRotation(), element.Value->GetComponentLocation())));
                        }
                    }
                    else {
                        retval.emplace_back(msr::airlib::Pose::nanPose());
                    }
                }
                else {
                    retval.emplace_back(msr::airlib::Pose::nanPose());
                }
            }
            else {
                retval.emplace_back(msr::airlib::Pose::nanPose());
            }
            }, true);
    }
    return retval;
}

bool ASimModeBase::SetMeshRGBAnnotationID(const std::string& annotation_name, const std::string& mesh_name, int object_id, bool is_name_regex, bool update_annotation) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return false;
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *FString(annotation_name.c_str()));
        return false;
    }
    if (annotators_[FString(annotation_name.c_str())].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to index mode."), *FString(annotation_name.c_str()));
        return false;
    }

    if (is_name_regex) {
        std::regex name_regex;
        name_regex.assign(mesh_name, std::regex_constants::icase);
        int changes = 0;
        for (auto It = annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().CreateConstIterator(); It; ++It)
        {
            if (std::regex_match(TCHAR_TO_UTF8(*It.Key()), name_regex)) {
                bool success;
                FString key = It.Key();
                UAirBlueprintLib::RunCommandOnGameThread([this, key, object_id, &success, annotation_name]() {
                    success = annotators_[FString(annotation_name.c_str())].SetComponentRGBColorByIndex(key, object_id);
                    }, true);
                changes++;
            }
        }
        if (update_annotation && changes > 0)updateAnnotation(FString(annotation_name.c_str()));
        return changes > 0;
    }
    else if (annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().Contains(mesh_name.c_str())) {
        bool success;
        FString key = mesh_name.c_str();
        UAirBlueprintLib::RunCommandOnGameThread([this, key, object_id, &success, annotation_name]() {
            success = annotators_[FString(annotation_name.c_str())].SetComponentRGBColorByIndex(key, object_id);
            }, true);
        if (update_annotation)updateAnnotation(FString(annotation_name.c_str()));
        return success;
    }
    else {
        return false;
    }
}

bool ASimModeBase::SetMeshRGBAnnotationColor(const std::string& annotation_name, const std::string& mesh_name, int r, int g, int b, bool is_name_regex, bool update_annotation) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return false;
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *FString(annotation_name.c_str()));
        return false;
    }
    if (!annotators_[FString(annotation_name.c_str())].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *FString(annotation_name.c_str()));
        return false;
    }

    FColor color = FColor(r, g, b);

    if (is_name_regex) {
        std::regex name_regex;
        name_regex.assign(mesh_name, std::regex_constants::icase);
        int changes = 0;
        for (auto It = annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().CreateConstIterator(); It; ++It)
        {
            if (std::regex_match(TCHAR_TO_UTF8(*It.Key()), name_regex)) {
                bool success;
                FString key = It.Key();
                UAirBlueprintLib::RunCommandOnGameThread([this, key, color, &success, annotation_name]() {
                    success = annotators_[FString(annotation_name.c_str())].SetComponentRGBColorByColor(key, color);
                    }, true);
                changes++;
            }
        }
        if (update_annotation && changes > 0)updateAnnotation(FString(annotation_name.c_str()));
        return changes > 0;
    }
    else if (annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().Contains(mesh_name.c_str())) {
        bool success;
        FString key = mesh_name.c_str();
        UAirBlueprintLib::RunCommandOnGameThread([this, key, color, &success, annotation_name]() {
            success = annotators_[FString(annotation_name.c_str())].SetComponentRGBColorByColor(key, color);
            }, true);
        if (update_annotation)updateAnnotation(FString(annotation_name.c_str()));
        return success;
    }
    else {
        return false;
    }

}

bool ASimModeBase::SetMeshGreyscaleAnnotationValue(const std::string& annotation_name, const std::string& mesh_name, float greyscale_value, bool is_name_regex, bool update_annotation) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return false;
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::Greyscale) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the greyscale type."), *FString(annotation_name.c_str()));
        return false;
    }
    
    if (is_name_regex) {
        std::regex name_regex;
        name_regex.assign(mesh_name, std::regex_constants::icase);
        int changes = 0;
        for (auto It = annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().CreateConstIterator(); It; ++It)
        {
            if (std::regex_match(TCHAR_TO_UTF8(*It.Key()), name_regex)) {
                bool success;
                FString key = It.Key();
                UAirBlueprintLib::RunCommandOnGameThread([this, key, greyscale_value, &success, annotation_name]() {
                    success = annotators_[FString(annotation_name.c_str())].SetComponentGreyScaleColorByValue(key, greyscale_value);
                    }, true);
                changes++;
            }
        }
        if (update_annotation && changes > 0)updateAnnotation(FString(annotation_name.c_str()));
        return changes > 0;
    }
    else if (annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().Contains(mesh_name.c_str())) {
        bool success;
        FString key = mesh_name.c_str();
        UAirBlueprintLib::RunCommandOnGameThread([this, key, greyscale_value, &success, annotation_name]() {
            success = annotators_[FString(annotation_name.c_str())].SetComponentGreyScaleColorByValue(key, greyscale_value);
            }, true);
        if (update_annotation)updateAnnotation(FString(annotation_name.c_str()));
        return success;
    }
    else {
        return false;
    }

}

float ASimModeBase::GetMeshGreyscaleAnnotationValue(const std::string& annotation_name, const std::string& mesh_name) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return 0;
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::Greyscale) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the greyscale type."), *FString(annotation_name.c_str()));
        return 0;
    }
    float greyscale_value = annotators_[FString(annotation_name.c_str())].GetComponentGreyscaleValue(mesh_name.c_str());
    return greyscale_value;
}

bool ASimModeBase::SetMeshTextureAnnotationPath(const std::string& annotation_name, const std::string& mesh_name, const std::string& texture_path, bool is_name_regex, bool update_annotation) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return false;
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *FString(annotation_name.c_str()));
        return false;
    }
    if (!annotators_[FString(annotation_name.c_str())].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *FString(annotation_name.c_str()));
        return false;
    }
    FString texture_path_fstring = FString(texture_path.c_str());

    if (is_name_regex) {
        std::regex name_regex;
        name_regex.assign(mesh_name, std::regex_constants::icase);
        int changes = 0;
        for (auto It = annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().CreateConstIterator(); It; ++It)
        {
            if (std::regex_match(TCHAR_TO_UTF8(*It.Key()), name_regex)) {
                bool success;
                FString key = It.Key();
                UAirBlueprintLib::RunCommandOnGameThread([this, key, texture_path_fstring, &success, annotation_name]() {
                    success = annotators_[FString(annotation_name.c_str())].SetComponentTextureByDirectPath(key, texture_path_fstring);
                    }, true);
                changes++;
            }
        }
        if (update_annotation && changes > 0)updateAnnotation(FString(annotation_name.c_str()));
        return changes > 0;
    }
    else if (annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().Contains(mesh_name.c_str())) {
        bool success;
        FString key = mesh_name.c_str();
        UAirBlueprintLib::RunCommandOnGameThread([this, key, texture_path_fstring, &success, annotation_name]() {
            success = annotators_[FString(annotation_name.c_str())].SetComponentTextureByDirectPath(key, texture_path_fstring);
            }, true);
        if (update_annotation)updateAnnotation(FString(annotation_name.c_str()));
        return success;
    }
    else {
        return false;
    }

}

bool ASimModeBase::EnableMeshTextureAnnotationByPath(const std::string& annotation_name, const std::string& mesh_name, bool is_name_regex, bool update_annotation) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return false;
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *FString(annotation_name.c_str()));
        return false;
    }
    if (annotators_[FString(annotation_name.c_str())].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to relative path mode."), *FString(annotation_name.c_str()));
        return false;
    }

    if (is_name_regex) {
        std::regex name_regex;
        name_regex.assign(mesh_name, std::regex_constants::icase);
        int changes = 0;
        for (auto It = annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().CreateConstIterator(); It; ++It)
        {
            if (std::regex_match(TCHAR_TO_UTF8(*It.Key()), name_regex)) {
                bool success;
                FString key = It.Key();
                UAirBlueprintLib::RunCommandOnGameThread([this, key, &success, annotation_name]() {
                    success = annotators_[FString(annotation_name.c_str())].SetComponentTextureByRelativePath(key);
                    }, true);
                changes++;
            }
        }
        if (update_annotation && changes > 0)updateAnnotation(FString(annotation_name.c_str()));
        return changes > 0;
    }
    else if (annotators_[FString(annotation_name.c_str())].GetNameToComponentMap().Contains(mesh_name.c_str())) {
        bool success;
        FString key = mesh_name.c_str();
        UAirBlueprintLib::RunCommandOnGameThread([this, key, &success, annotation_name]() {
            success = annotators_[FString(annotation_name.c_str())].SetComponentTextureByRelativePath(key);
            }, true);
        if (update_annotation)updateAnnotation(FString(annotation_name.c_str()));
        return success;
    }
    else {
        return false;
    }
}

std::string ASimModeBase::GetMeshTextureAnnotationPath(const std::string& annotation_name, const std::string& mesh_name) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return "";
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::Texture) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the texture type."), *FString(annotation_name.c_str()));
        return "";
    }
    FString texture_path = annotators_[FString(annotation_name.c_str())].GetComponentTexturePath(mesh_name.c_str());
    return TCHAR_TO_UTF8(*texture_path);
}

int ASimModeBase::GetMeshRGBAnnotationID(const std::string& annotation_name, const std::string& mesh_name) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return -1;
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *FString(annotation_name.c_str()));
        return -1;
    }
    if (annotators_[FString(annotation_name.c_str())].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to index mode."), *FString(annotation_name.c_str()));
        return -1;
    }
    return annotators_[FString(annotation_name.c_str())].GetComponentIndex(mesh_name.c_str());
}

std::string ASimModeBase::GetMeshRGBAnnotationColor(const std::string& annotation_name, const std::string& mesh_name) {
    if (annotators_.Contains(FString(annotation_name.c_str())) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *FString(annotation_name.c_str()), *FString(annotation_name.c_str()));
        return "";
    }
    if (annotators_[FString(annotation_name.c_str())].GetType() != FObjectAnnotator::AnnotatorType::RGB) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not of the RGB type."), *FString(annotation_name.c_str()));
        return "";
    }
    if (!annotators_[FString(annotation_name.c_str())].IsDirect()) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: This annotation layer is not set to direct mode."), *FString(annotation_name.c_str()));
        return "";
    }
    FString color = annotators_[FString(annotation_name.c_str())].GetComponentRGBColor(mesh_name.c_str());
    return TCHAR_TO_UTF8(*color);
}

bool ASimModeBase::AddNewActorToInstanceSegmentation(AActor* Actor, bool update_annotation){
   
	bool success = instance_segmentation_annotator_.AnnotateNewActor(Actor);
    if(success && update_annotation)updateInstanceSegmentationAnnotation();        
    return success;
}

bool ASimModeBase::DeleteActorFromInstanceSegmentation(AActor* Actor, bool update_annotation) {

    bool success = instance_segmentation_annotator_.DeleteActor(Actor);
    if (success && update_annotation)updateInstanceSegmentationAnnotation();
    return success;
}

void ASimModeBase::ForceUpdateInstanceSegmentation() {
	instance_segmentation_annotator_.UpdateAnnotationComponents(this->GetWorld());
    updateInstanceSegmentationAnnotation();
}

bool ASimModeBase::DoesAnnotationLayerExist(FString annotation_name) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    return true;
}

bool ASimModeBase::AddNewActorToAnnotation(FString annotation_name, AActor* Actor, bool update_annotation) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    bool success = annotators_[annotation_name].AnnotateNewActor(Actor);
    if (success && update_annotation)updateAnnotation(annotation_name);
    return success;
}

bool ASimModeBase::DeleteActorFromAnnotation(FString annotation_name, AActor* Actor, bool update_annotation) {

    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
        return false;
    }
    bool success = annotators_[annotation_name].DeleteActor(Actor);
    if (success && update_annotation)updateAnnotation(annotation_name);
    return success;
}

void ASimModeBase::ForceUpdateAnnotation(FString annotation_name) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
    }else{
        annotators_[annotation_name].UpdateAnnotationComponents(this->GetWorld());
        updateAnnotation(annotation_name);
    }
}

void ASimModeBase::updateInstanceSegmentationAnnotation() {
    TArray<TWeakObjectPtr<UPrimitiveComponent>> current_segmentation_components = instance_segmentation_annotator_.GetAnnotationComponents();

    TArray<AActor*> cameras_found;
    UAirBlueprintLib::RunCommandOnGameThread([this, &cameras_found]() {
        UGameplayStatics::GetAllActorsOfClass(this, APIPCamera::StaticClass(), cameras_found);
        }, true);
    if (cameras_found.Num() >= 0) {
        for (auto camera_actor : cameras_found) {
            APIPCamera* cur_camera = static_cast<APIPCamera*>(camera_actor);
            cur_camera->updateInstanceSegmentationAnnotation(current_segmentation_components);
        }
    }
    TArray<AActor*> lidar_cameras_found;
    UAirBlueprintLib::RunCommandOnGameThread([this, &lidar_cameras_found]() {
        UGameplayStatics::GetAllActorsOfClass(this, ALidarCamera::StaticClass(), lidar_cameras_found);
        }, true);
    if (cameras_found.Num() >= 0) {
        for (auto lidar_camera_actor : lidar_cameras_found) {
            ALidarCamera* cur_lidar_camera = static_cast<ALidarCamera*>(lidar_camera_actor);
            if(cur_lidar_camera->instance_segmentation_ && cur_lidar_camera->generate_groundtruth_)
                cur_lidar_camera->updateInstanceSegmentationAnnotation(current_segmentation_components);
        }
    }
    if (CameraDirector != nullptr) {
        if(CameraDirector->getFpvCamera() != nullptr)
			CameraDirector->getFpvCamera()->updateInstanceSegmentationAnnotation(current_segmentation_components, true);
        if (CameraDirector->getExternalCamera() != nullptr)
            CameraDirector->getExternalCamera()->updateInstanceSegmentationAnnotation(current_segmentation_components, true);
        if (CameraDirector->getBackupCamera() != nullptr)
            CameraDirector->getBackupCamera()->updateInstanceSegmentationAnnotation(current_segmentation_components, true);
        if (CameraDirector->getFrontCamera() != nullptr)
            CameraDirector->getFrontCamera()->updateInstanceSegmentationAnnotation(current_segmentation_components, true);
    }
}

void ASimModeBase::updateAnnotation(FString annotation_name) {
    if (annotators_.Contains(annotation_name) == false) {
        UE_LOG(LogTemp, Log, TEXT("AirSim Annotation [%s]: Could not find annotation layer %s"), *annotation_name, *annotation_name);
    }
    else {
        TArray<TWeakObjectPtr<UPrimitiveComponent>> current_annotation_components = annotators_[annotation_name].GetAnnotationComponents();
        TArray<AActor*> cameras_found;
        UAirBlueprintLib::RunCommandOnGameThread([this, &cameras_found]() {
            UGameplayStatics::GetAllActorsOfClass(this, APIPCamera::StaticClass(), cameras_found);
            }, true);
        UAirBlueprintLib::FindAllActor<APIPCamera>(this, cameras_found);
        if (cameras_found.Num() >= 0) {
            for (auto camera_actor : cameras_found) {
                APIPCamera* cur_camera = static_cast<APIPCamera*>(camera_actor);
                cur_camera->updateAnnotation(current_annotation_components, annotation_name);
            }
        }
        TArray<AActor*> lidar_cameras_found;
        UAirBlueprintLib::RunCommandOnGameThread([this, &lidar_cameras_found]() {
            UGameplayStatics::GetAllActorsOfClass(this, ALidarCamera::StaticClass(), lidar_cameras_found);
            }, true);
        UAirBlueprintLib::FindAllActor<ALidarCamera>(this, lidar_cameras_found);
        if (cameras_found.Num() >= 0) {
            for (auto lidar_camera_actor : lidar_cameras_found) {
                ALidarCamera* cur_lidar_camera = static_cast<ALidarCamera*>(lidar_camera_actor);
                if (!cur_lidar_camera->instance_segmentation_ && cur_lidar_camera->generate_groundtruth_ && cur_lidar_camera->annotation_name_ == annotation_name)
                    cur_lidar_camera->updateAnnotation(current_annotation_components);
            }
        }
        if (CameraDirector != nullptr) {
            if (CameraDirector->getFpvCamera() != nullptr)
                CameraDirector->getFpvCamera()->updateInstanceSegmentationAnnotation(current_annotation_components, true);
            if (CameraDirector->getExternalCamera() != nullptr)
                CameraDirector->getExternalCamera()->updateInstanceSegmentationAnnotation(current_annotation_components, true);
            if (CameraDirector->getBackupCamera() != nullptr)
                CameraDirector->getBackupCamera()->updateInstanceSegmentationAnnotation(current_annotation_components, true);
            if (CameraDirector->getFrontCamera() != nullptr)
                CameraDirector->getFrontCamera()->updateInstanceSegmentationAnnotation(current_annotation_components, true);
        }
    }
}

void ASimModeBase::AddAnnotatorCamera(FString name, FObjectAnnotator::AnnotatorType type, float max_view_distance) {
    TArray<AActor*> cameras_found;
    UAirBlueprintLib::FindAllActor<APIPCamera>(this, cameras_found);
    if (cameras_found.Num() >= 0) {
        for (auto camera_actor : cameras_found) {
            APIPCamera* cur_camera = static_cast<APIPCamera*>(camera_actor);
            cur_camera->addAnnotationCamera(name, type, max_view_distance);
        }
    }
}


bool ASimModeBase::SetWorldLightVisibility(const std::string& light_name, bool is_visible)
{
    if (world_lights_.Contains(FString(light_name.c_str())) == true)
    {
        UAirBlueprintLib::RunCommandOnGameThread([this, light_name, is_visible]() {
            ALight* light = world_lights_[FString(light_name.c_str())];
            light->SetEnabled(is_visible);
        }, true);        
        return true;
    }
    return false;
}

bool ASimModeBase::SetWorldLightIntensity(const std::string& light_name, float intensity)
{
    if (world_lights_.Contains(FString(light_name.c_str())) == true)
    {
        UAirBlueprintLib::RunCommandOnGameThread([this, light_name, intensity]() {
            ALight* light = world_lights_[FString(light_name.c_str())];
            light->SetBrightness(intensity);
        }, true);
        return true;
    }
    return false;
}

void ASimModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FRecordingThread::stopRecording();
    FRecordingThread::killRecording();
    world_sim_api_.reset();
    api_provider_.reset();
    api_server_.reset();
    global_ned_transform_.reset();

    CameraDirector = nullptr;
    sky_sphere_ = nullptr;
    sun_ = nullptr;

    spawned_actors_.Empty();
    vehicle_sim_apis_.clear();

    instance_segmentation_annotator_.EndPlay();
    for(auto& annotator : annotators_){
		annotator.Value.EndPlay();
	}
    annotators_.Empty();

    Super::EndPlay(EndPlayReason);
}

void ASimModeBase::initializeTimeOfDay()
{
    sky_sphere_ = nullptr;
    sun_ = nullptr;

    TArray<AActor*> sky_spheres;
    UGameplayStatics::GetAllActorsOfClass(this->GetWorld(), sky_sphere_class_, sky_spheres);

    if (sky_spheres.Num() > 1)
        UAirBlueprintLib::LogMessage(TEXT("More than BP_Sky_Sphere were found. "),
                                     TEXT("TimeOfDay settings would be applied to first one."),
                                     LogDebugLevel::Failure);

    if (sky_spheres.Num() >= 1) {
        sky_sphere_ = sky_spheres[0];
        static const FName sun_prop_name(TEXT("Directional light actor"));
        auto* p = sky_sphere_class_->FindPropertyByName(sun_prop_name);

#if ((ENGINE_MAJOR_VERSION > 4) || (ENGINE_MAJOR_VERSION == 4 && ENGINE_MINOR_VERSION >= 27))
        FObjectProperty* sun_prop = CastFieldChecked<FObjectProperty>(p);
#else
        FObjectProperty* sun_prop = Cast<FObjectProperty>(p);
#endif

        UObject* sun_obj = sun_prop->GetObjectPropertyValue_InContainer(sky_sphere_);
        sun_ = Cast<ADirectionalLight>(sun_obj);
        if (sun_)
            default_sun_rotation_ = sun_->GetActorRotation();
    }
}

void ASimModeBase::setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
                                float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
    bool enabled_currently = tod_enabled_;

    if (is_enabled) {

        if (!sun_) {
            UAirBlueprintLib::LogMessage(TEXT("BP_Sky_Sphere was not found. "),
                                         TEXT("TimeOfDay settings would be ignored."),
                                         LogDebugLevel::Failure);
        }
        else {
            sun_->GetRootComponent()->Mobility = EComponentMobility::Movable;

            // this is a bit odd but given how advanceTimeOfDay() works currently,
            // tod_sim_clock_start_ needs to be reset here.
            tod_sim_clock_start_ = ClockFactory::get()->nowNanos();

            tod_last_update_ = 0;
            if (start_datetime != "")
                tod_start_time_ = Utils::to_time_t(start_datetime, is_start_datetime_dst) * 1E9;
            else
                tod_start_time_ = std::time(nullptr) * 1E9;
        }
    }
    else if (enabled_currently) {
        // Going from enabled to disabled
        if (sun_) {
            setSunRotation(default_sun_rotation_);
            UAirBlueprintLib::LogMessageString("DateTime: ", Utils::to_string(ClockFactory::get()->nowNanos() / 1E9), LogDebugLevel::Informational);
        }
    }

    // do these in the end to ensure that advanceTimeOfDay() doesn't see
    // any inconsistent state.
    tod_enabled_ = is_enabled;
    tod_celestial_clock_speed_ = celestial_clock_speed;
    tod_update_interval_secs_ = update_interval_secs;
    tod_move_sun_ = move_sun;
}

bool ASimModeBase::isPaused() const
{
    return UGameplayStatics::IsGamePaused(this->GetWorld());
}

void ASimModeBase::pause(bool is_paused)
{
    UGameplayStatics::SetGamePaused(this->GetWorld(), is_paused);
}

void ASimModeBase::continueForTime(double seconds)
{
    //should be overridden by derived class
    unused(seconds);
    throw std::domain_error("continueForTime is not implemented by SimMode");
}

void ASimModeBase::continueForFrames(uint32_t frames)
{
    //should be overriden by derived class
    unused(frames);
    throw std::domain_error("continueForFrames is not implemented by SimMode");
}

void ASimModeBase::setWind(const msr::airlib::Vector3r& wind) const
{
    // should be overridden by derived class
    unused(wind);
    throw std::domain_error("setWind not implemented by SimMode");
}

void ASimModeBase::setExtForce(const msr::airlib::Vector3r& ext_force) const
{
    // should be overridden by derived class
    unused(ext_force);
    throw std::domain_error("setExtForce not implemented by SimMode");
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeBase::createApiServer() const
{
    //this will be the case when compilation with RPCLIB is disabled or simmode doesn't support APIs
    return nullptr;
}

void ASimModeBase::setupClockSpeed()
{
    //default setup - this should be overridden in derived modes as needed

    float clock_speed = getSettings().clock_speed;

    //setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock")
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    else if (clock_type == "SteppableClock")
        ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
            static_cast<msr::airlib::TTimeDelta>(msr::airlib::SteppableClock::DefaultStepSize * clock_speed)));
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}

void ASimModeBase::setupPhysicsLoopPeriod()
{
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    if (isRecording())
        ++record_tick_count;

    advanceTimeOfDay();

    showClockStats();

    updateDebugReport(debug_reporter_);

    //drawLidarDebugPoints();

    drawDistanceSensorDebugPoints();

    Super::Tick(DeltaSeconds);
}

void ASimModeBase::showClockStats()
{
    float clock_speed = getSettings().clock_speed;
    if (clock_speed != 1) {
        UAirBlueprintLib::LogMessageString("ClockSpeed config, actual: ",
                                           Utils::stringf("%f, %f", clock_speed, ClockFactory::get()->getTrueScaleWrtWallClock()),
                                           LogDebugLevel::Informational);
    }
}

void ASimModeBase::advanceTimeOfDay()
{
    const auto& settings = getSettings();

    if (tod_enabled_ && sky_sphere_ && sun_ && tod_move_sun_) {
        auto secs = ClockFactory::get()->elapsedSince(tod_last_update_);
        if (secs > tod_update_interval_secs_) {
            tod_last_update_ = ClockFactory::get()->nowNanos();

            auto interval = ClockFactory::get()->elapsedSince(tod_sim_clock_start_) * tod_celestial_clock_speed_;
            uint64_t cur_time = ClockFactory::get()->addTo(tod_start_time_, interval) / 1E9;

            UAirBlueprintLib::LogMessageString("DateTime: ", Utils::to_string(cur_time), LogDebugLevel::Informational);

            auto coord = msr::airlib::EarthCelestial::getSunCoordinates(cur_time, settings.origin_geopoint.home_geo_point.latitude, settings.origin_geopoint.home_geo_point.longitude);

            setSunRotation(FRotator(-coord.altitude, coord.azimuth, 0));
        }
    }
}

void ASimModeBase::setSunRotation(FRotator rotation)
{
    if (sun_ && sky_sphere_) {
        UAirBlueprintLib::RunCommandOnGameThread([this, rotation]() {
            sun_->SetActorRotation(rotation);

            FOutputDeviceNull ar;
            sky_sphere_->CallFunctionByNameWithArguments(TEXT("UpdateSunDirection"), ar, NULL, true);
        },
                                                 true /*wait*/);
    }
}

void ASimModeBase::reset()
{
    //default implementation
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        for (auto& api : getApiProvider()->getVehicleSimApis()) {
            api->reset();
        }
    }, true);
    
    FOnResetEvent.Broadcast();
}

std::string ASimModeBase::getDebugReport()
{
    return debug_reporter_.getOutput();
}

void ASimModeBase::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("InputEventResetAll", EKeys::BackSpace, this, &ASimModeBase::reset);
}

ECameraDirectorMode ASimModeBase::getInitialViewMode() const
{
    return Utils::toEnum<ECameraDirectorMode>(getSettings().initial_view_mode);
}

const msr::airlib::AirSimSettings& ASimModeBase::getSettings() const
{
    return AirSimSettings::singleton();
}

void ASimModeBase::initializeCameraDirector(const FTransform& camera_transform, float follow_distance)
{
    TArray<AActor*> camera_dirs;
    UAirBlueprintLib::FindAllActor<AAirSimCameraDirector>(this, camera_dirs);
    if (camera_dirs.Num() == 0) {
        //create director
        FActorSpawnParameters camera_spawn_params;
        camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
        camera_spawn_params.Name = "CameraDirector";
        CameraDirector = this->GetWorld()->SpawnActor<AAirSimCameraDirector>(camera_director_class_,
                                                                       camera_transform,
                                                                       camera_spawn_params);
        CameraDirector->setFollowDistance(follow_distance);
        CameraDirector->setCameraRotationLagEnabled(false);
        //create external camera required for the director
        camera_spawn_params.Name = "ExternalCamera";
        CameraDirector->ExternalCamera = this->GetWorld()->SpawnActor<APIPCamera>(external_camera_class_,
                                                                                  camera_transform,
                                                                                  camera_spawn_params);
    }
    else {
        CameraDirector = static_cast<AAirSimCameraDirector*>(camera_dirs[0]);
    }
}

bool ASimModeBase::toggleRecording()
{
    if (isRecording())
        stopRecording();
    else
        startRecording();

    return isRecording();
}

void ASimModeBase::stopRecording()
{
    FRecordingThread::stopRecording();
}

void ASimModeBase::startRecording()
{
    FRecordingThread::startRecording(getSettings().recording_setting, getApiProvider()->getVehicleSimApis());
}

bool ASimModeBase::isRecording() const
{
    return FRecordingThread::isRecording();
}

void ASimModeBase::toggleTraceAll()
{
    for (auto sim_api : getApiProvider()->getVehicleSimApis()) {
        auto* pawn_sim_api = static_cast<PawnSimApi*>(sim_api);
        pawn_sim_api->toggleTrace();
    }
}

const APIPCamera* ASimModeBase::getCamera(const msr::airlib::CameraDetails& camera_details) const
{
    return getVehicleSimApi(camera_details.vehicle_name)->getCamera(camera_details.camera_name);
}

const UnrealImageCapture* ASimModeBase::getImageCapture(const std::string& vehicle_name) const
{
    return getVehicleSimApi(vehicle_name)->getImageCapture();
}

//API server start/stop
void ASimModeBase::startApiServer()
{
    if (getSettings().enable_rpc) {

#ifdef AIRLIB_NO_RPC
        api_server_.reset();
#else
        api_server_ = createApiServer();
#endif

        try {
            api_server_->start(false, spawned_actors_.Num() + 4);
        }
        catch (std::exception& ex) {
            UAirBlueprintLib::LogMessageString("Cannot start RpcLib Server", ex.what(), LogDebugLevel::Failure);
        }
    }
    else
        UAirBlueprintLib::LogMessageString("API server is disabled in settings", "", LogDebugLevel::Informational);
}
void ASimModeBase::stopApiServer()
{
    if (api_server_ != nullptr) {
        api_server_->stop();
        api_server_.reset(nullptr);
    }
}
bool ASimModeBase::isApiServerStarted()
{
    return api_server_ != nullptr;
}

void ASimModeBase::updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter)
{
    debug_reporter.update();
    debug_reporter.setEnable(EnableReport);

    if (debug_reporter.canReport()) {
        debug_reporter.clearReport();

        for (auto& api : getApiProvider()->getVehicleSimApis()) {
            PawnSimApi* vehicle_sim_api = static_cast<PawnSimApi*>(api);
            msr::airlib::StateReporter& reporter = *debug_reporter.getReporter();
            std::string vehicle_name = vehicle_sim_api->getVehicleName();

            reporter.writeHeading(std::string("Vehicle: ").append(vehicle_name == "" ? "(default)" : vehicle_name));

            vehicle_sim_api->reportState(reporter);
        }
    }
}

FRotator ASimModeBase::toFRotator(const msr::airlib::AirSimSettings::Rotation& rotation, const FRotator& default_val)
{
    FRotator frotator = default_val;
    if (!std::isnan(rotation.yaw))
        frotator.Yaw = rotation.yaw;
    if (!std::isnan(rotation.pitch))
        frotator.Pitch = rotation.pitch;
    if (!std::isnan(rotation.roll))
        frotator.Roll = rotation.roll;

    return frotator;
}

APawn* ASimModeBase::createVehiclePawn(const AirSimSettings::VehicleSetting& vehicle_setting)
{
    //get UU origin of global NED frame
    const FTransform uu_origin = getGlobalNedTransform().getGlobalTransform();

    // compute initial pose
    FVector spawn_position = uu_origin.GetLocation();
    Vector3r settings_position = vehicle_setting.position;
    if (!VectorMath::hasNan(settings_position))
        spawn_position = getGlobalNedTransform().fromGlobalNed(settings_position);

    FRotator spawn_rotation = toFRotator(vehicle_setting.rotation, uu_origin.Rotator());

    std::string vehicle_name = vehicle_setting.vehicle_name;

    //spawn vehicle pawn
    FActorSpawnParameters pawn_spawn_params;
    pawn_spawn_params.Name = FName(vehicle_name.c_str());
    pawn_spawn_params.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
        getSettings().pawn_paths.at(getVehiclePawnPathName(vehicle_setting)).pawn_bp);
    APawn* spawned_pawn = static_cast<APawn*>(this->GetWorld()->SpawnActor(
        vehicle_bp_class, &spawn_position, &spawn_rotation, pawn_spawn_params));

    spawned_actors_.Add(spawned_pawn);

    return spawned_pawn;
}

std::unique_ptr<PawnSimApi> ASimModeBase::createVehicleApi(APawn* vehicle_pawn)
{
    initializeVehiclePawn(vehicle_pawn);

    //create vehicle sim api
    const auto& ned_transform = getGlobalNedTransform();
    const auto& pawn_ned_pos = ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
    const auto& home_geopoint = msr::airlib::EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);
    const std::string vehicle_name(TCHAR_TO_UTF8(*(vehicle_pawn->GetName())));

    PawnSimApi::Params pawn_sim_api_params(vehicle_pawn, &getGlobalNedTransform(), getVehiclePawnEvents(vehicle_pawn), getVehiclePawnCameras(vehicle_pawn), pip_camera_class, getVehiclePawnLights(vehicle_pawn), collision_display_template, home_geopoint, vehicle_name);

    std::unique_ptr<PawnSimApi> vehicle_sim_api = createVehicleSimApi(pawn_sim_api_params);
    auto vehicle_sim_api_p = vehicle_sim_api.get();
    auto vehicle_api = getVehicleApi(pawn_sim_api_params, vehicle_sim_api_p);
    getApiProvider()->insert_or_assign(vehicle_name, vehicle_api, vehicle_sim_api_p);

    return vehicle_sim_api;
}

bool ASimModeBase::createVehicleAtRuntime(const std::string& vehicle_name, const std::string& vehicle_type,
                                          const msr::airlib::Pose& pose, const std::string& pawn_path)
{
    // Convert to lowercase as done during settings loading
    const std::string vehicle_type_lower = Utils::toLower(vehicle_type);
    if (!isVehicleTypeSupported(vehicle_type_lower)) {
        Utils::log(Utils::stringf("Vehicle type %s is not supported in this game mode", vehicle_type.c_str()), Utils::kLogLevelWarn);
        return false;
    }

    // TODO: Figure out a better way to add more fields
    //       Maybe allow passing a JSON string for the vehicle settings?

    // Retroactively adjust AirSimSettings, so it's like we knew about this vehicle all along
    AirSimSettings::singleton().addVehicleSetting(vehicle_name, vehicle_type_lower, pose, pawn_path);
    const auto* vehicle_setting = getSettings().getVehicleSetting(vehicle_name);

    auto spawned_pawn = createVehiclePawn(*vehicle_setting);
    auto vehicle_sim_api = createVehicleApi(spawned_pawn);

    // Usually physics registration happens at init, in ASimModeWorldBase::initializeForPlay(), but not in this case
    registerPhysicsBody(vehicle_sim_api.get());

    vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));

    return true;
}

void ASimModeBase::setupVehiclesAndCamera()
{
    //get UU origin of global NED frame
    const FTransform uu_origin = getGlobalNedTransform().getGlobalTransform();

    //determine camera director camera default pose and spawn it
    const auto& camera_director_setting = getSettings().camera_director;
    FVector camera_director_position_uu = uu_origin.GetLocation() +
                                          getGlobalNedTransform().fromLocalNed(camera_director_setting.position);
    FTransform camera_transform(toFRotator(camera_director_setting.rotation, FRotator::ZeroRotator),
                                camera_director_position_uu);
    initializeCameraDirector(camera_transform, camera_director_setting.follow_distance);

    //find all vehicle pawns
    {
        TArray<AActor*> pawns;
        getExistingVehiclePawns(pawns);
        bool haveUEPawns = pawns.Num() > 0;
        APawn* fpv_pawn = nullptr;

        if (haveUEPawns) {
            fpv_pawn = static_cast<APawn*>(pawns[0]);
        }
        else {
            //add vehicles from settings
            for (const auto& vehicle_setting_pair : getSettings().vehicles) {
                //if vehicle is of type for derived SimMode and auto creatable
                const auto& vehicle_setting = *vehicle_setting_pair.second;
                if (vehicle_setting.auto_create &&
                    isVehicleTypeSupported(vehicle_setting.vehicle_type)) {

                    auto spawned_pawn = createVehiclePawn(vehicle_setting);
                    pawns.Add(spawned_pawn);

                    if (vehicle_setting.is_fpv_vehicle)
                        fpv_pawn = spawned_pawn;
                }
            }
        }


        //add beacons from settings
        for (auto const& beacon_setting_pair : getSettings().beacons)
        {
            //if vehicle is of type for derived SimMode and auto creatable
            const auto& beacon_setting = *beacon_setting_pair.second;
            //if (beacon_setting.auto_create &&
                //isVehicleTypeSupported(beacon_setting.beacon_type)) {
            if (beacon_setting.auto_create) {
                //compute initial pose
                FVector spawn_position = uu_origin.GetLocation();
                msr::airlib::Vector3r settings_position = beacon_setting.position;
                /*FVector globalOffset = getGlobalNedTransform().getGlobalOffset();

                settings_position.x() += globalOffset.X;
                settings_position.y() += globalOffset.Y;
                settings_position.z() += globalOffset.Z;*/


                if (!msr::airlib::VectorMath::hasNan(settings_position))
                    spawn_position = getGlobalNedTransform().fromGlobalNed(settings_position);
                FRotator spawn_rotation = toFRotator(beacon_setting.rotation, uu_origin.Rotator());

                //spawn beacon pawn
                FActorSpawnParameters pawn_spawn_params;
                FString comboName = beacon_setting.beacon_name.c_str() + FString(":") + beacon_setting.beacon_pawn_name.c_str();
                pawn_spawn_params.Name = FName(*comboName);
                pawn_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

                //auto beacon_bp_class = UAirBlueprintLib::LoadClass(getSettings().pawn_paths.at(beacon_setting.beacon_pawn_name).pawn_bp);
                //FActorSpawnParameters SpawnInfo;
                // TODO: Make the child sim modes responsible for casting the types.
                //ATemplateBeacon* spawned_beacon = static_cast<ATemplateBeacon*>(this->GetWorld()->SpawnActor(beacon_bp_class, &spawn_position, &spawn_rotation, pawn_spawn_params2));

                if (beacon_setting.beacon_type.compare("fiducialmarker") == 0) {
                    AFiducialBeacon* spawned_beacon = static_cast<AFiducialBeacon*>(GetWorld()->SpawnActor<AFiducialBeacon>(spawn_position, spawn_rotation, pawn_spawn_params));
                    spawned_beacon->setScale(beacon_setting.scale);
                }
                else if (beacon_setting.beacon_type.compare("uwbbeacon") == 0) {
                    AUWBBeacon* spawned_beacon = static_cast<AUWBBeacon*>(GetWorld()->SpawnActor<AUWBBeacon>(spawn_position, spawn_rotation, pawn_spawn_params));
                }
                else if (beacon_setting.beacon_type.compare("wifibeacon") == 0) {
                    AWifiBeacon* spawned_beacon = static_cast<AWifiBeacon*>(GetWorld()->SpawnActor<AWifiBeacon>(spawn_position, spawn_rotation, pawn_spawn_params));
                }
                else if (beacon_setting.beacon_type.compare("dynamicblockbeacon") == 0) {
                    ADynamicBlockBeacon* spawned_beacon = static_cast<ADynamicBlockBeacon*>(GetWorld()->SpawnActor<ADynamicBlockBeacon>(spawn_position, spawn_rotation, pawn_spawn_params));
                }
                else if (beacon_setting.beacon_type.compare("dynamicrackbeacon") == 0) {
                    ADynamicRackBeacon* spawned_beacon = static_cast<ADynamicRackBeacon*>(GetWorld()->SpawnActor<ADynamicRackBeacon>(spawn_position, spawn_rotation, pawn_spawn_params));
                }
                else {
                    ATemplateBeacon* spawned_beacon = static_cast<ATemplateBeacon*>(GetWorld()->SpawnActor<ATemplateBeacon>(spawn_position, spawn_rotation, pawn_spawn_params));
                }

                //this->GetWorld()->SpawnActor<AActor>(ATemplateBeacon)
                /*AActor* spawned_actor = static_cast<AActor*>(this->GetWorld()->SpawnActor(
                    beacon_bp_class, &spawn_position, &spawn_rotation, pawn_spawn_params2));

                AirsimVehicle* spawned_pawn2 = dynamic_cast<AirsimVehicle*>(spawned_actor);

                spawned_actors_.Add(spawned_pawn2->GetPawn());
                pawns.Add(spawned_pawn2);

                if (beacon_setting.is_fpv_vehicle)
                    fpv_pawn = spawned_pawn2->GetPawn();*/
            }
        }

        //add passive echo beacons from settings
        for (auto const& passive_echo_beacon_setting_pair : getSettings().passive_echo_beacons)
        {
            const auto& passive_echo_beacon_setting = *passive_echo_beacon_setting_pair.second;
            //compute initial pose
            if (passive_echo_beacon_setting.enable) {
                FVector spawn_position = FVector(0, 0, 0);
                msr::airlib::Vector3r settings_position = passive_echo_beacon_setting.position;
                if (!msr::airlib::VectorMath::hasNan(settings_position))
                    spawn_position = global_ned_transform_->toFVector(settings_position, 100, true);
                FRotator spawn_rotation = toFRotator(passive_echo_beacon_setting.rotation, FRotator());

                //spawn passive echo beacon actor
                FActorSpawnParameters actor_spawn_params;
                actor_spawn_params.Name = FName(passive_echo_beacon_setting.name.c_str());
                actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
                actor_spawn_params.bDeferConstruction = true;
                APassiveEchoBeacon* spawned_passive_echo_beacon = static_cast<APassiveEchoBeacon*>(GetWorld()->SpawnActor<APassiveEchoBeacon>(spawn_position, spawn_rotation, actor_spawn_params));
                spawned_passive_echo_beacon->enable_ = passive_echo_beacon_setting.enable;
                spawned_passive_echo_beacon->name_ = FString(passive_echo_beacon_setting.name.c_str());
                spawned_passive_echo_beacon->initial_directions_ = passive_echo_beacon_setting.initial_directions;
                spawned_passive_echo_beacon->initial_lower_azimuth_limit_ = passive_echo_beacon_setting.initial_lower_azimuth_limit;
                spawned_passive_echo_beacon->initial_upper_azimuth_limit_ = passive_echo_beacon_setting.initial_upper_azimuth_limit;
                spawned_passive_echo_beacon->initial_lower_elevation_limit_ = passive_echo_beacon_setting.initial_lower_elevation_limit;
                spawned_passive_echo_beacon->initial_upper_elevation_limit_ = passive_echo_beacon_setting.initial_upper_elevation_limit;
                spawned_passive_echo_beacon->attenuation_limit_ = passive_echo_beacon_setting.attenuation_limit;
                spawned_passive_echo_beacon->reflection_distance_limit_ = passive_echo_beacon_setting.reflection_distance_limit;
                spawned_passive_echo_beacon->reflection_only_final_ = passive_echo_beacon_setting.reflection_only_final;
                spawned_passive_echo_beacon->attenuation_per_distance_ = passive_echo_beacon_setting.attenuation_per_distance;
                spawned_passive_echo_beacon->attenuation_per_reflection_ = passive_echo_beacon_setting.attenuation_per_reflection;
                spawned_passive_echo_beacon->distance_limit_ = passive_echo_beacon_setting.distance_limit;
                spawned_passive_echo_beacon->reflection_limit_ = passive_echo_beacon_setting.reflection_limit;
                spawned_passive_echo_beacon->draw_debug_location_ = passive_echo_beacon_setting.draw_debug_location;
                spawned_passive_echo_beacon->draw_debug_all_points_ = passive_echo_beacon_setting.draw_debug_all_points;
                spawned_passive_echo_beacon->draw_debug_all_lines_ = passive_echo_beacon_setting.draw_debug_all_lines;
                spawned_passive_echo_beacon->draw_debug_duration_ = passive_echo_beacon_setting.draw_debug_duration;
                spawned_passive_echo_beacon->FinishSpawning(FTransform(spawn_rotation, spawn_position));
            }
        }

        // Add world lights from settings
        for (auto const& world_light_setting_pair : getSettings().world_lights)
        {
            const auto& world_light_settings = *world_light_setting_pair.second;
            //compute initial pose
            if (world_light_settings.enable) {
                FVector spawn_position = FVector(0, 0, 0);
                msr::airlib::Vector3r settings_position = world_light_settings.position;
                if (!msr::airlib::VectorMath::hasNan(settings_position))
                    spawn_position = global_ned_transform_->toFVector(settings_position, 100, true);
                FRotator spawn_rotation = toFRotator(world_light_settings.rotation, FRotator());

                FActorSpawnParameters light_spawn_params;
                light_spawn_params.Name = FName(world_light_settings.name.c_str());
                light_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
                light_spawn_params.bDeferConstruction = true;

                //spawn the right type of light, set specific settings and attach to pawn
                ALight* light = nullptr;
                if (world_light_settings.type ==2) // Rect light
                {
                    ARectLight* rectLight = GetWorld()->SpawnActor<ARectLight>(spawn_position, spawn_rotation, light_spawn_params);
                    rectLight->SetMobility(EComponentMobility::Movable);
                    if (IsValid(rectLight))
                    {
                        URectLightComponent* rectLightComponent = rectLight->GetComponentByClass<URectLightComponent>();
                        rectLightComponent->SetSourceWidth(world_light_settings.source_width);
                        rectLightComponent->SetSourceHeight(world_light_settings.source_height);
                        rectLightComponent->SetBarnDoorAngle(world_light_settings.barn_door_angle);
                        rectLightComponent->SetBarnDoorLength(world_light_settings.barn_door_length);                
                        rectLightComponent->SetAttenuationRadius(world_light_settings.attenuation_radius);
                        if (world_light_settings.intensity_unit == 2)
                        {
                            rectLightComponent->SetIntensityUnits(ELightUnits::EV);
                        }else if (world_light_settings.intensity_unit == 1)
                        {
                            rectLightComponent->SetIntensityUnits(ELightUnits::Lumens);
                        }else
                        {
                            rectLightComponent->SetIntensityUnits(ELightUnits::Candelas);
                        }
                        rectLight->FinishSpawning(FTransform(spawn_rotation, spawn_position));
                        light = static_cast<ALight*>(rectLight);
                    }           
                }else if (world_light_settings.type == 1) // Point light
                {
                    APointLight* pointLight = GetWorld()->SpawnActor<APointLight>(spawn_position, spawn_rotation, light_spawn_params);
                    pointLight->SetMobility(EComponentMobility::Movable);
                    if (IsValid(pointLight))
                    {
                        UPointLightComponent* pointLightComponent = pointLight->GetComponentByClass<UPointLightComponent>();
                        pointLightComponent->SetSourceRadius(world_light_settings.source_radius);
                        pointLightComponent->SetSoftSourceRadius(world_light_settings.source_soft_radius);
                        pointLightComponent->SetAttenuationRadius(world_light_settings.attenuation_radius);
                        if (world_light_settings.intensity_unit == 2)
                        {
                            pointLightComponent->SetIntensityUnits(ELightUnits::EV);
                        }else if (world_light_settings.intensity_unit == 1)
                        {
                            pointLightComponent->SetIntensityUnits(ELightUnits::Lumens);
                        }else
                        {
                            pointLightComponent->SetIntensityUnits(ELightUnits::Candelas);
                        }
                        pointLight->FinishSpawning(FTransform(spawn_rotation, spawn_position));
                        light = static_cast<ALight*>(pointLight);
                    }
                }else // Spot Light
                {
                    ASpotLight* spotLight = GetWorld()->SpawnActor<ASpotLight>(spawn_position, spawn_rotation, light_spawn_params);
                    spotLight->SetMobility(EComponentMobility::Movable);
                    if (IsValid(spotLight))
                    {
                        USpotLightComponent* spotLightComponent = spotLight->GetComponentByClass<USpotLightComponent>();
                        spotLightComponent->SetSourceRadius(world_light_settings.source_radius);
                        spotLightComponent->SetSoftSourceRadius(world_light_settings.source_soft_radius);                        
                        spotLightComponent->SetOuterConeAngle(world_light_settings.outer_cone_angle);
                        spotLightComponent->SetInnerConeAngle(world_light_settings.inner_cone_angle);
                        spotLightComponent->SetAttenuationRadius(world_light_settings.attenuation_radius);
                        if (world_light_settings.intensity_unit == 2)
                        {
                            spotLightComponent->SetIntensityUnits(ELightUnits::EV);
                        }else if (world_light_settings.intensity_unit == 1)
                        {
                            spotLightComponent->SetIntensityUnits(ELightUnits::Lumens);
                        }else
                        {
                            spotLightComponent->SetIntensityUnits(ELightUnits::Candelas);
                        }
                        spotLight->FinishSpawning(FTransform(spawn_rotation, spawn_position));
                        light = static_cast<ALight*>(spotLight);
                    }
                }

                if (IsValid(light))
                {
                    // Set other common settings
                    ULightComponent* lightComponent = light->GetComponentByClass<ULightComponent>();
                    lightComponent->SetIntensity(world_light_settings.intensity);            
                    lightComponent->SetCastShadows(world_light_settings.cast_shadows);
                    if (world_light_settings.cast_shadows)
                    {
                        lightComponent->SetCastVolumetricShadow(true);                
                    }
                    lightComponent->SetUseTemperature(true);
                    lightComponent->SetTemperature(world_light_settings.temperature);            
                    lightComponent->SetVisibility(world_light_settings.enable);
                    lightComponent->SetLightColor(FColor(world_light_settings.color_r, world_light_settings.color_g, world_light_settings.color_b));
                    FString lightName = UTF8_TO_TCHAR(world_light_settings.name.c_str());
                    world_lights_.Add(lightName, light);
                }
            }
        }

        //create API objects for each pawn we have
        for (AActor* pawn : pawns) {
            auto vehicle_pawn = static_cast<APawn*>(pawn);

            auto vehicle_sim_api = createVehicleApi(vehicle_pawn);
            std::string vehicle_name = vehicle_sim_api->getVehicleName();

            if ((fpv_pawn == vehicle_pawn || !getApiProvider()->hasDefaultVehicle()) && vehicle_name != "")
                getApiProvider()->makeDefaultVehicle(vehicle_name);

            vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));
        }
    }

    if (getApiProvider()->hasDefaultVehicle()) {
        //TODO: better handle no FPV vehicles scenario
        getVehicleSimApi()->possess();
        CameraDirector->initializeForBeginPlay(getInitialViewMode(), getVehicleSimApi()->getPawn(), getVehicleSimApi()->getCamera("fpv"), getVehicleSimApi()->getCamera("back_center"), nullptr);
    }
    else
        CameraDirector->initializeForBeginPlay(getInitialViewMode(), nullptr, nullptr, nullptr, nullptr);

    checkVehicleReady();
}

void ASimModeBase::registerPhysicsBody(msr::airlib::VehicleSimApiBase* physicsBody)
{
    // derived class shoudl override this method to add new vehicle to the physics engine
}

void ASimModeBase::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    //derived class should override this method to retrieve types of pawns they support
}

bool ASimModeBase::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    //derived class should override this method to retrieve types of pawns they support
    return false;
}

std::string ASimModeBase::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //derived class should override this method to retrieve types of pawns they support
    return "";
}

std::string ASimModeBase::getBeaconPawnPathName(const AirSimSettings::BeaconSetting& beacon_setting) const
{
    //derived class should override this method to retrieve types of pawns they support
    return "";
}

PawnEvents* ASimModeBase::getVehiclePawnEvents(APawn* pawn) const
{
    unused(pawn);

    //derived class should override this method to retrieve types of pawns they support
    return nullptr;
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeBase::getVehiclePawnCameras(APawn* pawn) const
{
    unused(pawn);

    //derived class should override this method to retrieve types of pawns they support
    return common_utils::UniqueValueMap<std::string, APIPCamera*>();
}
const common_utils::UniqueValueMap<std::string, ALight*> ASimModeBase::getVehiclePawnLights(APawn* pawn) const
{
    unused(pawn);

    //derived class should override this method to retrieve types of pawns they support
    return common_utils::UniqueValueMap<std::string, ALight*>();
}
void ASimModeBase::initializeVehiclePawn(APawn* pawn)
{
    unused(pawn);
    //derived class should override this method to retrieve types of pawns they support
}
std::unique_ptr<PawnSimApi> ASimModeBase::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
    unused(pawn_sim_api_params);
    auto sim_api = std::unique_ptr<PawnSimApi>();
    sim_api->initialize();

    return sim_api;
}
msr::airlib::VehicleApiBase* ASimModeBase::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                         const PawnSimApi* sim_api) const
{
    //derived class should override this method to retrieve types of pawns they support
    return nullptr;
}

// Draw debug-point on main viewport for Distance sensor hit
void ASimModeBase::drawDistanceSensorDebugPoints()
{
    if (getApiProvider() == nullptr)
        return;

    for (auto& sim_api : getApiProvider()->getVehicleSimApis()) {
        PawnSimApi* pawn_sim_api = static_cast<PawnSimApi*>(sim_api);
        std::string vehicle_name = pawn_sim_api->getVehicleName();

        msr::airlib::VehicleApiBase* api = getApiProvider()->getVehicleApi(vehicle_name);

        if (api != nullptr) {
            msr::airlib::uint count_distance_sensors = api->getSensors().size(SensorType::Distance);
            Pose vehicle_pose = pawn_sim_api->getGroundTruthKinematics()->pose;

            for (msr::airlib::uint i = 0; i < count_distance_sensors; i++) {
                const msr::airlib::DistanceSimple* distance_sensor =
                    static_cast<const msr::airlib::DistanceSimple*>(api->getSensors().getByType(SensorType::Distance, i));

                if (distance_sensor != nullptr && distance_sensor->getParams().draw_debug_points) {
                    msr::airlib::DistanceSensorData distance_sensor_data = distance_sensor->getOutput();

                    // Find position of point hit
                    // Similar to UnrealDistanceSensor.cpp#L19
                    // order of Pose addition is important here because it also adds quaternions which is not commutative!
                    Pose distance_sensor_pose = distance_sensor_data.relative_pose + vehicle_pose;
                    Vector3r start = distance_sensor_pose.position;
                    Vector3r point = start + VectorMath::rotateVector(VectorMath::front(),
                                                                      distance_sensor_pose.orientation,
                                                                      true) *
                                                 distance_sensor_data.distance;

                    FVector uu_point = pawn_sim_api->getNedTransform().fromLocalNed(point);

                    DrawDebugPoint(
                        this->GetWorld(),
                        uu_point,
                        10, // size
                        FColor::Green,
                        false, // persistent (never goes away)
                        0.03 // LifeTime: point leaves a trail on moving object
                    );
                }
            }
        }
    }
}
