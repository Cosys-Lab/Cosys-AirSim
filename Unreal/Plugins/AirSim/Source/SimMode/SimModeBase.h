#pragma once

#include "CoreMinimal.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "GameFramework/Actor.h"
#include "Components/MeshComponent.h"
#include "ParticleDefinitions.h"
#include "Annotation/ObjectAnnotator.h"
#include <string>
#include "AirSimCameraDirector.h"
#include "common/AirSimSettings.hpp"
#include "AssetRegistry/AssetData.h"
#include "common/ClockFactory.hpp"
#include "Engine/Light.h"
#include "api/ApiServerBase.hpp"
#include "api/ApiProvider.hpp"
#include "PawnSimApi.h"
#include "common/StateReporterWrapper.hpp"
#include "LoadingScreenWidget.h"
#include "UnrealImageCapture.h"
#include "Beacons/TemplateBeacon.h"
#include "Beacons/FiducialBeacon.h"
#include "SimModeBase.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FLevelLoaded);

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnResetEvent);


UCLASS()
class AIRSIM_API ASimModeBase : public AActor
{
public:
    GENERATED_BODY()

    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "airsim | Utils")
    FLevelLoaded OnLevelLoaded;

    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "airsim | Utils")
    FLevelLoaded FOnResetEvent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "airsim | Utils")
    AAirSimCameraDirector* CameraDirector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "airsim | Utils")
    bool EnableReport = false;

    UFUNCTION(BlueprintCallable, Category = "airsim | Utils")
    bool toggleRecording();

	UFUNCTION(BlueprintCallable, Category = "airsim | Instance Segmentation")
	bool AddNewActorToInstanceSegmentation(AActor* Actor, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Instance Segmentation")
    bool DeleteActorFromInstanceSegmentation(AActor* Actor, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Instance Segmentation")
    void ForceUpdateInstanceSegmentation();

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool DoesAnnotationLayerExist(FString annotation_name);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddNewActorToAnnotation(FString annotation_name, AActor* Actor, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddRGBDirectAnnotationTagToActor(FString annotation_name, AActor* actor, FColor color, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateRGBDirectAnnotationTagToActor(FString annotation_name, AActor* actor, FColor color, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddRGBIndexAnnotationTagToActor(FString annotation_name, AActor* actor, int32 index, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateRGBIndexAnnotationTagToActor(FString annotation_name, AActor* actor, int32 index, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddRGBDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, FColor color, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateRGBDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, FColor color, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddRGBIndexAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, int32 index, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateRGBIndexAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, int32 index, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddGreyscaleAnnotationTagToActor(FString annotation_name, AActor* actor, float greyscale_value, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateGreyscaleAnnotationTagToActor(FString annotation_name, AActor* actor, float greyscale_value, bool update_annotation = true);
  
    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddGreyscaleAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, float greyscale_value, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateGreyscaleAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, float greyscale_value, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddTextureDirectAnnotationTagToActorByPath(FString annotation_name, AActor* actor, FString texture_path, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateTextureDirectAnnotationTagToActorByPath(FString annotation_name, AActor* actor, FString texture_path, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddTextureDirectAnnotationTagToComponentByPath(FString annotation_name, UMeshComponent* component, FString texture_path, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateTextureDirectAnnotationTagToComponentByPath(FString annotation_name, UMeshComponent* component, FString texture_path, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddTextureDirectAnnotationTagToActor(FString annotation_name, AActor* actor, UTexture* texture, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateTextureDirectAnnotationTagToActor(FString annotation_name, AActor* actor, UTexture* texture, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool AddTextureDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, UTexture* texture, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool UpdateTextureDirectAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, UTexture* texture, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool EnableTextureByPathAnnotationTagToActor(FString annotation_name, AActor* actor, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool EnableTextureByPathAnnotationTagToComponent(FString annotation_name, UMeshComponent* component, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool DeleteActorFromAnnotation(FString annotation_name, AActor* Actor, bool update_annotation = true);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    void ForceUpdateAnnotation(FString annotation_name);

    UFUNCTION(BlueprintCallable, Category = "airsim | Annotation")
    bool IsAnnotationRGBValid(FString annotation_name, FColor color);

public:
    UFUNCTION(BlueprintPure, Category = "airsim | Utils")
    static ASimModeBase* getSimMode();

    UFUNCTION(BlueprintCallable, Category = "airsim | Utils")
    void toggleLoadingScreen(bool is_visible);

    UFUNCTION(BlueprintCallable, Category = "airsim | Utils")
    virtual void reset();

    // Sets default values for this actor's properties
    ASimModeBase();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    //additional overridable methods
    virtual std::string getDebugReport();
    virtual ECameraDirectorMode getInitialViewMode() const;

    virtual bool isPaused() const;
    virtual void pause(bool is_paused);
    virtual void continueForTime(double seconds);
    virtual void continueForFrames(uint32_t frames);

    virtual void setWind(const msr::airlib::Vector3r& wind) const;
    virtual void setExtForce(const msr::airlib::Vector3r& ext_force) const;

    virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
                              float celestial_clock_speed, float update_interval_secs, bool move_sun);

    virtual void startRecording();
    virtual void stopRecording();
    virtual bool isRecording() const;

    virtual void toggleTraceAll();

    void startApiServer();
    void stopApiServer();
    bool isApiServerStarted();

    bool createVehicleAtRuntime(const std::string& vehicle_name, const std::string& vehicle_type,
                                const msr::airlib::Pose& pose, const std::string& pawn_path = "");

    const NedTransform& getGlobalNedTransform();

    msr::airlib::ApiProvider* getApiProvider() const
    {
        return api_provider_.get();
    }
    const PawnSimApi* getVehicleSimApi(const std::string& vehicle_name = "") const
    {
        return static_cast<PawnSimApi*>(api_provider_->getVehicleSimApi(vehicle_name));
    }
    PawnSimApi* getVehicleSimApi(const std::string& vehicle_name = "")
    {
        return static_cast<PawnSimApi*>(api_provider_->getVehicleSimApi(vehicle_name));
    }
	std::vector<std::string> GetAllInstanceSegmentationMeshIDs();
    std::vector<msr::airlib::Pose> GetAllInstanceSegmentationMeshPoses(bool ned = true, bool only_visible = false);
    TMap<UMeshComponent*, FString> GetInstanceSegmentationComponentToNameMap();
    std::vector<msr::airlib::Vector3r> GetInstanceSegmentationColorMap();

	bool SetMeshInstanceSegmentationID(const std::string& mesh_name, int object_id, bool is_name_regex, bool update_annotation = true);
    int GetMeshInstanceSegmentationID(const std::string& mesh_name);

    std::vector<std::string> GetAllAnnotationMeshIDs(const std::string& annotation_name);
    std::vector<msr::airlib::Pose> GetAllAnnotationMeshPoses(const std::string& annotation_name, bool ned = true, bool only_visible = false);

    bool SetMeshRGBAnnotationID(const std::string& annotation_name, const std::string& mesh_name, int object_id, bool is_name_regex, bool update_annotation = true);
    bool SetMeshRGBAnnotationColor(const std::string& annotation_name, const std::string& mesh_name, int r, int g, int b, bool is_name_regex, bool update_annotation = true);
    int GetMeshRGBAnnotationID(const std::string& annotation_name, const std::string& mesh_name);
    std::string GetMeshRGBAnnotationColor(const std::string& annotation_name, const std::string& mesh_name);

    bool SetMeshGreyscaleAnnotationValue(const std::string& annotation_name, const std::string& mesh_name, float greyscale_value, bool is_name_regex, bool update_annotation = true);
    float GetMeshGreyscaleAnnotationValue(const std::string& annotation_name, const std::string& mesh_name);

    bool EnableMeshTextureAnnotationByPath(const std::string& annotation_name, const std::string& mesh_name, bool is_name_regex, bool update_annotation = true);
    bool SetMeshTextureAnnotationPath(const std::string& annotation_name, const std::string& mesh_name, const std::string& texture_path, bool is_name_regex, bool update_annotation = true);
    std::string GetMeshTextureAnnotationPath(const std::string& annotation_name, const std::string& mesh_name);

    bool SetWorldLightVisibility(const std::string& light_name, bool is_visible);
    bool SetWorldLightIntensity(const std::string& light_name, float intensity);
    
	static void RunCommandOnGameThread(TFunction<void()> InFunction, bool wait = false, const TStatId InStatId = TStatId());

    const APIPCamera* getCamera(const msr::airlib::CameraDetails& camera_details) const;

    APIPCamera* getCamera(const msr::airlib::CameraDetails& camera_details)
    {
        return const_cast<APIPCamera*>(static_cast<const ASimModeBase*>(this)->getCamera(camera_details));
    }

    const UnrealImageCapture* getImageCapture(const std::string& vehicle_name = "") const;

    TMap<FString, FAssetData> asset_map;
    TMap<FString, AActor*> scene_object_map;
    UMaterial* domain_rand_material_;

protected: //must overrides
    typedef msr::airlib::AirSimSettings AirSimSettings;

    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const;
    virtual void getExistingVehiclePawns(TArray<AActor*>& pawns) const;
    virtual bool isVehicleTypeSupported(const std::string& vehicle_type) const;
    virtual std::string getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const;
    virtual std::string getBeaconPawnPathName(const AirSimSettings::BeaconSetting& beacon_setting) const;
    virtual PawnEvents* getVehiclePawnEvents(APawn* pawn) const;
    virtual const common_utils::UniqueValueMap<std::string, APIPCamera*> getVehiclePawnCameras(APawn* pawn) const;
    virtual const common_utils::UniqueValueMap<std::string, ALight*> getVehiclePawnLights(APawn* pawn) const;
    virtual void initializeVehiclePawn(APawn* pawn);
    virtual std::unique_ptr<PawnSimApi> createVehicleSimApi(
        const PawnSimApi::Params& pawn_sim_api_params) const;
    virtual msr::airlib::VehicleApiBase* getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                       const PawnSimApi* sim_api) const;
    virtual void registerPhysicsBody(msr::airlib::VehicleSimApiBase* physicsBody);

protected: //optional overrides
    virtual APawn* createVehiclePawn(const AirSimSettings::VehicleSetting& vehicle_setting);
    virtual std::unique_ptr<PawnSimApi> createVehicleApi(APawn* vehicle_pawn);
    virtual void setupVehiclesAndCamera();
    virtual void setupInputBindings();
    //called when SimMode should handle clock speed setting
    virtual void setupClockSpeed();
    void initializeCameraDirector(const FTransform& camera_transform, float follow_distance);
    void checkVehicleReady(); //checks if vehicle is available to use
    virtual void updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter);

    virtual void updateInstanceSegmentationAnnotation();
    virtual void updateAnnotation(FString annotation_name);

protected: //Utility methods for derived classes
    virtual const AirSimSettings& getSettings() const;
    FRotator toFRotator(const AirSimSettings::Rotation& rotation, const FRotator& default_val);

protected:
    int record_tick_count;
    UPROPERTY()
    UClass* pip_camera_class;
    UPROPERTY()
    UParticleSystem* collision_display_template;

private:
    typedef common_utils::Utils Utils;
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;
    typedef msr::airlib::SensorBase::SensorType SensorType;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::VectorMath VectorMath;

private:
    //assets loaded in constructor
    UPROPERTY()
    UClass* external_camera_class_;
    UPROPERTY()
    UClass* camera_director_class_;
    UPROPERTY()
    UClass* sky_sphere_class_;
    UPROPERTY()
    ULoadingScreenWidget* loading_screen_widget_;

    UPROPERTY()
    AActor* sky_sphere_;
    UPROPERTY()
    ADirectionalLight* sun_;
    FRotator default_sun_rotation_;
    TTimePoint tod_sim_clock_start_; // sim start in local time
    TTimePoint tod_last_update_;
    TTimePoint tod_start_time_; // tod, configurable
    bool tod_enabled_;
    float tod_celestial_clock_speed_;
    float tod_update_interval_secs_;
    bool tod_move_sun_;

    std::unique_ptr<NedTransform> global_ned_transform_;
    std::unique_ptr<msr::airlib::WorldSimApiBase> world_sim_api_;
    std::unique_ptr<msr::airlib::ApiProvider> api_provider_;
    std::unique_ptr<msr::airlib::ApiServerBase> api_server_;
    msr::airlib::StateReporterWrapper debug_reporter_;

    std::vector<std::unique_ptr<msr::airlib::VehicleSimApiBase>> vehicle_sim_apis_;

    UPROPERTY()
    TArray<AActor*> spawned_actors_; //keep refs alive from Unreal GC

    static ASimModeBase* SIMMODE;

    FObjectAnnotator instance_segmentation_annotator_;
    TMap<FString, FObjectAnnotator> annotators_;

    TMap<FString, ALight*> world_lights_;

private:
    void InitializeInstanceSegmentation();
    void InitializeAnnotation();
    void AddAnnotatorCamera(FString name, FObjectAnnotator::AnnotatorType type, float max_view_distance = -1.0f);
	void InitializeMaterialStencils();
    void initializeTimeOfDay();
    void advanceTimeOfDay();
    void setSunRotation(FRotator rotation);
    void setupPhysicsLoopPeriod();
    void showClockStats();
    void drawDistanceSensorDebugPoints();
};
