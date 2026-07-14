// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/MeshComponent.h"
#include "Components/ActorComponent.h"
#include "Components/SceneComponent.h"
#include "TextureResource.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkinnedMeshComponent.h"
#include "ObjectFilter.h"
#include <string>
#include "DetectionComponent.generated.h"

USTRUCT()
struct FDetectionInfo
{
    GENERATED_BODY()

    UPROPERTY()
    AActor* Actor;

    UPROPERTY()    
    UMeshComponent* Component;

    UPROPERTY()
    FString DetectionName;

    UPROPERTY()
    FBox2D Box2D;

    UPROPERTY()
    FBox Box3D;

    UPROPERTY()
    FTransform RelativeTransform;

    FDetectionInfo()
        : Actor(nullptr)
        , Component(nullptr)
        , DetectionName(TEXT(""))
        , Box2D(FBox2D(EForceInit::ForceInit))
        , Box3D(FBox())
        , RelativeTransform(FTransform::Identity)
    {
    }
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class AIRSIM_API UDetectionComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    // Sets default values for this component's properties
    UDetectionComponent();

protected:
    // Called when the game starts
    virtual void BeginPlay() override;

public:
    // Called every frame
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    const TArray<FDetectionInfo>& getDetections(TMap<UMeshComponent*, FString>  component_to_name_map, bool component_based = true);

    void addMeshName(const std::string& mesh_name);
    void setFilterRadius(const float radius_cm);
    void clearMeshNames();

private:
    bool calcBoundingFromViewInfo(AActor* actor, FBox2D& box_out);
    bool calcBoundingFromViewInfoComponent(UMeshComponent* component, FBox2D& box_out);

    FVector getRelativeLocation(FVector in_location);

    FRotator getRelativeRotation(FVector in_location, FRotator in_rotation);

public:
    UPROPERTY()
    UTextureRenderTarget2D* texture_target_;

private:
    UPROPERTY()
    FObjectFilter object_filter_;

    UPROPERTY(EditAnywhere, Category = "Tracked Actors")
    float max_distance_to_camera_;

    UPROPERTY()
    USceneCaptureComponent2D* scene_capture_component_2D_;

    UPROPERTY()
    TArray<FDetectionInfo> cached_detections_;
};
