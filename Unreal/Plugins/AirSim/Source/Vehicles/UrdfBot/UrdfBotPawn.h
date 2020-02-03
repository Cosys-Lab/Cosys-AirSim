#pragma once

#include "common/AirSimSettings.hpp"

#include "CoreMinimal.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "UObject/ConstructorHelpers.h"
#include "Runtime/Engine/Classes/Components/BoxComponent.h"
#include "Runtime/Engine/Classes/Components/SphereComponent.h"
#include "Runtime/Engine/Classes/Components/CapsuleComponent.h"
#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"
#include "Runtime/Engine/Classes/PhysicsEngine/ConstraintInstance.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Core/Public/Internationalization/Regex.h"

#include "PIPCamera.h"
#include "common/common_utils/UniqueValueMap.hpp"
#include "common/common_utils/Utils.hpp"
#include "AirBlueprintLib.h"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"

#include "UrdfParser/UrdfGeometry.h"
#include "UrdfParser/UrdfParser.h"
#include "UrdfLink.h"

#include "ControlledMotionComponent/ControlledMotionComponentFactory.h"
#include "ControlledMotionComponent/ControlledMotionComponent.h"

#include "MeshGeneration/StaticMeshGenerator.h"

#include "PawnEvents.h"

#include "Vehicles/AirsimVehicle.h"

#include "PhysXIncludes.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"

#include "UrdfBotPawn.generated.h"

UCLASS()
class AUrdfBotPawn : public APawn, public AirsimVehicle
{
    GENERATED_BODY()

    public:
        AUrdfBotPawn();
        ~AUrdfBotPawn();

        virtual void BeginPlay() override;
        virtual void Tick(float delta) override;
        virtual void EndPlay(const EEndPlayReason::Type endPlayReason) override;
        virtual void NotifyHit(class UPrimitiveComponent* myComp, class AActor* other, class UPrimitiveComponent* otherComp, bool bSelfMoved, FVector hitLocation,
            FVector hitNormal, FVector normalImpulse, const FHitResult &hit) override;

        void InitializeForBeginPlay();
        common_utils::UniqueValueMap<std::string, APIPCamera*> GetCameras() const;

        // TODO: Can this be const?
        PawnEvents* GetPawnEvents();

        TMap<FString, AUrdfLink*> GetLinkComponents() const;
        AUrdfLink* GetRootLinkComponent() const;

        TMap<FString, ControlledMotionComponent*> GetControlledMotionComponents() const;

        virtual USceneComponent* GetComponent(FString componentName) override;
        virtual AUrdfLink* GetLink(FString linkName);
        virtual APawn* GetPawn() override { return this; }
        virtual void GetComponentReferenceTransform(FString componentName, FVector& translation, FRotator& rotation) override;
        virtual bool PawnUsesNedCoords() override { return false; }

        virtual void TeleportToLocation(FVector position, FQuat orientation, bool teleport) override;

    private:
        typedef msr::airlib::AirSimSettings AirSimSettings;

        void ConstructFromFile(FString fileName);

        AUrdfLink* CreateLinkFromSpecification(const UrdfLinkSpecification &linkSpecification);
        void AttachChildren(AUrdfLink* parentLink, const UrdfLinkSpecification &parentLinkSpecification, AUrdfLink* childLink, const UrdfLinkSpecification &childLinkSpecification, UrdfJointSpecification *jointSpecification);

        FConstraintInstance CreateConstraint(const UrdfJointSpecification &jointSpecification);
        void ResizeLink(AUrdfLink* link, UrdfGeometry* geometry);

        UrdfLinkSpecification* FindRootNodeSpecification(TMap<FString, UrdfLinkSpecification*> links);
        FConstraintInstance CreateDefaultFixedConstraintInstance();
        bool ConstraintNeedsControlledMotionComponent(const UrdfJointSpecification &spec);

        // For limited translation joints, we need to move the link to the center of the allowable motion before setting the constraint.
        // This is due to a limitation in the PhysX engine - offsets are symmetric from reference poses of joints when initConstraint() is called.
        // Returns a FTransform that undoes the transform. This will be either a pure rotation (for revolute joints) or a pure translation (for prismatic joints)
        // The intent is to call this function to move the component to the center, initialize the joint, then apply the given vector to move the component back.
        FVector MoveChildLinkForLimitedXAxisMotion(AUrdfLink* parentLink, AUrdfLink* childLink, const UrdfJointSpecification &jointSpecification);

        void DrawDebug();
        void MoveAllComponents(FVector translation, FRotator rotation);

        UPROPERTY()
        TMap<FString, AUrdfLink*> components_;

        UPROPERTY()
        TMap<FString, bool> component_visited_as_constraint_parent_;

        UPROPERTY()
        AUrdfLink* root_component_;

        PawnEvents pawn_events_;

        UPROPERTY()
        UStaticMesh* baseBoxMesh_;

        UPROPERTY()
        UStaticMesh* baseCylinderMesh_;

        UPROPERTY()
        UStaticMesh* baseSphereMesh_;

        UPROPERTY()
        USceneComponent* rootComponent_;

        UPROPERTY()
        TMap<FString, UMaterialInterface*> materials_;

        UPROPERTY()
        TMap<FString, UStaticMesh*> user_static_meshes_;

        UPROPERTY()
        TMap<FString, UStaticMesh*> procedural_mesh_cache_;

        TMap<FString, TArray<FRegexPattern>> collision_blacklist_;

        TMap<FString, TTuple<UrdfJointType, UPhysicsConstraintComponent*>> constraints_;

        TMap<FString, ControlledMotionComponent*> controlled_motion_components_;

        StaticMeshGenerator staticMeshGenerator_;

        common_utils::UniqueValueMap<std::string, APIPCamera*> cameras_;

        float world_scale_;
        bool draw_debug_ = false;
        float debug_symbol_scale_ = 0.0f;
};