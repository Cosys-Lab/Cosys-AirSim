#pragma once

#include "Runtime/Core/Public/HAL/ThreadSafeBool.h"

#include "Components/StaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "UrdfParser/UrdfForceSpecification.h"
#include "DrawDebugHelpers.h"

#include "physics/Kinematics.hpp"

#include "AirBlueprintLib.h"

#include "ProceduralMeshComponent.h"

#include "UrdfLink.generated.h"

class UrdfBotPawn;

USTRUCT()
struct FAUrdfLinkSecondaryTickFunction : public FTickFunction
{
    GENERATED_USTRUCT_BODY()

    class AUrdfLink* target_;

    virtual void ExecuteTick(
        float deltaTime,
        ELevelTick tickType,
        ENamedThreads::Type currentThread,
        const FGraphEventRef &MyCompletionGraphEvent) override;

    virtual FString DiagnosticMessage() override;
};

template<>
struct TStructOpsTypeTraits<FAUrdfLinkSecondaryTickFunction> : public TStructOpsTypeTraitsBase2<FAUrdfLinkSecondaryTickFunction>
{
    enum
    {
        WithCopy = false
    };
};

UCLASS()
class AUrdfLink : public AActor
{
    GENERATED_BODY()

    public:
        AUrdfLink();
        ~AUrdfLink();

        virtual void BeginPlay() override;

        //virtual void TickComponent(float deltaTime, ELevelTick tickType, FActorComponentTickFunction* thisTickFunction) override;
        virtual void Tick(float delta) override;
        void TickPostPhysics(float deltaSeconds, ELevelTick tickType, FAUrdfLinkSecondaryTickFunction& thisTickFunction);
        virtual void NotifyHit(class UPrimitiveComponent* myComp, class AActor* other, class UPrimitiveComponent* otherComp, bool bSelfMoved, FVector hitLocation,
            FVector hitNormal, FVector normalImpulse, const FHitResult &hit) override;

        void SetCollisionComponent(UPrimitiveComponent* collisionComponent);
        UPrimitiveComponent* GetCollisionComponent();

        const msr::airlib::Kinematics::State& GetKinematics() const;

        void GetReferenceFrameLocation(FVector &translation, FRotator &rotation);
        void SetReferenceFrameLocation(FVector &translation, FRotator &rotation);

        void RecordVisualOffset(FVector &translation, FRotator &rotation);

        // AUrdfLink now owns the ForceSpecification wrt memory allocation / deallocation
        void AddForceSpecification(UrdfForceSpecification* forceSpecification);

        // This will delete the ForceSpecification
        void RemoveForceSpecification(UrdfForceSpecification* forceSpecification);
        void SetForceMagnitude(FString forceName, float magnitude);

        TMap<FString, UrdfForceSpecification*> GetForceSpecifications() const;
        void ResetForceMagnitudes();

        // results are in UNREAL UNITS
        FVector WorldFromReferenceOffset(FVector referenceOffset, bool inputInMeters);

        void ComputeForces(float deltaTime, bool inSubstep);

        void SetOwningActor(AUrdfBotPawn* owner);
        AUrdfBotPawn* GetOwningActor();

        void UpdateKinematics(float dt);
        msr::airlib::Pose GetPose();

        void SetMeshFromStaticMeshComponent(UStaticMeshComponent* mesh);
        void SetMeshFromProceduralMeshComponent(UProceduralMeshComponent* proceduralMeshComponent);
        void SetMass(float massInKg);
        void SetMaterial(UMaterial* material);
        UMeshComponent* GetRootMesh();
        FVector GetPhysicsLinearVelocity();
        FVector GetPhysicsAngularVelocityInRadians();


    private:
        void SubstepTick(float deltaTime, FBodyInstance* bodyInstance);
        void AcquireForceLock();
        void ReleaseForceLock();
        void InitPostSetMesh();

        const float SiForceToUU_ = 100.0f;
        const float SiTorqueToUU_ = 100.0f * 100.0f;

        FCalculateCustomPhysics onCalculateCustomPhysics;

        FThreadSafeBool forceLock_;
        class AUrdfBotPawn* owner_ = nullptr;
        int frameCount_;
        
        UPrimitiveComponent* collisionComponent_;

        FAUrdfLinkSecondaryTickFunction secondaryComponentTick_;

        FVector referenceTranslation_;
        FRotator referenceRotation_;

        FVector visualOffsetTranslation_;
        FRotator visualOffsetRotation_;

        TMap<FString, UrdfForceSpecification*> forceSpecifications_;
        TMap<FString, float> lastSetMagnitudes_;

        msr::airlib::Kinematics::State kinematics_;

        UMeshComponent* mesh_root_ = nullptr;
};