#pragma once

#include "CoreMinimal.h"

#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "ControlledMotionComponent.h"
#include "Vehicles/UrdfBot/UrdfLink.h"
#include "Vehicles/UrdfBot/UrdfParser/UrdfJointSpecification.h"

class LinearActuator : public ControlledMotionComponent
{
    public:
        virtual ~LinearActuator() override {};

        virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent) override;
        virtual void SetControl(TMap<FString, float> controlSignals) override;
        virtual void ComputeForces(float delta) override;
        virtual TMap<FString, FString> GetState() override;

    private:
        float maxActuationRate_ = 1.0f;

        FVector baseToActuatorZero_;
        FVector baseToActuatorOne_;

        float deadZone_ = 0.05f;
        float controlSignalSetPoint_ = 0;
        float actualActuatorPosition_ = 0;
        float invRange_ = 1;
        float range_ = 0;
        float lower_ = 0;
        bool runOneTick_ = false;
};