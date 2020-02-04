#pragma once

#include "CoreMinimal.h"

#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "AirLib/include/common/common_utils/Utils.hpp"
#include "ControlledMotionComponent.h"
#include "Vehicles/UrdfBot/UrdfLink.h"
#include "Vehicles/UrdfBot/UrdfParser/UrdfJointSpecification.h"

class Motor : public ControlledMotionComponent
{
public:
    virtual ~Motor() override {};

    virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent) override;
    virtual void SetControl(TMap<FString, float> controlSignals) override;
    virtual void ComputeForces(float delta) override;
    virtual TMap<FString, FString> GetState() override;

private:
    float controlSignalSetPoint_ = 0;
    float actualMotorSpeed_ = 0;

    FVector rotationAxis_;

    float maxVelocity_ = 0.0f;
    float deadZone_ = 0.05f;
};