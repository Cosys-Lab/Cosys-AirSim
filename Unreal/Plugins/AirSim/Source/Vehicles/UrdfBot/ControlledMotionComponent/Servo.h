#pragma once

#include "CoreMinimal.h"

#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "AirLib/include/common/common_utils/Utils.hpp"
#include "ControlledMotionComponent.h"
#include "Vehicles/UrdfBot/UrdfLink.h"
#include "Vehicles/UrdfBot/UrdfParser/UrdfJointSpecification.h"

class Servo : public ControlledMotionComponent
{
public:
    virtual ~Servo() override {};

    virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent) override;
    virtual void SetControl(TMap<FString, float> controlSignals) override;
    virtual void ComputeForces(float delta) override;
    virtual TMap<FString, FString> GetState() override;

private:
    float controlSignalSetPoint_ = 0.0f;
    float actualServoPosition_ = 0.0f;

    float maxRotationRate_ = 0.0f;
    float rotationRange_ = 0.0f;
    float deadZone_ = 0.05f;
    float invRotationRange_ = 1.0f;
    float lower_ = 0.0f;
    bool runOneTick_ = false;

};