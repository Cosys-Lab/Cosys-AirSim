#pragma once

#include "CoreMinimal.h"

#include "Runtime/Engine/Classes/PhysicsEngine/PhysicsConstraintComponent.h"

#include "AirBlueprintLib.h"
#include "Vehicles/UrdfBot/UrdfLink.h"
#include "Vehicles/UrdfBot/UrdfParser/UrdfJointSpecification.h"

class ControlledMotionComponent
{
    public:
        virtual ~ControlledMotionComponent() {};

        virtual FString GetName()
        {
            return this->name_;
        }

        // Inserts a UrdfForceSpecification into the two objects
        virtual void Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent)
        {
            this->baseLink_ = baseLink;
            this->actuationLink_ = actuationLink;
            this->worldScale = UAirBlueprintLib::GetWorldToMetersScale(baseLink);
            this->jointSpecification_ = jointSpecification;
            this->constraintComponent_ = constraintComponent;
            this->name_ = jointSpecification->Name;
        }

        virtual void SetControl(TMap<FString, float> controlSignals) = 0;
        virtual TMap<FString, FString> GetState() = 0;

        // Should be called on each tick
        virtual void ComputeForces(float delta) = 0;


    protected:
        bool GetFloatFromConfiguration(FString configName, TMap<FString, FString> configuration, float& out, bool throwIfNotExist = false, bool throwIfNonNumeric = false);

        FString name_;
        float worldScale = 100.0f;

        bool isAttached_ = false;

        AUrdfLink* baseLink_;
        AUrdfLink* actuationLink_;
        UrdfJointSpecification *jointSpecification_;
        UPhysicsConstraintComponent* constraintComponent_;
};