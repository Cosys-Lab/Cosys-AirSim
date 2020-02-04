#include "LinearActuator.h"

void LinearActuator::Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent)
{
    ControlledMotionComponent::Attach(baseLink, actuationLink, jointSpecification, constraintComponent);

    if (jointSpecification->Type != PRISMATIC_TYPE)
    {
        throw std::runtime_error("Attempting to attach linear actuator to joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "', which is not of type prismatic.");
    }

    if (jointSpecification->Limit->Upper <= jointSpecification->Limit->Lower)
    {
        std::string jointName = std::string(TCHAR_TO_UTF8(*jointSpecification->Name));
        std::string lower = std::to_string(jointSpecification->Limit->Lower);
        std::string upper = std::to_string(jointSpecification->Limit->Upper);

        throw std::runtime_error("Joint '" + jointName + "' has a lower of '" + lower + "', which is >= the upper of '" + upper + "'.");
    }

    this->range_ = (jointSpecification->Limit->Upper - jointSpecification->Limit->Lower) * this->worldScale;
    this->invRange_ = 1.0f / this->range_;
    this->maxActuationRate_ = jointSpecification->Limit->Velocity;

    // Compute min and max transform points
    FVector parentLocation = this->baseLink_->GetActorLocation();
    FRotator parentRotation = this->baseLink_->GetActorRotation();

    FVector jointCenterLocation = constraintComponent->GetComponentLocation();

    FVector actuationAxis = parentRotation.RotateVector(jointSpecification->Axis);

    if (!actuationAxis.Normalize())
    {
        throw std::runtime_error("Cannot normalize joint axis in Linear Actuator.");
    }

    FVector actuatorOne = jointCenterLocation + (0.5 * actuationAxis * this->range_);
    FVector actuatorZero = jointCenterLocation - (0.5 * actuationAxis * this->range_);

    this->baseToActuatorZero_ = actuatorZero - parentLocation;
    this->baseToActuatorOne_ = actuatorOne - parentLocation;
    this->lower_ = jointSpecification->Limit->Lower;

    this->isAttached_ = true;
    this->runOneTick_ = false;
}

void LinearActuator::SetControl(TMap<FString, float> controlSignals)
{
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error("LinearActuator '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "' is missing the required 'Value' parameter in SetControl().");
    }

    float value = controlSignals[TEXT("Value")];

    if (value < 0 || value > 1)
        throw std::runtime_error("LinearActuator control signal has value '" + std::to_string(value) + "', which is outside the expected range of 0-1.");

    this->controlSignalSetPoint_ = value;
}

void LinearActuator::ComputeForces(float delta)
{
    if (!this->isAttached_)
        return;

    // Get the min and max points in world coordinates
    FVector basePosition = this->baseLink_->GetActorLocation();
    FVector actuatorPosition = this->actuationLink_->GetActorLocation();
    FVector actuatorZero = basePosition + this->baseToActuatorZero_;
    FVector actuatorOne = basePosition + this->baseToActuatorOne_;
    FVector actuatorOneHalf = (actuatorZero + actuatorOne) * 0.5f;

    FVector zeroToOne = (actuatorOne - actuatorZero);
    float setPointDistance = this->controlSignalSetPoint_ * this->range_;

    // For the first tick, initialize the clock and initial position
    if (!this->runOneTick_)
    {
        float actualDistance = FVector::Distance(actuatorPosition, actuatorZero);
        float noopControlSignal = actualDistance * this->invRange_;

        if (noopControlSignal < 0.0f)
        {
            throw std::runtime_error("The initial state of the actuator could not be resolved. The actuator is compressed shorter than the minimal allowable distance.");
        }
        else if (noopControlSignal > 1.0f)
        {
            throw std::runtime_error("The initial state of the actuator could not be resolved. The actuator is extended longer than the maximal allowable distance.");
        }

        // Start with no-op. User will set control signal in future frames.
        this->controlSignalSetPoint_ = noopControlSignal;
        this->actualActuatorPosition_ = noopControlSignal;
        this->runOneTick_ = true;
        return;
    }

    float error = this->controlSignalSetPoint_ - this->actualActuatorPosition_;
    float maxActuationAmount = this->maxActuationRate_ * delta;

    error = std::max(std::min(error, maxActuationAmount), -1.0f * maxActuationAmount);
    this->actualActuatorPosition_ += error;

    FVector setVec = zeroToOne * (this->actualActuatorPosition_ - 0.5f);
    FVector ss = this->jointSpecification_->Axis.Rotation().RotateVector(setVec);
    //this->actuationLink_->GetRootComponent()->WakeAllRigidBodies();
    //this->baseLink_->GetRootComponent()->WakeAllRigidBodies();
    this->constraintComponent_->SetLinearPositionTarget(ss);
}

TMap<FString, FString> LinearActuator::GetState()
{
    TMap<FString, FString> state;
    state.Add(TEXT("SetPoint"), FString::SanitizeFloat(this->controlSignalSetPoint_));

    // Actually query the position rather than use cached values, as cached values may be incorrect.
    FVector basePosition = this->baseLink_->GetActorLocation();
    FVector actuatorPosition = this->actuationLink_->GetActorLocation();
    FVector actuatorZero = basePosition + this->baseToActuatorZero_;
    float actualDistance = FVector::Distance(actuatorPosition, actuatorZero);
    float actuatorPoint = (actualDistance - this->lower_) * this->invRange_;

    state.Add(TEXT("ActualPoint"), FString::SanitizeFloat(actuatorPoint)); // convert back to 0-1 range
    return state;
}
