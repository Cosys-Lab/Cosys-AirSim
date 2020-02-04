#include "Motor.h"

void Motor::Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent)
{
    ControlledMotionComponent::Attach(baseLink, actuationLink, jointSpecification, constraintComponent);

    if (jointSpecification->Type != CONTINUOUS_TYPE)
    {
        throw std::runtime_error("Attempting to initialize a motor on joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "', which is not of type continuous.");
    }

    this->rotationAxis_ = jointSpecification->Axis;
    this->maxVelocity_ = jointSpecification->Limit->Velocity;
    this->controlSignalSetPoint_ = 0.0f;
}

void Motor::SetControl(TMap<FString, float> controlSignals)
{
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error("Motor '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "' is missing the required 'Value' parameter in SetControl().");
    }

    float value = controlSignals[TEXT("Value")];

    if (value < -1 || value > 1)
        throw std::runtime_error("Motor control signal has value '" + std::to_string(value) + "', which is outside the expected range of -1 to 1.");

    this->controlSignalSetPoint_ = value;
}
void Motor::ComputeForces(float delta)
{
    FVector targetAngularVelocity = FVector(this->maxVelocity_, 0, 0) * this->controlSignalSetPoint_;
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetAngularVelocityTarget(targetAngularVelocity);
}

TMap<FString, FString> Motor::GetState()
{
    TMap<FString, FString> state;
    state.Add(TEXT("SetPoint"), FString::SanitizeFloat(this->controlSignalSetPoint_));

    FVector targetAngularVelocity = this->actuationLink_->GetPhysicsAngularVelocityInRadians();
    FVector baseAngularVelocity = this->actuationLink_->GetPhysicsAngularVelocityInRadians();
    FVector worldDiffAngularVelocity = targetAngularVelocity - baseAngularVelocity;

    FRotator componentRotation = this->constraintComponent_->GetComponentRotation();
    FVector localDiffAngularVelocity = componentRotation.RotateVector(worldDiffAngularVelocity);

    float actualSetPoint = localDiffAngularVelocity[0] / this->maxVelocity_;
    state.Add(TEXT("ActualPoint"), FString::SanitizeFloat(actualSetPoint));

    return state;
}