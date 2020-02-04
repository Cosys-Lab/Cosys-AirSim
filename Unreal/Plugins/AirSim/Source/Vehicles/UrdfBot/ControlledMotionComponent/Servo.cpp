#include "Servo.h"

void Servo::Attach(AUrdfLink* baseLink, AUrdfLink* actuationLink, UrdfJointSpecification *jointSpecification, UPhysicsConstraintComponent* constraintComponent)
{
    ControlledMotionComponent::Attach(baseLink, actuationLink, jointSpecification, constraintComponent);

    if (jointSpecification->Type != REVOLUTE_TYPE)
    {
        throw std::runtime_error("Trying to initialize a servo on joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "', which is not of type revolute.");
    }

    if (jointSpecification->Limit->Upper <= jointSpecification->Limit->Lower)
    {
        std::string jointName = std::string(TCHAR_TO_UTF8(*jointSpecification->Name));
        std::string lower = std::to_string(jointSpecification->Limit->Lower);
        std::string upper = std::to_string(jointSpecification->Limit->Upper);

        throw std::runtime_error("Joint '" + jointName + "' has a lower of '" + lower + "', which is >= the upper of '" + upper + "'.");
    }

    if (jointSpecification->Limit->Lower > 0)
    {
        throw std::runtime_error("Joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "' has a lower of '" + std::to_string(jointSpecification->Limit->Lower) + "', which is > 0.");
    }

    if (jointSpecification->Limit->Upper < 0)
    {
        throw std::runtime_error("Joint '" + std::string(TCHAR_TO_UTF8(*jointSpecification->Name)) + "' has an upper of '" + std::to_string(jointSpecification->Limit->Upper) + "', which is < 0.");
    }

    this->rotationRange_ = FMath::RadiansToDegrees(jointSpecification->Limit->Upper - jointSpecification->Limit->Lower);
    this->invRotationRange_ = 1.0f / this->rotationRange_;
    this->maxRotationRate_ = jointSpecification->Limit->Velocity;
    this->lower_ = FMath::RadiansToDegrees(jointSpecification->Limit->Lower);

    this->isAttached_ = true;
    this->runOneTick_ = false;
}

void Servo::SetControl(TMap<FString, float> controlSignals)
{
    if (!controlSignals.Contains(TEXT("Value")))
    {
        throw std::runtime_error("Servo '" + std::string(TCHAR_TO_UTF8(*(this->GetName()))) + "' is missing the required 'Value' parameter in SetControl().");
    }

    float value = controlSignals[TEXT("Value")];

    if (value < 0 || value > 1)
        throw std::runtime_error("Servo control signal has value '" + std::to_string(value) + "', which is outside the expected range of 0-1.");

    this->controlSignalSetPoint_ = value;
}

void Servo::ComputeForces(float delta)
{
    if (!this->isAttached_)
        return;

    FQuat baseLinkQuat = this->baseLink_->GetActorQuat();
    float setPointAngle = this->controlSignalSetPoint_ * this->rotationRange_;

    if (!this->runOneTick_)
    {
        float noopControlSignal = (0.0f - this->lower_) * this->invRotationRange_;

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
        this->actualServoPosition_ = noopControlSignal;
        this->runOneTick_ = true;

        return;
    }

    float error = (this->controlSignalSetPoint_ - this->actualServoPosition_) ;
    float maxRotationAmount = this->maxRotationRate_ * delta;

    error = std::max(std::min(error, maxRotationAmount), -1.0f * maxRotationAmount);
    this->actualServoPosition_ += error;

    float rotateDegrees = (this->actualServoPosition_ * this->rotationRange_) - (0.5 * this->rotationRange_);
    FRotator setRotator = FRotator(0, 0, rotateDegrees);
    this->actuationLink_->GetRootMesh()->WakeAllRigidBodies();
    this->baseLink_->GetRootMesh()->WakeAllRigidBodies();
    this->constraintComponent_->SetAngularOrientationTarget(setRotator);
}

TMap<FString, FString> Servo::GetState()
{
    TMap<FString, FString> state;
    state.Add(TEXT("SetPoint"), FString::SanitizeFloat(this->controlSignalSetPoint_));

    FQuat baseLinkQuat = this->baseLink_->GetActorQuat();
    FQuat actuationQuat = this->actuationLink_->GetActorQuat();
    state.Add(TEXT("AcutalPoint"), FString::SanitizeFloat(this->actualServoPosition_));
    return state;
}
