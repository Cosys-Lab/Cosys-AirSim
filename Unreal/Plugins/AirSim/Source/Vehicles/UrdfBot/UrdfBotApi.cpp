#include "UrdfBotApi.h"

UrdfBotApi::UrdfBotApi(AUrdfBotPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint, 
    std::function<const msr::airlib::Kinematics::State*(std::string)> state_provider_fxn, const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting,
    std::shared_ptr<msr::airlib::SensorFactory> sensor_factory, const msr::airlib::Environment& environment)
    : UrdfBotApiBase(vehicle_setting, sensor_factory, state_provider_fxn, environment),
        pawn_(pawn), pawn_kinematics_(pawn_kinematics), home_geopoint_(home_geopoint)
{
}

UrdfBotApi::~UrdfBotApi()
{
}

void UrdfBotApi::addLinearForce(const AddLinearForce& addedForce, const std::string& vehicle_name)
{
    // TODO: For now, only use one vehicle_name.
    TMap<FString, AUrdfLink*> components = this->pawn_->GetLinkComponents();
    FString linkName = FString(addedForce.link_name.c_str());
    if (components.Contains(linkName))
    {
        auto forceSpecification = new UrdfLinearForceSpecification(addedForce);

        if (!forceSpecification)
            throw std::runtime_error("OOM when attempting to allocate linear force specification.");

        components[linkName]->AddForceSpecification(forceSpecification);
        this->force_link_mapping_.Add(FString(addedForce.force_name.c_str()), components[linkName]);
    }
}

void UrdfBotApi::addAngularForce(const AddAngularForce& addedForce, const std::string& vehicle_name)
{
    // TODO: For now, only use one vehicle_name.
    TMap<FString, AUrdfLink*> components = this->pawn_->GetLinkComponents();
    FString linkName = FString(addedForce.link_name.c_str());
    if (components.Contains(linkName))
    {
        auto forceSpecification = new UrdfAngularForceSpecification(addedForce);

        if (!forceSpecification)
            throw std::runtime_error("OOM when attempting to allocate angular force specification.");

        components[linkName]->AddForceSpecification(forceSpecification);
        this->force_link_mapping_.Add(FString(addedForce.force_name.c_str()), components[linkName]);
    }
}

void UrdfBotApi::updateForceMagnitude(const UpdateForceMagnitude& updatedForce, const std::string& vehicle_name)
{
    FString forceName = FString(updatedForce.force_name.c_str());

    if (this->force_link_mapping_.Contains(forceName))
    {
        AUrdfLink* link = this->force_link_mapping_[forceName];
        link->SetForceMagnitude(forceName, updatedForce.magnitude);
    }
}

void UrdfBotApi::updateControlledMotionComponentControlSignal(const UrdfBotControlledMotionComponentControlSignal& updated_control_signal, const std::string& vehicle_name)
{
    TMap<FString, ControlledMotionComponent*> components = this->pawn_->GetControlledMotionComponents();
    FString componentName = FString(updated_control_signal.component_name.c_str());
    if (components.Contains(componentName))
    {
        ControlledMotionComponent* component = components[componentName];
        TMap<FString, float> controlSignalValues;
        for (auto const &kvp : updated_control_signal.control_signal_values)
        {
            controlSignalValues.Add(FString(kvp.first.c_str()), kvp.second);
        }
        component->SetControl(controlSignalValues);
    }
}

const msr::airlib::UrdfBotApiBase::UrdfBotState UrdfBotApi::getBotState(const std::string& vehicle_name)
{
    msr::airlib::UrdfBotApiBase::UrdfBotState botState;
    TMap<FString, AUrdfLink*> linkComponents = this->pawn_->GetLinkComponents();
    TMap<FString, ControlledMotionComponent*> motionComponents = this->pawn_->GetControlledMotionComponents();
    AUrdfLink* rootLinkComponent = this->pawn_->GetRootLinkComponent();

    FVector rootLinkLocation = rootLinkComponent->GetActorLocation();
    FRotator rootLinkRotation = rootLinkComponent->GetActorRotation();
    FQuat rootLinkRotationQuat = rootLinkRotation.Quaternion();

    FVector rootLinkVelocity = rootLinkComponent->GetPhysicsLinearVelocity();
    FVector rootLinkAngularVelocity = rootLinkComponent->GetPhysicsAngularVelocityInRadians();

    // pawn_kinematics_ is updated in PawnSim.cpp
    //botState.kinematics_estimated = *this->pawn_kinematics_;
    botState.kinematics_estimated = rootLinkComponent->GetKinematics();

    for (auto &kvp : linkComponents)
    {
        // TODO: add, acceleration
        LinkInformation currentLinkInformation;
        AUrdfLink* currentLink = kvp.Value;
        msr::airlib::Pose relativePose;
        msr::airlib::Twist relativeTwist;

        FVector relativeLocation = currentLink->GetActorLocation() - rootLinkLocation;
        FQuat relativeRotation = (currentLink->GetActorRotation() - rootLinkRotation).Quaternion();

        FVector relativeVelocity = currentLink->GetPhysicsLinearVelocity() - rootLinkVelocity;
        FVector relativeAngularVelocity = currentLink->GetPhysicsAngularVelocityInRadians() - rootLinkAngularVelocity;

        relativePose.position = msr::airlib::Vector3r(relativeLocation[0], relativeLocation[1], relativeLocation[2]);
        relativePose.orientation = msr::airlib::Quaternionr(relativeRotation.W, relativeRotation.X, relativeRotation.Y, relativeRotation.Z);
        
        relativeTwist.linear = msr::airlib::Vector3r(relativeVelocity[0], relativeVelocity[1], relativeVelocity[2]);
        relativeTwist.angular = msr::airlib::Vector3r(relativeAngularVelocity[0], relativeAngularVelocity[1], relativeAngularVelocity[2]);

        currentLinkInformation.link_name = TCHAR_TO_UTF8(*(kvp.Key));
        currentLinkInformation.link_relative_pose = relativePose;
        currentLinkInformation.link_relative_twist = relativeTwist;

        botState.link_information.emplace_back(currentLinkInformation);
    }

    for (auto const &kvp : motionComponents)
    {
        TMap<FString, FString> state = kvp.Value->GetState();

        std::map<std::string, std::string> state_converted;
        for (const auto &state_kvp : state)
        {
            state_converted[std::string(TCHAR_TO_UTF8(*(state_kvp.Key)))] = std::string(TCHAR_TO_UTF8(*(state_kvp.Value)));
        }
        
        std::string component_name = std::string(TCHAR_TO_UTF8(*(kvp.Key)));

        botState.controlled_motion_component_states[component_name] = state_converted;
    }
    
    return botState;
}

void UrdfBotApi::reset()
{
    msr::airlib::UrdfBotApiBase::reset();

    //TODO: Reset all forces on object to 0
    for (auto &linkKvp : this->pawn_->GetLinkComponents())
    {
        linkKvp.Value->ResetForceMagnitudes();
    }
}

void UrdfBotApi::update()
{
    msr::airlib::UrdfBotApiBase::update();
}

msr::airlib::GeoPoint UrdfBotApi::getHomeGeoPoint() const
{
    return this->home_geopoint_;
}

void UrdfBotApi::enableApiControl(bool is_enabled)
{
    this->api_control_enabled_ = is_enabled;
}

bool UrdfBotApi::isApiControlEnabled() const
{
    return this->api_control_enabled_;
}

// TODO: Figure out what this does.
bool UrdfBotApi::armDisarm(bool arm)
{
    unused(arm);
    return true;
}