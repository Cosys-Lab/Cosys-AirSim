#include "UrdfBotPawn.h"

AUrdfBotPawn::AUrdfBotPawn()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> baseBoxMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> baseCylinderMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> baseSphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));

    if (baseBoxMesh.Succeeded())
        this->baseBoxMesh_ = baseBoxMesh.Object;

    if (baseCylinderMesh.Succeeded())
        this->baseCylinderMesh_ = baseCylinderMesh.Object;

    if (baseSphereMesh.Succeeded())
        this->baseSphereMesh_ = baseSphereMesh.Object;


    // Create a default root component. Needed for spawning to work properly.
    auto DefaultCapsule = CreateDefaultSubobject<UCapsuleComponent>(FName("Default"));
    DefaultCapsule->InitCapsuleSize(0.01f, 0.01f);
    DefaultCapsule->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    DefaultCapsule->SetCollisionResponseToAllChannels(ECR_Ignore);
    DefaultCapsule->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
    DefaultCapsule->SetVisibility(false);
    RootComponent = DefaultCapsule;

    // Parse the URDF file to get a list of materials and static meshes.
    // These can only be created in the constructor as far as I can tell. 
    // Cache them for use later.
    //
    // Also, the unreal editor calls the constructor. At this point, the settings file is not loaded, which happens during SimMode initialization.
    // This causes the UrdfPath to be null, throwing an exception. For now, catch that exception.
    // If the path is improper, BeginPlay() will throw.
    try
    {
        AirSimSettings& settings = AirSimSettings::AirSimSettings::singleton();
        if (settings.pawn_paths.find("UrdfBot") == settings.pawn_paths.end())
            throw std::runtime_error("Cannot find pawn path. Make sure settings.json has a member inside 'PawnPaths' named 'Urdfbot'.");

        FString urdfPath = FString(settings.pawn_paths["UrdfBot"].urdf_path.c_str());
        if (urdfPath.Len() == 0)
            throw std::runtime_error("No pawn path specified. Make sure that the 'PawnPaths' member named 'UrdfBot' has member 'urdf_path' specified.");

        this->components_.Empty();

        UrdfParser parser;
        parser.Parse(urdfPath);

        this->materials_.Empty();

        for (auto kvp : parser.GetMaterials())
        {
            FString materialName = kvp.Value->Name;
            FString materialPath = kvp.Value->TextureFile;
            if (materialPath.Len() > 0)
            {
                ConstructorHelpers::FObjectFinder<UMaterialInterface> material(*materialPath);

                if (material.Object != NULL)
                {
                    this->materials_.Add(materialName, static_cast<UMaterialInterface*>(material.Object));
                }
            }
        }

        this->user_static_meshes_.Empty();
        for (auto kvp : parser.GetLinks())
        {
            UrdfMesh* linkGeometry = nullptr;
            UrdfLinkVisualSpecification* visualSpecification = kvp.Value->VisualSpecification;
            UrdfLinkCollisionSpecification* collisionSpecification = kvp.Value->CollisionSpecification;

            if (visualSpecification != nullptr && visualSpecification->Geometry->GetGeometryType() == MESH)
            {
                linkGeometry = static_cast<UrdfMesh*>(visualSpecification->Geometry);
            }
            else if (collisionSpecification != nullptr && collisionSpecification->Geometry->GetGeometryType() == MESH)
            {
                linkGeometry = static_cast<UrdfMesh*>(visualSpecification->Geometry);
            }

            if (linkGeometry == nullptr || linkGeometry->FileType != UNREAL_MESH)
            {
                continue;
            }

            ConstructorHelpers::FObjectFinder<UStaticMesh> mesh(*linkGeometry->FileLocation);

            if (mesh.Object != NULL)
            {
                this->user_static_meshes_.Add(linkGeometry->FileLocation, static_cast<UStaticMesh*>(mesh.Object));
            }
        }

        this->staticMeshGenerator_.Initialize(this->baseBoxMesh_, this->baseCylinderMesh_, this->baseSphereMesh_, this->user_static_meshes_);
    }
    catch (std::exception e)
    {
        int j = 0;
    }
}

AUrdfBotPawn::~AUrdfBotPawn()
{
}

void AUrdfBotPawn::BeginPlay()
{
    Super::BeginPlay();
}

void AUrdfBotPawn::Tick(float delta)
{
    Super::Tick(delta);

    for (const auto& kvp : this->controlled_motion_components_)
    {
        kvp.Value->ComputeForces(delta);
    }

    for (const auto& kvp : this->components_)
    {
        kvp.Value->Tick(delta);
    }

    this->pawn_events_.getPawnTickSignal().emit(delta);

    if (this->draw_debug_)
    {
        this->DrawDebug();
    }
}

void AUrdfBotPawn::EndPlay(const EEndPlayReason::Type endPlayReason)
{
}

void AUrdfBotPawn::NotifyHit(class UPrimitiveComponent* myComp, class AActor* other, class UPrimitiveComponent* otherComp, bool bSelfMoved, FVector hitLocation,
    FVector hitNormal, FVector normalImpulse, const FHitResult &hit)
{
    //myComp is the hit link
    //other is the actor that is being hit

    if (!IsValid(myComp))
    {
        return;
    }

    if (!IsValid(other))
    {
        return;
    }

    if (!IsValid(otherComp))
    {
        return;
    }

    FString meshCompName = myComp->GetName();
    FString otherCompName = other->GetName();
    if (this->collision_blacklist_.Contains(meshCompName))
    {
        TArray<FRegexPattern> patterns = this->collision_blacklist_[meshCompName];
        for (int i = 0; i < patterns.Num(); i++)
        {
            FRegexMatcher matcher(patterns[i], otherCompName);
            if (matcher.FindNext())
            {
                return;
            }
        }
    }

    this->pawn_events_.getCollisionSignal().emit(myComp, other, otherComp, bSelfMoved, hitLocation,
        hitNormal, normalImpulse, hit);
}

void AUrdfBotPawn::InitializeForBeginPlay()
{
    AirSimSettings& settings = AirSimSettings::AirSimSettings::singleton();
    if (settings.pawn_paths.find("UrdfBot") == settings.pawn_paths.end())
        throw std::runtime_error("Cannot find pawn path. Make sure settings.json has a member inside 'PawnPaths' named 'Urdfbot'.");

    FString urdfPath = FString(settings.pawn_paths["UrdfBot"].urdf_path.c_str());
    if (urdfPath.Len() == 0)
        throw std::runtime_error("No pawn path specified. Make sure that the 'PawnPaths' member named 'UrdfBot' has member 'urdf_path' specified.");

    // TODO: change when multiple vehicles are supported.
    this->debug_symbol_scale_ = settings.vehicles["UrdfBot"].get()->debug_symbol_scale;
    this->draw_debug_ = !msr::airlib::Utils::isApproximatelyZero(this->debug_symbol_scale_);

    if (this->draw_debug_ && this->debug_symbol_scale_ < 0)
    {
        throw std::runtime_error("Debug symbol scale is < 0. Must be a positive number.");
    }

    this->ConstructFromFile(urdfPath);

    this->collision_blacklist_.Empty();
    for (auto &kvp : settings.vehicles["UrdfBot"].get()->collision_blacklist)
    {
        FString bot_mesh = FString(kvp.first.c_str()) + TEXT("_visual");
        FString regex = FString(kvp.second.c_str());

        if (!this->collision_blacklist_.Contains(bot_mesh))
        {
            this->collision_blacklist_.Add(bot_mesh, TArray<FRegexPattern>());
        }

        this->collision_blacklist_[bot_mesh].Add(FRegexPattern(regex));
    }
}

common_utils::UniqueValueMap<std::string, APIPCamera*> AUrdfBotPawn::GetCameras() const
{
    return this->cameras_;
}

PawnEvents* AUrdfBotPawn::GetPawnEvents()
{
    return &(this->pawn_events_);
}

TMap<FString, AUrdfLink*> AUrdfBotPawn::GetLinkComponents() const
{
    return this->components_;
}

AUrdfLink* AUrdfBotPawn::GetRootLinkComponent() const
{
    return this->root_component_;
}

TMap<FString, ControlledMotionComponent*> AUrdfBotPawn::GetControlledMotionComponents() const
{
    return this->controlled_motion_components_;
}

void AUrdfBotPawn::TeleportToLocation(FVector position, FQuat orientation, bool teleport)
{
    FVector translation = (position * UAirBlueprintLib::GetWorldToMetersScale(this)) - this->GetActorLocation();
    FRotator rotation = (orientation * this->GetActorQuat().Inverse()).Rotator();

    this->MoveAllComponents(translation, rotation);

    AirsimVehicle::TeleportToLocation(position * UAirBlueprintLib::GetWorldToMetersScale(this), orientation, teleport);
}

USceneComponent* AUrdfBotPawn::GetComponent(FString componentName)
{
    // For debug
    if (!this->components_.Contains(componentName))
    {
        throw std::runtime_error("Requested component named " + std::string(TCHAR_TO_UTF8(*componentName)) + " in GetComponent(), which does not exist.");
    }

    return this->components_[componentName]->GetRootComponent();
}

AUrdfLink* AUrdfBotPawn::GetLink(FString linkName)
{
    // For debug
    if (!this->components_.Contains(linkName))
    {
        throw std::runtime_error("Requested component named " + std::string(TCHAR_TO_UTF8(*linkName)) + " in GetComponent(), which does not exist.");
    }

    return this->components_[linkName];
}

void AUrdfBotPawn::GetComponentReferenceTransform(FString componentName, FVector& translation, FRotator& rotation)
{
    // For debug
    if (!this->components_.Contains(componentName))
    {
        throw std::runtime_error("Requested component named " + std::string(TCHAR_TO_UTF8(*componentName)) + " in GetComponent(), which does not exist.");
    }

    this->components_[componentName]->GetReferenceFrameLocation(translation, rotation);
}

void AUrdfBotPawn::ConstructFromFile(FString fileName)
{
    this->components_.Empty();
    this->component_visited_as_constraint_parent_.Empty();
    this->constraints_.Empty();

    this->world_scale_ = UAirBlueprintLib::GetWorldToMetersScale(this);

    UrdfParser parser;
    parser.Parse(fileName);
    TMap<FString, UrdfLinkSpecification*> links = parser.GetLinks();
    TMap<FString, UrdfJointSpecification*> joints = parser.GetJoints();
    TMap<FString, UrdfForceSpecification*> forces = parser.GetForces();

    for (auto kvp : links)
    {
        AUrdfLink* createdLink = this->CreateLinkFromSpecification(*kvp.Value);
        this->components_.Add(kvp.Key, createdLink);
        this->component_visited_as_constraint_parent_.Add(kvp.Key, false);
    }

    auto rootLocation = this->GetActorLocation();
    auto rootRotation = this->GetActorRotation();

    UrdfLinkSpecification* rootLinkSpecification = this->FindRootNodeSpecification(links);
    this->root_component_ = this->components_[rootLinkSpecification->Name];
    this->root_component_->SetReferenceFrameLocation(this->GetActorLocation(), this->GetActorRotation());
    this->root_component_->GetRootComponent()->AttachTo(RootComponent, NAME_None, EAttachLocation::KeepRelativeOffset); 

    this->SetRootComponent(this->root_component_->GetRootComponent());

    UPrimitiveComponent* rootCollisionComponent = this->root_component_->GetCollisionComponent();
    if (rootCollisionComponent != nullptr)
    {
        rootCollisionComponent->SetWorldLocationAndRotation(this->GetActorLocation(), this->GetActorRotation());
    }

    this->component_visited_as_constraint_parent_[rootLinkSpecification->Name] = true;

    for (auto kvp : rootLinkSpecification->Children)
    {
        UrdfLinkSpecification* childLinkSpecification = kvp.Key;
        UrdfJointSpecification* jointSpecification = kvp.Value;
        AUrdfLink* childLink = this->components_[childLinkSpecification->Name];

        this->AttachChildren(this->root_component_, *rootLinkSpecification, childLink, *childLinkSpecification, jointSpecification);
    }

    for (auto kvp : forces)
    {
        UrdfForceSpecification* forceSpecification = kvp.Value;
        AUrdfLink* link = this->components_[kvp.Value->LinkName];

        link->AddForceSpecification(forceSpecification);
    }
}

AUrdfLink* AUrdfBotPawn::CreateLinkFromSpecification(const UrdfLinkSpecification &linkSpecification)
{
    UrdfGeometry* visualGeometry = nullptr;
    UrdfGeometry* collisionGeometry = nullptr;

    if (linkSpecification.VisualSpecification != nullptr)
    {
        visualGeometry = linkSpecification.VisualSpecification->Geometry;
        collisionGeometry = linkSpecification.VisualSpecification->Geometry;
    }

    if (linkSpecification.CollisionSpecification != nullptr)
    {
        collisionGeometry = linkSpecification.CollisionSpecification->Geometry;
        if (visualGeometry == nullptr)
        {
            visualGeometry = linkSpecification.CollisionSpecification->Geometry;
        }
    }

    if (visualGeometry == nullptr || collisionGeometry == nullptr)
        throw std::runtime_error("Unable to create link " + std::string(TCHAR_TO_UTF8(*linkSpecification.Name)) + ". No visual or geometry node specified.");

    AUrdfLink* link = NewObject<AUrdfLink>(this, AUrdfLink::StaticClass(), FName(linkSpecification.Name.GetCharArray().GetData()));

    this->staticMeshGenerator_.CreateUnscaledMeshForLink(linkSpecification.Name, linkSpecification.VisualSpecification->MaterialName, visualGeometry, collisionGeometry, this, link, this->materials_);

    this->ResizeLink(link, collisionGeometry);

    link->SetMass(linkSpecification.InertialSpecification->Mass);
    link->SetOwningActor(this);

    return link;
}

void AUrdfBotPawn::AttachChildren(AUrdfLink* parentLink, const UrdfLinkSpecification &parentLinkSpecification, AUrdfLink *childLink, const UrdfLinkSpecification &childLinkSpecification, UrdfJointSpecification *jointSpecification)
{
    this->component_visited_as_constraint_parent_[parentLink->GetName()] = true;

    // Weld joints
    UPrimitiveComponent* childCollisionComponent = childLink->GetCollisionComponent();

    FVector parentReferenceTranslation(0, 0, 0);
    FRotator parentReferenceRotation(0, 0, 0);
    parentLink->GetReferenceFrameLocation(parentReferenceTranslation, parentReferenceRotation);

    FVector referenceTranslation = parentReferenceTranslation + parentReferenceRotation.RotateVector(jointSpecification->Origin.Origin) * this->world_scale_;
    FRotator referenceRotation = parentReferenceRotation + FMath::RadiansToDegrees(jointSpecification->Origin.RollPitchYaw);
    childLink->SetReferenceFrameLocation(referenceTranslation, referenceRotation);

    if (childCollisionComponent != nullptr)
    {
        if (childLinkSpecification.CollisionSpecification != nullptr)
        {
            childCollisionComponent->SetWorldLocationAndRotation(referenceTranslation + (childLinkSpecification.CollisionSpecification->Origin.Origin * this->world_scale_), referenceRotation + (FMath::RadiansToDegrees(childLinkSpecification.CollisionSpecification->Origin.RollPitchYaw)));
        }
        else
        {
            childCollisionComponent->SetWorldLocationAndRotation(referenceTranslation, referenceRotation);
        }
    }

    FRotator visualOffsetRotator = FMath::RadiansToDegrees(childLinkSpecification.VisualSpecification->Origin.RollPitchYaw);
    FVector visualOffsetVector = visualOffsetRotator.RotateVector(childLinkSpecification.VisualSpecification->Origin.Origin) * this->world_scale_;
    childLink->SetActorLocationAndRotation(referenceTranslation + visualOffsetVector, referenceRotation + visualOffsetRotator);
    childLink->RecordVisualOffset(visualOffsetVector, visualOffsetRotator);

    // Create constraint component
    UPhysicsConstraintComponent* constraint = NewObject<UPhysicsConstraintComponent>((USceneComponent*)parentLink, FName(*(parentLinkSpecification.Name + FString(TEXT("_")) + childLinkSpecification.Name)));
    FConstraintInstance constraintInstance = this->CreateConstraint(*jointSpecification);

    constraint->ConstraintInstance = constraintInstance;

    FRotator rotation = FRotator::ZeroRotator;
    if (jointSpecification->Type != FIXED_TYPE)
    {
        // Rotate such that the local X axis aligns with the specified axis
        FVector unitX(1, 0, 0);
        FVector unitY(0, 1, 0);
        FVector unitZ(0, 0, 1);

        FRotator jointAxisRotator(FMath::RadiansToDegrees(jointSpecification->RollPitchYaw.Pitch), FMath::RadiansToDegrees(jointSpecification->RollPitchYaw.Yaw), FMath::RadiansToDegrees(jointSpecification->RollPitchYaw.Roll));
        FVector jointAxisInParentFrame = parentReferenceRotation.RotateVector(jointSpecification->Axis);
        FVector jointAxis = jointAxisRotator.RotateVector(jointAxisInParentFrame);
        FVector planeToParentRotAxis = unitX ^ jointAxis;
        if (planeToParentRotAxis.Normalize())
        {
            float rotRad = acosf(FVector::DotProduct(unitX, jointAxisInParentFrame));
            FQuat rotQuat(planeToParentRotAxis, rotRad);
            FRotator rotRot = rotQuat.Rotator();
            rotation += rotRot;
        }

        // Specal case - jointSpecification->Axis = - X. Then, roll 180 degrees
        if (jointSpecification->Axis.X == -1 && jointSpecification->Axis.Y == 0 && jointSpecification->Axis.Z == 0)
        {
            float rotRad = FMath::DegreesToRadians(-180);
            FQuat rotQuat(unitX, rotRad);
            FRotator rotRot = rotQuat.Rotator();
            rotation += rotRot;
        }

        // Center the joint for revolute joints
        if (jointSpecification->Type == REVOLUTE_TYPE)
        {
            float medianPosition = FMath::RadiansToDegrees((jointSpecification->Limit->Upper + jointSpecification->Limit->Lower) * 0.5f);
            constraint->ConstraintInstance.AngularRotationOffset = FRotator(0, 0, medianPosition);
        }

        constraint->SetWorldRotation(rotation, false, nullptr, ETeleportType::TeleportPhysics);
    }

    // If required, move child component to set up the physics constraint
    FVector inverseTransform = this->MoveChildLinkForLimitedXAxisMotion(parentLink, childLink, *jointSpecification);

    constraint->SetDisableCollision(true);

    constraint->SetWorldLocation(childLink->GetActorLocation());

    constraint->AttachToComponent(parentLink->GetRootComponent(), FAttachmentTransformRules(EAttachmentRule::KeepWorld, (jointSpecification->Type == FIXED_TYPE)));

    constraint->ConstraintActor1 = this;
    constraint->ConstraintActor2 = this;

    constraint->SetConstrainedComponents(parentLink->GetRootMesh(), NAME_None, childLink->GetRootMesh(), NAME_None);

    this->constraints_.Add(jointSpecification->Name, TTuple<UrdfJointType, UPhysicsConstraintComponent*>(jointSpecification->Type, constraint));

    if (this->ConstraintNeedsControlledMotionComponent(*jointSpecification))
    {
        ControlledMotionComponent* component = ControlledMotionComponentFactory::CreateControlledMotionComponent(parentLink, childLink, jointSpecification, constraint);
        this->controlled_motion_components_.Add(component->GetName(), component);
    }

    // Reset Child if needed
    FVector cl = childLink->GetActorLocation();
    childLink->SetActorLocation(cl + inverseTransform, false, nullptr, ETeleportType::TeleportPhysics);
    FVector cl2 = childLink->GetActorLocation();

    //Attach all of the child node's children
    if (!this->component_visited_as_constraint_parent_[childLinkSpecification.Name])
    {
        for (auto kvp : childLinkSpecification.Children)
        {
            UrdfLinkSpecification* nextChildLinkSpecification = kvp.Key;
            UrdfJointSpecification* nextJointSpecification = kvp.Value;
            AUrdfLink* nextChildLink = this->components_[nextChildLinkSpecification->Name];
            this->AttachChildren(childLink, childLinkSpecification, nextChildLink, *nextChildLinkSpecification, nextJointSpecification);
        }
    }
}

FConstraintInstance AUrdfBotPawn::CreateConstraint(const UrdfJointSpecification &jointSpecification)
{
    FConstraintInstance constraintInstance = this->CreateDefaultFixedConstraintInstance();

    constraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = false;
    constraintInstance.ProfileInstance.ConeLimit.bSoftConstraint = false;
    constraintInstance.ProfileInstance.AngularDrive.AngularDriveMode = EAngularDriveMode::TwistAndSwing;


    float range = 0.0f;
    switch (jointSpecification.Type)
    {
    case FIXED_TYPE:
        break;
    case FLOATING_TYPE:
        constraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
        constraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
        constraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
        break;
    case PRISMATIC_TYPE:
        range = (jointSpecification.Limit->Upper - jointSpecification.Limit->Lower) * this->world_scale_ * 0.5f;
        constraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, range);
        if (jointSpecification.Limit != nullptr && jointSpecification.Limit->Effort > 0)
        {
            constraintInstance.SetLinearDriveParams(jointSpecification.Limit->Effort * this->world_scale_, 5.0f, jointSpecification.Limit->Effort * this->world_scale_);
            constraintInstance.SetLinearPositionDrive(true, false, false);
        }
        else
        {
            constraintInstance.SetLinearDriveParams(0, 0, 0);
            constraintInstance.SetLinearPositionDrive(false, false, false);
        }
        break;
    case REVOLUTE_TYPE:
        range = FMath::RadiansToDegrees(jointSpecification.Limit->Upper - jointSpecification.Limit->Lower) * 0.5f;
        constraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, range);
        if (jointSpecification.Limit != nullptr && jointSpecification.Limit->Effort > 0)
        {
            constraintInstance.SetAngularPositionDrive(false, true);
            constraintInstance.SetAngularDriveParams(jointSpecification.Limit->Effort * this->world_scale_ * this->world_scale_, 5.0f, jointSpecification.Limit->Effort * this->world_scale_ * this->world_scale_);
        }
        else
        {
            constraintInstance.SetAngularPositionDrive(false, false);
            constraintInstance.SetAngularDriveParams(0, 0, 0);
        }
        break;
    case CONTINUOUS_TYPE:
        constraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
        if (jointSpecification.Limit != nullptr && jointSpecification.Limit->Effort > 0)
        {
            constraintInstance.SetAngularVelocityDrive(false, true);
            constraintInstance.SetAngularDriveParams(5.0f, jointSpecification.Limit->Effort * this->world_scale_ * this->world_scale_, jointSpecification.Limit->Effort * this->world_scale_ * this->world_scale_);
        }
        else
        {
            constraintInstance.SetAngularVelocityDrive(false, false);
            constraintInstance.SetAngularDriveParams(0, 0, 0);
        }
        break;
    case PLANAR_TYPE:
        range = (jointSpecification.Limit->Upper - jointSpecification.Limit->Lower) * this->world_scale_ * 0.5f;
        constraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, range);
        constraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, range);
        constraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
        break;
    }

    return constraintInstance;
}

void AUrdfBotPawn::ResizeLink(AUrdfLink* link, UrdfGeometry* geometry)
{
    FVector resizeSpec = FVector(0, 0, 0);
    UrdfBox* boxSpecification = nullptr;
    UrdfCylinder* cylinderSpecification = nullptr;
    UrdfSphere* sphereSpecification = nullptr;
    switch (geometry->GetGeometryType())
    {
    case BOX:
        boxSpecification = static_cast<UrdfBox*>(geometry);
        resizeSpec = boxSpecification->Size;
        break;
    case CYLINDER:
        cylinderSpecification = static_cast<UrdfCylinder*>(geometry);
        resizeSpec = FVector(cylinderSpecification->Radius * 2.0f, cylinderSpecification->Radius * 2.0f, cylinderSpecification->Length);
        break;
    case SPHERE:
        sphereSpecification = static_cast<UrdfSphere*>(geometry);
        resizeSpec = FVector(sphereSpecification->Radius * 2.0f, sphereSpecification->Radius * 2.0f, sphereSpecification->Radius * 2.0f);
        break;
    case MESH:
        resizeSpec = FVector(1, 1, 1);
        break;
    default:
        throw std::runtime_error("Unable to construct shape component due to unrecognized mesh shape.");
    }

    link->GetRootComponent()->SetWorldScale3D(resizeSpec);
}

UrdfLinkSpecification* AUrdfBotPawn::FindRootNodeSpecification(TMap<FString, UrdfLinkSpecification*> links)
{
    for (auto kvp : links)
    {
        UrdfLinkSpecification* candidateLink = kvp.Value;
        if (candidateLink->ParentLink == nullptr)
            return candidateLink;
    }

    throw std::runtime_error("Cannot construct the bot. No root node.");
}

FConstraintInstance AUrdfBotPawn::CreateDefaultFixedConstraintInstance()
{
    FConstraintInstance ConstraintInstance;
    ConstraintInstance.SetDisableCollision(true);
    ConstraintInstance.SetLinearXMotion(ELinearConstraintMotion::LCM_Locked);
    ConstraintInstance.SetLinearYMotion(ELinearConstraintMotion::LCM_Locked);
    ConstraintInstance.SetLinearZMotion(ELinearConstraintMotion::LCM_Locked);
    ConstraintInstance.SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
    ConstraintInstance.SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
    ConstraintInstance.SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
    ConstraintInstance.SetAngularSwing1Motion(EAngularConstraintMotion::ACM_Locked);
    ConstraintInstance.SetAngularSwing2Motion(EAngularConstraintMotion::ACM_Locked);
    ConstraintInstance.SetAngularTwistMotion(EAngularConstraintMotion::ACM_Locked);
    ConstraintInstance.SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
    ConstraintInstance.SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
    ConstraintInstance.SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
    ConstraintInstance.AngularRotationOffset = FRotator(0, 0, 0);

    return ConstraintInstance;
}

bool AUrdfBotPawn::ConstraintNeedsControlledMotionComponent(const UrdfJointSpecification &spec)
{
    if (spec.Type == PRISMATIC_TYPE || spec.Type == REVOLUTE_TYPE || spec.Type == CONTINUOUS_TYPE)
    {
        return (spec.Limit->Effort > 0) && !(this->controlled_motion_components_.Contains(spec.Name));
    }

    return false;
}

FVector AUrdfBotPawn::MoveChildLinkForLimitedXAxisMotion(AUrdfLink* parentLink, AUrdfLink* childLink, const UrdfJointSpecification& jointSpecification)
{
    FVector inverseTransform = FVector::ZeroVector;

    // This transform is only needed for prismatic joints. 
    if (jointSpecification.Type != PRISMATIC_TYPE)
    {
        return inverseTransform;
    }

    // If prismatic joint does not have limit set, then no transform is needed
    if (jointSpecification.Limit == nullptr || jointSpecification.Limit->Lower < 0 - 1 || jointSpecification.Limit->Upper < 0)
    {
        return inverseTransform;
    }

    if (jointSpecification.Limit->Lower >= jointSpecification.Limit->Upper)
    {
        std::string jointName = std::string(TCHAR_TO_UTF8(*jointSpecification.Name));
        std::string lower = std::to_string(jointSpecification.Limit->Lower);
        std::string upper = std::to_string(jointSpecification.Limit->Upper);

        throw std::runtime_error("Joint '" + jointName + "' has a lower of '" + lower + "', which is >= the upper of '" + upper + "'.");
    }

    FVector parentLocation = parentLink->GetActorLocation();
    FVector childLocation = childLink->GetActorLocation();
    FRotator parentRotation = parentLink->GetActorRotation();
    FRotator childRotation = childLink->GetActorRotation();

    float lowerUU = jointSpecification.Limit->Lower * this->world_scale_;
    float upperUU = jointSpecification.Limit->Upper * this->world_scale_;

    FVector xAxisInParentFrame = parentRotation.UnrotateVector(FVector(1, 0, 0));
    FRotator axisRotator = jointSpecification.Axis.Rotation();
    FVector unitVectorOfAxis = axisRotator.UnrotateVector(xAxisInParentFrame);

    if (!unitVectorOfAxis.Normalize())
    {
        std::string jointName = std::string(TCHAR_TO_UTF8(*jointSpecification.Name));
        throw std::runtime_error("Cannot normalize axis for join '" + jointName + "'.");
    }

    FVector childMinLocation = parentLocation + (unitVectorOfAxis * lowerUU);
    FVector childMaxLocation = parentLocation + (unitVectorOfAxis * upperUU);
    FVector requiredPositionParent = (childMinLocation + childMaxLocation) * 0.5f;
    FVector transform = requiredPositionParent - parentLocation;
    inverseTransform = -1 * transform;

    childLink->SetActorLocation(childLocation + transform, false, nullptr, ETeleportType::TeleportPhysics);

    return inverseTransform;
}

void AUrdfBotPawn::DrawDebug()
{
    UWorld* world = this->GetWorld();

    FColor jointColor = FColor::Emerald;
    bool persistant = false;
    float persistSeconds = -1;
    uint8 priority = (uint8)'\002';
    float thickness = 1.0f;
    int32 jointCircleSegments = 32;
    float jointCircleRadius = this->debug_symbol_scale_ / 2.0f;
    float axisLineLength = this->debug_symbol_scale_;

    // Draw debug axis
    for (auto& kvp : this->components_)
    {
        AUrdfLink* component = kvp.Value;

        FVector actualLocation = component->GetActorLocation();
        FRotator actualRotation = component->GetActorRotation();

        FVector xx(axisLineLength, 0, 0);
        FVector yy(0, axisLineLength, 0);
        FVector zz(0, 0, axisLineLength);
        FVector zzz(0, 0, 0);

        FVector xxt = actualRotation.RotateVector(xx) + actualLocation;
        FVector yyt = actualRotation.RotateVector(yy) + actualLocation;
        FVector zzt = actualRotation.RotateVector(zz) + actualLocation;
        FVector zzzt = actualRotation.RotateVector(zzz) + actualLocation;

        DrawDebugLine(world, zzzt, xxt, FColor::Red, persistant, persistSeconds, priority, thickness);
        DrawDebugLine(world, zzzt, yyt, FColor::Green, persistant, persistSeconds, priority, thickness);
        DrawDebugLine(world, zzzt, zzt, FColor::Blue, persistant, persistSeconds, priority, thickness);
    }

    // Draw constraints
    for (auto& kvp : this->constraints_)
    {
        UPhysicsConstraintComponent* component = kvp.Value.Value;
        UrdfJointType jointType = kvp.Value.Key;

        if (jointType == REVOLUTE_TYPE)
        {
            FVector directionLocal(0, jointCircleRadius / 2, 0);
            FRotator componentRotator = component->GetComponentRotation();
            FVector unitX(1, 0, 0);
            FVector localX = componentRotator.RotateVector(unitX);

            float rotationAmount = component->ConstraintInstance.GetAngularTwistLimit();
            float center = component->ConstraintInstance.AngularRotationOffset.Roll;
            float minRotation = center - rotationAmount;
            float maxRotation = center + rotationAmount;

            FVector directionMin = FRotator(0, 0, -minRotation).RotateVector(directionLocal);
            FVector directionMax = FRotator(0, 0, -maxRotation).RotateVector(directionLocal);

            FVector minDirectionGlobal = componentRotator.RotateVector(directionMin);
            FVector maxDirectionGlobal = componentRotator.RotateVector(directionMax);

            DrawDebugLine(world, component->GetComponentLocation(), component->GetComponentLocation() + minDirectionGlobal, FColor::Yellow, persistant, persistSeconds, priority, thickness * 2);
            DrawDebugLine(world, component->GetComponentLocation(), component->GetComponentLocation() + maxDirectionGlobal, FColor::Magenta, persistant, persistSeconds, priority, thickness * 2);


            FVector unitXLocal = componentRotator.RotateVector(unitX);
            FVector start = component->GetComponentLocation();
            FVector end = start + (unitXLocal * jointCircleRadius / 2);

            DrawDebugLine(world, start, end, FColor::Black, persistant, persistSeconds, priority, thickness * 2);
        }
        else if (jointType == CONTINUOUS_TYPE)
        {
            FVector zAxisLocal(0, 0, 1);
            FVector yAxisLocal(0, 1, 0);

            FVector zAxisWorld = component->GetComponentRotation().RotateVector(zAxisLocal);
            FVector yAxisWorld = component->GetComponentRotation().RotateVector(yAxisLocal);
            DrawDebugCircle(world, component->GetComponentLocation(), jointCircleRadius, jointCircleSegments, jointColor, persistant, persistSeconds, priority, thickness, yAxisWorld, zAxisWorld, false);
        }
        else if (jointType == PRISMATIC_TYPE)
        {
            float extents = component->ConstraintInstance.ProfileInstance.LinearLimit.Limit;
            FVector startLocal(-extents, 0, 0);
            FVector endLocal(extents, 0, 0);
            FVector componentLocation = component->GetComponentLocation();
            FRotator componentRotation = component->GetComponentRotation();
            FVector startOffset = componentRotation.RotateVector(startLocal);
            FVector endOffset = componentRotation.RotateVector(endLocal);
            FVector startWorld = componentLocation + startOffset;
            FVector endWorld = componentLocation + endOffset;

            DrawDebugLine(world, startWorld, endWorld, jointColor, persistant, persistSeconds, priority, thickness / 2);

            DrawDebugPoint(world, startWorld, thickness * 4, FColor::Yellow, persistant, persistSeconds, priority);
            DrawDebugPoint(world, endWorld, thickness * 4, FColor::Magenta, persistant, persistSeconds, priority);
        }
        else if (jointType == FIXED_TYPE)
        {
            DrawDebugPoint(world, component->GetComponentLocation(), jointCircleRadius, jointColor, persistant, persistSeconds, priority);
        }
        else if (jointType == FLOATING_TYPE)
        {
            DrawDebugSphere(world, component->GetComponentLocation(), jointCircleRadius, jointCircleSegments, jointColor, persistant, persistSeconds, priority, thickness);
        }
        else if (jointType == PLANAR_TYPE)
        {
            float extents = component->ConstraintInstance.ProfileInstance.LinearLimit.Limit;
            FVector localBoxExtents(extents, extents, 0.001f);
            FVector boxExtents = component->GetComponentRotation().RotateVector(localBoxExtents);
            DrawDebugBox(world, component->GetComponentLocation(), boxExtents, jointColor, persistant, persistSeconds, priority, thickness);
        }
        else
        {
            throw std::runtime_error("Unrecognized joint type in DrawDebug()");
        }
    }
}

void AUrdfBotPawn::MoveAllComponents(FVector translation, FRotator rotation)
{
    for (auto kvp : this->components_)
    {
        AUrdfLink* component = kvp.Value;
        FVector componentLocation = component->GetActorLocation();
        FRotator actorRotation = component->GetActorRotation();

        component->SetActorLocationAndRotation(translation + componentLocation, rotation + actorRotation);
    }
}
