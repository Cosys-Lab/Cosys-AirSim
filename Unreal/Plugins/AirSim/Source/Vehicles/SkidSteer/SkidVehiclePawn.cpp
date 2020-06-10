#include "SkidVehiclePawn.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Controller.h"
#include "Components/TextRenderComponent.h"
#include "Components/AudioComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Sound/SoundCue.h"
#include "SkidVehicleMovementComponent.h"

#include "AirBlueprintLib.h"
#include <vector>
#include "common/common_utils/Utils.hpp"
#include "common/ClockFactory.hpp"


#define LOCTEXT_NAMESPACE "VehiclePawn"

ASkidVehiclePawn::ASkidVehiclePawn()
{
	static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
	pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;

	const auto& car_mesh_paths = AirSimSettings::singleton().pawn_paths["CPHusky"];
	auto slippery_mat = Cast<UPhysicalMaterial>(
		UAirBlueprintLib::LoadObject(car_mesh_paths.slippery_mat));
	auto non_slippery_mat = Cast<UPhysicalMaterial>(
		UAirBlueprintLib::LoadObject(car_mesh_paths.non_slippery_mat));
	if (slippery_mat)
		slippery_mat_ = slippery_mat;
	else
		UAirBlueprintLib::LogMessageString("Failed to load Slippery physics material", "", LogDebugLevel::Failure);
	if (non_slippery_mat)
		non_slippery_mat_ = non_slippery_mat;
	else
		UAirBlueprintLib::LogMessageString("Failed to load NonSlippery physics material", "", LogDebugLevel::Failure);

	// Create In-Car camera component 
	camera_front_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_base_"));
	camera_front_center_base_->SetRelativeLocation(FVector(45, 0, 45)); //center
	camera_front_center_base_->SetupAttachment(GetMesh());
	camera_front_left_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_left_base_"));
	camera_front_left_base_->SetRelativeLocation(FVector(45, -20, 45)); //left
	camera_front_left_base_->SetupAttachment(GetMesh());
	camera_front_right_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_right_base_"));
	camera_front_right_base_->SetRelativeLocation(FVector(45, 20, 45)); //right
	camera_front_right_base_->SetupAttachment(GetMesh());
	camera_driver_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_driver_base_"));
	camera_driver_base_->SetRelativeLocation(FVector(3, 0, 75)); //driver
	camera_driver_base_->SetupAttachment(GetMesh());
	camera_back_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_back_center_base_"));
	camera_back_center_base_->SetRelativeLocation(FVector(-190, 0, 75)); //rear
	camera_back_center_base_->SetupAttachment(GetMesh());

	// In car HUD
	// Create text render component for in car speed display
	speed_text_render_ = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarSpeed"));
	speed_text_render_->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
	speed_text_render_->SetRelativeLocation(FVector(0.0f, 0.0f, 5.0f));
	speed_text_render_->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
	speed_text_render_->SetupAttachment(GetMesh());
	speed_text_render_->SetVisibility(true);

	// Create text render component for in car gear display
	gear_text_render_ = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarGear"));
	gear_text_render_->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
	gear_text_render_->SetRelativeLocation(FVector(0.0f, 0.0f, 5.0f));
	gear_text_render_->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
	gear_text_render_->SetupAttachment(GetMesh());
	gear_text_render_->SetVisibility(true);

	// Setup the audio component and allocate it a sound cue
	ConstructorHelpers::FObjectFinder<USoundCue> SoundCue(TEXT("/AirSim/VehicleAdv/Sound/Engine_Loop_Cue.Engine_Loop_Cue"));
	engine_sound_audio_ = CreateDefaultSubobject<UAudioComponent>(TEXT("EngineSound"));
	engine_sound_audio_->SetSound(SoundCue.Object);
	engine_sound_audio_->SetupAttachment(GetMesh());

	// Colors for the in-car gear display. One for normal one for reverse
	last_gear_display_reverse_color_ = FColor(255, 0, 0, 255);
	last_gear_display_color_ = FColor(255, 255, 255, 255);

	is_low_friction_ = false;
}

void ASkidVehiclePawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
	FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
	pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
		HitNormal, NormalImpulse, Hit);
}

USkidVehicleMovementComponent* ASkidVehiclePawn::getVehicleMovementComponent() const
{
	return GetSkidVehicleMovement();
}

void ASkidVehiclePawn::initializeForBeginPlay(bool engine_sound)
{
	if (engine_sound)
		engine_sound_audio_->Activate();
	else
		engine_sound_audio_->Deactivate();


	//put camera little bit above vehicle
	FTransform camera_transform(FVector::ZeroVector);
	FActorSpawnParameters camera_spawn_params;
	camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

	camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_center"));
	camera_front_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
	camera_front_center_->AttachToComponent(camera_front_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

	camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_left"));
	camera_front_left_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
	camera_front_left_->AttachToComponent(camera_front_left_base_, FAttachmentTransformRules::KeepRelativeTransform);

	camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_right"));
	camera_front_right_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
	camera_front_right_->AttachToComponent(camera_front_right_base_, FAttachmentTransformRules::KeepRelativeTransform);

	camera_spawn_params.Name = FName(*(this->GetName() + "_camera_driver"));
	camera_driver_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
	camera_driver_->AttachToComponent(camera_driver_base_, FAttachmentTransformRules::KeepRelativeTransform);

	camera_spawn_params.Name = FName(*(this->GetName() + "_camera_back_center"));
	camera_back_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_,
		FTransform(FRotator(0, -180, 0), FVector::ZeroVector), camera_spawn_params);
	camera_back_center_->AttachToComponent(camera_back_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

	setupInputBindings();
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> ASkidVehiclePawn::getCameras() const
{
	common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
	cameras.insert_or_assign("front_center", camera_front_center_);
	cameras.insert_or_assign("front_right", camera_front_right_);
	cameras.insert_or_assign("front_left", camera_front_left_);
	cameras.insert_or_assign("fpv", camera_driver_);
	cameras.insert_or_assign("back_center", camera_back_center_);

	cameras.insert_or_assign("0", camera_front_center_);
	cameras.insert_or_assign("1", camera_front_right_);
	cameras.insert_or_assign("2", camera_front_left_);
	cameras.insert_or_assign("3", camera_driver_);
	cameras.insert_or_assign("4", camera_back_center_);

	cameras.insert_or_assign("", camera_front_center_);

	return cameras;
}

void ASkidVehiclePawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	camera_front_center_ = nullptr;
	camera_front_left_ = nullptr;
	camera_front_right_ = nullptr;
	camera_driver_ = nullptr;
	camera_back_center_ = nullptr;

	camera_front_center_base_ = nullptr;
	camera_front_left_base_ = nullptr;
	camera_front_right_base_ = nullptr;
	camera_driver_base_ = nullptr;
	camera_back_center_base_ = nullptr;
}

void ASkidVehiclePawn::Tick(float Delta)
{
	Super::Tick(Delta);

	// update physics material
	updatePhysicsMaterial();

	// Update the strings used in the HUD (in-car and on-screen)
	updateHUDStrings();

	// Set the string in the in-car HUD
	updateInCarHUD();

	// Pass the engine RPM to the sound component
	float RPMToAudioScale = 2500.0f / GetVehicleMovement()->GetEngineMaxRotationSpeed();
	engine_sound_audio_->SetFloatParameter(FName("RPM"), GetVehicleMovement()->GetEngineRotationSpeed()*RPMToAudioScale);

	pawn_events_.getPawnTickSignal().emit(Delta);
}

void ASkidVehiclePawn::BeginPlay()
{
	Super::BeginPlay();

	// Start an engine sound playing
	engine_sound_audio_->Play();
}

void ASkidVehiclePawn::updateHUDStrings()
{

	float speed_unit_factor = AirSimSettings::singleton().speed_unit_factor;
	FText speed_unit_label = FText::FromString(FString(AirSimSettings::singleton().speed_unit_label.c_str()));
	float vel = FMath::Abs(GetVehicleMovement()->GetForwardSpeed() / 100); //cm/s -> m/s
	float vel_rounded = FMath::FloorToInt(vel * 10 * speed_unit_factor) / 10.0f;
	int32 Gear = GetVehicleMovement()->GetCurrentGear();

	// Using FText because this is display text that should be localizable
	last_speed_ = FText::Format(LOCTEXT("SpeedFormat", "{0} {1}"), FText::AsNumber(vel_rounded), speed_unit_label);

	if (GetVehicleMovement()->GetCurrentGear() < 0)
	{
		last_gear_ = FText(LOCTEXT("ReverseGear", "R"));
	}
	else
	{
		last_gear_ = (Gear == 0) ? LOCTEXT("N", "N") : FText::AsNumber(Gear);
	}


	UAirBlueprintLib::LogMessage(TEXT("Speed: "), last_speed_.ToString(), LogDebugLevel::Informational);
	UAirBlueprintLib::LogMessage(TEXT("Gear: "), last_gear_.ToString(), LogDebugLevel::Informational);
	UAirBlueprintLib::LogMessage(TEXT("RPM: "), FText::AsNumber(GetVehicleMovement()->GetEngineRotationSpeed()).ToString(), LogDebugLevel::Informational);
}

void ASkidVehiclePawn::updateInCarHUD()
{
	APlayerController* PlayerController = Cast<APlayerController>(GetController());
	if ((PlayerController != nullptr) && (speed_text_render_ != nullptr) && (gear_text_render_ != nullptr))
	{
		// Setup the text render component strings
		speed_text_render_->SetText(last_speed_);
		gear_text_render_->SetText(last_gear_);

		if (GetVehicleMovement()->GetCurrentGear() >= 0)
		{
			gear_text_render_->SetTextRenderColor(last_gear_display_color_);
		}
		else
		{
			gear_text_render_->SetTextRenderColor(last_gear_display_reverse_color_);
		}
	}
}

USceneComponent* ASkidVehiclePawn::GetComponent(FString componentName)
{
	// Debugging
	if (componentName.Len() > 0)
	{
		throw std::runtime_error("Request for component " + std::string(TCHAR_TO_UTF8(*componentName)) + " in GetComponent in a pawn that does not have components.");
	}

	return this->RootComponent;
}

void ASkidVehiclePawn::GetComponentReferenceTransform(FString componentName, FVector& translation, FRotator& rotation)
{
	// Debugging
	if (componentName.Len() > 0)
	{
		throw std::runtime_error("Request for component " + std::string(TCHAR_TO_UTF8(*componentName)) + " in GetComponent in a pawn that does not have components.");
	}

	USceneComponent* component = this->GetComponent(componentName);

	translation = component->GetComponentLocation();
	rotation = component->GetComponentRotation();
}

void ASkidVehiclePawn::updatePhysicsMaterial()
{
	if (GetActorUpVector().Z < 0)
	{
		if (is_low_friction_ == true)
		{
			GetMesh()->SetPhysMaterialOverride(non_slippery_mat_);
			is_low_friction_ = false;
		}
		else
		{
			GetMesh()->SetPhysMaterialOverride(slippery_mat_);
			is_low_friction_ = true;
		}
	}
}

/******************* bindings*******************/
//This method must be in pawn because Unreal doesn't allow key bindings to non UObject pointers
void ASkidVehiclePawn::setupInputBindings()
{
	UAirBlueprintLib::EnableInput(this);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveY", EKeys::Up, 0.5), this,
		this, &ASkidVehiclePawn::onMoveY);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveY", EKeys::Down, -0.5), this,
		this, &ASkidVehiclePawn::onMoveY);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveX", EKeys::Right, 0.34), this,
		this, &ASkidVehiclePawn::onMoveX);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveX", EKeys::Left, -0.34), this,
		this, &ASkidVehiclePawn::onMoveX);

	UAirBlueprintLib::BindActionToKey("Break", EKeys::SpaceBar, this, &ASkidVehiclePawn::onBreakPressed, true);
	UAirBlueprintLib::BindActionToKey("Break", EKeys::SpaceBar, this, &ASkidVehiclePawn::onBreakReleased, false);


	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveX", EKeys::Gamepad_LeftX, 1), this,
		this, &ASkidVehiclePawn::onMoveX);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveY", EKeys::Gamepad_LeftY, 1), this,
		this, &ASkidVehiclePawn::onMoveY);

	UAirBlueprintLib::BindActionToKey("Break", EKeys::Gamepad_FaceButton_Right, this, &ASkidVehiclePawn::onBreakPressed, true);
	UAirBlueprintLib::BindActionToKey("Break", EKeys::Gamepad_FaceButton_Right, this, &ASkidVehiclePawn::onBreakReleased, false);
}

void ASkidVehiclePawn::onMoveX(float Val)
{
	keyboard_controls_.steering = Val;
}

void ASkidVehiclePawn::onMoveY(float Val)
{
	keyboard_controls_.throttle = Val;
}

void ASkidVehiclePawn::onBreakPressed()
{
	keyboard_controls_.brake = 1;
}

void ASkidVehiclePawn::onBreakReleased()
{
	keyboard_controls_.brake = 0;
}

#undef LOCTEXT_NAMESPACE

