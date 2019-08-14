#include "CPHuskyPawn.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Controller.h"
#include "Components/TextRenderComponent.h"
#include "Components/AudioComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Sound/SoundCue.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "CPHuskyWheel.h"
#include "AirBlueprintLib.h"
#include <vector>
#include "common/common_utils/Utils.hpp"
#include "common/ClockFactory.hpp"


#define LOCTEXT_NAMESPACE "VehiclePawn"

ACPHuskyPawn::ACPHuskyPawn()
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

	setupVehicleMovementComponent();
	
	show_top_panel_ = true;

	// Spawn the top panel
	if (show_top_panel_) {
		ConstructorHelpers::FObjectFinder<UStaticMesh> top_panel_static(TEXT("StaticMesh'/AirSim/VehicleAdv/CPHusky/CPHusky_TopPanel.CPHusky_TopPanel'"));
		top_panel_component_ = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TopPanel"));
		top_panel_component_->SetStaticMesh(top_panel_static.Object);
		top_panel_component_->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		top_panel_component_->SetupAttachment(GetMesh(), FName("TopSocket"));
		top_panel_component_->SetRelativeLocation(FVector(0, 0, -28.016001));
	}

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
	speed_text_render_->SetRelativeLocation(FVector(35.0f, -6.0f, 20.0f));
	speed_text_render_->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
	speed_text_render_->SetupAttachment(GetMesh());
	speed_text_render_->SetVisibility(true);

	// Create text render component for in car gear display
	gear_text_render_ = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarGear"));
	gear_text_render_->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
	gear_text_render_->SetRelativeLocation(FVector(35.0f, 5.0f, 20.0f));
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

void ACPHuskyPawn::setupVehicleMovementComponent()
{
	UWheeledVehicleMovementComponent4W* movement = CastChecked<UWheeledVehicleMovementComponent4W>(getVehicleMovementComponent());
	check(movement->WheelSetups.Num() == 4);

	// Wheels/Tires
	// Setup the wheels
	movement->WheelSetups[0].WheelClass = UCPHuskyWheel::StaticClass();
	movement->WheelSetups[0].BoneName = FName("CPHusky_Wheel_FL");

	movement->WheelSetups[1].WheelClass = UCPHuskyWheel::StaticClass();
	movement->WheelSetups[1].BoneName = FName("CPHusky_Wheel_FR");

	movement->WheelSetups[2].WheelClass = UCPHuskyWheel::StaticClass();
	movement->WheelSetups[2].BoneName = FName("CPHusky_Wheel_BL");

	movement->WheelSetups[3].WheelClass = UCPHuskyWheel::StaticClass();
	movement->WheelSetups[3].BoneName = FName("CPHusky_Wheel_BR");

	// Adjust the tire loading
	movement->MinNormalizedTireLoad = 0.0f;
	movement->MinNormalizedTireLoadFiltered = 0.2308f;
	movement->MaxNormalizedTireLoad = 2.0f;
	movement->MaxNormalizedTireLoadFiltered = 2.0f;

	// Engine 
	// Torque setup
	movement->EngineSetup.MaxRPM = 15.0f;
	movement->EngineSetup.TorqueCurve.GetRichCurve()->Reset();
	movement->EngineSetup.TorqueCurve.GetRichCurve()->AddKey(260.0f, 6408.0f);

	// Adjust the steering 
	movement->SteeringCurve.GetRichCurve()->Reset();
	movement->SteeringCurve.GetRichCurve()->AddKey(0.0f, 1.0f);
	movement->SteeringCurve.GetRichCurve()->AddKey(40.0f, 0.7f);
	movement->SteeringCurve.GetRichCurve()->AddKey(120.0f, 0.6f);

	// Transmission	
	// We want 4wd
	movement->DifferentialSetup.DifferentialType = EVehicleDifferential4W::LimitedSlip_4W;

	// Drive the front wheels a little more than the rear
	movement->DifferentialSetup.FrontRearSplit = 0.5;

	// Automatic gearbox
	movement->TransmissionSetup.bUseGearAutoBox = true;
	movement->TransmissionSetup.GearSwitchTime = 0.15f;
	movement->TransmissionSetup.GearAutoBoxLatency = 1.0f;

	// Disable reverse as brake, this is needed for SetBreakInput() to take effect
	movement->bReverseAsBrake = false;

	// Physics settings
	// Adjust the center of mass - the buggy is quite low
	UPrimitiveComponent* primitive = Cast<UPrimitiveComponent>(movement->UpdatedComponent);
	if (primitive)
	{
		primitive->BodyInstance.COMNudge = FVector(8.0f, 0.0f, 0.0f);
	}

	// Set the inertia scale. This controls how the mass of the vehicle is distributed.
	movement->InertiaTensorScale = FVector(1.0f, 1.333f, 1.2f);
	movement->bDeprecatedSpringOffsetMode = true;
}

void ACPHuskyPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
	FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
	pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
		HitNormal, NormalImpulse, Hit);
}

UWheeledVehicleMovementComponent* ACPHuskyPawn::getVehicleMovementComponent() const
{
	return GetVehicleMovement();
}

void ACPHuskyPawn::initializeForBeginPlay(bool engine_sound)
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

const common_utils::UniqueValueMap<std::string, APIPCamera*> ACPHuskyPawn::getCameras() const
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

void ACPHuskyPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
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

void ACPHuskyPawn::Tick(float Delta)
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

void ACPHuskyPawn::BeginPlay()
{
	Super::BeginPlay();

	// Start an engine sound playing
	engine_sound_audio_->Play();
}

void ACPHuskyPawn::updateHUDStrings()
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

void ACPHuskyPawn::updateInCarHUD()
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



void ACPHuskyPawn::updatePhysicsMaterial()
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

/******************* Keyboard bindings*******************/
//This method must be in pawn because Unreal doesn't allow key bindings to non UObject pointers
void ACPHuskyPawn::setupInputBindings()
{
	UAirBlueprintLib::EnableInput(this);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Up, 1), this,
		this, &ACPHuskyPawn::onMoveForward);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Down, -1), this,
		this, &ACPHuskyPawn::onMoveForward);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Right, 0.5), this,
		this, &ACPHuskyPawn::onMoveRight);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Left, -0.5), this,
		this, &ACPHuskyPawn::onMoveRight);

	UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACPHuskyPawn::onHandbrakePressed, true);
	UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACPHuskyPawn::onHandbrakeReleased, false);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::SpaceBar, 1), this,
		this, &ACPHuskyPawn::onFootBrake);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Gamepad_LeftX, 1), this,
		this, &ACPHuskyPawn::onMoveRight);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Gamepad_RightTriggerAxis, 1), this,
		this, &ACPHuskyPawn::onMoveForward);

	UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::Gamepad_LeftTriggerAxis, 1), this,
		this, &ACPHuskyPawn::onFootBrake);

	//below is not needed
	//UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACPHuskyPawn::onReversePressed, true);
	//UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACPHuskyPawn::onReverseReleased, false);
}

void ACPHuskyPawn::onMoveForward(float Val)
{
	if (Val < 0)
		onReversePressed();
	else
		onReverseReleased();

	keyboard_controls_.throttle = Val;
}

void ACPHuskyPawn::onMoveRight(float Val)
{
	keyboard_controls_.steering = Val;
}

void ACPHuskyPawn::onHandbrakePressed()
{
	keyboard_controls_.handbrake = true;
}

void ACPHuskyPawn::onHandbrakeReleased()
{
	keyboard_controls_.handbrake = false;
}

void ACPHuskyPawn::onFootBrake(float Val)
{
	keyboard_controls_.brake = Val;
}

void ACPHuskyPawn::onReversePressed()
{
	if (keyboard_controls_.manual_gear >= 0) {
		keyboard_controls_.is_manual_gear = true;
		keyboard_controls_.manual_gear = -1;
		keyboard_controls_.gear_immediate = true;
	}
}

void ACPHuskyPawn::onReverseReleased()
{
	if (keyboard_controls_.manual_gear < 0) {
		keyboard_controls_.is_manual_gear = false;
		keyboard_controls_.manual_gear = 0;
		keyboard_controls_.gear_immediate = true;
	}
}

#undef LOCTEXT_NAMESPACE
