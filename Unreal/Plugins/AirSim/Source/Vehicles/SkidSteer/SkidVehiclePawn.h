#pragma once

#include "CoreMinimal.h"
#include "Vehicles/SkidSteer/SkidVehicle.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "UObject/ConstructorHelpers.h"

#include "physics/Kinematics.hpp"
#include "vehicles/car/api/CarApiBase.hpp"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "common/common_utils/Signal.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "PawnEvents.h"
#include "PIPCamera.h"

#include "SkidVehiclePawn.generated.h"

class UPhysicalMaterial;
class UCameraComponent;
class USpringArmComponent;
class UTextRenderComponent;
class UInputComponent;
class UAudioComponent;

UCLASS(config = Game)
class AIRSIM_API ASkidVehiclePawn : public ASkidVehicle
{
	GENERATED_BODY()
	
public:
	ASkidVehiclePawn();

	virtual void BeginPlay() override;
	virtual void Tick(float Delta) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
		FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

	//interface
	void initializeForBeginPlay(bool engine_sound);
	const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
	PawnEvents* getPawnEvents()
	{
		return &pawn_events_;
	}
	USkidVehicleMovementComponent* getVehicleMovementComponent() const;
	const msr::airlib::CarApiBase::CarControls& getKeyBoardControls() const
	{
		return keyboard_controls_;
	}

	UPROPERTY(BluePrintReadWrite) USceneComponent* camera_front_center_base_;
	UPROPERTY(BluePrintReadWrite) USceneComponent* camera_front_left_base_;
	UPROPERTY(BluePrintReadWrite) USceneComponent* camera_front_right_base_;
	UPROPERTY(BluePrintReadWrite) USceneComponent* camera_driver_base_;
	UPROPERTY(BluePrintReadWrite) USceneComponent* camera_back_center_base_;

private:
	void updateHUDStrings();
	void updateInCarHUD();
	void updatePhysicsMaterial();

	void setupInputBindings();

private:
	typedef msr::airlib::AirSimSettings AirSimSettings;

	UClass* pip_camera_class_;

	PawnEvents pawn_events_;

	bool is_low_friction_;
	UPhysicalMaterial* slippery_mat_;
	UPhysicalMaterial* non_slippery_mat_;



	UPROPERTY() APIPCamera* camera_front_center_;
	UPROPERTY() APIPCamera* camera_front_left_;
	UPROPERTY() APIPCamera* camera_front_right_;
	UPROPERTY() APIPCamera* camera_driver_;
	UPROPERTY() APIPCamera* camera_back_center_;

	UTextRenderComponent* speed_text_render_;
	UTextRenderComponent* gear_text_render_;
	UAudioComponent* engine_sound_audio_;

	msr::airlib::CarApiBase::CarControls keyboard_controls_;
	void onMoveX(float Val);
	void onMoveY(float Val);
	void onBreakPressed();
	void onBreakReleased();

	FText last_speed_;
	FText last_gear_;
	FColor	last_gear_display_color_;
	FColor	last_gear_display_reverse_color_;
	UStaticMeshComponent* top_panel_component_;
	bool show_top_panel_;
};
