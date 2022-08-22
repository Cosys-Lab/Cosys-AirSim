#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerInput.h"

#include "ManualPoseController.generated.h"

UCLASS()
class AIRSIM_API UManualPoseController : public UObject {
    GENERATED_BODY()

public:
    void initializeForPlay();
    void setActor(AActor* actor);
    AActor* getActor() const;
    void updateActorPose(float dt);
    void getDeltaPose(FVector& delta_position, FRotator& delta_rotation) const;
    void resetDelta();
    void updateDeltaPosition(float dt);
	bool getHold();

private:
    void inputManualLeft(float val);
    void inputManualRight(float val);
    void inputManualForward(float val);
    void inputManualBackward(float val);
    void inputManualMoveUp(float val);
    void inputManualDown(float val);
    void inputManualLeftYaw(float val);
    void inputManualUpPitch(float val);
    void inputManualRightYaw(float val);
    void inputManualDownPitch(float val);
	void inputManualShift(float val);
	void inputManualSpace(float val);

    void setupInputBindings();	
    void removeInputBindings();
    void clearBindings();

private:
    FInputAxisBinding *left_binding_, *right_binding_, *up_binding_, *down_binding_;
    FInputAxisBinding *forward_binding_, *backward_binding_, *left_yaw_binding_, *up_pitch_binding_;
    FInputAxisBinding *right_yaw_binding_, *down_pitch_binding_;
	FInputAxisBinding *shift_binding_, *space_binding_;


    FInputAxisKeyMapping left_mapping_, right_mapping_, up_mapping_, down_mapping_;
    FInputAxisKeyMapping forward_mapping_, backward_mapping_, left_yaw_mapping_, up_pitch_mapping_;
    FInputAxisKeyMapping right_yaw_mapping_, down_pitch_mapping_;
	FInputAxisKeyMapping shift_mapping_, space_mapping_;

    FVector delta_position_;
    FRotator delta_rotation_;

    AActor *actor_;

    float acceleration_ = 0;
	bool hold_ = false;
	float acceleration_factor_ = 60;
    FVector input_positive_, inpute_negative_;
    FVector last_velocity_;
};