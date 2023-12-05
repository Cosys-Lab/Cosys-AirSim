// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NedTransform.h"
#include "AirBlueprintLib.h"
#include "AirLib/include/common/AirSimSettings.hpp"
#include "UnrealSensors/UnrealEchoSensor.h"

#include "PassiveEchoBeacon.generated.h"

UCLASS()
class AIRSIM_API APassiveEchoBeacon : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	APassiveEchoBeacon();
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		int32 initial_directions_ = 1024;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_lower_azimuth_limit_ = -90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_upper_azimuth_limit_ = -90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_lower_elevation_limit_ = -90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_upper_elevation_limit_ = -90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float wave_length_ = 0.01;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_limit_ = -100;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float reflection_depth_ = 3;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float reflection_distance_limit_ = 10;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_per_distance_ = 0;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_per_reflection_ = 0;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		bool enable_ = true;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_location_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_initial_points_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_initial_lines_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_all_lines_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_all_points_ = false;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


private:

	void generateSampleDirections();

	UPROPERTY(EditAnywhere)
		UStaticMeshComponent* Mesh;

private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;
	float draw_time_;
	float line_thinkness_ = 1;
	const NedTransform* ned_transform_;
	float saved_clockspeed_;
	msr::airlib::vector<msr::airlib::Vector3r> sample_directions_;
	msr::airlib::vector<msr::airlib::Vector3r> spread_directions_;
	msr::airlib::Pose sensor_reference_frame_;
};
