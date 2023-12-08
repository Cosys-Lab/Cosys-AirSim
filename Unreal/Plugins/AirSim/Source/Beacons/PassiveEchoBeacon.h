// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/ArrowComponent.h"
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
		bool enable_ = true;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		int32 initial_directions_ = 1000;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_lower_azimuth_limit_ = -90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_upper_azimuth_limit_ = 90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_lower_elevation_limit_ = -90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_upper_elevation_limit_ = 90;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_limit_ = -100;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float reflection_distance_limit_ = 0.4;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_per_distance_ = 0;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_per_reflection_ = 0;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float distance_limit_ = 3;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		int reflection_limit_ = 3;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_location_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_all_points_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_all_lines_ = false;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		float draw_debug_duration_ = -1.f;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY()
		UArrowComponent* arrow_ = nullptr;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	TArray<UnrealEchoSensor::EchoPoint> getPoints();

private:

	void generateSampleDirectionPoints();
	void getPointCloud();
	void parsePointCloud();

private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;
	float line_thickness_ = 1;
	const NedTransform* ned_transform_;
	msr::airlib::vector<msr::airlib::Vector3r> sample_direction_points_;
	msr::airlib::vector<msr::airlib::real_T> point_cloud_;
	msr::airlib::vector<std::string> groundtruth_;
	TArray<UnrealEchoSensor::EchoPoint> points_;
	msr::airlib::Pose beacon_reference_frame_;
	TArray<AActor*> ignore_actors_;
};
