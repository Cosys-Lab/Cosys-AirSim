// Developed by Cosys-Lab, University of Antwerp

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/ArrowComponent.h"
#include "NedTransform.h"
#include "AirBlueprintLib.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "AirLib/include/common/AirSimSettings.hpp"
#include "UnrealSensors/UnrealEchoCommon.h"

#include "PassiveEchoBeacon.generated.h"

UCLASS()
class AIRSIM_API APassiveEchoBeacon : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	APassiveEchoBeacon();

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
	FString	name_ = "MyPassiveEchoBeacon";

	/** Toggle the beacon on or off. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		bool enable_ = true;
	
	/** Amount of traces (rays) being cast. This defines the resolution of the resulting reflection point cloud. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		int32 initial_directions_ = 1000;

	/** The lower azimuth angle limit in degrees for sending out the initial rays of the source. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_lower_azimuth_limit_ = -90;

	/** The upper azimuth angle limit in degrees for sending out the initial rays of the source. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_upper_azimuth_limit_ = 90;

	/** The lower elevation angle limit in degrees for sending out the initial rays of the source. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_lower_elevation_limit_ = -90;

	/** The upper elevation angle limit in degrees for sending out the initial rays of the source. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float initial_upper_elevation_limit_ = 90;

	/** Maximum amount of reflections that can happen. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		int reflection_limit_ = 3;

	/** Maximum distance between two reflections (meters) */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float reflection_distance_limit_ = 0.4;

	/** Only save the final reflection along a trace. This will ignore all other reflections that happen along the trace in the data */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		bool reflection_only_final_ = false;

	/** Attenuation of signal wrt distance traveled (dB/m) */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_per_distance_ = 0;

	/** Attenuation of signal wrt reflections (dB) */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_per_reflection_ = 0;

	/** Attenuation at which the signal is considered dissipated (dB) */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float attenuation_limit_ = -100;

	/** Maximum distance a reflection can travel (meters) */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|General")
		float distance_limit_ = 3;

	/** Draw debug points in world where reflected points are happening due to this source. It will also show the reflection direction with a line. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_all_points_ = false;

	/** Draw all lines that are being cast from the source to the reflections, not only the ones that are reflected. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_all_lines_ = false;

	/** Draw a 3D axes shown where the source is. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		bool draw_debug_location_ = false;

	/** Duration in seconds that the debug points and lines will be shown in the world. -1 is infinite. */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "PassiveEchoBeacon|Debug")
		float draw_debug_duration_ = -1.f;

	/** Generate the sampling for this passive echo beacon. */
	UFUNCTION(BlueprintCallable, Category = "PassiveEchoBeacon")
	void StartSampling();

	/** If the startup sampling has already been called. */
	UFUNCTION(BlueprintCallable, Category = "PassiveEchoBeacon")
	bool IsStarted();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY()
		UArrowComponent* arrow_ = nullptr;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	TArray<UnrealEchoCommon::EchoPoint> getPoints();

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
	TArray<UnrealEchoCommon::EchoPoint> points_;
	msr::airlib::Pose beacon_reference_frame_;
	TArray<AActor*> ignore_actors_;
	float reflection_distance_limit_cm_;
	bool started_ = false;
	msr::airlib::vector<FVector> point_cloud_draw_reflected_points_;
};
