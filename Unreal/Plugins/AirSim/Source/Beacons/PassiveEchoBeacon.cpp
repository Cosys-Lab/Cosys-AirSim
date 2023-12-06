// Fill out your copyright notice in the Description page of Project Settings.

#include "AirBlueprintLib.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"
#include "PassiveEchoBeacon.h"

// Sets default values
APassiveEchoBeacon::APassiveEchoBeacon()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	arrow_ = CreateDefaultSubobject<UArrowComponent>(TEXT("PassiveEchoBeaconArrow"));
	arrow_->SetMobility(EComponentMobility::Type::Static);
	arrow_->SetArrowColor(FLinearColor(177, 0, 151));
	arrow_->SetRelativeScale3D(FVector(0.1f));
	this->SetRootComponent(arrow_);

	Super::SetActorHiddenInGame(true);
}

// initializes information based on echo configuration
void APassiveEchoBeacon::generateSampleDirections()
{
	UnrealEchoSensor::sampleSphereCap(initial_directions_, initial_lower_azimuth_limit_, initial_upper_azimuth_limit_, -initial_upper_elevation_limit_, -initial_lower_elevation_limit_, sample_directions_);
}

void APassiveEchoBeacon::getPointCloud()
{
	FVector trace_start_position;
	trace_start_position = ned_transform_->fromLocalNed(beacon_reference_frame_.position);

	// Shoot traces (rays)
	point_cloud_.clear();
	groundtruth_.clear();
	for (auto direction_count = 0u; direction_count < sample_directions_.size(); ++direction_count)
	{
		Vector3r sample_direction = sample_directions_[direction_count];

		FVector trace_direction = UnrealEchoSensor::Vector3rToFVector(VectorMath::rotateVector(sample_direction, beacon_reference_frame_.orientation, 1)); // sensor_reference_frame_.orientation

		float trace_length = ned_transform_->fromNed(UnrealEchoSensor::remainingDistance(0, 0, attenuation_limit_, distance_limit_));  // Maximum possible distance for emitted signal (0 distance, 0 attenuation)
		FVector trace_end_position = trace_start_position + trace_direction * trace_length;

		// Shoot trace and get the impact point and remaining attenuation, if any returns
		UnrealEchoSensor::traceDirection(trace_start_position, trace_end_position, point_cloud_, groundtruth_, ned_transform_, beacon_reference_frame_,
			distance_limit_, reflection_limit_, attenuation_limit_, reflection_distance_limit_, 0, attenuation_per_distance_, attenuation_per_reflection_, ignore_actors_, this, false,
			draw_debug_duration_, line_thickness_, false, draw_debug_all_lines_, false, draw_debug_all_points_, false, false);
	}

	return;
}

// Called when the game starts or when spawned
void APassiveEchoBeacon::BeginPlay()
{


	Super::BeginPlay();	

	ned_transform_ = new NedTransform(this, NedTransform(this->GetWorld()->GetFirstPlayerController()->GetViewTarget()->GetActorTransform(), UAirBlueprintLib::GetWorldToMetersScale(this)));

	beacon_reference_frame_ = ned_transform_->toLocalNed(Super::GetActorTransform());

	generateSampleDirections();

	if (draw_debug_location_) {
		UAirBlueprintLib::DrawCoordinateSystem(this->GetWorld(), Super::GetActorLocation(), Super::GetActorRotation(), 25, false, draw_debug_duration_, 10);
	}

	getPointCloud();
}

// Called every frame
void APassiveEchoBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

