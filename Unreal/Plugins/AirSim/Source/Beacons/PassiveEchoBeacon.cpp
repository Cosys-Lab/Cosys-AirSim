// Developed by Cosys-Lab, University of Antwerp

#include "PassiveEchoBeacon.h"
#include "AirBlueprintLib.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"


// Sets default values
APassiveEchoBeacon::APassiveEchoBeacon()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	arrow_ = CreateDefaultSubobject<UArrowComponent>(TEXT("PassiveEchoBeaconArrow"));
	arrow_->SetMobility(EComponentMobility::Type::Static);
	arrow_->SetArrowColor(FLinearColor(177, 0, 151));
	arrow_->SetRelativeScale3D(FVector(0.33f));
	this->SetRootComponent(arrow_);

	Super::SetActorHiddenInGame(true);
	name_ = this->GetName();
}

// initializes information based on echo configuration
void APassiveEchoBeacon::generateSampleDirectionPoints()
{
	UnrealEchoCommon::sampleSphereCap(initial_directions_, initial_lower_azimuth_limit_, initial_upper_azimuth_limit_, -initial_upper_elevation_limit_, -initial_lower_elevation_limit_, sample_direction_points_);
}

void APassiveEchoBeacon::getPointCloud()
{
	FVector trace_start_position;
	trace_start_position = ned_transform_->fromLocalNed(beacon_reference_frame_.position);



	point_cloud_.emplace_back(ned_transform_->getGlobalTransform().GetLocation().X);
	point_cloud_.emplace_back(ned_transform_->getGlobalTransform().GetLocation().Y);
	point_cloud_.emplace_back(ned_transform_->getGlobalTransform().GetLocation().Z);
	point_cloud_.emplace_back(0);
	point_cloud_.emplace_back(0);
	point_cloud_.emplace_back(0);
	point_cloud_.emplace_back(ned_transform_->getGlobalTransform().GetRotation().Rotator().Vector().X);
	point_cloud_.emplace_back(ned_transform_->getGlobalTransform().GetRotation().Rotator().Vector().Y);
	point_cloud_.emplace_back(ned_transform_->getGlobalTransform().GetRotation().Rotator().Vector().Z);
	groundtruth_.emplace_back(std::string(TCHAR_TO_UTF8(*name_)));
	groundtruth_.emplace_back(std::string(TCHAR_TO_UTF8(*name_)));
	for (auto sample_direction_point_count = 0u; sample_direction_point_count < sample_direction_points_.size(); ++sample_direction_point_count)
	{
		Vector3r sample_direction_point = sample_direction_points_[sample_direction_point_count];

		FVector trace_direction = UnrealEchoCommon::Vector3rToFVector(VectorMath::rotateVector(sample_direction_point, beacon_reference_frame_.orientation, 1)); // sensor_reference_frame_.orientation

		float trace_length = ned_transform_->fromNed(UnrealEchoCommon::remainingDistance(0, 0, attenuation_limit_, distance_limit_));  // Maximum possible distance for emitted signal (0 distance, 0 attenuation)
		FVector trace_end_position = trace_start_position + trace_direction * trace_length;

		// Shoot trace and get the impact point and remaining attenuation, if any returns
		UnrealEchoCommon::traceDirection(0, false, trace_start_position, trace_end_position, point_cloud_, groundtruth_, point_cloud_draw_reflected_points_, ned_transform_, beacon_reference_frame_,
			distance_limit_, reflection_limit_, attenuation_limit_, reflection_distance_limit_cm_, 0, attenuation_per_distance_, attenuation_per_reflection_, ignore_actors_, this, false, true,
			draw_debug_duration_, line_thickness_ / 2, false, draw_debug_all_lines_, false, false, false, false, true, true, reflection_only_final_, std::string(TCHAR_TO_UTF8(*name_)));
	}
}

void APassiveEchoBeacon::parsePointCloud()
{
	bool persistent_lines = false;
	if (draw_debug_duration_ == -1)persistent_lines = true;

	int float_stride = 9;
	int string_stride = 2;
	for (auto point_count = 0u; point_count < (int) point_cloud_.size() / (float)float_stride; ++point_count)
	{
		FVector point = FVector(point_cloud_[point_count * float_stride], point_cloud_[point_count * float_stride + 1], point_cloud_[point_count * float_stride + 2]);
		float signal_attenuation = point_cloud_[point_count * float_stride + 3];
		float signal_distance = point_cloud_[point_count * float_stride + 4];
		float reflections = point_cloud_[point_count * float_stride + 5];
		FVector direction = FVector(point_cloud_[point_count * float_stride + 6], point_cloud_[point_count * float_stride + 7], point_cloud_[point_count * float_stride + 8]);	
		std::string reflection_object = groundtruth_[point_count * string_stride];
		std::string source_object = groundtruth_[point_count * string_stride + 1];

		UnrealEchoCommon::EchoPoint echo_point;
		echo_point.point = point;
		echo_point.direction = direction;
		echo_point.reflection_object = reflection_object;
		echo_point.source_object = source_object;
		echo_point.total_distance = signal_distance;
		echo_point.total_attenuation = signal_attenuation;
		echo_point.reflections = reflections;
		points_.Add(echo_point);

		if (draw_debug_all_points_) {
			UAirBlueprintLib::DrawPoint(this->GetWorld(), point, 5, FColor::Red, persistent_lines, draw_debug_duration_);
			FVector draw_line_end_point = point + direction * 100;
			FColor line_color = FColor::MakeRedToGreenColorFromScalar(1 - (signal_attenuation / attenuation_limit_));
			UAirBlueprintLib::DrawLine(this->GetWorld(), point, draw_line_end_point, line_color, persistent_lines, draw_debug_duration_, 0, line_thickness_);
		}		
	}
}

TArray<UnrealEchoCommon::EchoPoint> APassiveEchoBeacon::getPoints() {
	return points_;
}

// Called when the game starts or when spawned
void APassiveEchoBeacon::BeginPlay()
{
	Super::BeginPlay();
}

void APassiveEchoBeacon::StartSampling() {

	if (!started_) {
		ned_transform_ = new NedTransform(this, NedTransform(Super::GetActorTransform(), UAirBlueprintLib::GetWorldToMetersScale(this)));

		beacon_reference_frame_ = msr::airlib::Pose();

		reflection_distance_limit_cm_ = ned_transform_->fromNed(reflection_distance_limit_);

		if (draw_debug_location_) {
			bool persistent_lines = false;
			if (draw_debug_duration_ == -1)persistent_lines = true;
			UAirBlueprintLib::DrawCoordinateSystem(this->GetWorld(), Super::GetActorLocation(), Super::GetActorRotation(), 25, persistent_lines, draw_debug_duration_, 10);
		}
		if (enable_) {
			generateSampleDirectionPoints();
			point_cloud_.clear();
			groundtruth_.clear();
			getPointCloud();
			parsePointCloud();
		}
		started_ = true;
	}
}

bool APassiveEchoBeacon::IsStarted() {
	return started_;
}

// Called every frame
void APassiveEchoBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

