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
	draw_time_ = 30;

	// set the mesh
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Beacon mesh"));
	// attach to root component
	Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh

	static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/passiveEchoBeaconMesh.passiveEchoBeaconMesh'"));
	// check if path is valid
	if (loadedMesh.Succeeded())
	{
		// mesh = valid path
		Mesh->SetStaticMesh(loadedMesh.Object);
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	}
	Super::SetActorHiddenInGame(true);
}

// initializes information based on echo configuration
void APassiveEchoBeacon::generateSampleDirections()
{
	UnrealEchoSensor::sampleSphereCap(initial_directions_, initial_lower_azimuth_limit_, initial_upper_azimuth_limit_, initial_upper_elevation_limit_, initial_lower_elevation_limit_, sample_directions_);
}

// Called when the game starts or when spawned
void APassiveEchoBeacon::BeginPlay()
{
	Super::BeginPlay();	
	if (draw_debug_location_) {
		UAirBlueprintLib::DrawCoordinateSystem(this->GetWorld(), Super::GetActorLocation(), Super::GetActorRotation(), 25, false, draw_time_, 10);
	}
}

// Called every frame
void APassiveEchoBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

