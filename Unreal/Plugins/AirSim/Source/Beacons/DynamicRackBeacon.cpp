// Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicRackBeacon.h"
#include "AirBlueprintLib.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"


// Sets default values
ADynamicRackBeacon::ADynamicRackBeacon()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	// set the mesh
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Beacon mesh"));
	this->SetRootComponent(Mesh);
	// attach to root component
	//Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh

	static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'"));
	// check if path is validBlueprint'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'
	if (loadedMesh.Succeeded())
	{
		// mesh = valid path
		Mesh->SetStaticMesh(loadedMesh.Object);
	}
}


// Called when the game starts or when spawned
void ADynamicRackBeacon::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ADynamicRackBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

