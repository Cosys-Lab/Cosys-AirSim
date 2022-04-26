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
	// attach to root component
	Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh

	// There are 5 rack options
	int rackId = rand() % 5;

	if (rackId ==0){
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'"));
		// check if path is validBlueprint'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'
		if (loadedMesh.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh.Object);
		}
	}else if (rackId == 1) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/dynamicRacks/DynamicRackBeacon_barrel2.DynamicRackBeacon_barrel2'"));
		// check if path is validBlueprint'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'
		if (loadedMesh.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh.Object);
		}
	}
	else if (rackId == 2) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/dynamicRacks/DynamicRackBeacon_barrel.DynamicRackBeacon_barrel'"));
		// check if path is validBlueprint'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'
		if (loadedMesh.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh.Object);
		}
	}
	else if (rackId == 3) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/dynamicRacks/DynamicRackBeacon_box.DynamicRackBeacon_box'"));
		// check if path is validBlueprint'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'
		if (loadedMesh.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh.Object);
		}
	}
	else if (rackId == 4) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/dynamicRacks/DynamicRackBeacon_hose.DynamicRackBeacon_hose'"));
		// check if path is validBlueprint'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'
		if (loadedMesh.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh.Object);
		}
	}
	else {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'"));
		// check if path is validBlueprint'/AirSim/Beacons/DynamicRackBeacon.DynamicRackBeacon'
		if (loadedMesh.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh.Object);
		}
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

