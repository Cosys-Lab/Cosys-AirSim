// Fill out your copyright notice in the Description page of Project Settings.


#include "WifiBeacon.h"
#include "AirBlueprintLib.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"


// Sets default values
AWifiBeacon::AWifiBeacon()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	// set the mesh
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Beacon mesh"));
	this->SetRootComponent(Mesh);
	// attach to root component
	//Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh

	static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/wifiBeacon.wifiBeacon'"));
	// check if path is valid
	if (loadedMesh.Succeeded())
	{
		// mesh = valid path
		Mesh->SetStaticMesh(loadedMesh.Object);
	}
}

// Called when the game starts or when spawned
void AWifiBeacon::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AWifiBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

