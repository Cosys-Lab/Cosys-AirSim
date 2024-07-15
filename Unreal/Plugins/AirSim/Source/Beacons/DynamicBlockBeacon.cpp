// Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicBlockBeacon.h"
#include "AirBlueprintLib.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"


// Sets default values
ADynamicBlockBeacon::ADynamicBlockBeacon()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	// set the mesh
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Passive Source Mesh"));
	this->SetRootComponent(Mesh);
	// attach to root component
	//Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh

	static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/DynamicBlockBeacon.DynamicBlockBeacon'"));
	// check if path is valid
	if (loadedMesh.Succeeded())
	{
		// mesh = valid path
		Mesh->SetStaticMesh(loadedMesh.Object);
	}
}

// Called when the game starts or when spawned
void ADynamicBlockBeacon::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ADynamicBlockBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

