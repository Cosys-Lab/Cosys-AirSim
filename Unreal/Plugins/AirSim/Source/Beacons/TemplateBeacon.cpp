// Fill out your copyright notice in the Description page of Project Settings.


#include "TemplateBeacon.h"
#include "AirBlueprintLib.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"


// Sets default values
ATemplateBeacon::ATemplateBeacon()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	// set the mesh
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Beacon mesh"));
	this->SetRootComponent(Mesh);
	// attach to root component
	//Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh

	static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/templateBeacon.templateBeacon'"));
	// check if path is valid
	if (loadedMesh.Succeeded())
	{
		// mesh = valid path
		Mesh->SetStaticMesh(loadedMesh.Object);
	}
}

// Called when the game starts or when spawned
void ATemplateBeacon::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ATemplateBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

