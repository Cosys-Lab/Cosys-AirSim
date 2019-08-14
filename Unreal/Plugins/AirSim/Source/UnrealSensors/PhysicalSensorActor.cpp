// Fill out your copyright notice in the Description page of Project Settings.

#include "PhysicalSensorActor.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "AirBlueprintLib.h"

// Sets default values
APhysicalSensorActor::APhysicalSensorActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	static ConstructorHelpers::FObjectFinder<UBlueprint> ItemBlueprint(TEXT("Blueprint'/AirSim/Sensors/Sensor_Plane.Sensor_Plane'"));
	if (ItemBlueprint.Object) {
		sensor_blueprint_ = (UClass*)ItemBlueprint.Object->GeneratedClass;
	}
}

// Called when the game starts or when spawned
void APhysicalSensorActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void APhysicalSensorActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}



