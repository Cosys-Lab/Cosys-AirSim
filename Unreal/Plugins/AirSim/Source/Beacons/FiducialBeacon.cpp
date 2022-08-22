// Fill out your copyright notice in the Description page of Project Settings.


#include "FiducialBeacon.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"

// Sets default values
AFiducialBeacon::AFiducialBeacon()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	FString BeaconName, BeaconType;
	this->GetFName().ToString().Split(TEXT(":"), &BeaconName, &BeaconType);
	int beaconType = FCString::Atoi(*BeaconType);

	// set the mesh
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Beacon mesh"));
	// attach to root component
	Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh

	if (beaconType == 0) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh0(TEXT("StaticMesh'/AirSim/Beacons/fiducial_0.fiducial_0'"));
		// check if path is valid
		if (loadedMesh0.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh0.Object);
		}
	}
	else if (beaconType == 1) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh1(TEXT("StaticMesh'/AirSim/Beacons/fiducial_1.fiducial_1'"));
		// check if path is valid
		if (loadedMesh1.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh1.Object);
		}
	}
	else if (beaconType == 2) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh2(TEXT("StaticMesh'/AirSim/Beacons/fiducial_2.fiducial_2'"));
		// check if path is valid
		if (loadedMesh2.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh2.Object);
		}
	}
	else if (beaconType == 3) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh3(TEXT("StaticMesh'/AirSim/Beacons/fiducial_3.fiducial_3'"));
		// check if path is valid
		if (loadedMesh3.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh3.Object);
		}
	}
	else if (beaconType == 4) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh4(TEXT("StaticMesh'/AirSim/Beacons/fiducial_4.fiducial_4'"));
		// check if path is valid
		if (loadedMesh4.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh4.Object);
		}
	}
	else if (beaconType == 5) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh5(TEXT("StaticMesh'/AirSim/Beacons/fiducial_5.fiducial_5'"));
		// check if path is valid
		if (loadedMesh5.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh5.Object);
		}
	}
	else if (beaconType == 6) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh6(TEXT("StaticMesh'/AirSim/Beacons/fiducial_6.fiducial_6'"));
		// check if path is valid
		if (loadedMesh6.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh6.Object);
		}
	}
	else if (beaconType == 7) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh7(TEXT("StaticMesh'/AirSim/Beacons/fiducial_7.fiducial_7'"));
		// check if path is valid
		if (loadedMesh7.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh7.Object);
		}
	}
	else if (beaconType == 8) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh8(TEXT("StaticMesh'/AirSim/Beacons/fiducial_8.fiducial_8'"));
		// check if path is valid
		if (loadedMesh8.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh8.Object);
		}
	}
	else if (beaconType == 9) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh9(TEXT("StaticMesh'/AirSim/Beacons/fiducial_9.fiducial_9'"));
		// check if path is valid
		if (loadedMesh9.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh9.Object);
		}
	}
	else if (beaconType == 10) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh10(TEXT("StaticMesh'/AirSim/Beacons/fiducial_10.fiducial_10'"));
		// check if path is valid
		if (loadedMesh10.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh10.Object);
		}
	}
	else if (beaconType == 11) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh11(TEXT("StaticMesh'/AirSim/Beacons/fiducial_11.fiducial_11'"));
		// check if path is valid
		if (loadedMesh11.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh11.Object);
		}
	}
	else if (beaconType == 12) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh12(TEXT("StaticMesh'/AirSim/Beacons/fiducial_12.fiducial_12'"));
		// check if path is valid
		if (loadedMesh12.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh12.Object);
		}
	}
	else if (beaconType == 13) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh13(TEXT("StaticMesh'/AirSim/Beacons/fiducial_13.fiducial_13'"));
		// check if path is valid
		if (loadedMesh13.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh13.Object);
		}
	}
	else if (beaconType == 14) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh14(TEXT("StaticMesh'/AirSim/Beacons/fiducial_14.fiducial_14'"));
		// check if path is valid
		if (loadedMesh14.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh14.Object);
		}
	}
	else if (beaconType == 15) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh15(TEXT("StaticMesh'/AirSim/Beacons/fiducial_15.fiducial_15'"));
		// check if path is valid
		if (loadedMesh15.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh15.Object);
		}
	}
	else if (beaconType == 16) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh16(TEXT("StaticMesh'/AirSim/Beacons/fiducial_16.fiducial_16'"));
		// check if path is valid
		if (loadedMesh16.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh16.Object);
		}
	}
	else if (beaconType == 17) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh17(TEXT("StaticMesh'/AirSim/Beacons/fiducial_17.fiducial_17'"));
		// check if path is valid
		if (loadedMesh17.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh17.Object);
		}
	}
	else if (beaconType == 18) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh18(TEXT("StaticMesh'/AirSim/Beacons/fiducial_18.fiducial_18'"));
		// check if path is valid
		if (loadedMesh18.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh18.Object);
		}
	}
	else if (beaconType == 19) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh19(TEXT("StaticMesh'/AirSim/Beacons/fiducial_19.fiducial_19'"));
		// check if path is valid
		if (loadedMesh19.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh19.Object);
		}
	}
	else if (beaconType == 20) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh20(TEXT("StaticMesh'/AirSim/Beacons/fiducial_20.fiducial_20'"));
		// check if path is valid
		if (loadedMesh20.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh20.Object);
		}
	}
	else if (beaconType == 21) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh21(TEXT("StaticMesh'/AirSim/Beacons/fiducial_21.fiducial_21'"));
		// check if path is valid
		if (loadedMesh21.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh21.Object);
		}
	}
	else if (beaconType == 22) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh22(TEXT("StaticMesh'/AirSim/Beacons/fiducial_22.fiducial_22'"));
		// check if path is valid
		if (loadedMesh22.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh22.Object);
		}
	}
	else if (beaconType == 23) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh23(TEXT("StaticMesh'/AirSim/Beacons/fiducial_23.fiducial_23'"));
		// check if path is valid
		if (loadedMesh23.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh23.Object);
		}
	}
	else if (beaconType == 24) {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh24(TEXT("StaticMesh'/AirSim/Beacons/fiducial_24.fiducial_24'"));
		// check if path is valid
		if (loadedMesh24.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh24.Object);
		}
	}
	else {
		static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Beacons/templateBeacon.templateBeacon'"));
		// check if path is valid
		if (loadedMesh.Succeeded())
		{
			// mesh = valid path
			Mesh->SetStaticMesh(loadedMesh.Object);
		}
	}
	//this->Mesh = Mesh;
}

void AFiducialBeacon::setScale(float scale) {
	this->Mesh->SetRelativeScale3D(FVector(scale, scale, scale));
}

// Called when the game starts or when spawned
void AFiducialBeacon::BeginPlay()
{
	Super::BeginPlay();

}

// Called every frame
void AFiducialBeacon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}