// Copyright (c) Cosys-Lab, University of Antwerp. 2026
#include "Beacons/FiducialBeacon.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"

// Sets default values
#pragma warning(disable: 4883)
AFiducialBeacon::AFiducialBeacon()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	FString BeaconName, BeaconType;
	this->GetFName().ToString().Split(TEXT(":"), &BeaconName, &BeaconType);
	int beaconType = FCString::Atoi(*BeaconType);

	// set the mesh
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Beacon mesh"));
	this->SetRootComponent(Mesh);
	// attach to root component
	//Mesh->SetupAttachment(GetRootComponent());
	// set path for static mesh
	static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh0(TEXT("StaticMesh'/AirSim/CosysAirSim/Meshes/Beacons/Fiducials/SM_Aruco.SM_Aruco'"));

	//
	if (loadedMesh0.Succeeded())
	{
		// mesh = valid path
		Mesh->SetStaticMesh(loadedMesh0.Object);

		// Add material to mesh
		static ConstructorHelpers::FObjectFinder<UMaterial> FoundMaterial(TEXT("Material'/AirSim/CosysAirSim/Materials/Beacons/Fiducials/M_6x6_Default.M_6x6_Default'"));
		if (FoundMaterial.Succeeded()) {
			Material = FoundMaterial.Object;
			DynamicMaterial = UMaterialInstanceDynamic::Create(Material, Mesh);
		}

		//if (beaconType == 0) {
		//	static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_0.T_DICT_6X6_1000_0'"));
		//	if (TextureFinder.Succeeded()) {
		//		Texture = TextureFinder.Object;
		//		DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
		//	}
		//}

		//
		//
		// BEGIN PASTED TEXT

		if (beaconType == 0) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_0.T_DICT_6X6_1000_0'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 1) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_1.T_DICT_6X6_1000_1'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 2) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_2.T_DICT_6X6_1000_2'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 3) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_3.T_DICT_6X6_1000_3'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 4) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_4.T_DICT_6X6_1000_4'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 5) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_5.T_DICT_6X6_1000_5'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 6) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_6.T_DICT_6X6_1000_6'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 7) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_7.T_DICT_6X6_1000_7'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 8) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_8.T_DICT_6X6_1000_8'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 9) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_9.T_DICT_6X6_1000_9'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 10) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_10.T_DICT_6X6_1000_10'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 11) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_11.T_DICT_6X6_1000_11'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 12) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_12.T_DICT_6X6_1000_12'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 13) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_13.T_DICT_6X6_1000_13'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 14) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_14.T_DICT_6X6_1000_14'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 15) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_15.T_DICT_6X6_1000_15'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 16) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_16.T_DICT_6X6_1000_16'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 17) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_17.T_DICT_6X6_1000_17'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 18) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_18.T_DICT_6X6_1000_18'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 19) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_19.T_DICT_6X6_1000_19'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 20) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_20.T_DICT_6X6_1000_20'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 21) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_21.T_DICT_6X6_1000_21'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 22) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_22.T_DICT_6X6_1000_22'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 23) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_23.T_DICT_6X6_1000_23'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 24) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_24.T_DICT_6X6_1000_24'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 25) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_25.T_DICT_6X6_1000_25'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 26) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_26.T_DICT_6X6_1000_26'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 27) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_27.T_DICT_6X6_1000_27'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 28) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_28.T_DICT_6X6_1000_28'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 29) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_29.T_DICT_6X6_1000_29'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 30) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_30.T_DICT_6X6_1000_30'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 31) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_31.T_DICT_6X6_1000_31'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 32) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_32.T_DICT_6X6_1000_32'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 33) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_33.T_DICT_6X6_1000_33'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 34) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_34.T_DICT_6X6_1000_34'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 35) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_35.T_DICT_6X6_1000_35'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 36) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_36.T_DICT_6X6_1000_36'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 37) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_37.T_DICT_6X6_1000_37'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 38) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_38.T_DICT_6X6_1000_38'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 39) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_39.T_DICT_6X6_1000_39'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 40) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_40.T_DICT_6X6_1000_40'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 41) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_41.T_DICT_6X6_1000_41'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 42) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_42.T_DICT_6X6_1000_42'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 43) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_43.T_DICT_6X6_1000_43'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 44) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_44.T_DICT_6X6_1000_44'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 45) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_45.T_DICT_6X6_1000_45'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 46) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_46.T_DICT_6X6_1000_46'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 47) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_47.T_DICT_6X6_1000_47'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 48) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_48.T_DICT_6X6_1000_48'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 49) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_49.T_DICT_6X6_1000_49'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 50) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_50.T_DICT_6X6_1000_50'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 51) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_51.T_DICT_6X6_1000_51'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 52) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_52.T_DICT_6X6_1000_52'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 53) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_53.T_DICT_6X6_1000_53'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 54) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_54.T_DICT_6X6_1000_54'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 55) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_55.T_DICT_6X6_1000_55'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 56) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_56.T_DICT_6X6_1000_56'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 57) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_57.T_DICT_6X6_1000_57'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 58) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_58.T_DICT_6X6_1000_58'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 59) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_59.T_DICT_6X6_1000_59'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 60) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_60.T_DICT_6X6_1000_60'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 61) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_61.T_DICT_6X6_1000_61'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 62) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_62.T_DICT_6X6_1000_62'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 63) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_63.T_DICT_6X6_1000_63'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 64) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_64.T_DICT_6X6_1000_64'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 65) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_65.T_DICT_6X6_1000_65'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 66) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_66.T_DICT_6X6_1000_66'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 67) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_67.T_DICT_6X6_1000_67'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 68) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_68.T_DICT_6X6_1000_68'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 69) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_69.T_DICT_6X6_1000_69'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 70) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_70.T_DICT_6X6_1000_70'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 71) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_71.T_DICT_6X6_1000_71'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 72) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_72.T_DICT_6X6_1000_72'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 73) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_73.T_DICT_6X6_1000_73'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 74) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_74.T_DICT_6X6_1000_74'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 75) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_75.T_DICT_6X6_1000_75'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 76) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_76.T_DICT_6X6_1000_76'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 77) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_77.T_DICT_6X6_1000_77'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 78) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_78.T_DICT_6X6_1000_78'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 79) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_79.T_DICT_6X6_1000_79'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 80) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_80.T_DICT_6X6_1000_80'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 81) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_81.T_DICT_6X6_1000_81'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 82) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_82.T_DICT_6X6_1000_82'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 83) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_83.T_DICT_6X6_1000_83'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 84) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_84.T_DICT_6X6_1000_84'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 85) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_85.T_DICT_6X6_1000_85'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 86) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_86.T_DICT_6X6_1000_86'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 87) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_87.T_DICT_6X6_1000_87'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 88) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_88.T_DICT_6X6_1000_88'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 89) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_89.T_DICT_6X6_1000_89'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 90) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_90.T_DICT_6X6_1000_90'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 91) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_91.T_DICT_6X6_1000_91'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 92) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_92.T_DICT_6X6_1000_92'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 93) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_93.T_DICT_6X6_1000_93'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 94) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_94.T_DICT_6X6_1000_94'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 95) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_95.T_DICT_6X6_1000_95'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 96) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_96.T_DICT_6X6_1000_96'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 97) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_97.T_DICT_6X6_1000_97'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 98) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_98.T_DICT_6X6_1000_98'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 99) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_99.T_DICT_6X6_1000_99'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 100) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_100.T_DICT_6X6_1000_100'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 101) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_101.T_DICT_6X6_1000_101'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 102) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_102.T_DICT_6X6_1000_102'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 103) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_103.T_DICT_6X6_1000_103'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 104) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_104.T_DICT_6X6_1000_104'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 105) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_105.T_DICT_6X6_1000_105'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 106) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_106.T_DICT_6X6_1000_106'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 107) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_107.T_DICT_6X6_1000_107'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 108) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_108.T_DICT_6X6_1000_108'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 109) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_109.T_DICT_6X6_1000_109'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 110) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_110.T_DICT_6X6_1000_110'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 111) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_111.T_DICT_6X6_1000_111'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 112) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_112.T_DICT_6X6_1000_112'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 113) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_113.T_DICT_6X6_1000_113'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 114) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_114.T_DICT_6X6_1000_114'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 115) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_115.T_DICT_6X6_1000_115'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 116) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_116.T_DICT_6X6_1000_116'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 117) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_117.T_DICT_6X6_1000_117'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 118) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_118.T_DICT_6X6_1000_118'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 119) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_119.T_DICT_6X6_1000_119'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 120) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_120.T_DICT_6X6_1000_120'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 121) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_121.T_DICT_6X6_1000_121'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 122) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_122.T_DICT_6X6_1000_122'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 123) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_123.T_DICT_6X6_1000_123'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 124) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_124.T_DICT_6X6_1000_124'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 125) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_125.T_DICT_6X6_1000_125'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 126) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_126.T_DICT_6X6_1000_126'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 127) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_127.T_DICT_6X6_1000_127'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 128) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_128.T_DICT_6X6_1000_128'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 129) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_129.T_DICT_6X6_1000_129'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 130) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_130.T_DICT_6X6_1000_130'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 131) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_131.T_DICT_6X6_1000_131'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 132) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_132.T_DICT_6X6_1000_132'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 133) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_133.T_DICT_6X6_1000_133'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 134) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_134.T_DICT_6X6_1000_134'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 135) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_135.T_DICT_6X6_1000_135'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 136) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_136.T_DICT_6X6_1000_136'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 137) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_137.T_DICT_6X6_1000_137'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 138) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_138.T_DICT_6X6_1000_138'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 139) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_139.T_DICT_6X6_1000_139'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 140) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_140.T_DICT_6X6_1000_140'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 141) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_141.T_DICT_6X6_1000_141'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 142) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_142.T_DICT_6X6_1000_142'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 143) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_143.T_DICT_6X6_1000_143'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 144) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_144.T_DICT_6X6_1000_144'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 145) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_145.T_DICT_6X6_1000_145'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 146) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_146.T_DICT_6X6_1000_146'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 147) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_147.T_DICT_6X6_1000_147'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 148) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_148.T_DICT_6X6_1000_148'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 149) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_149.T_DICT_6X6_1000_149'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 150) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_150.T_DICT_6X6_1000_150'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 151) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_151.T_DICT_6X6_1000_151'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 152) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_152.T_DICT_6X6_1000_152'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 153) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_153.T_DICT_6X6_1000_153'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 154) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_154.T_DICT_6X6_1000_154'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 155) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_155.T_DICT_6X6_1000_155'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 156) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_156.T_DICT_6X6_1000_156'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 157) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_157.T_DICT_6X6_1000_157'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 158) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_158.T_DICT_6X6_1000_158'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 159) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_159.T_DICT_6X6_1000_159'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 160) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_160.T_DICT_6X6_1000_160'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 161) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_161.T_DICT_6X6_1000_161'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 162) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_162.T_DICT_6X6_1000_162'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 163) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_163.T_DICT_6X6_1000_163'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 164) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_164.T_DICT_6X6_1000_164'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 165) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_165.T_DICT_6X6_1000_165'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 166) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_166.T_DICT_6X6_1000_166'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 167) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_167.T_DICT_6X6_1000_167'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 168) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_168.T_DICT_6X6_1000_168'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 169) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_169.T_DICT_6X6_1000_169'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 170) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_170.T_DICT_6X6_1000_170'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 171) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_171.T_DICT_6X6_1000_171'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 172) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_172.T_DICT_6X6_1000_172'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 173) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_173.T_DICT_6X6_1000_173'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 174) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_174.T_DICT_6X6_1000_174'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 175) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_175.T_DICT_6X6_1000_175'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 176) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_176.T_DICT_6X6_1000_176'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 177) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_177.T_DICT_6X6_1000_177'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 178) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_178.T_DICT_6X6_1000_178'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 179) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_179.T_DICT_6X6_1000_179'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 180) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_180.T_DICT_6X6_1000_180'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 181) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_181.T_DICT_6X6_1000_181'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 182) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_182.T_DICT_6X6_1000_182'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 183) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_183.T_DICT_6X6_1000_183'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 184) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_184.T_DICT_6X6_1000_184'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 185) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_185.T_DICT_6X6_1000_185'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 186) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_186.T_DICT_6X6_1000_186'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 187) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_187.T_DICT_6X6_1000_187'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 188) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_188.T_DICT_6X6_1000_188'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 189) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_189.T_DICT_6X6_1000_189'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 190) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_190.T_DICT_6X6_1000_190'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 191) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_191.T_DICT_6X6_1000_191'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 192) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_192.T_DICT_6X6_1000_192'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 193) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_193.T_DICT_6X6_1000_193'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 194) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_194.T_DICT_6X6_1000_194'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 195) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_195.T_DICT_6X6_1000_195'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 196) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_196.T_DICT_6X6_1000_196'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 197) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_197.T_DICT_6X6_1000_197'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 198) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_198.T_DICT_6X6_1000_198'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 199) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_199.T_DICT_6X6_1000_199'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 200) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_200.T_DICT_6X6_1000_200'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 201) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_201.T_DICT_6X6_1000_201'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 202) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_202.T_DICT_6X6_1000_202'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 203) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_203.T_DICT_6X6_1000_203'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 204) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_204.T_DICT_6X6_1000_204'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 205) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_205.T_DICT_6X6_1000_205'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 206) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_206.T_DICT_6X6_1000_206'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 207) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_207.T_DICT_6X6_1000_207'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 208) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_208.T_DICT_6X6_1000_208'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 209) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_209.T_DICT_6X6_1000_209'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 210) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_210.T_DICT_6X6_1000_210'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 211) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_211.T_DICT_6X6_1000_211'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 212) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_212.T_DICT_6X6_1000_212'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 213) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_213.T_DICT_6X6_1000_213'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 214) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_214.T_DICT_6X6_1000_214'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 215) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_215.T_DICT_6X6_1000_215'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 216) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_216.T_DICT_6X6_1000_216'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 217) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_217.T_DICT_6X6_1000_217'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 218) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_218.T_DICT_6X6_1000_218'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 219) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_219.T_DICT_6X6_1000_219'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 220) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_220.T_DICT_6X6_1000_220'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 221) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_221.T_DICT_6X6_1000_221'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 222) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_222.T_DICT_6X6_1000_222'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 223) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_223.T_DICT_6X6_1000_223'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 224) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_224.T_DICT_6X6_1000_224'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 225) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_225.T_DICT_6X6_1000_225'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 226) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_226.T_DICT_6X6_1000_226'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 227) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_227.T_DICT_6X6_1000_227'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 228) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_228.T_DICT_6X6_1000_228'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 229) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_229.T_DICT_6X6_1000_229'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 230) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_230.T_DICT_6X6_1000_230'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 231) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_231.T_DICT_6X6_1000_231'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 232) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_232.T_DICT_6X6_1000_232'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 233) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_233.T_DICT_6X6_1000_233'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 234) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_234.T_DICT_6X6_1000_234'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 235) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_235.T_DICT_6X6_1000_235'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 236) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_236.T_DICT_6X6_1000_236'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 237) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_237.T_DICT_6X6_1000_237'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 238) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_238.T_DICT_6X6_1000_238'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 239) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_239.T_DICT_6X6_1000_239'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 240) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_240.T_DICT_6X6_1000_240'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 241) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_241.T_DICT_6X6_1000_241'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 242) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_242.T_DICT_6X6_1000_242'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 243) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_243.T_DICT_6X6_1000_243'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 244) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_244.T_DICT_6X6_1000_244'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 245) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_245.T_DICT_6X6_1000_245'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 246) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_246.T_DICT_6X6_1000_246'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 247) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_247.T_DICT_6X6_1000_247'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 248) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_248.T_DICT_6X6_1000_248'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 249) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_249.T_DICT_6X6_1000_249'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 250) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_250.T_DICT_6X6_1000_250'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 251) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_251.T_DICT_6X6_1000_251'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 252) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_252.T_DICT_6X6_1000_252'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 253) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_253.T_DICT_6X6_1000_253'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 254) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_254.T_DICT_6X6_1000_254'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 255) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_255.T_DICT_6X6_1000_255'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 256) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_256.T_DICT_6X6_1000_256'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 257) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_257.T_DICT_6X6_1000_257'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 258) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_258.T_DICT_6X6_1000_258'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 259) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_259.T_DICT_6X6_1000_259'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 260) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_260.T_DICT_6X6_1000_260'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 261) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_261.T_DICT_6X6_1000_261'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 262) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_262.T_DICT_6X6_1000_262'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 263) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_263.T_DICT_6X6_1000_263'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 264) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_264.T_DICT_6X6_1000_264'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 265) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_265.T_DICT_6X6_1000_265'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 266) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_266.T_DICT_6X6_1000_266'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 267) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_267.T_DICT_6X6_1000_267'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 268) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_268.T_DICT_6X6_1000_268'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 269) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_269.T_DICT_6X6_1000_269'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 270) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_270.T_DICT_6X6_1000_270'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 271) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_271.T_DICT_6X6_1000_271'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 272) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_272.T_DICT_6X6_1000_272'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 273) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_273.T_DICT_6X6_1000_273'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 274) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_274.T_DICT_6X6_1000_274'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 275) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_275.T_DICT_6X6_1000_275'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 276) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_276.T_DICT_6X6_1000_276'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 277) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_277.T_DICT_6X6_1000_277'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 278) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_278.T_DICT_6X6_1000_278'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 279) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_279.T_DICT_6X6_1000_279'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 280) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_280.T_DICT_6X6_1000_280'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 281) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_281.T_DICT_6X6_1000_281'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 282) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_282.T_DICT_6X6_1000_282'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 283) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_283.T_DICT_6X6_1000_283'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 284) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_284.T_DICT_6X6_1000_284'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 285) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_285.T_DICT_6X6_1000_285'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 286) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_286.T_DICT_6X6_1000_286'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 287) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_287.T_DICT_6X6_1000_287'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 288) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_288.T_DICT_6X6_1000_288'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 289) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_289.T_DICT_6X6_1000_289'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 290) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_290.T_DICT_6X6_1000_290'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 291) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_291.T_DICT_6X6_1000_291'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 292) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_292.T_DICT_6X6_1000_292'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 293) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_293.T_DICT_6X6_1000_293'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 294) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_294.T_DICT_6X6_1000_294'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 295) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_295.T_DICT_6X6_1000_295'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 296) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_296.T_DICT_6X6_1000_296'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 297) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_297.T_DICT_6X6_1000_297'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 298) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_298.T_DICT_6X6_1000_298'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 299) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_299.T_DICT_6X6_1000_299'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 300) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_300.T_DICT_6X6_1000_300'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 301) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_301.T_DICT_6X6_1000_301'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 302) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_302.T_DICT_6X6_1000_302'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 303) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_303.T_DICT_6X6_1000_303'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 304) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_304.T_DICT_6X6_1000_304'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 305) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_305.T_DICT_6X6_1000_305'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 306) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_306.T_DICT_6X6_1000_306'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 307) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_307.T_DICT_6X6_1000_307'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 308) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_308.T_DICT_6X6_1000_308'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 309) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_309.T_DICT_6X6_1000_309'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 310) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_310.T_DICT_6X6_1000_310'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 311) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_311.T_DICT_6X6_1000_311'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 312) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_312.T_DICT_6X6_1000_312'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 313) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_313.T_DICT_6X6_1000_313'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 314) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_314.T_DICT_6X6_1000_314'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 315) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_315.T_DICT_6X6_1000_315'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 316) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_316.T_DICT_6X6_1000_316'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 317) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_317.T_DICT_6X6_1000_317'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 318) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_318.T_DICT_6X6_1000_318'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 319) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_319.T_DICT_6X6_1000_319'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 320) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_320.T_DICT_6X6_1000_320'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 321) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_321.T_DICT_6X6_1000_321'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 322) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_322.T_DICT_6X6_1000_322'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 323) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_323.T_DICT_6X6_1000_323'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 324) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_324.T_DICT_6X6_1000_324'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 325) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_325.T_DICT_6X6_1000_325'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 326) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_326.T_DICT_6X6_1000_326'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 327) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_327.T_DICT_6X6_1000_327'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 328) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_328.T_DICT_6X6_1000_328'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 329) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_329.T_DICT_6X6_1000_329'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 330) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_330.T_DICT_6X6_1000_330'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 331) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_331.T_DICT_6X6_1000_331'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 332) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_332.T_DICT_6X6_1000_332'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 333) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_333.T_DICT_6X6_1000_333'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 334) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_334.T_DICT_6X6_1000_334'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 335) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_335.T_DICT_6X6_1000_335'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 336) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_336.T_DICT_6X6_1000_336'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 337) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_337.T_DICT_6X6_1000_337'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 338) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_338.T_DICT_6X6_1000_338'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 339) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_339.T_DICT_6X6_1000_339'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 340) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_340.T_DICT_6X6_1000_340'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 341) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_341.T_DICT_6X6_1000_341'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 342) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_342.T_DICT_6X6_1000_342'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 343) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_343.T_DICT_6X6_1000_343'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 344) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_344.T_DICT_6X6_1000_344'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 345) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_345.T_DICT_6X6_1000_345'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 346) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_346.T_DICT_6X6_1000_346'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 347) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_347.T_DICT_6X6_1000_347'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 348) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_348.T_DICT_6X6_1000_348'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 349) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_349.T_DICT_6X6_1000_349'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 350) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_350.T_DICT_6X6_1000_350'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 351) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_351.T_DICT_6X6_1000_351'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 352) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_352.T_DICT_6X6_1000_352'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 353) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_353.T_DICT_6X6_1000_353'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 354) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_354.T_DICT_6X6_1000_354'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 355) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_355.T_DICT_6X6_1000_355'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 356) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_356.T_DICT_6X6_1000_356'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 357) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_357.T_DICT_6X6_1000_357'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 358) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_358.T_DICT_6X6_1000_358'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 359) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_359.T_DICT_6X6_1000_359'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 360) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_360.T_DICT_6X6_1000_360'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 361) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_361.T_DICT_6X6_1000_361'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 362) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_362.T_DICT_6X6_1000_362'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 363) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_363.T_DICT_6X6_1000_363'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 364) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_364.T_DICT_6X6_1000_364'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 365) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_365.T_DICT_6X6_1000_365'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 366) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_366.T_DICT_6X6_1000_366'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 367) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_367.T_DICT_6X6_1000_367'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 368) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_368.T_DICT_6X6_1000_368'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 369) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_369.T_DICT_6X6_1000_369'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 370) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_370.T_DICT_6X6_1000_370'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 371) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_371.T_DICT_6X6_1000_371'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 372) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_372.T_DICT_6X6_1000_372'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 373) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_373.T_DICT_6X6_1000_373'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 374) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_374.T_DICT_6X6_1000_374'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 375) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_375.T_DICT_6X6_1000_375'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 376) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_376.T_DICT_6X6_1000_376'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 377) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_377.T_DICT_6X6_1000_377'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 378) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_378.T_DICT_6X6_1000_378'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 379) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_379.T_DICT_6X6_1000_379'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 380) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_380.T_DICT_6X6_1000_380'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 381) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_381.T_DICT_6X6_1000_381'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 382) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_382.T_DICT_6X6_1000_382'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 383) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_383.T_DICT_6X6_1000_383'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 384) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_384.T_DICT_6X6_1000_384'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 385) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_385.T_DICT_6X6_1000_385'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 386) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_386.T_DICT_6X6_1000_386'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 387) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_387.T_DICT_6X6_1000_387'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 388) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_388.T_DICT_6X6_1000_388'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 389) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_389.T_DICT_6X6_1000_389'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 390) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_390.T_DICT_6X6_1000_390'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 391) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_391.T_DICT_6X6_1000_391'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 392) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_392.T_DICT_6X6_1000_392'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 393) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_393.T_DICT_6X6_1000_393'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 394) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_394.T_DICT_6X6_1000_394'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 395) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_395.T_DICT_6X6_1000_395'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 396) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_396.T_DICT_6X6_1000_396'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 397) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_397.T_DICT_6X6_1000_397'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 398) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_398.T_DICT_6X6_1000_398'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 399) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_399.T_DICT_6X6_1000_399'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 400) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_400.T_DICT_6X6_1000_400'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 401) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_401.T_DICT_6X6_1000_401'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 402) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_402.T_DICT_6X6_1000_402'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 403) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_403.T_DICT_6X6_1000_403'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 404) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_404.T_DICT_6X6_1000_404'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 405) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_405.T_DICT_6X6_1000_405'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 406) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_406.T_DICT_6X6_1000_406'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 407) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_407.T_DICT_6X6_1000_407'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 408) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_408.T_DICT_6X6_1000_408'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 409) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_409.T_DICT_6X6_1000_409'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 410) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_410.T_DICT_6X6_1000_410'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 411) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_411.T_DICT_6X6_1000_411'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 412) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_412.T_DICT_6X6_1000_412'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 413) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_413.T_DICT_6X6_1000_413'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 414) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_414.T_DICT_6X6_1000_414'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 415) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_415.T_DICT_6X6_1000_415'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 416) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_416.T_DICT_6X6_1000_416'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 417) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_417.T_DICT_6X6_1000_417'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 418) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_418.T_DICT_6X6_1000_418'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 419) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_419.T_DICT_6X6_1000_419'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 420) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_420.T_DICT_6X6_1000_420'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 421) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_421.T_DICT_6X6_1000_421'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 422) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_422.T_DICT_6X6_1000_422'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 423) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_423.T_DICT_6X6_1000_423'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 424) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_424.T_DICT_6X6_1000_424'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 425) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_425.T_DICT_6X6_1000_425'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 426) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_426.T_DICT_6X6_1000_426'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 427) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_427.T_DICT_6X6_1000_427'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 428) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_428.T_DICT_6X6_1000_428'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 429) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_429.T_DICT_6X6_1000_429'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 430) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_430.T_DICT_6X6_1000_430'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 431) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_431.T_DICT_6X6_1000_431'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 432) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_432.T_DICT_6X6_1000_432'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 433) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_433.T_DICT_6X6_1000_433'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 434) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_434.T_DICT_6X6_1000_434'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 435) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_435.T_DICT_6X6_1000_435'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 436) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_436.T_DICT_6X6_1000_436'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 437) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_437.T_DICT_6X6_1000_437'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 438) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_438.T_DICT_6X6_1000_438'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 439) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_439.T_DICT_6X6_1000_439'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 440) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_440.T_DICT_6X6_1000_440'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 441) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_441.T_DICT_6X6_1000_441'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 442) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_442.T_DICT_6X6_1000_442'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 443) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_443.T_DICT_6X6_1000_443'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 444) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_444.T_DICT_6X6_1000_444'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 445) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_445.T_DICT_6X6_1000_445'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 446) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_446.T_DICT_6X6_1000_446'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 447) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_447.T_DICT_6X6_1000_447'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 448) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_448.T_DICT_6X6_1000_448'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 449) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_449.T_DICT_6X6_1000_449'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 450) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_450.T_DICT_6X6_1000_450'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 451) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_451.T_DICT_6X6_1000_451'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 452) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_452.T_DICT_6X6_1000_452'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 453) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_453.T_DICT_6X6_1000_453'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 454) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_454.T_DICT_6X6_1000_454'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 455) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_455.T_DICT_6X6_1000_455'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 456) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_456.T_DICT_6X6_1000_456'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 457) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_457.T_DICT_6X6_1000_457'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 458) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_458.T_DICT_6X6_1000_458'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 459) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_459.T_DICT_6X6_1000_459'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 460) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_460.T_DICT_6X6_1000_460'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 461) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_461.T_DICT_6X6_1000_461'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 462) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_462.T_DICT_6X6_1000_462'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 463) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_463.T_DICT_6X6_1000_463'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 464) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_464.T_DICT_6X6_1000_464'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 465) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_465.T_DICT_6X6_1000_465'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 466) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_466.T_DICT_6X6_1000_466'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 467) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_467.T_DICT_6X6_1000_467'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 468) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_468.T_DICT_6X6_1000_468'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 469) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_469.T_DICT_6X6_1000_469'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 470) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_470.T_DICT_6X6_1000_470'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 471) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_471.T_DICT_6X6_1000_471'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 472) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_472.T_DICT_6X6_1000_472'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 473) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_473.T_DICT_6X6_1000_473'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 474) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_474.T_DICT_6X6_1000_474'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 475) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_475.T_DICT_6X6_1000_475'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 476) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_476.T_DICT_6X6_1000_476'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 477) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_477.T_DICT_6X6_1000_477'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 478) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_478.T_DICT_6X6_1000_478'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 479) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_479.T_DICT_6X6_1000_479'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 480) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_480.T_DICT_6X6_1000_480'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 481) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_481.T_DICT_6X6_1000_481'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 482) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_482.T_DICT_6X6_1000_482'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 483) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_483.T_DICT_6X6_1000_483'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 484) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_484.T_DICT_6X6_1000_484'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 485) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_485.T_DICT_6X6_1000_485'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 486) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_486.T_DICT_6X6_1000_486'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 487) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_487.T_DICT_6X6_1000_487'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 488) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_488.T_DICT_6X6_1000_488'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 489) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_489.T_DICT_6X6_1000_489'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 490) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_490.T_DICT_6X6_1000_490'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 491) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_491.T_DICT_6X6_1000_491'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 492) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_492.T_DICT_6X6_1000_492'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 493) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_493.T_DICT_6X6_1000_493'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 494) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_494.T_DICT_6X6_1000_494'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 495) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_495.T_DICT_6X6_1000_495'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 496) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_496.T_DICT_6X6_1000_496'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 497) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_497.T_DICT_6X6_1000_497'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 498) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_498.T_DICT_6X6_1000_498'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 499) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_499.T_DICT_6X6_1000_499'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 500) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_500.T_DICT_6X6_1000_500'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 501) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_501.T_DICT_6X6_1000_501'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 502) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_502.T_DICT_6X6_1000_502'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 503) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_503.T_DICT_6X6_1000_503'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 504) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_504.T_DICT_6X6_1000_504'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 505) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_505.T_DICT_6X6_1000_505'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 506) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_506.T_DICT_6X6_1000_506'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 507) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_507.T_DICT_6X6_1000_507'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 508) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_508.T_DICT_6X6_1000_508'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 509) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_509.T_DICT_6X6_1000_509'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 510) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_510.T_DICT_6X6_1000_510'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 511) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_511.T_DICT_6X6_1000_511'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 512) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_512.T_DICT_6X6_1000_512'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 513) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_513.T_DICT_6X6_1000_513'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 514) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_514.T_DICT_6X6_1000_514'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 515) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_515.T_DICT_6X6_1000_515'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 516) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_516.T_DICT_6X6_1000_516'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 517) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_517.T_DICT_6X6_1000_517'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 518) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_518.T_DICT_6X6_1000_518'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 519) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_519.T_DICT_6X6_1000_519'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 520) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_520.T_DICT_6X6_1000_520'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 521) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_521.T_DICT_6X6_1000_521'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 522) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_522.T_DICT_6X6_1000_522'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 523) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_523.T_DICT_6X6_1000_523'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 524) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_524.T_DICT_6X6_1000_524'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 525) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_525.T_DICT_6X6_1000_525'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 526) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_526.T_DICT_6X6_1000_526'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 527) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_527.T_DICT_6X6_1000_527'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 528) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_528.T_DICT_6X6_1000_528'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 529) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_529.T_DICT_6X6_1000_529'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 530) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_530.T_DICT_6X6_1000_530'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 531) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_531.T_DICT_6X6_1000_531'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 532) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_532.T_DICT_6X6_1000_532'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 533) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_533.T_DICT_6X6_1000_533'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 534) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_534.T_DICT_6X6_1000_534'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 535) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_535.T_DICT_6X6_1000_535'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 536) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_536.T_DICT_6X6_1000_536'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 537) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_537.T_DICT_6X6_1000_537'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 538) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_538.T_DICT_6X6_1000_538'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 539) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_539.T_DICT_6X6_1000_539'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 540) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_540.T_DICT_6X6_1000_540'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 541) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_541.T_DICT_6X6_1000_541'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 542) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_542.T_DICT_6X6_1000_542'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 543) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_543.T_DICT_6X6_1000_543'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 544) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_544.T_DICT_6X6_1000_544'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 545) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_545.T_DICT_6X6_1000_545'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 546) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_546.T_DICT_6X6_1000_546'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 547) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_547.T_DICT_6X6_1000_547'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 548) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_548.T_DICT_6X6_1000_548'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 549) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_549.T_DICT_6X6_1000_549'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 550) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_550.T_DICT_6X6_1000_550'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 551) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_551.T_DICT_6X6_1000_551'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 552) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_552.T_DICT_6X6_1000_552'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 553) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_553.T_DICT_6X6_1000_553'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 554) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_554.T_DICT_6X6_1000_554'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 555) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_555.T_DICT_6X6_1000_555'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 556) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_556.T_DICT_6X6_1000_556'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 557) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_557.T_DICT_6X6_1000_557'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 558) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_558.T_DICT_6X6_1000_558'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 559) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_559.T_DICT_6X6_1000_559'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 560) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_560.T_DICT_6X6_1000_560'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 561) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_561.T_DICT_6X6_1000_561'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 562) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_562.T_DICT_6X6_1000_562'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 563) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_563.T_DICT_6X6_1000_563'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 564) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_564.T_DICT_6X6_1000_564'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 565) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_565.T_DICT_6X6_1000_565'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 566) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_566.T_DICT_6X6_1000_566'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 567) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_567.T_DICT_6X6_1000_567'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 568) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_568.T_DICT_6X6_1000_568'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 569) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_569.T_DICT_6X6_1000_569'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 570) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_570.T_DICT_6X6_1000_570'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 571) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_571.T_DICT_6X6_1000_571'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 572) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_572.T_DICT_6X6_1000_572'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 573) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_573.T_DICT_6X6_1000_573'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 574) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_574.T_DICT_6X6_1000_574'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 575) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_575.T_DICT_6X6_1000_575'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 576) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_576.T_DICT_6X6_1000_576'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 577) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_577.T_DICT_6X6_1000_577'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 578) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_578.T_DICT_6X6_1000_578'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 579) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_579.T_DICT_6X6_1000_579'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 580) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_580.T_DICT_6X6_1000_580'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 581) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_581.T_DICT_6X6_1000_581'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 582) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_582.T_DICT_6X6_1000_582'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 583) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_583.T_DICT_6X6_1000_583'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 584) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_584.T_DICT_6X6_1000_584'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 585) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_585.T_DICT_6X6_1000_585'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 586) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_586.T_DICT_6X6_1000_586'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 587) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_587.T_DICT_6X6_1000_587'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 588) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_588.T_DICT_6X6_1000_588'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 589) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_589.T_DICT_6X6_1000_589'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 590) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_590.T_DICT_6X6_1000_590'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 591) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_591.T_DICT_6X6_1000_591'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 592) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_592.T_DICT_6X6_1000_592'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 593) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_593.T_DICT_6X6_1000_593'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 594) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_594.T_DICT_6X6_1000_594'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 595) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_595.T_DICT_6X6_1000_595'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 596) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_596.T_DICT_6X6_1000_596'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 597) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_597.T_DICT_6X6_1000_597'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 598) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_598.T_DICT_6X6_1000_598'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 599) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_599.T_DICT_6X6_1000_599'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 600) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_600.T_DICT_6X6_1000_600'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 601) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_601.T_DICT_6X6_1000_601'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 602) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_602.T_DICT_6X6_1000_602'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 603) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_603.T_DICT_6X6_1000_603'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 604) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_604.T_DICT_6X6_1000_604'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 605) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_605.T_DICT_6X6_1000_605'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 606) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_606.T_DICT_6X6_1000_606'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 607) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_607.T_DICT_6X6_1000_607'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 608) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_608.T_DICT_6X6_1000_608'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 609) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_609.T_DICT_6X6_1000_609'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 610) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_610.T_DICT_6X6_1000_610'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 611) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_611.T_DICT_6X6_1000_611'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 612) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_612.T_DICT_6X6_1000_612'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 613) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_613.T_DICT_6X6_1000_613'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 614) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_614.T_DICT_6X6_1000_614'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 615) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_615.T_DICT_6X6_1000_615'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 616) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_616.T_DICT_6X6_1000_616'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 617) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_617.T_DICT_6X6_1000_617'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 618) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_618.T_DICT_6X6_1000_618'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 619) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_619.T_DICT_6X6_1000_619'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 620) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_620.T_DICT_6X6_1000_620'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 621) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_621.T_DICT_6X6_1000_621'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 622) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_622.T_DICT_6X6_1000_622'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 623) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_623.T_DICT_6X6_1000_623'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 624) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_624.T_DICT_6X6_1000_624'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 625) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_625.T_DICT_6X6_1000_625'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 626) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_626.T_DICT_6X6_1000_626'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 627) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_627.T_DICT_6X6_1000_627'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 628) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_628.T_DICT_6X6_1000_628'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 629) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_629.T_DICT_6X6_1000_629'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 630) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_630.T_DICT_6X6_1000_630'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 631) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_631.T_DICT_6X6_1000_631'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 632) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_632.T_DICT_6X6_1000_632'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 633) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_633.T_DICT_6X6_1000_633'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 634) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_634.T_DICT_6X6_1000_634'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 635) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_635.T_DICT_6X6_1000_635'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 636) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_636.T_DICT_6X6_1000_636'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 637) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_637.T_DICT_6X6_1000_637'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 638) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_638.T_DICT_6X6_1000_638'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 639) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_639.T_DICT_6X6_1000_639'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 640) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_640.T_DICT_6X6_1000_640'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 641) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_641.T_DICT_6X6_1000_641'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 642) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_642.T_DICT_6X6_1000_642'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 643) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_643.T_DICT_6X6_1000_643'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 644) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_644.T_DICT_6X6_1000_644'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 645) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_645.T_DICT_6X6_1000_645'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 646) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_646.T_DICT_6X6_1000_646'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 647) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_647.T_DICT_6X6_1000_647'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 648) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_648.T_DICT_6X6_1000_648'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 649) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_649.T_DICT_6X6_1000_649'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 650) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_650.T_DICT_6X6_1000_650'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 651) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_651.T_DICT_6X6_1000_651'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 652) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_652.T_DICT_6X6_1000_652'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 653) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_653.T_DICT_6X6_1000_653'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 654) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_654.T_DICT_6X6_1000_654'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 655) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_655.T_DICT_6X6_1000_655'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 656) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_656.T_DICT_6X6_1000_656'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 657) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_657.T_DICT_6X6_1000_657'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 658) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_658.T_DICT_6X6_1000_658'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 659) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_659.T_DICT_6X6_1000_659'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 660) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_660.T_DICT_6X6_1000_660'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 661) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_661.T_DICT_6X6_1000_661'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 662) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_662.T_DICT_6X6_1000_662'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 663) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_663.T_DICT_6X6_1000_663'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 664) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_664.T_DICT_6X6_1000_664'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 665) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_665.T_DICT_6X6_1000_665'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 666) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_666.T_DICT_6X6_1000_666'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 667) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_667.T_DICT_6X6_1000_667'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 668) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_668.T_DICT_6X6_1000_668'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 669) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_669.T_DICT_6X6_1000_669'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 670) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_670.T_DICT_6X6_1000_670'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 671) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_671.T_DICT_6X6_1000_671'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 672) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_672.T_DICT_6X6_1000_672'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 673) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_673.T_DICT_6X6_1000_673'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 674) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_674.T_DICT_6X6_1000_674'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 675) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_675.T_DICT_6X6_1000_675'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 676) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_676.T_DICT_6X6_1000_676'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 677) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_677.T_DICT_6X6_1000_677'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 678) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_678.T_DICT_6X6_1000_678'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 679) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_679.T_DICT_6X6_1000_679'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 680) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_680.T_DICT_6X6_1000_680'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 681) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_681.T_DICT_6X6_1000_681'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 682) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_682.T_DICT_6X6_1000_682'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 683) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_683.T_DICT_6X6_1000_683'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 684) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_684.T_DICT_6X6_1000_684'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 685) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_685.T_DICT_6X6_1000_685'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 686) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_686.T_DICT_6X6_1000_686'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 687) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_687.T_DICT_6X6_1000_687'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 688) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_688.T_DICT_6X6_1000_688'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 689) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_689.T_DICT_6X6_1000_689'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 690) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_690.T_DICT_6X6_1000_690'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 691) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_691.T_DICT_6X6_1000_691'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 692) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_692.T_DICT_6X6_1000_692'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 693) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_693.T_DICT_6X6_1000_693'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 694) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_694.T_DICT_6X6_1000_694'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 695) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_695.T_DICT_6X6_1000_695'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 696) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_696.T_DICT_6X6_1000_696'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 697) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_697.T_DICT_6X6_1000_697'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 698) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_698.T_DICT_6X6_1000_698'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 699) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_699.T_DICT_6X6_1000_699'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 700) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_700.T_DICT_6X6_1000_700'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 701) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_701.T_DICT_6X6_1000_701'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 702) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_702.T_DICT_6X6_1000_702'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 703) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_703.T_DICT_6X6_1000_703'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 704) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_704.T_DICT_6X6_1000_704'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 705) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_705.T_DICT_6X6_1000_705'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 706) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_706.T_DICT_6X6_1000_706'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 707) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_707.T_DICT_6X6_1000_707'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 708) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_708.T_DICT_6X6_1000_708'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 709) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_709.T_DICT_6X6_1000_709'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 710) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_710.T_DICT_6X6_1000_710'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 711) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_711.T_DICT_6X6_1000_711'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 712) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_712.T_DICT_6X6_1000_712'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 713) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_713.T_DICT_6X6_1000_713'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 714) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_714.T_DICT_6X6_1000_714'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 715) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_715.T_DICT_6X6_1000_715'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 716) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_716.T_DICT_6X6_1000_716'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 717) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_717.T_DICT_6X6_1000_717'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 718) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_718.T_DICT_6X6_1000_718'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 719) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_719.T_DICT_6X6_1000_719'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 720) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_720.T_DICT_6X6_1000_720'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 721) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_721.T_DICT_6X6_1000_721'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 722) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_722.T_DICT_6X6_1000_722'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 723) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_723.T_DICT_6X6_1000_723'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 724) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_724.T_DICT_6X6_1000_724'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 725) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_725.T_DICT_6X6_1000_725'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 726) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_726.T_DICT_6X6_1000_726'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 727) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_727.T_DICT_6X6_1000_727'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 728) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_728.T_DICT_6X6_1000_728'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 729) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_729.T_DICT_6X6_1000_729'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 730) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_730.T_DICT_6X6_1000_730'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 731) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_731.T_DICT_6X6_1000_731'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 732) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_732.T_DICT_6X6_1000_732'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 733) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_733.T_DICT_6X6_1000_733'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 734) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_734.T_DICT_6X6_1000_734'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 735) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_735.T_DICT_6X6_1000_735'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 736) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_736.T_DICT_6X6_1000_736'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 737) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_737.T_DICT_6X6_1000_737'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 738) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_738.T_DICT_6X6_1000_738'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 739) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_739.T_DICT_6X6_1000_739'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 740) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_740.T_DICT_6X6_1000_740'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 741) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_741.T_DICT_6X6_1000_741'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 742) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_742.T_DICT_6X6_1000_742'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 743) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_743.T_DICT_6X6_1000_743'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 744) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_744.T_DICT_6X6_1000_744'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 745) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_745.T_DICT_6X6_1000_745'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 746) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_746.T_DICT_6X6_1000_746'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 747) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_747.T_DICT_6X6_1000_747'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 748) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_748.T_DICT_6X6_1000_748'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 749) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_749.T_DICT_6X6_1000_749'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 750) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_750.T_DICT_6X6_1000_750'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 751) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_751.T_DICT_6X6_1000_751'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 752) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_752.T_DICT_6X6_1000_752'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 753) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_753.T_DICT_6X6_1000_753'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 754) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_754.T_DICT_6X6_1000_754'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 755) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_755.T_DICT_6X6_1000_755'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 756) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_756.T_DICT_6X6_1000_756'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 757) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_757.T_DICT_6X6_1000_757'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 758) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_758.T_DICT_6X6_1000_758'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 759) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_759.T_DICT_6X6_1000_759'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 760) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_760.T_DICT_6X6_1000_760'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 761) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_761.T_DICT_6X6_1000_761'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 762) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_762.T_DICT_6X6_1000_762'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 763) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_763.T_DICT_6X6_1000_763'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 764) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_764.T_DICT_6X6_1000_764'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 765) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_765.T_DICT_6X6_1000_765'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 766) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_766.T_DICT_6X6_1000_766'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 767) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_767.T_DICT_6X6_1000_767'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 768) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_768.T_DICT_6X6_1000_768'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 769) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_769.T_DICT_6X6_1000_769'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 770) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_770.T_DICT_6X6_1000_770'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 771) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_771.T_DICT_6X6_1000_771'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 772) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_772.T_DICT_6X6_1000_772'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 773) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_773.T_DICT_6X6_1000_773'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 774) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_774.T_DICT_6X6_1000_774'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 775) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_775.T_DICT_6X6_1000_775'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 776) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_776.T_DICT_6X6_1000_776'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 777) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_777.T_DICT_6X6_1000_777'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 778) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_778.T_DICT_6X6_1000_778'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 779) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_779.T_DICT_6X6_1000_779'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 780) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_780.T_DICT_6X6_1000_780'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 781) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_781.T_DICT_6X6_1000_781'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 782) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_782.T_DICT_6X6_1000_782'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 783) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_783.T_DICT_6X6_1000_783'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 784) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_784.T_DICT_6X6_1000_784'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 785) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_785.T_DICT_6X6_1000_785'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 786) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_786.T_DICT_6X6_1000_786'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 787) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_787.T_DICT_6X6_1000_787'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 788) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_788.T_DICT_6X6_1000_788'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 789) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_789.T_DICT_6X6_1000_789'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 790) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_790.T_DICT_6X6_1000_790'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 791) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_791.T_DICT_6X6_1000_791'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 792) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_792.T_DICT_6X6_1000_792'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 793) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_793.T_DICT_6X6_1000_793'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 794) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_794.T_DICT_6X6_1000_794'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 795) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_795.T_DICT_6X6_1000_795'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 796) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_796.T_DICT_6X6_1000_796'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 797) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_797.T_DICT_6X6_1000_797'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 798) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_798.T_DICT_6X6_1000_798'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 799) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_799.T_DICT_6X6_1000_799'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 800) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_800.T_DICT_6X6_1000_800'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 801) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_801.T_DICT_6X6_1000_801'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 802) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_802.T_DICT_6X6_1000_802'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 803) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_803.T_DICT_6X6_1000_803'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 804) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_804.T_DICT_6X6_1000_804'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 805) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_805.T_DICT_6X6_1000_805'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 806) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_806.T_DICT_6X6_1000_806'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 807) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_807.T_DICT_6X6_1000_807'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 808) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_808.T_DICT_6X6_1000_808'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 809) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_809.T_DICT_6X6_1000_809'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 810) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_810.T_DICT_6X6_1000_810'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 811) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_811.T_DICT_6X6_1000_811'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 812) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_812.T_DICT_6X6_1000_812'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 813) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_813.T_DICT_6X6_1000_813'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 814) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_814.T_DICT_6X6_1000_814'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 815) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_815.T_DICT_6X6_1000_815'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 816) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_816.T_DICT_6X6_1000_816'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 817) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_817.T_DICT_6X6_1000_817'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 818) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_818.T_DICT_6X6_1000_818'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 819) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_819.T_DICT_6X6_1000_819'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 820) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_820.T_DICT_6X6_1000_820'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 821) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_821.T_DICT_6X6_1000_821'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 822) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_822.T_DICT_6X6_1000_822'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 823) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_823.T_DICT_6X6_1000_823'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 824) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_824.T_DICT_6X6_1000_824'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 825) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_825.T_DICT_6X6_1000_825'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 826) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_826.T_DICT_6X6_1000_826'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 827) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_827.T_DICT_6X6_1000_827'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 828) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_828.T_DICT_6X6_1000_828'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 829) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_829.T_DICT_6X6_1000_829'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 830) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_830.T_DICT_6X6_1000_830'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 831) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_831.T_DICT_6X6_1000_831'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 832) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_832.T_DICT_6X6_1000_832'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 833) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_833.T_DICT_6X6_1000_833'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 834) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_834.T_DICT_6X6_1000_834'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 835) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_835.T_DICT_6X6_1000_835'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 836) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_836.T_DICT_6X6_1000_836'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 837) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_837.T_DICT_6X6_1000_837'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 838) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_838.T_DICT_6X6_1000_838'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 839) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_839.T_DICT_6X6_1000_839'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 840) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_840.T_DICT_6X6_1000_840'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 841) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_841.T_DICT_6X6_1000_841'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 842) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_842.T_DICT_6X6_1000_842'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 843) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_843.T_DICT_6X6_1000_843'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 844) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_844.T_DICT_6X6_1000_844'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 845) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_845.T_DICT_6X6_1000_845'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 846) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_846.T_DICT_6X6_1000_846'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 847) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_847.T_DICT_6X6_1000_847'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 848) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_848.T_DICT_6X6_1000_848'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 849) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_849.T_DICT_6X6_1000_849'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 850) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_850.T_DICT_6X6_1000_850'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 851) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_851.T_DICT_6X6_1000_851'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 852) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_852.T_DICT_6X6_1000_852'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 853) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_853.T_DICT_6X6_1000_853'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 854) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_854.T_DICT_6X6_1000_854'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 855) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_855.T_DICT_6X6_1000_855'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 856) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_856.T_DICT_6X6_1000_856'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 857) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_857.T_DICT_6X6_1000_857'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 858) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_858.T_DICT_6X6_1000_858'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 859) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_859.T_DICT_6X6_1000_859'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 860) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_860.T_DICT_6X6_1000_860'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 861) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_861.T_DICT_6X6_1000_861'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 862) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_862.T_DICT_6X6_1000_862'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 863) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_863.T_DICT_6X6_1000_863'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 864) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_864.T_DICT_6X6_1000_864'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 865) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_865.T_DICT_6X6_1000_865'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 866) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_866.T_DICT_6X6_1000_866'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 867) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_867.T_DICT_6X6_1000_867'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 868) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_868.T_DICT_6X6_1000_868'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 869) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_869.T_DICT_6X6_1000_869'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 870) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_870.T_DICT_6X6_1000_870'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 871) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_871.T_DICT_6X6_1000_871'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 872) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_872.T_DICT_6X6_1000_872'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 873) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_873.T_DICT_6X6_1000_873'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 874) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_874.T_DICT_6X6_1000_874'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 875) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_875.T_DICT_6X6_1000_875'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 876) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_876.T_DICT_6X6_1000_876'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 877) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_877.T_DICT_6X6_1000_877'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 878) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_878.T_DICT_6X6_1000_878'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 879) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_879.T_DICT_6X6_1000_879'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 880) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_880.T_DICT_6X6_1000_880'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 881) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_881.T_DICT_6X6_1000_881'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 882) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_882.T_DICT_6X6_1000_882'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 883) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_883.T_DICT_6X6_1000_883'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 884) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_884.T_DICT_6X6_1000_884'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 885) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_885.T_DICT_6X6_1000_885'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 886) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_886.T_DICT_6X6_1000_886'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 887) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_887.T_DICT_6X6_1000_887'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 888) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_888.T_DICT_6X6_1000_888'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 889) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_889.T_DICT_6X6_1000_889'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 890) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_890.T_DICT_6X6_1000_890'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 891) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_891.T_DICT_6X6_1000_891'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 892) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_892.T_DICT_6X6_1000_892'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 893) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_893.T_DICT_6X6_1000_893'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 894) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_894.T_DICT_6X6_1000_894'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 895) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_895.T_DICT_6X6_1000_895'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 896) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_896.T_DICT_6X6_1000_896'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 897) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_897.T_DICT_6X6_1000_897'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 898) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_898.T_DICT_6X6_1000_898'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 899) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_899.T_DICT_6X6_1000_899'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 900) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_900.T_DICT_6X6_1000_900'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 901) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_901.T_DICT_6X6_1000_901'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 902) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_902.T_DICT_6X6_1000_902'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 903) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_903.T_DICT_6X6_1000_903'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 904) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_904.T_DICT_6X6_1000_904'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 905) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_905.T_DICT_6X6_1000_905'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 906) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_906.T_DICT_6X6_1000_906'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 907) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_907.T_DICT_6X6_1000_907'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 908) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_908.T_DICT_6X6_1000_908'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 909) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_909.T_DICT_6X6_1000_909'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 910) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_910.T_DICT_6X6_1000_910'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 911) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_911.T_DICT_6X6_1000_911'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 912) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_912.T_DICT_6X6_1000_912'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 913) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_913.T_DICT_6X6_1000_913'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 914) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_914.T_DICT_6X6_1000_914'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 915) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_915.T_DICT_6X6_1000_915'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 916) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_916.T_DICT_6X6_1000_916'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 917) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_917.T_DICT_6X6_1000_917'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 918) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_918.T_DICT_6X6_1000_918'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 919) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_919.T_DICT_6X6_1000_919'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 920) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_920.T_DICT_6X6_1000_920'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 921) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_921.T_DICT_6X6_1000_921'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 922) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_922.T_DICT_6X6_1000_922'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 923) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_923.T_DICT_6X6_1000_923'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 924) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_924.T_DICT_6X6_1000_924'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 925) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_925.T_DICT_6X6_1000_925'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 926) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_926.T_DICT_6X6_1000_926'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 927) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_927.T_DICT_6X6_1000_927'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 928) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_928.T_DICT_6X6_1000_928'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 929) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_929.T_DICT_6X6_1000_929'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 930) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_930.T_DICT_6X6_1000_930'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 931) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_931.T_DICT_6X6_1000_931'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 932) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_932.T_DICT_6X6_1000_932'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 933) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_933.T_DICT_6X6_1000_933'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 934) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_934.T_DICT_6X6_1000_934'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 935) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_935.T_DICT_6X6_1000_935'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 936) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_936.T_DICT_6X6_1000_936'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 937) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_937.T_DICT_6X6_1000_937'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 938) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_938.T_DICT_6X6_1000_938'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 939) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_939.T_DICT_6X6_1000_939'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 940) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_940.T_DICT_6X6_1000_940'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 941) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_941.T_DICT_6X6_1000_941'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 942) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_942.T_DICT_6X6_1000_942'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 943) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_943.T_DICT_6X6_1000_943'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 944) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_944.T_DICT_6X6_1000_944'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 945) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_945.T_DICT_6X6_1000_945'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 946) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_946.T_DICT_6X6_1000_946'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 947) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_947.T_DICT_6X6_1000_947'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 948) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_948.T_DICT_6X6_1000_948'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 949) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_949.T_DICT_6X6_1000_949'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 950) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_950.T_DICT_6X6_1000_950'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 951) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_951.T_DICT_6X6_1000_951'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 952) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_952.T_DICT_6X6_1000_952'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 953) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_953.T_DICT_6X6_1000_953'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 954) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_954.T_DICT_6X6_1000_954'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 955) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_955.T_DICT_6X6_1000_955'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 956) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_956.T_DICT_6X6_1000_956'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 957) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_957.T_DICT_6X6_1000_957'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 958) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_958.T_DICT_6X6_1000_958'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 959) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_959.T_DICT_6X6_1000_959'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 960) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_960.T_DICT_6X6_1000_960'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 961) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_961.T_DICT_6X6_1000_961'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 962) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_962.T_DICT_6X6_1000_962'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 963) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_963.T_DICT_6X6_1000_963'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 964) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_964.T_DICT_6X6_1000_964'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 965) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_965.T_DICT_6X6_1000_965'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 966) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_966.T_DICT_6X6_1000_966'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 967) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_967.T_DICT_6X6_1000_967'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 968) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_968.T_DICT_6X6_1000_968'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 969) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_969.T_DICT_6X6_1000_969'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 970) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_970.T_DICT_6X6_1000_970'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 971) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_971.T_DICT_6X6_1000_971'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 972) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_972.T_DICT_6X6_1000_972'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 973) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_973.T_DICT_6X6_1000_973'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 974) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_974.T_DICT_6X6_1000_974'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 975) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_975.T_DICT_6X6_1000_975'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 976) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_976.T_DICT_6X6_1000_976'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 977) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_977.T_DICT_6X6_1000_977'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 978) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_978.T_DICT_6X6_1000_978'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 979) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_979.T_DICT_6X6_1000_979'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 980) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_980.T_DICT_6X6_1000_980'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 981) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_981.T_DICT_6X6_1000_981'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 982) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_982.T_DICT_6X6_1000_982'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 983) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_983.T_DICT_6X6_1000_983'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 984) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_984.T_DICT_6X6_1000_984'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 985) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_985.T_DICT_6X6_1000_985'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 986) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_986.T_DICT_6X6_1000_986'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 987) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_987.T_DICT_6X6_1000_987'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 988) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_988.T_DICT_6X6_1000_988'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 989) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_989.T_DICT_6X6_1000_989'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 990) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_990.T_DICT_6X6_1000_990'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 991) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_991.T_DICT_6X6_1000_991'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 992) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_992.T_DICT_6X6_1000_992'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 993) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_993.T_DICT_6X6_1000_993'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 994) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_994.T_DICT_6X6_1000_994'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 995) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_995.T_DICT_6X6_1000_995'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 996) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_996.T_DICT_6X6_1000_996'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 997) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_997.T_DICT_6X6_1000_997'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 998) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_998.T_DICT_6X6_1000_998'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}
		if (beaconType == 999) {
			static ConstructorHelpers::FObjectFinder<UTexture> TextureFinder(TEXT("Texture2D'/AirSim/CosysAirSim/Textures/Beacons/Fiducials/T_DICT_6X6_1000_999.T_DICT_6X6_1000_999'"));
			if (TextureFinder.Succeeded()) {
				Texture = TextureFinder.Object;
				DynamicMaterial->SetTextureParameterValue(FName(TEXT("DynamicTexture")), Texture);
			}
		}

		// END PASTED TEXT
		//
		//

		Mesh->SetMaterial(0, DynamicMaterial);




		/*if (FoundMaterial.Succeeded()) {
			Material = FoundMaterial.Object;
			DynamicMaterial = UMaterialInstanceDynamic::Create(Material, Mesh);
			Mesh->SetMaterial(0, DynamicMaterial);
		}*/
	}


	//if (beaconType == 0) {
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