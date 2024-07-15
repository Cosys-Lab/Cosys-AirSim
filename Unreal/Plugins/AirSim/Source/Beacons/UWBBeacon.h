// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/StaticMeshComponent.h"
#include <Engine/StaticMesh.h>
#include "UWBBeacon.generated.h"

UCLASS()
class AIRSIM_API AUWBBeacon : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AUWBBeacon();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, Category = "UWBBeacon|General")
	UStaticMeshComponent* Mesh;

	//msr::airlib::AirSimSettings::BeaconSetting* beaconSetting;
};
