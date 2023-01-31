// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FiducialBeacon.generated.h"

UCLASS()
class AIRSIM_API AFiducialBeacon : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFiducialBeacon();
	void setScale(float scale);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere)
	UStaticMeshComponent* Mesh;
	UMaterial* Material;
	UMaterialInstanceDynamic* DynamicMaterial;
	UTexture* Texture;
};