// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/StaticMeshComponent.h"
#include <Engine/StaticMesh.h>
#include "Engine/Texture2D.h"
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

	UPROPERTY(EditAnywhere, Category = "FiducialBeacon|General")
	UStaticMeshComponent* Mesh;
	UMaterial* Material;
	UMaterialInstanceDynamic* DynamicMaterial;
	UTexture* Texture;
};