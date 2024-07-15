// Fill out your copyright notice in the Description page of Project Settings.


#include "RandomPropSpawner.h"

// Sets default values
ARandomPropSpawner::ARandomPropSpawner()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ARandomPropSpawner::BeginPlay()
{
	Super::BeginPlay();

	TArray<UStaticMeshComponent*> StaticComps;
	GetComponents<UStaticMeshComponent>(StaticComps);
	int randomIndex = FMath::RandRange(0, StaticComps.Num() - 1 + AllowNone);

	for (int i = StaticComps.Num() - 1; i >= 0; i--) {
		if (i != randomIndex) {
			if (StaticComps[i] != nullptr)
			{
				StaticComps[i]->DestroyComponent();
			}
		}
		else {
			if (StaticComps[i] != nullptr)
			{
				float randomRotationOffset = FMath::RandRange(-MaximumRotationOffset, MaximumRotationOffset);
				float randomXPositionOffset = FMath::RandRange(-MaximumXPositionOffset, MaximumXPositionOffset);
				float randomYPositionOffset = FMath::RandRange(-MaximumYPositionOffset, MaximumYPositionOffset);
				StaticComps[i]->SetVisibility(true);
				StaticComps[i]->SetRelativeLocation(FVector(randomXPositionOffset, randomYPositionOffset, 0));
				StaticComps[i]->SetRelativeRotation(FRotator(0, randomRotationOffset, 0));
			}
		}
	}
	
}

// Called every frame
void ARandomPropSpawner::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

