// Fill out your copyright notice in the Description page of Project Settings.


#include "LaunchParameterHelper.h"

int32 ULaunchParameterHelper::GetStartSeed()
{
	int32 startSeed;
	if (FParse::Value(FCommandLine::Get(), TEXT("startSeed"), startSeed)) {
		UE_LOG(LogTemp, Display, TEXT("startSeed found: %i"), startSeed);
		if (startSeed == 0) {
			startSeed = FMath::Rand();
			UE_LOG(LogTemp, Display, TEXT("startSeed set to 0, chose random: %i"), startSeed);
		}
	}
	else {
		startSeed = FMath::Rand();
		UE_LOG(LogTemp, Display, TEXT("startSeed not found, chose random: %i"), startSeed);
	}

	return startSeed;
}

int32 ULaunchParameterHelper::GetRandomStartSeed()
{
	int32 startSeed;
	startSeed = FMath::Rand();
	UE_LOG(LogTemp, Display, TEXT("Set startSeed: %i"), startSeed);
	
	return startSeed;
}

bool ULaunchParameterHelper::IsWithEditor(const AActor* actor)
{
	auto world = actor->GetWorld();
	auto worldType = world->WorldType;
	auto worldTypeText = (worldType == EWorldType::Editor) ? TEXT("Editor")
		: (worldType == EWorldType::PIE) ? TEXT("PIE")
		: (worldType == EWorldType::EditorPreview) ? TEXT("EditorPreview")
		: (worldType == EWorldType::Game) ? TEXT("Game")
		: (worldType == EWorldType::GamePreview) ? TEXT("GamePreview")
		: (worldType == EWorldType::EditorPreview) ? TEXT("EditorPreview")
		: (worldType == EWorldType::Inactive) ? TEXT("Inactive")
		: (worldType == EWorldType::None) ? TEXT("None")
		: TEXT("I dont know");
	UE_LOG(LogTemp, Display, TEXT("WorldType: %s"), worldTypeText);
	if (worldType == EWorldType::Editor || worldType == EWorldType::PIE || worldType == EWorldType::EditorPreview) {
		return true;
	}
	return false;
}

bool ULaunchParameterHelper::IsStatic() {
	bool isStatic = false;
	if (FParse::Bool(FCommandLine::Get(), TEXT("isStatic"), isStatic)) {
		UE_LOG(LogTemp, Display, TEXT("isStatic found"));
	}

	return isStatic;
}

bool ULaunchParameterHelper::SpawnAI() {
	bool spawnAI = true;
	if (FParse::Bool(FCommandLine::Get(), TEXT("spawnAI"), spawnAI)) {
		UE_LOG(LogTemp, Display, TEXT("spawnAI found"));
	}
	return spawnAI;
}