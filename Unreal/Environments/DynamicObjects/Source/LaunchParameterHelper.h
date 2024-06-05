// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine.h"
#include "Engine/World.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "LaunchParameterHelper.generated.h"

/**
 * 
 */
UCLASS()
class HYSLAM_API ULaunchParameterHelper : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// Get seed from launch parameters if available, otherwise set random
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetStartSeed();
	// Get random seed
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetRandomStartSeed();
	// Allow the blueprint to determine whether we are running with the editor or not
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static bool IsWithEditor(const AActor* actor);
	// Get boolean from launch parameters if available, otherwise false
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static bool IsStatic();
	// Get boolean from launch parameters if available, otherwise true
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static bool SpawnAI();
};