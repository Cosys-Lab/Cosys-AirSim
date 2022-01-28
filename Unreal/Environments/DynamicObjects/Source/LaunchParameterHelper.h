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
class HS_BM_2_API ULaunchParameterHelper : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()



private:

	static void ShuffleArrayWithStream_impl(void* TargetArray, const UArrayProperty* ArrayProperty, const FRandomStream& Stream); // Real implementation

public:

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Shuffle actor array by name", CompactNodeTitle = "SORTACTORARRAY", ArrayParm = "TargetArray"), Category = "Utility")
		static void SortActorArray(UPARAM(ref) TArray<AActor*>& TargetArray, const bool Reversed);
	UFUNCTION(BlueprintCallable, CustomThunk, meta = (DisplayName = "Shuffle by Stream", CompactNodeTitle = "SHUFFLESTREAM", ArrayParm = "TargetArray"), Category = "Utility")
		static void ShuffleArrayWithStream(const TArray<int32>& TargetArray, int32 Seed); // Stub function
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static bool GetLaunchFile(FString& launchFileName);
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 ParseLaunchFile(const FString& launchFileName);
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static void GetLaunchConfig(const FString& launchFileName, const int32 index, int32 &startSeed, int32 &removePercentage, int32 &movePercentage, int32 &moveOffsetValue, int32 &moveRotationValue);
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetStartSeed();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetRandomStartSeed();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetStartPoint();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static bool SpawnAI();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetRemovePercentage();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetMovePercentage();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetMoveOffsetValue();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static int32 GetMoveRotationValue();
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static bool IsWithEditor(const AActor* actor);
	UFUNCTION(BlueprintCallable, Category = "LaunchParameters")
		static bool IsStatic();


	DECLARE_FUNCTION(execShuffleArrayWithStream)
	{
		Stack.MostRecentProperty = nullptr;
		Stack.StepCompiledIn<UArrayProperty>(NULL);
		void* ArrayAddr = Stack.MostRecentPropertyAddress;
		UArrayProperty* ArrayProperty = Cast<UArrayProperty>(Stack.MostRecentProperty);
		if (!ArrayProperty)
		{
			Stack.bArrayContextFailed = true;
			return;
		}
		P_GET_STRUCT_REF(FRandomStream, Stream);
		P_FINISH;
		P_NATIVE_BEGIN;
		ShuffleArrayWithStream_impl(ArrayAddr, ArrayProperty, Stream);
		P_NATIVE_END;
	}
};