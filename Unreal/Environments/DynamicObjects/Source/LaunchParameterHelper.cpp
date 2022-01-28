// Fill out your copyright notice in the Description page of Project Settings.


#include "LaunchParameterHelper.h"

bool ULaunchParameterHelper::GetLaunchFile(FString& launchFileName)
{
	FString launchFileContent;
	if (FParse::Value(FCommandLine::Get(), TEXT("launchFile"), launchFileName)) {
		launchFileName = launchFileName.Replace(TEXT("="), TEXT("")).Replace(TEXT("\""), TEXT(""));
		if (FFileHelper::LoadFileToString(launchFileContent, *launchFileName)) {
			UE_LOG(LogTemp, Display, TEXT("LaunchFile found: %s"), *FString(launchFileName));
			return true;
		}else{
			UE_LOG(LogTemp, Display, TEXT("Launchfile not found: %s"), *FString(launchFileName));
			return true;
		}	
	}
	else {
		UE_LOG(LogTemp, Display, TEXT("LaunchFile not set"));
		return false;
	}
}

int32 ULaunchParameterHelper::ParseLaunchFile(const FString& launchFileName) {
	FString launchFileContent;
	TArray<int32> launchArray;
	TArray<FString> lines;

	FFileHelper::LoadFileToString(launchFileContent, *launchFileName);

	int32 lineCount = launchFileContent.ParseIntoArray(lines, TEXT("\n"), true);
	for (int i = 0; i < lineCount; i++) {
		UE_LOG(LogTemp, Display, TEXT("Launch config %i: %s"), i+1, *FString(lines[i]));
	}
	return lineCount;
}

void ULaunchParameterHelper::GetLaunchConfig(const FString& launchFileName, const int32 index, int32 &startSeed, int32 &removePercentage, int32 &movePercentage, int32 &moveOffsetValue, int32 &moveRotationValue) {
	FString launchFileContent;
	TArray<FString> launchConfigArrayString;
	TArray<FString> lines;

	FFileHelper::LoadFileToString(launchFileContent, *launchFileName);
	launchFileContent.ParseIntoArray(lines, TEXT("\n"), true);
	lines[index].ParseIntoArray(launchConfigArrayString, TEXT(","), true);
	startSeed = FCString::Atoi(*launchConfigArrayString[0]);
	removePercentage = FCString::Atoi(*launchConfigArrayString[1]);
	movePercentage = FCString::Atoi(*launchConfigArrayString[2]);
	moveOffsetValue = FCString::Atoi(*launchConfigArrayString[3]);
	moveRotationValue = FCString::Atoi(*launchConfigArrayString[4]);
	UE_LOG(LogTemp, Display, TEXT("Launch config %i set:\n  startSeed: %i\n  removePercentage: %i\n  movePercentage: %i\n  moveOffsetValue: %i\n  moveRotationValue: %i"), index + 1, startSeed, removePercentage, movePercentage, moveOffsetValue, moveRotationValue);
}

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
int32 ULaunchParameterHelper::GetStartPoint()
{
	int32 startPoint = 1;
	if (FParse::Value(FCommandLine::Get(), TEXT("startPoint"), startPoint)) {
		UE_LOG(LogTemp, Display, TEXT("startPoint found: %i"), startPoint);
	}
	return startPoint;
}

bool ULaunchParameterHelper::SpawnAI()
{
	bool isSpawnAI = 1;
	if (FParse::Bool(FCommandLine::Get(), TEXT("spawnAI"), isSpawnAI)) {
		UE_LOG(LogTemp, Display, TEXT("spawnAI found: %i"), isSpawnAI);
	}
	return isSpawnAI;
}

int32 ULaunchParameterHelper::GetRemovePercentage()
{
	int32 removePercentage = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("removePercentage"), removePercentage)) {
		UE_LOG(LogTemp, Display, TEXT("removePercentage found: %i"), removePercentage);
	}
	return removePercentage;
}

int32 ULaunchParameterHelper::GetMovePercentage()
{
	int32 movePercentage = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("movePercentage"), movePercentage)) {
		UE_LOG(LogTemp, Display, TEXT("movePercentage found: %i"), movePercentage);
	}
	return movePercentage;
}

int32 ULaunchParameterHelper::GetMoveOffsetValue()
{
	int32 moveOffsetValue = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("moveOffsetValue"), moveOffsetValue)) {
		UE_LOG(LogTemp, Display, TEXT("moveOffsetValue found: %i"), moveOffsetValue);
	}
	return moveOffsetValue;
}

int32 ULaunchParameterHelper::GetMoveRotationValue()
{
	int32 moveRotationValue = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("moveRotationValue"), moveRotationValue)) {
		UE_LOG(LogTemp, Display, TEXT("moveRotationValue found: %i"), moveRotationValue);
	}
	return moveRotationValue;
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
		: (worldType == EWorldType::Preview) ? TEXT("Preview")
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

void ULaunchParameterHelper::SortActorArray(UPARAM(ref) TArray<AActor*>& TargetArray, const bool Reversed)
{
	if (!Reversed)
	{
		TargetArray.Sort([](const AActor& A, const AActor& B)
		{
			return A.GetName() < B.GetName();
		});
	}
	else
	{
		TargetArray.Sort([](const AActor& A, const AActor& B)
		{
			return A.GetName() > B.GetName();
		});
	}
}

void ULaunchParameterHelper::ShuffleArrayWithStream(const TArray<int32>& TargetArray, int32 Seed)
{
	UE_LOG(LogTemp, Error, TEXT("Stub shuffle func called - should not happen"));
	check(0);
}

void ULaunchParameterHelper::ShuffleArrayWithStream_impl(void* TargetArray, const UArrayProperty* ArrayProperty, const FRandomStream& Stream)
{

	if (TargetArray)
	{

		FScriptArrayHelper ArrayHelper(ArrayProperty, TargetArray);
		int32 LastIndex = ArrayHelper.Num() - 1;

		for (int32 i = 0; i <= LastIndex; ++i)
		{
			int32 Index = Stream.RandRange(i, LastIndex-1);
			if (i != Index)
			{
				ArrayHelper.SwapValues(i, Index);
			}
		}
	}
}