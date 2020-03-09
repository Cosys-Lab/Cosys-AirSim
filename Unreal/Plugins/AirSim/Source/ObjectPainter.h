// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "ObjectPainter.generated.h"


/**
*
*/
UCLASS()
class AIRSIM_API UObjectPainter : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

	static bool AddNewActorColor(AActor* Actor, TMap<FString, uint32>* Id2Color, TMap<FString, AActor*>* Id2Actor);

	/** Reset this to uninitialized state */
	static void Reset(ULevel* InLevel, TMap<FString, uint32>* Id2Color, TMap<FString, AActor*>* Id2Actor);
	/** The assigned color for each object */
	;
	/** A list of paintable objects */

	/** Vertex paint one object with Flood-Fill */
	static bool PaintObject(AActor* Actor, const FColor& Color, bool IsColorGammaEncoded = true);

	/** Get a pointer to an object */
	AActor* GetObject(FString ActorId, TMap<FString, AActor*> Id2Actor);

	/** Return a list of actors in the level */
	void GetObjectList(TMap<FString, AActor*> Id2Actor);

	/** Get the object color */
	static uint32 GetActorColor(FString ActorId, TMap<FString, uint32> Id2Color);

	static bool SetActorColor(FString ActorId, uint32 id, TMap<FString, uint32>* Id2Color, TMap<FString, AActor*> Id2Actor);
};
