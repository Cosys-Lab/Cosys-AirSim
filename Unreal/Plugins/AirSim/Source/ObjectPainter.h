// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Components/MeshComponent.h"
#include "ObjectPainter.generated.h"


/**
*
*/
UCLASS()
class AIRSIM_API UObjectPainter : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

	/** Paint all components of an newly added actor */
	static bool PaintNewActor(AActor* actor, TMap<FString, uint32>* name_to_colorindex_map, TMap<FString, UMeshComponent*>* name_to_component_map, TMap<FString, FString>* color_to_name_map);

	/** Reset all objects to their initial vertex color */
	static void Reset(ULevel* InLevel, TMap<FString, uint32>* name_to_colorindex_map, TMap<FString, UMeshComponent*>* name_to_component_map, TMap<FString, FString>* color_to_name_map);

	/** Vertex paint one object (static or skinned) */
	static bool PaintComponent(UMeshComponent* component, const FColor& color);

	/** Get the object color */
	static uint32 GetComponentColor(FString component_id, TMap<FString, uint32> name_to_colorindex_map);

	/** Set the object color */
	static bool SetComponentColor(FString component_id, uint32 color_index, TMap<FString, uint32>* name_to_colorindex_map, TMap<FString, UMeshComponent*> name_to_component_map, TMap<FString, FString>* color_to_name_map);

	/** Set camera ViewMode for vertex color */
	static void SetViewForVertexColor(FEngineShowFlags& show_flags);

	static void GetAnnotationComponents(UWorld* World, TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList);

	static int GammaCorrectionTable[256];


};
