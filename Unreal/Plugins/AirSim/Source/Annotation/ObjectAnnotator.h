// Weichao Qiu @ 2017
// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.
#pragma once

#include "Runtime/Engine/Classes/GameFramework/Actor.h"

class FColorGenerator
{
public:
	FColor GetColorFromColorMap(int32 ObjectIndex);
	int GetGammaCorrectedColor(int color_index);

private:
	int32 GetChannelValue(uint32 Index);
	void GetColors(int32 max_val, bool enable_1, bool enable_2, bool enable_3, TArray<FColor>& color_map, TArray<int32>& ok_values);
	static int GammaCorrectionTable_[256];
};

class AIRSIM_API FObjectAnnotator
{
public:
	FObjectAnnotator();

	bool DeleteActor(AActor* actor);
	bool AnnotateNewActor(AActor* actor);
	
	void GenerateEntireLevel(ULevel* InLevel);

	uint32 GetComponentIndex(FString component_id);
	bool SetComponentRGBColorByIndex(FString component_id, uint32 color_index);
	void UpdateAnnotationComponents(UWorld* World);
	TArray<TWeakObjectPtr<UPrimitiveComponent>> GetAnnotationComponents();

    static void SetViewForRGBAnnotationRender(FEngineShowFlags& show_flags);

	std::vector<std::string> GetAllMeshIDs();
	TMap<FString, UMeshComponent*> GetNameToComponentMap();

private:
	FColorGenerator ColorGenerator_;

	bool PaintRGBComponent(UMeshComponent* component, const FColor& color);
	bool UpdatePaintRGBComponent(UMeshComponent* component, const FColor& color);
	bool DeleteComponent(UMeshComponent* component);	

	void getPaintableComponentMeshes(AActor* actor, TMap<FString, UMeshComponent*>* paintable_components_meshes);
	bool IsPaintable(AActor* actor);

private:
	TMap<FString, uint32> name_to_color_index_map_;
	TMap<FString, FString> color_to_name_map_;
	TMap<FString, UMeshComponent*> name_to_component_map_;
	TArray<TWeakObjectPtr<UPrimitiveComponent>> annotation_component_list_;
};
