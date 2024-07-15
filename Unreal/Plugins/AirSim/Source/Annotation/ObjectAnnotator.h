// Weichao Qiu @ 2017
// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.
#pragma once

#include <vector>
#include <string>
#include "Components/MeshComponent.h"
#include "Components/SceneComponent.h"
#include "UObject/ObjectMacros.h"
#include "Components/StaticMeshComponent.h"
#include <Engine/StaticMesh.h>
#include "ShowFlags.h"
#include "UObject/ScriptMacros.h"
#include "Runtime/Engine/Classes/GameFramework/Actor.h"

class FColorGenerator
{
public:
	FColor GetColorFromColorMap(int32 ObjectIndex);
	int GetIndexForColor(FColor color);
	int GetGammaCorrectedColor(int color_index);
	TArray<FColor> GetColorMap();

private:
	int32 GetChannelValue(uint32 Index);
	void GetColors(int32 max_val, bool enable_1, bool enable_2, bool enable_3, TArray<FColor>& color_map, TArray<int32>& ok_values);
	static int GammaCorrectionTable_[256];
	static TArray<FColor> color_map_;
};

class AIRSIM_API FObjectAnnotator
{
public:

	enum class AnnotatorType : uint32
	{
		RGB = 0,
		Greyscale = 1,
		Texture = 2,
		InstanceSegmentation = 3
	};

	FObjectAnnotator();

	FObjectAnnotator(FString name, AnnotatorType type = AnnotatorType::RGB, bool show_by_default = true, bool set_direct = false, FString texture_path = FString(""), FString texture_prefix = FString(""), float max_view_distance = -1.0f);

	void Initialize(ULevel* level);
	void InitializeInstanceSegmentation(ULevel* level);
	void InitializeRGB(ULevel* level);
	void InitializeGreyscale(ULevel* level);
	void InitializeTexture(ULevel* level);

	bool DeleteActor(AActor* actor);

	bool AnnotateNewActor(AActor* actor);
	bool AnnotateNewActorInstanceSegmentation(AActor* actor);
	bool AnnotateNewActorRGB(AActor* actor);
	bool AnnotateNewActorGreyscale(AActor* actor);
	bool AnnotateNewActorTexture(AActor* actor);

	uint32 GetComponentIndex(FString component_id);

	bool IsRGBColorValid(FColor color);
	FString GetComponentRGBColor(FString component_id);
	bool SetComponentRGBColorByIndex(FString component_id, uint32 color_index);
	bool SetComponentRGBColorByColor(FString component_id, FColor color);

	bool SetComponentGreyScaleColorByValue(FString component_id, float greyscale_value);
	float GetComponentGreyscaleValue(FString component_id);

	bool SetComponentTextureByDirectPath(FString component_id, FString path);
	bool SetComponentTextureByRelativePath(FString component_id);

	FString GetComponentTexturePath(FString component_id);

	void UpdateAnnotationComponents(UWorld* World);
	TArray<TWeakObjectPtr<UPrimitiveComponent>> GetAnnotationComponents();

	static void SetViewForAnnotationRender(FEngineShowFlags& show_flags);

	TArray<FColor> GetColorMap();

	bool IsDirect();
	FObjectAnnotator::AnnotatorType GetType();

	void EndPlay();

	std::vector<std::string> GetAllComponentNames();
	TMap<FString, UMeshComponent*> GetNameToComponentMap();
	TMap<FString, FString> GetColorToComponentNameMap();
	TMap<FString, float> GetComponentToValueMap();
	TMap<UMeshComponent*, FString> GetComponentToNameMap();

private:
	FColorGenerator ColorGenerator_;

	AnnotatorType type_;
	FString name_;
	bool show_by_default_;
	bool set_direct_;
	FString texture_path_;
	FString texture_prefix_;
	float max_view_distance_;

	bool PaintRGBComponent(UMeshComponent* component, const FColor& color, const FString& component_name);
	bool UpdatePaintRGBComponent(UMeshComponent* component, const FColor& color, const FString& component_name);

	bool PaintTextureComponent(UMeshComponent* component, const FString& texture_path, const FString& component_name);
	bool UpdatePaintTextureComponent(UMeshComponent* component, const FString& texture_path, const FString& component_name);

	bool DeleteComponent(UMeshComponent* component);

	void getPaintableComponentMeshes(AActor* actor, TMap<FString, UMeshComponent*>* paintable_components_meshes);
	void getPaintableComponentMeshesAndTags(AActor* actor, TMap<FString, UMeshComponent*>* paintable_components_meshes, TMap<FString, TArray<FName>>* paintable_components_tags);
	bool IsPaintable(AActor* actor);

private:
	TMap<FString, uint32> name_to_color_index_map_;
	TMap<FString, FString> name_to_gammacorrected_color_map_;
	TMap<FString, float> name_to_value_map_;
	TMap<FString, FString> name_to_texture_path_map_;
	TMap<FString, FString> color_to_name_map_;
	TMap<FString, FString> gammacorrected_color_to_name_map_;
	TMap<FString, UMeshComponent*> name_to_component_map_;
	TMap<UMeshComponent*, FString> component_to_name_map_;
	TArray<TWeakObjectPtr<UPrimitiveComponent>> annotation_component_list_;
};

