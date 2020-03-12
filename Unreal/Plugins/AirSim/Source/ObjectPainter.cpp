// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.

#include "ObjectPainter.h"
#include "StaticMeshResources.h"
#include "Components/SkinnedMeshComponent.h"
#include "Slate/SceneViewport.h"
#include "AirBlueprintLib.h"

int32 GetChannelValue(uint32 index)
{
	static int32 values[256] = { 0 };
	static bool init = false;
	if (!init)
	{
		float step = 256;
		uint32 iter = 0;
		values[0] = 0;
		while (step >= 1)
		{
			for (uint32 value = step - 1; value <= 256; value += step * 2)
			{
				iter++;
				values[iter] = value;
			}
			step /= 2;
		}
		init = true;
	}
	if (index >= 0 && index <= 255)
	{
		return values[index];
	}
	else
	{
		check(false);
		return -1;
	}
}

void GetColors(int32 max_val, bool enable_1, bool enable_2, bool enable_3, TArray<FColor>& color_map)
{
	for (int32 I = 0; I <= (enable_1 ? 0 : max_val - 1); I++)
	{
		for (int32 J = 0; J <= (enable_2 ? 0 : max_val - 1); J++)
		{
			for (int32 K = 0; K <= (enable_3 ? 0 : max_val - 1); K++)
			{
				uint8 R = GetChannelValue(enable_1 ? max_val : I);
				uint8 G = GetChannelValue(enable_2 ? max_val : J);
				uint8 B = GetChannelValue(enable_3 ? max_val : K);
				if (R != 76 && B != 76 && G != 76) {
					FColor color(R, G, B, 255);
					color_map.Add(color);
				}
			}
		}
	}
}


FColor GetColorFromColorMap(int32 color_index)
{
	static TArray<FColor> color_map_;
	int num_per_channel = 128;
	if (color_map_.Num() == 0)
	{
		for (int32 max_channel_index = 0; max_channel_index < num_per_channel; max_channel_index++)
		{
			GetColors(max_channel_index, false, false, true, color_map_);
			GetColors(max_channel_index, false, true, false, color_map_);
			GetColors(max_channel_index, false, true, true, color_map_);
			GetColors(max_channel_index, true, false, false, color_map_);
			GetColors(max_channel_index, true, false, true, color_map_);
			GetColors(max_channel_index, true, true, false, color_map_);
			GetColors(max_channel_index, true, true, true, color_map_);
		}
	}
	if (color_index < 0 || color_index >= pow(num_per_channel, 3))
	{
		UE_LOG(LogTemp, Error, TEXT("AirSim Segmentation: Object index %d is out of the available color map boundary [%d, %d]"), color_index, 0, pow(num_per_channel, 3));
		return FColor(0, 0, 0);
	}
	else {
		return color_map_[color_index];
	}	
}

bool IsPaintable(AActor* actor)
{
	TArray<UMeshComponent*> paintable_components;
	actor->GetComponents<UMeshComponent>(paintable_components);
	if (paintable_components.Num() == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void getPaintableComponentMeshes(AActor* actor, TMap<FString, UMeshComponent*>* paintable_components_meshes)
{
	TArray<UMeshComponent*> paintable_components;
	actor->GetComponents<UMeshComponent>(paintable_components);
	int index = 0;
	for (auto component : paintable_components)
	{
		if (paintable_components.Num() == 1) {
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				if (actor->GetParentActor()) {
					FString component_name = staticmesh_component->GetStaticMesh()->GetName();
					component_name.Append("_");
					component_name.Append(FString::FromInt(0));
					component_name.Append("_");
					if (actor->GetRootComponent()->GetAttachParent()) {
						component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
						component_name.Append("_");
					}
					component_name.Append(actor->GetParentActor()->GetName());
					paintable_components_meshes->Emplace(component_name, component);
				}
				else {
					paintable_components_meshes->Emplace(actor->GetName(), component);
				}
			}
			if (USkinnedMeshComponent*  SkinnedMeshComponent = Cast<USkinnedMeshComponent>(component)) {
				paintable_components_meshes->Emplace(actor->GetName(), component);
			}
		}
		else {
			FString component_name;
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				component_name = staticmesh_component->GetStaticMesh()->GetName();
				component_name.Append("_");				
				component_name.Append(FString::FromInt(index));
				component_name.Append("_");
				if (actor->GetParentActor()) {
					if (actor->GetRootComponent()->GetAttachParent()) {
						component_name.Append(actor->GetRootComponent()->GetAttachParent()->GetName());
						component_name.Append("_");
					}
					component_name.Append(actor->GetParentActor()->GetName());
				}
				else {
					component_name.Append(actor->GetName());
				}				
			}
			if (USkinnedMeshComponent*  skinnedmesh_component = Cast<USkinnedMeshComponent>(component)) {
				component_name = actor->GetName();
			}
			paintable_components_meshes->Emplace(component_name, component);
			index++;
		}		
	}
} 

bool UObjectPainter::SetComponentColor(FString component_id, uint32 color_index, TMap<FString, uint32>* name_to_colorindex_map, TMap<FString, UMeshComponent*> name_to_component_map)
{
	if (name_to_component_map.Contains(component_id))
	{
		FColor color = GetColorFromColorMap(color_index);
		UMeshComponent* actor = name_to_component_map[component_id];
		if (PaintComponent(actor, color))
		{
			name_to_colorindex_map->Emplace(component_id, color_index);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool UObjectPainter::PaintNewActor(AActor* actor, TMap<FString, uint32>* name_to_colorindex_map, TMap<FString, UMeshComponent*>* name_to_component_map)
{
	if (actor && IsPaintable(actor)) {
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		getPaintableComponentMeshes(actor, &paintable_components_meshes);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			if (name_to_component_map->Contains(it.Key())) {
				FColor Color = GetColorFromColorMap(name_to_colorindex_map->FindRef(it.Key()));
				check(PaintComponent(it.Value(), Color));				
			}
			else {
				uint32 ObjectIndex = name_to_component_map->Num();
				name_to_component_map->Emplace(it.Key(), it.Value());
				FColor NewColor = GetColorFromColorMap(ObjectIndex);
				name_to_colorindex_map->Emplace(it.Key(), ObjectIndex);
				check(PaintComponent(it.Value(), NewColor));
				UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Added new object %s with ID # %s"), *it.Key(), *FString::FromInt(ObjectIndex));
			}
		}
		return true;
	}
	else {
		return false;
	}
}

uint32 UObjectPainter::GetComponentColor(FString component_id, TMap<FString, uint32> name_to_colorindex_map)
{
	if (name_to_colorindex_map.Num() == 0)
	{
		return -1;
	}
	if (name_to_colorindex_map.Contains(component_id))
	{
		return name_to_colorindex_map[component_id];
	}
	else
	{
		return -1;
	}
}

void UObjectPainter::Reset(ULevel* InLevel, TMap<FString, uint32>* name_to_colorindex_map, TMap<FString, UMeshComponent*>* name_to_component_map)
{
	uint32 color_index = 0;
	UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Starting initial random instance segmentation"));
	for (AActor* actor : InLevel->Actors)
	{
		if (actor && IsPaintable(actor))
		{
			TMap<FString, UMeshComponent*> paintable_components_meshes;
			getPaintableComponentMeshes(actor, &paintable_components_meshes);
			for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
			{
				name_to_component_map->Emplace(it.Key(), it.Value());
				FColor new_color = GetColorFromColorMap(color_index);
				name_to_colorindex_map->Emplace(it.Key(), color_index);
				check(PaintComponent(it.Value(), new_color));
				UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Added new object %s with ID # %s"), *it.Key(), *FString::FromInt(color_index));

				color_index++;
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Completed initial random instance segmentation"));

}

bool UObjectPainter::PaintComponent(UMeshComponent* component, const FColor& color)
{
	if (!component) return false;
	if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component))
	{
		UStaticMesh* staticmesh;
		staticmesh = staticmesh_component->GetStaticMesh(); 
		if (staticmesh)
		{
			uint32 num_lod_level = staticmesh->RenderData->LODResources.Num();
			for (uint32 painting_mesh_lod_index = 0; painting_mesh_lod_index < num_lod_level; painting_mesh_lod_index++)
			{
				FStaticMeshLODResources& lod_model = staticmesh->RenderData->LODResources[painting_mesh_lod_index];
				FStaticMeshComponentLODInfo* instance_mesh_lod_info = NULL;

				// painting_mesh_lod_index + 1 is the minimum requirement, enlarge if not satisfied
				staticmesh_component->SetLODDataCount(painting_mesh_lod_index + 1, staticmesh_component->LODData.Num());
				instance_mesh_lod_info = &staticmesh_component->LODData[painting_mesh_lod_index];

				instance_mesh_lod_info->ReleaseOverrideVertexColorsAndBlock();
				{
					instance_mesh_lod_info->OverrideVertexColors = new FColorVertexBuffer;
					instance_mesh_lod_info->OverrideVertexColors->InitFromSingleColor(FColor::White, lod_model.GetNumVertices());
				}

				uint32 num_vertices = lod_model.GetNumVertices();
				check(instance_mesh_lod_info->OverrideVertexColors);
				check(num_vertices <= instance_mesh_lod_info->OverrideVertexColors->GetNumVertices());

				for (uint32 color_index = 0; color_index < num_vertices; ++color_index)
				{
					// Need to initialize the vertex buffer first
					uint32 num_override_vertex_colors = instance_mesh_lod_info->OverrideVertexColors->GetNumVertices();
					uint32 num_painted_vertices = instance_mesh_lod_info->PaintedVertices.Num();
					instance_mesh_lod_info->OverrideVertexColors->VertexColor(color_index) = color;
				}
				BeginInitResource(instance_mesh_lod_info->OverrideVertexColors);
				staticmesh_component->MarkRenderStateDirty();
					
			}
		}
	}
	if (USkinnedMeshComponent*  skinnedmesh_component = Cast<USkinnedMeshComponent>(component))
	{
		skinnedmesh_component->SetAllVertexColorOverride(color);
	}
	return true;
}

void UObjectPainter::SetViewForVertexColor(FEngineShowFlags& show_flags)
{
	show_flags.SetMaterials(false);
	show_flags.SetLighting(false);
	show_flags.SetBSPTriangles(true);
	show_flags.SetVertexColors(true);
	show_flags.SetPostProcessing(false);
	show_flags.SetHMDDistortion(false);
	show_flags.SetTonemapper(false);
	show_flags.SetEyeAdaptation(false);
}
