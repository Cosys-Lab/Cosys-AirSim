// Weichao Qiu @ 2017
// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.
#include "ObjectAnnotator.h"
#include "Runtime/Engine/Public/EngineUtils.h"
#include "Runtime/Launch/Resources/Version.h"
#include "AnnotationComponent.h"
#include "AirBlueprintLib.h"

// For UE4 < 17
// check https://github.com/unrealcv/unrealcv/blob/1369a72be8428547318d8a52ae2d63e1eb57a001/Source/UnrealCV/Private/Controller/ObjectAnnotator.cpp#L1


FObjectAnnotator::FObjectAnnotator()
{
	name_ = FString("InstanceSegmentation");
	type_ = AnnotatorType::InstanceSegmentation;
	default_type_ = AnnotatorDefault::NoRender;
	direct_ = false;
}

FObjectAnnotator::FObjectAnnotator(FString name, AnnotatorType type, AnnotatorDefault default_type, bool direct)
{
	name_ = name;
	type_ = type;
	default_type_ = default_type;
	direct_ = direct;
}

void FObjectAnnotator::Initialize(ULevel* level) {
	switch (type_)
	{
	case AnnotatorType::RGB:
		UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: RGB not implemented yet."));
		break;
	case AnnotatorType::Greyscale:
		UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Greyscale not implemented yet"));
		break;
	case AnnotatorType::Texture:
		UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Texture not implemented yet."));
		break;
	case AnnotatorType::InstanceSegmentation:
		GenerateEntireLevel(level);
		break;
	}
}

bool FObjectAnnotator::IsPaintable(AActor* actor)
{
	if (!IsValid(actor))
	{
		return false;
	}
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

void FObjectAnnotator::getPaintableComponentMeshes(AActor* actor, TMap<FString, UMeshComponent*>* paintable_components_meshes)
{
	TArray<UMeshComponent*> paintable_components;
	actor->GetComponents<UMeshComponent>(paintable_components);
	int index = 0;
	for (auto component : paintable_components)
	{
		if (paintable_components.Num() == 1) {
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				if (actor->GetParentActor()) {
					if (staticmesh_component->GetStaticMesh() != nullptr) {
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
				}
				else {
					paintable_components_meshes->Emplace(actor->GetName(), component);
				}
			}
			if (USkinnedMeshComponent* SkinnedMeshComponent = Cast<USkinnedMeshComponent>(component)) {
				paintable_components_meshes->Emplace(actor->GetName(), component);
			}
		}
		else {
			FString component_name;
			if (UStaticMeshComponent* staticmesh_component = Cast<UStaticMeshComponent>(component)) {
				if (staticmesh_component->GetStaticMesh() != nullptr) {
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
			}
			if (USkinnedMeshComponent* skinnedmesh_component = Cast<USkinnedMeshComponent>(component)) {
				component_name = actor->GetName();
			}
			paintable_components_meshes->Emplace(component_name, component);
			index++;
		}
	}
}

bool FObjectAnnotator::SetComponentRGBColorByIndex(FString component_id, uint32 color_index)
{
	if (name_to_component_map_.Contains(component_id))
	{
		FColor color = ColorGenerator_.GetColorFromColorMap(color_index);
		UMeshComponent* actor = name_to_component_map_[component_id];
		if (UpdatePaintRGBComponent(actor, color))
		{
			FString color_string = FString::FromInt(color.R) + "," + FString::FromInt(color.G) + "," + FString::FromInt(color.B);
			FString color_string_gammacorrected = FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(color.R)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(color.G)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(color.B));
			color_to_name_map_.Emplace(color_string, component_id);
			name_to_color_index_map_.Emplace(component_id, color_index);
			UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Adjusted object %s to new ID # %s (RGB: %s)"), *component_id, *FString::FromInt(color_index), *color_string_gammacorrected);
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

bool FObjectAnnotator::AnnotateNewActor(AActor* actor)
{
	if (actor && IsPaintable(actor)) {
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		getPaintableComponentMeshes(actor, &paintable_components_meshes);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			if (name_to_component_map_.Contains(it.Key())) {
				FColor Color = ColorGenerator_.GetColorFromColorMap(name_to_color_index_map_.FindRef(it.Key()));
				check(UpdatePaintRGBComponent(it.Value(), Color));
			}
			else {
				uint32 ObjectIndex = name_to_component_map_.Num();
				name_to_component_map_.Emplace(it.Key(), it.Value());
				FColor new_color = ColorGenerator_.GetColorFromColorMap(ObjectIndex);
				name_to_color_index_map_.Emplace(it.Key(), ObjectIndex);
				FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
				FString color_string_gammacorrected = FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.R)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.G)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.B));
				color_to_name_map_.Emplace(color_string, it.Key());
				check(PaintRGBComponent(it.Value(), new_color));
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Added new object %s with ID # %s (RGB: %s)"), *it.Key(), *FString::FromInt(ObjectIndex), *color_string_gammacorrected);
			}
		}
		return true;
	}
	else {
		return false;
	}
}

bool FObjectAnnotator::DeleteActor(AActor* actor)
{
	if (actor && IsPaintable(actor)) {
		TMap<FString, UMeshComponent*> paintable_components_meshes;
		getPaintableComponentMeshes(actor, &paintable_components_meshes);
		for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
		{
			if (name_to_component_map_.Contains(it.Key())) {
				check(DeleteComponent(it.Value()));
				name_to_component_map_.Remove(it.Key());
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Deleted object %s."), *it.Key());

			}
			else {
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: could not delete object %s."), *it.Key());
				return false;
			}
		}
		return true;
	}
	else {
		return false;
	}
}

uint32 FObjectAnnotator::GetComponentIndex(FString component_id)
{
	if (name_to_color_index_map_.Num() == 0)
	{
		return -1;
	}
	if (name_to_color_index_map_.Contains(component_id))
	{
		return name_to_color_index_map_[component_id];
	}
	else
	{
		return -1;
	}
}

void FObjectAnnotator::GenerateEntireLevel(ULevel* InLevel)
{
	uint32 color_index = 0;
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Starting full level index-based RGB annotation."));
	for (AActor* actor : InLevel->Actors)
	{
		if (actor && IsPaintable(actor))
		{
			TMap<FString, UMeshComponent*> paintable_components_meshes;
			getPaintableComponentMeshes(actor, &paintable_components_meshes);
			for (auto it = paintable_components_meshes.CreateConstIterator(); it; ++it)
			{
				name_to_component_map_.Emplace(it.Key(), it.Value());
				FColor new_color = ColorGenerator_.GetColorFromColorMap(color_index);
				name_to_color_index_map_.Emplace(it.Key(), color_index);
				FString color_string = FString::FromInt(new_color.R) + "," + FString::FromInt(new_color.G) + "," + FString::FromInt(new_color.B);
				FString color_string_gammacorrected = FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.R)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.G)) + "," + FString::FromInt(ColorGenerator_.GetGammaCorrectedColor(new_color.B));
				color_to_name_map_.Emplace(color_string, it.Key());
				check(PaintRGBComponent(it.Value(), new_color));
				UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Added new object %s with ID # %s (RGB: %s)"), *it.Key(), *FString::FromInt(color_index), *color_string_gammacorrected);

				color_index++;
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("AirSim Annotation: Completed full level index-based RGB annotation."));

}

bool FObjectAnnotator::PaintRGBComponent(UMeshComponent* component, const FColor& color)
{
	if (!component) return false;

	FLinearColor LinearColor = FLinearColor(color);
	const FColor NewColor = LinearColor.ToFColor(false);
	UAnnotationComponent* AnnotationComponent = NewObject<UAnnotationComponent>(component);
	AnnotationComponent->SetupAttachment(component);
	AnnotationComponent->RegisterComponent();
	AnnotationComponent->SetAnnotationColor(NewColor);
	AnnotationComponent->MarkRenderStateDirty();
	return true;
}

bool FObjectAnnotator::UpdatePaintRGBComponent(UMeshComponent* component, const FColor& color)
{
	if (!component) return false;

	FLinearColor LinearColor = FLinearColor(color);
	const FColor NewColor = LinearColor.ToFColor(false);
	TArray<UActorComponent*> AnnotationComponents = component->GetAttachmentRootActor()->K2_GetComponentsByClass(UAnnotationComponent::StaticClass());

	if (AnnotationComponents.Num() == 0)return PaintRGBComponent(component, color);

	for (UActorComponent* Component : AnnotationComponents)
	{
		UAnnotationComponent* AnnotationComponent = Cast<UAnnotationComponent>(Component);
		AnnotationComponent->SetAnnotationColor(NewColor);
		AnnotationComponent->MarkRenderStateDirty();
	}
	return true;
}

bool FObjectAnnotator::DeleteComponent(UMeshComponent* component)
{
	if (!component) return false;

	TArray<UActorComponent*> AnnotationComponents = component->GetAttachmentRootActor()->K2_GetComponentsByClass(UAnnotationComponent::StaticClass());
	for (UActorComponent* Component : AnnotationComponents)
	{
		Component->DestroyComponent();
	}
	return true;
}

void FObjectAnnotator::SetViewForRGBAnnotationRender(FEngineShowFlags& show_flags)
{
	show_flags.SetMaterials(false);
	show_flags.SetLighting(false);
	show_flags.SetBSPTriangles(true);
	show_flags.SetPostProcessing(false);
	show_flags.SetHMDDistortion(false);
	show_flags.SetTonemapper(false);
	show_flags.SetEyeAdaptation(false);
	show_flags.SetFog(false);
	show_flags.SetPaper2DSprites(false);
	show_flags.SetBloom(false);
	show_flags.SetMotionBlur(false);
	show_flags.SetSkyLighting(false);
	show_flags.SetAmbientOcclusion(false);
	show_flags.SetInstancedFoliage(false);
	show_flags.SetInstancedGrass(false);
	show_flags.SetTextRender(false);
	show_flags.SetTemporalAA(false);
	show_flags.SetDecals(false);
}

void FObjectAnnotator::UpdateAnnotationComponents(UWorld* World)
{
	if (!IsValid(World))
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Can not get AnnotationComponents, World is invalid."));
		return;
	}

	// Check how much time is spent here!
	TArray<UObject*> UObjectList;
	bool bIncludeDerivedClasses = false;
	EObjectFlags ExclusionFlags = EObjectFlags::RF_ClassDefaultObject;
	EInternalObjectFlags ExclusionInternalFlags = EInternalObjectFlags::AllFlags;
	GetObjectsOfClass(UAnnotationComponent::StaticClass(), UObjectList, bIncludeDerivedClasses, ExclusionFlags, ExclusionInternalFlags);

	for (UObject* Object : UObjectList)
	{
		UPrimitiveComponent* Component = Cast<UPrimitiveComponent>(Object);

		if (Component->GetWorld() == World
			&& !annotation_component_list_.Contains(Component))
		{
			annotation_component_list_.Add(Component);
		}
	}

	if (annotation_component_list_.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: No annotation in the scene to show."));
	}
}

TArray<TWeakObjectPtr<UPrimitiveComponent>>  FObjectAnnotator::GetAnnotationComponents(){
	return annotation_component_list_;
}

std::vector<std::string> FObjectAnnotator::GetAllMeshIDs() {
	std::vector<std::string> retval;
	TMap<FString, uint32> nameToColorIndexMapTemp = name_to_color_index_map_;
	for (auto const& element : nameToColorIndexMapTemp) {
		retval.emplace_back(std::string(TCHAR_TO_UTF8(*element.Key)));
	}
	return retval;
}


TMap<FString, UMeshComponent*> FObjectAnnotator::GetNameToComponentMap() {
	return name_to_component_map_;
}

int32 FColorGenerator::GetChannelValue(uint32 index)
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

void FColorGenerator::GetColors(int32 max_val, bool enable_1, bool enable_2, bool enable_3, TArray<FColor>& color_map, TArray<int32>& ok_values)
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
				if (ok_values.Contains(R) && ok_values.Contains(G) && ok_values.Contains(B) && R != 149 && B != 149 && G != 149) {
					FColor color(R, G, B, 255);
					color_map.Add(color);
				}
			}
		}
	}
}

FColor FColorGenerator::GetColorFromColorMap(int32 color_index)
{
	static TArray<FColor> color_map_;
	static TArray<int32> ok_values_;
	int num_per_channel = 256;
	int uneven_start = 79;
	int full_start = 149;
	int uneven_count = FMath::FloorToInt((full_start - uneven_start + 2) / 2.0f);
	if (color_map_.Num() == 0) {

		for (int32 i = uneven_start; i <= full_start; i += 2) {
			ok_values_.Emplace(i);
		}
		for (int32 i = full_start + 1; i < num_per_channel; i++) {
			ok_values_.Emplace(i);
		}
		for (int32 max_channel_index = 0; max_channel_index < num_per_channel; max_channel_index++)
		{
			GetColors(max_channel_index, false, false, true, color_map_, ok_values_);
			GetColors(max_channel_index, false, true, false, color_map_, ok_values_);
			GetColors(max_channel_index, false, true, true, color_map_, ok_values_);
			GetColors(max_channel_index, true, false, false, color_map_, ok_values_);
			GetColors(max_channel_index, true, false, true, color_map_, ok_values_);
			GetColors(max_channel_index, true, true, false, color_map_, ok_values_);
			GetColors(max_channel_index, true, true, true, color_map_, ok_values_);
		}
	}
	if (color_index < 0 || color_index >= pow((num_per_channel - full_start) + uneven_count - 3, 3))
	{
		UE_LOG(LogTemp, Error, TEXT("AirSim Segmentation: Object index %d is out of the available color map boundary [%d, %d]"), color_index, 0, pow((num_per_channel - full_start) + uneven_count - 3, 3));
		return FColor(0, 0, 0);
	}
	else {
		return color_map_[color_index];
	}
}

int FColorGenerator::GetGammaCorrectedColor(int color_index) {
	return GammaCorrectionTable_[color_index];
}

int32 FColorGenerator::GammaCorrectionTable_[256] =
{
	0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 79, 0,
	81, 0, 83, 0, 85, 0, 86, 0, 88, 0,
	90, 0, 93, 0, 95, 0, 96, 0, 98, 0,
	101, 0, 102, 0, 105, 0, 106, 0, 109, 0,
	110, 0, 113, 0, 114, 0, 117, 0, 119, 0,
	120, 0, 122, 0, 125, 0, 127, 0, 129, 0,
	131, 0, 133, 0, 135, 0, 137, 0, 139, 0,
	141, 0, 143, 0, 145, 0, 147, 147, 148, 150,
	151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
	161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
	171, 172, 173, 174, 175, 176, 177, 178, 179, 180,
	181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 196, 197, 198, 199, 200,
	201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
	211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
	221, 222, 223, 224, 225, 226, 227, 228, 229, 230,
	231, 232, 233, 234, 235, 236, 237, 238, 239, 240,
	241, 242, 243, 244, 245, 246, 247, 248, 249, 250,
	251, 252, 253, 254, 255
};
