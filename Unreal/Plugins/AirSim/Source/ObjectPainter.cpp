// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.

#include "ObjectPainter.h"
#include "StaticMeshResources.h"
#include "Components/SkinnedMeshComponent.h"
#include "Slate/SceneViewport.h"
#include "AirBlueprintLib.h"

int32 GetChannelValue(uint32 Index)
{
	static int32 Values[256] = { 0 };
	static bool Init = false;
	if (!Init)
	{
		float Step = 256;
		uint32 Iter = 0;
		Values[0] = 0;
		while (Step >= 1)
		{
			for (uint32 Value = Step - 1; Value <= 256; Value += Step * 2)
			{
				Iter++;
				Values[Iter] = Value;
			}
			Step /= 2;
		}
		Init = true;
	}
	if (Index >= 0 && Index <= 255)
	{
		return Values[Index];
	}
	else
	{
		check(false);
		return -1;
	}
}

void GetColors(int32 MaxVal, bool Fix1, bool Fix2, bool Fix3, TArray<FColor>& ColorMap)
{
	for (int32 I = 0; I <= (Fix1 ? 0 : MaxVal - 1); I++)
	{
		for (int32 J = 0; J <= (Fix2 ? 0 : MaxVal - 1); J++)
		{
			for (int32 K = 0; K <= (Fix3 ? 0 : MaxVal - 1); K++)
			{
				uint8 R = GetChannelValue(Fix1 ? MaxVal : I);
				uint8 G = GetChannelValue(Fix2 ? MaxVal : J);
				uint8 B = GetChannelValue(Fix3 ? MaxVal : K);
				if (R != 76 && B != 76 && G != 76) {
					FColor Color(R, G, B, 255);
					ColorMap.Add(Color);
				}
			}
		}
	}
}


FColor GetColorFromColorMap(int32 ObjectIndex)
{
	static TArray<FColor> ColorMap;
	int NumPerChannel = 255;
	if (ColorMap.Num() == 0)
	{
		for (int32 MaxChannelIndex = 0; MaxChannelIndex < NumPerChannel; MaxChannelIndex++)
		{
			GetColors(MaxChannelIndex, false, false, true, ColorMap);
			GetColors(MaxChannelIndex, false, true, false, ColorMap);
			GetColors(MaxChannelIndex, false, true, true, ColorMap);
			GetColors(MaxChannelIndex, true, false, false, ColorMap);
			GetColors(MaxChannelIndex, true, false, true, ColorMap);
			GetColors(MaxChannelIndex, true, true, false, ColorMap);
			GetColors(MaxChannelIndex, true, true, true, ColorMap);
		}
	}
	return ColorMap[ObjectIndex];
}

bool IsPaintable(AActor* Actor)
{
	TArray<UMeshComponent*> PaintableComponents;
	Actor->GetComponents<UMeshComponent>(PaintableComponents);
	if (PaintableComponents.Num() == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void getPaintableComponentMeshes(AActor* Actor, TMap<FString, UMeshComponent*>* PaintableComponentsMeshes)
{
	TArray<UMeshComponent*> PaintableComponents;
	Actor->GetComponents<UMeshComponent>(PaintableComponents);
	int index = 0;
	for (auto Component : PaintableComponents)
	{
		if (PaintableComponents.Num() == 1) {
			if (UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(Component)) {
				if (Actor->GetParentActor()) {
					FString component_name = StaticMeshComponent->GetStaticMesh()->GetName();
					component_name.Append("_");
					component_name.Append(FString::FromInt(0));
					component_name.Append("_");
					if (Actor->GetRootComponent()->GetAttachParent()) {
						component_name.Append(Actor->GetRootComponent()->GetAttachParent()->GetName());
						component_name.Append("_");
					}
					component_name.Append(Actor->GetParentActor()->GetName());
					PaintableComponentsMeshes->Emplace(component_name, Component);
				}
				else {
					PaintableComponentsMeshes->Emplace(Actor->GetName(), Component);
				}
			}
			if (USkinnedMeshComponent*  SkinnedMeshComponent = Cast<USkinnedMeshComponent>(Component)) {
				PaintableComponentsMeshes->Emplace(Actor->GetName(), Component);
			}
		}
		else {
			FString component_name;
			if (UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(Component)) {
				component_name = StaticMeshComponent->GetStaticMesh()->GetName();
				component_name.Append("_");				
				component_name.Append(FString::FromInt(index));
				component_name.Append("_");
				if (Actor->GetParentActor()) {
					if (Actor->GetRootComponent()->GetAttachParent()) {
						component_name.Append(Actor->GetRootComponent()->GetAttachParent()->GetName());
						component_name.Append("_");
					}
					component_name.Append(Actor->GetParentActor()->GetName());
				}
				else {
					component_name.Append(Actor->GetName());
				}				
			}
			if (USkinnedMeshComponent*  SkinnedMeshComponent = Cast<USkinnedMeshComponent>(Component)) {
				component_name = Actor->GetName();
			}
			PaintableComponentsMeshes->Emplace(component_name, Component);
			index++;
		}		
	}
} 

bool UObjectPainter::SetActorColor(FString ActorId, uint32 id, TMap<FString, uint32>* Id2Color, TMap<FString, UMeshComponent*> Id2Actor)
{
	if (Id2Actor.Contains(ActorId))
	{
		FColor Color = GetColorFromColorMap(id);
		UMeshComponent* Actor = Id2Actor[ActorId];
		if (PaintObject(Actor, Color))
		{
			Id2Color->Emplace(ActorId, id);
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

bool UObjectPainter::AddNewActorColor(AActor* Actor, TMap<FString, uint32>* Id2Color, TMap<FString, UMeshComponent*>* Id2Actor)
{
	if (Actor && IsPaintable(Actor)) {
		TMap<FString, UMeshComponent*> PaintableComponentsMeshes;
		getPaintableComponentMeshes(Actor, &PaintableComponentsMeshes);
		for (auto It = PaintableComponentsMeshes.CreateConstIterator(); It; ++It)
		{
			if (Id2Actor->Contains(It.Key())) {
				FColor Color = GetColorFromColorMap(Id2Color->FindRef(It.Key()));
				check(PaintObject(It.Value(), Color));				
			}
			else {
				uint32 ObjectIndex = Id2Actor->Num();
				Id2Actor->Emplace(It.Key(), It.Value());
				FColor NewColor = GetColorFromColorMap(ObjectIndex);
				Id2Color->Emplace(It.Key(), ObjectIndex);
				check(PaintObject(It.Value(), NewColor));
				UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Added new object %s with ID # %s"), *It.Key(), *FString::FromInt(ObjectIndex));
			}
		}
		return true;
	}
	else {
		return false;
	}
}

uint32 UObjectPainter::GetActorColor(FString ActorId, TMap<FString, uint32> Id2Color)
{
	if (Id2Color.Num() == 0)
	{
		return -1;
	}
	if (Id2Color.Contains(ActorId))
	{
		return Id2Color[ActorId];
	}
	else
	{
		return -1;
	}
}

void UObjectPainter::GetObjectList(TMap<FString, UMeshComponent*> Id2Actor)
{
	TArray<FString> Keys;
	Id2Actor.GetKeys(Keys);
	FString Message = "";
	for (auto ActorId : Keys)
	{
		Message += ActorId + " ";
	}
}

void UObjectPainter::Reset(ULevel* InLevel, TMap<FString, uint32>* Id2Color, TMap<FString, UMeshComponent*>* Id2Actor)
{

	uint32 ObjectIndex = 0;
	UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Starting initial random instance segmentation"));
	for (AActor* Actor : InLevel->Actors)
	{
		if (Actor && IsPaintable(Actor))
		{
			TMap<FString, UMeshComponent*> PaintableComponentsMeshes;
			getPaintableComponentMeshes(Actor, &PaintableComponentsMeshes);
			for (auto It = PaintableComponentsMeshes.CreateConstIterator(); It; ++It)
			{
				Id2Actor->Emplace(It.Key(), It.Value());
				FColor NewColor = GetColorFromColorMap(ObjectIndex);
				Id2Color->Emplace(It.Key(), ObjectIndex);
				check(PaintObject(It.Value(), NewColor));
				UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Added new object %s with ID # %s"), *It.Key(), *FString::FromInt(ObjectIndex));

				ObjectIndex++;
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("AirSim Segmentation: Completed initial random instance segmentation"));

}

bool UObjectPainter::PaintObject(UMeshComponent* Actor, const FColor& Color, bool IsColorGammaEncoded)
{
	if (!Actor) return false;

	FColor NewColor;
	if (IsColorGammaEncoded)
	{
		FLinearColor LinearColor = FLinearColor::FromPow22Color(Color);
		NewColor = LinearColor.ToFColor(false);
	}
	else
	{
		NewColor = Color;
	}

	if (UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(Actor))
	{
		UStaticMesh* StaticMesh;
		StaticMesh = StaticMeshComponent->GetStaticMesh(); 
		if (StaticMesh)
		{
			uint32 NumLODLevel = StaticMesh->RenderData->LODResources.Num();
			for (uint32 PaintingMeshLODIndex = 0; PaintingMeshLODIndex < NumLODLevel; PaintingMeshLODIndex++)
			{
				FStaticMeshLODResources& LODModel = StaticMesh->RenderData->LODResources[PaintingMeshLODIndex];
				FStaticMeshComponentLODInfo* InstanceMeshLODInfo = NULL;

				// PaintingMeshLODIndex + 1 is the minimum requirement, enlarge if not satisfied
				StaticMeshComponent->SetLODDataCount(PaintingMeshLODIndex + 1, StaticMeshComponent->LODData.Num());
				InstanceMeshLODInfo = &StaticMeshComponent->LODData[PaintingMeshLODIndex];

				InstanceMeshLODInfo->ReleaseOverrideVertexColorsAndBlock();
				{
					InstanceMeshLODInfo->OverrideVertexColors = new FColorVertexBuffer;

					FColor FillColor = FColor(255, 255, 255, 255);
					InstanceMeshLODInfo->OverrideVertexColors->InitFromSingleColor(FColor::White, LODModel.GetNumVertices());
				}

				uint32 NumVertices = LODModel.GetNumVertices();
				check(InstanceMeshLODInfo->OverrideVertexColors);
				check(NumVertices <= InstanceMeshLODInfo->OverrideVertexColors->GetNumVertices());

				for (uint32 ColorIndex = 0; ColorIndex < NumVertices; ++ColorIndex)
				{
					// Need to initialize the vertex buffer first
					uint32 NumOverrideVertexColors = InstanceMeshLODInfo->OverrideVertexColors->GetNumVertices();
					uint32 NumPaintedVertices = InstanceMeshLODInfo->PaintedVertices.Num();
					InstanceMeshLODInfo->OverrideVertexColors->VertexColor(ColorIndex) = NewColor;
				}
				BeginInitResource(InstanceMeshLODInfo->OverrideVertexColors);
				StaticMeshComponent->MarkRenderStateDirty();
					
			}
		}
	}
	if (USkinnedMeshComponent*  SkinnedMeshComponent = Cast<USkinnedMeshComponent>(Actor))
	{
		SkinnedMeshComponent->SetAllVertexColorOverride(NewColor);
	}
	return true;
}
