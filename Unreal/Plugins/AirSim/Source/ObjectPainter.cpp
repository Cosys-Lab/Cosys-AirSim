#include "ObjectPainter.h"
#include "StaticMeshResources.h"
#include "Components/SkinnedMeshComponent.h"
#include "Slate/SceneViewport.h"
#include "ColorMap.h"


bool UObjectPainter::SetActorColor(FString ActorId, FColor Color, TMap<FString, FColor>* Id2Color, TMap<FString, AActor*> Id2Actor)
{
	if (Id2Actor.Contains(ActorId))
	{
		AActor* Actor = Id2Actor[ActorId];
		if (PaintObject(Actor, Color))
		{
			Id2Color->Emplace(ActorId, Color);
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

bool UObjectPainter::AddNewActorColor(AActor* Actor, TMap<FString, FColor>* Id2Color, TMap<FString, AActor*>* Id2Actor)
{
	if (Actor && IsPaintable(Actor)) {
		FString ActorId = Actor->GetHumanReadableName();
		if (Id2Actor->Contains(ActorId)) {
			return false;
		}
		else {
			uint32 ObjectIndex = Id2Actor->Num();
			Id2Actor->Emplace(ActorId, Actor);
			FColor NewColor = GetColorFromColorMap(ObjectIndex);
			Id2Color->Emplace(ActorId, NewColor);
			check(PaintObject(Actor, NewColor));
			return true;
		}
	}
	else {
		return false;
	}
}

FColor UObjectPainter::GetActorColor(FString ActorId, TMap<FString, FColor> Id2Color)
{
	// Make sure the object color map is initialized
	if (Id2Color.Num() == 0)
	{
		FColor ObjectColor = FColor(0,0,0,0);
		return ObjectColor;
	}
	if (Id2Color.Contains(ActorId))
	{
		FColor ObjectColor = Id2Color[ActorId]; // Make sure the object exist
		// FString Message = "%.3f %.3f %.3f %.3f";
		return ObjectColor;
	}
	else
	{
		FColor ObjectColor = FColor(0,0,0,0);
		return ObjectColor;
	}
}

void UObjectPainter::GetObjectList(TMap<FString, AActor*> Id2Actor)
{
	TArray<FString> Keys;
	Id2Actor.GetKeys(Keys);
	FString Message = "";
	for (auto ActorId : Keys)
	{
		Message += ActorId + " ";
	}
}

AActor* UObjectPainter::GetObject(FString ActorId, TMap<FString, AActor*> Id2Actor)
{
	/** Return the pointer of an object, return NULL if object not found */
	if (Id2Actor.Contains(ActorId))
	{
		return Id2Actor[ActorId];
	}
	else
	{
		return NULL;
	}
}

void UObjectPainter::Reset(ULevel* InLevel, TMap<FString, FColor>* Id2Color, TMap<FString, AActor*>* Id2Actor)
{

	uint32 ObjectIndex = 0;
	for (AActor* Actor : InLevel->Actors)
	{
		if (Actor && IsPaintable(Actor))
		{
			FString ActorId = Actor->GetHumanReadableName();
			
			FColor NewColor = GetColorFromColorMap(ObjectIndex);
			Id2Actor->Emplace(ActorId, Actor);
			Id2Color->Emplace(ActorId, NewColor);
			ObjectIndex++;
			PaintObject(Actor, NewColor);
		}
	}
}

/** DisplayColor is the color that the screen will show
If DisplayColor.R = 128, the display will show 0.5 voltage
To achieve this, UnrealEngine will do gamma correction.
The value on image will be 187.
https://en.wikipedia.org/wiki/Gamma_correction#Methods_to_perform_display_gamma_correction_in_computing
*/
bool UObjectPainter::PaintObject(AActor* Actor, const FColor& Color, bool IsColorGammaEncoded)
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

	TArray<UMeshComponent*> PaintableComponents;
	Actor->GetComponents<UMeshComponent>(PaintableComponents);


	for (auto MeshComponent : PaintableComponents)
	{
		if (UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(MeshComponent))
		{
			UStaticMesh* StaticMesh;
			StaticMesh = StaticMeshComponent->GetStaticMesh(); // This is a new function introduced in 4.14
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
					// Setup OverrideVertexColors
					// if (!InstanceMeshLODInfo->OverrideVertexColors) // TODO: Check this
					{
						InstanceMeshLODInfo->OverrideVertexColors = new FColorVertexBuffer;

						FColor FillColor = FColor(255, 255, 255, 255);
						InstanceMeshLODInfo->OverrideVertexColors->InitFromSingleColor(FColor::White, LODModel.GetNumVertices());
					}

					uint32 NumVertices = LODModel.GetNumVertices();
					check(InstanceMeshLODInfo->OverrideVertexColors);
					check(NumVertices <= InstanceMeshLODInfo->OverrideVertexColors->GetNumVertices());
					// StaticMeshComponent->CachePaintedDataIfNecessary();

					for (uint32 ColorIndex = 0; ColorIndex < NumVertices; ++ColorIndex)
					{
						// LODModel.ColorVertexBuffer.VertexColor(ColorIndex) = NewColor;  // This is vertex level
						// Need to initialize the vertex buffer first
						uint32 NumOverrideVertexColors = InstanceMeshLODInfo->OverrideVertexColors->GetNumVertices();
						uint32 NumPaintedVertices = InstanceMeshLODInfo->PaintedVertices.Num();
						// check(NumOverrideVertexColors == NumPaintedVertices);
						InstanceMeshLODInfo->OverrideVertexColors->VertexColor(ColorIndex) = NewColor;
						// InstanceMeshLODInfo->PaintedVertices[ColorIndex].Color = NewColor;
					}
					BeginInitResource(InstanceMeshLODInfo->OverrideVertexColors);
					StaticMeshComponent->MarkRenderStateDirty();
					// BeginUpdateResourceRHI(InstanceMeshLODInfo->OverrideVertexColors);


					/*
					// TODO: Need to check other LOD levels
					// Use flood fill to paint mesh vertices
					UE_LOG(LogUnrealCV, Warning, TEXT("%s:%s has %d vertices"), *Actor->GetActorLabel(), *StaticMeshComponent->GetName(), NumVertices);

					if (LODModel.ColorVertexBuffer.GetNumVertices() == 0)
					{
					// Mesh doesn't have a color vertex buffer yet!  We'll create one now.
					LODModel.ColorVertexBuffer.InitFromSingleColor(FColor(255, 255, 255, 255), LODModel.GetNumVertices());
					}

					*/
				}
			}
		}
		if (USkinnedMeshComponent*  SkinnedMeshComponent = Cast<USkinnedMeshComponent>(MeshComponent))
		{

			SkinnedMeshComponent->SetAllVertexColorOverride(NewColor);

			//Info.OverrideVertexColors = new FColorVertexBuffer;
			//Info.OverrideVertexColors->InitFromSingleColor(NewColor, numVerts);

			//BeginInitResource(Info.OverrideVertexColors);

			//SkinnedMeshComponent->MarkRenderStateDirty();

		}
	}
	return true;
}
