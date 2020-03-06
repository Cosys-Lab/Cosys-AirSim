#include "ObjectPainter.h"
#include "StaticMeshResources.h"
#include "Slate/SceneViewport.h"
#include "ColorMap.h"

FObjectPainter& FObjectPainter::Get()
{
	static FObjectPainter Singleton;
	return Singleton;
}

bool FObjectPainter::SetActorColor(FString ActorId, FColor Color)
{
	if (Id2Actor.Contains(ActorId))
	{
		AActor* Actor = Id2Actor[ActorId];
		if (PaintObject(Actor, Color))
		{
			Id2Color.Emplace(ActorId, Color);
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

FColor FObjectPainter::GetActorColor(FString ActorId)
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

void FObjectPainter::GetObjectList()
{
	TArray<FString> Keys;
	this->Id2Actor.GetKeys(Keys);
	FString Message = "";
	for (auto ActorId : Keys)
	{
		Message += ActorId + " ";
	}
}

AActor* FObjectPainter::GetObject(FString ActorId)
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

void FObjectPainter::Reset(ULevel* InLevel)
{
	this->Level = InLevel;
	this->Id2Color.Empty();
	this->Id2Actor.Empty();

	// This list needs to be generated everytime the game restarted.
	check(Level);

	uint32 ObjectIndex = 0;
	for (AActor* Actor : Level->Actors)
	{
		if (Actor && IsPaintable(Actor))
		{
			FString ActorId = Actor->GetHumanReadableName();
			Id2Actor.Emplace(ActorId, Actor);
			FColor NewColor = GetColorFromColorMap(ObjectIndex);
			Id2Color.Emplace(ActorId, NewColor);
			ObjectIndex++;
		}
	}

	for (auto& Elem : Id2Color)
	{
		FString ActorId = Elem.Key;
		FColor NewColor = Elem.Value;
		AActor* Actor = Id2Actor[ActorId];
		check(PaintObject(Actor, NewColor));
	}
}
