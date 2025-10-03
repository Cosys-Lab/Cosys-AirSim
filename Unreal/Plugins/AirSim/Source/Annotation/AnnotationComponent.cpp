// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.


#include "AnnotationComponent.h"
// Overwrite the material

#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Runtime/Engine/Public/Materials/Material.h"
#include "Runtime/Engine/Public/Materials/MaterialInstanceDynamic.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Engine/Classes/Components/SkeletalMeshComponent.h"
#include "Runtime/Launch/Resources/Version.h"
#include "Runtime/Engine/Public/MaterialShared.h"
#include "Runtime/Engine/Classes/Engine/Engine.h"
#include "AirBlueprintLib.h"

#if ENGINE_MAJOR_VERSION >= 5
//different header files in UE
#include "Runtime/Engine/Public/StaticMeshSceneProxy.h"
#include "Runtime/Engine/Public/SkeletalMeshSceneProxy.h"

#endif
#include "Runtime/Engine/Public/Rendering/SkeletalMeshRenderData.h"

/** A proxy class to get mesh data from StaticMesh, should be used together with AnnotationCamSensor.
Inheritance is needed because I need to access protected data
Use `show Material` command to see the effect of this component
Note that some area might be not colored, this is caused by the issue that
both the original mesh and the annotation mesh are rendered, this is not an issue for the AnnotationCamSensor, which will exclude original meshes.
*/
class FStaticAnnotationSceneProxy : public FStaticMeshSceneProxy
{
public:
	FMaterialRenderProxy* MaterialRenderProxy;

	//FStaticMeshSceneProxyDesc::InitializeFrom(UStaticMeshComponent* Component);

	FStaticAnnotationSceneProxy(UStaticMeshComponent* Component, bool bForceLODsShareStaticLighting, UMaterialInterface* AnnotationMID) :
		FStaticMeshSceneProxy(Component, bForceLODsShareStaticLighting)
	{
		MaterialRenderProxy = AnnotationMID->GetRenderProxy();
		// this->MaterialRelevance = AnnotationMID->GetRelevance(GetScene().GetFeatureLevel());
		// Note: This MaterailRelevance makes no difference?

		this->bVerifyUsedMaterials = false;
		// This is required, otherwise the code will fail

		bCastShadow = false;
	}

	virtual void GetDynamicMeshElements(
		const TArray < const FSceneView * > & Views,
		const FSceneViewFamily & ViewFamily,
		uint32 VisibilityMap,
		FMeshElementCollector & Collector) const override;

	virtual bool GetMeshElement
	(
		int32 LODIndex,
		int32 BatchIndex,
		int32 ElementIndex,
		uint8 InDepthPriorityGroup,
		bool bUseSelectedMaterial,
		bool bAllowPreCulledIndices,
		FMeshBatch & OutMeshBatch
	) const override;

	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView * View) const override;
};

FPrimitiveViewRelevance FStaticAnnotationSceneProxy::GetViewRelevance(const FSceneView * View) const
{
	if (View->Family->EngineShowFlags.Materials)
	{
		FPrimitiveViewRelevance ViewRelevance;
		ViewRelevance.bDrawRelevance = 0; 
		// This will make the AnnotationComponent gets ignored if the Materials flag is on
		// Which means it won't affect regulary rendering.
		return ViewRelevance;
	}
	else
	{
		return FStaticMeshSceneProxy::GetViewRelevance(View);
	}
}


void FStaticAnnotationSceneProxy::GetDynamicMeshElements(
	const TArray < const FSceneView * > & Views,
	const FSceneViewFamily & ViewFamily,
	uint32 VisibilityMap,
	FMeshElementCollector & Collector) const
{
	//if (MaterialRenderProxy->GetMaterialName().Contains("AnnotationMaterialMID")) {
	//	FStaticMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);
	//}	
	FStaticMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);

}

bool FStaticAnnotationSceneProxy::GetMeshElement(
	int32 LODIndex,
	int32 BatchIndex,
	int32 ElementIndex,
	uint8 InDepthPriorityGroup,
	bool bUseSelectedMaterial,
	bool bAllowPreCulledIndices,
	FMeshBatch & OutMeshBatch) const
{
	bool Ret = FStaticMeshSceneProxy::GetMeshElement(LODIndex, BatchIndex, ElementIndex, InDepthPriorityGroup,
		bUseSelectedMaterial, bAllowPreCulledIndices, OutMeshBatch);
	OutMeshBatch.MaterialRenderProxy = this->MaterialRenderProxy;
	return Ret;
}

class FSkeletalAnnotationSceneProxy : public FSkeletalMeshSceneProxy
{
public:
	FSkeletalAnnotationSceneProxy(const USkinnedMeshComponent* Component, FSkeletalMeshRenderData* InSkeletalMeshRenderData, UMaterialInterface* AnnotationMID)
	: FSkeletalMeshSceneProxy(Component, InSkeletalMeshRenderData)
	{
		// TODO: Update MaterialRelevance
		this->bVerifyUsedMaterials = false;
		// this->bCastShadow = false;
		this->bCastDynamicShadow = false;
		for(int32 LODIdx=0; LODIdx < LODSections.Num(); LODIdx++)
		{
			FLODSectionElements& LODSection = LODSections[LODIdx];
			for(int32 SectionIndex = 0; SectionIndex < LODSection.SectionElements.Num(); SectionIndex++)
			{
				if (IsValid(AnnotationMID))
				{
					LODSection.SectionElements[SectionIndex].Material = AnnotationMID;
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: AnnotationMaterial is Invalid in FSkeletalSceneProxy"));
				}
			}
		}
	}
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView * View) const override;

	virtual void GetDynamicMeshElements(
		const TArray<const FSceneView*>& Views,
		const FSceneViewFamily& ViewFamily,
		uint32 VisibilityMap,
		FMeshElementCollector& Collector) const;
};


void FSkeletalAnnotationSceneProxy::GetDynamicMeshElements(
	const TArray<const FSceneView*>& Views,
	const FSceneViewFamily& ViewFamily,
	uint32 VisibilityMap,
	FMeshElementCollector& Collector) const
{
	//if (LODSections.Num() > 0){
	//	if (LODSections[0].SectionElements.Num() > 0) {
	//		if (LODSections[0].SectionElements[0].Material->GetName().Contains("AnnotationMaterialMID")) {
	//			FSkeletalMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);
	//		}
	//	}
	//}
	FSkeletalMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);

}

FPrimitiveViewRelevance FSkeletalAnnotationSceneProxy::GetViewRelevance(const FSceneView * View) const
{
	if (View->Family->EngineShowFlags.Materials)
	{
		FPrimitiveViewRelevance ViewRelevance;
		ViewRelevance.bDrawRelevance = 0; // This will make it gets ignored, when materials flag is enabled.
		return ViewRelevance;
	}
	else
	{
		return FSkeletalMeshSceneProxy::GetViewRelevance(View);
	}
}

// FString MeterialPath = TEXT("MaterialInstanceConstant'/UnrealCV/AnnotationColor_Inst.AnnotationColor_Inst'");
// static ConstructorHelpers::FObjectFinder<UMaterialInstanceDynamic> AnnotationMaterialObject(*MaterialPath);
UAnnotationComponent::UAnnotationComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
	  // , ParentMeshInfo(nullptr)
{
	bSkeletalMesh = false;
	bTexture = false;

	FString MaterialPath = TEXT("Material'/AirSim/HUDAssets/AnnotationMaterial.AnnotationMaterial'");
	static ConstructorHelpers::FObjectFinder<UMaterial> AnnotationMaterialObject(*MaterialPath);
	if (AnnotationMaterialObject.Object == nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Annotation material is not valid."));
    }
    else
    {
        AnnotationMaterial = AnnotationMaterialObject.Object;
	}

	FString MaterialPathSphere = TEXT("Material'/AirSim/HUDAssets/AnnotationMaterialSphere.AnnotationMaterialSphere'");
	static ConstructorHelpers::FObjectFinder<UMaterial> SphereMaterialObject(*MaterialPathSphere);
	if (SphereMaterialObject.Object == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Sphere annotation material is not valid."));
	}
	else
	{
		SphereMaterial = SphereMaterialObject.Object;
	}

	// ParentMeshInfo = MakeShareable(new FParentMeshInfo(nullptr));
	// This will be invalid until attached to a MeshComponent
	this->PrimaryComponentTick.bCanEverTick = true;
}

void UAnnotationComponent::OnRegister()
{
	Super::OnRegister();

	if (this->GetFName().ToString().Contains("annotation_sphere")) {
		AnnotationMID = UMaterialInstanceDynamic::Create(SphereMaterial, this, TEXT("AnnotationMaterialMID"));
		if (!IsValid(AnnotationMID))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: SphereMaterial is not correctly initialized"));
			return;
		}
		FLinearColor LinearAnnotationColor = FLinearColor(0, 0, 0, 1.0);
		AnnotationMID->SetVectorParameterValue("AnnotationColor", LinearAnnotationColor);
	}
	else {
		// Note: This can not be placed in the constructor, MID means material instance dynamic
		AnnotationMID = UMaterialInstanceDynamic::Create(AnnotationMaterial, this, TEXT("AnnotationMaterialMID"));
		if (!IsValid(AnnotationMID))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: ColorAnnotationMaterial is not correctly initialized"));
			return;
		}
		const float OneOver255 = 1.0f / 255.0f;
		FLinearColor LinearAnnotationColor = FLinearColor(
			this->AnnotationColor.R * OneOver255,
			this->AnnotationColor.G * OneOver255,
			this->AnnotationColor.B * OneOver255,
			1.0
		);
		AnnotationMID->SetVectorParameterValue("AnnotationColor", LinearAnnotationColor);
	}
}

/** 
 * Note: The "exposure compensation" in "PostProcessVolume3" in the RR map will destroy the color
 * Saturate the color to 1. This is a mysterious behavior after tedious debug.
 */
void UAnnotationComponent::SetAnnotationColor(FColor NewAnnotationColor)
{
	if (NewAnnotationColor.R == 27)NewAnnotationColor.R = 26;
	if (NewAnnotationColor.G == 27)NewAnnotationColor.G = 26;
	if (NewAnnotationColor.B == 27)NewAnnotationColor.B = 26;
	if (NewAnnotationColor.R == 32)NewAnnotationColor.R = 31;
	if (NewAnnotationColor.G == 32)NewAnnotationColor.G = 31;
	if (NewAnnotationColor.B == 32)NewAnnotationColor.B = 31;
	if (NewAnnotationColor.R == 35)NewAnnotationColor.R = 34;
	if (NewAnnotationColor.G == 35)NewAnnotationColor.G = 34;
	if (NewAnnotationColor.B == 35)NewAnnotationColor.B = 34;
	if (NewAnnotationColor.R == 41)NewAnnotationColor.R = 40;
	if (NewAnnotationColor.G == 41)NewAnnotationColor.G = 40;
	if (NewAnnotationColor.B == 41)NewAnnotationColor.B = 40;
	if (NewAnnotationColor.R == 44)NewAnnotationColor.R = 43;
	if (NewAnnotationColor.G == 44)NewAnnotationColor.G = 43;
	if (NewAnnotationColor.B == 44)NewAnnotationColor.B = 43;
	if (NewAnnotationColor.R == 49)NewAnnotationColor.R = 48;
	if (NewAnnotationColor.G == 49)NewAnnotationColor.G = 48;
	if (NewAnnotationColor.B == 49)NewAnnotationColor.B = 48;
	if (NewAnnotationColor.R == 51)NewAnnotationColor.R = 50;
	if (NewAnnotationColor.G == 51)NewAnnotationColor.G = 50;
	if (NewAnnotationColor.B == 51)NewAnnotationColor.B = 50;
	this->AnnotationColor = NewAnnotationColor;
	const float OneOver255 = 1.0f / 255.0f; // TODO: Check 255 or 256?

	FLinearColor LinearAnnotationColor = FLinearColor(
		AnnotationColor.R * OneOver255,
		AnnotationColor.G * OneOver255,
		AnnotationColor.B * OneOver255,
		1.0
	);

	if (IsValid(AnnotationMID))
	{
		AnnotationMID->SetVectorParameterValue("AnnotationColor", LinearAnnotationColor);
	}
}

void UAnnotationComponent::SetAnnotationTexture(FString NewAnnotationTexturePath)
{
    bTexture = true;
	AnnotationMID->SetScalarParameterValue("TextureEnabled", 1);
    this->AnnotationTexturePath = NewAnnotationTexturePath;
    TArray<FString> splitPath;
    NewAnnotationTexturePath.ParseIntoArray(splitPath, TEXT("/"), true);
    FString TextureFileName = splitPath.Last();
	FString FullPath = FString::Printf(TEXT("%s.%s"), *NewAnnotationTexturePath, *TextureFileName);
	UTexture* AnnotationTexture = LoadObject<UTexture>(NULL, *FullPath);

    if (AnnotationTexture != nullptr)
    {       
        if (IsValid(AnnotationMID))
        {
			AnnotationMID->SetTextureParameterValue("AnnotationTexture", AnnotationTexture);
		}else
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Could not set annotation texture to %s cause something wrong with MID."), *FullPath);
		}
    }
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Could not set annotation texture to %s."), *FullPath);
	}
}

void UAnnotationComponent::SetAnnotationTexture(UTexture* NewAnnotationTexture)
{
	bTexture = true;
	AnnotationMID->SetScalarParameterValue("TextureEnabled", 1);
	TArray<FString> splitPath;
	NewAnnotationTexture->GetPathName().ParseIntoArray(splitPath, TEXT("."), true);
	FString TextureFilePath = splitPath[0];
	this->AnnotationTexturePath = TextureFilePath;
	if (IsValid(AnnotationMID))
	{
		AnnotationMID->SetTextureParameterValue("AnnotationTexture", NewAnnotationTexture);
	}
}

FColor UAnnotationComponent::GetAnnotationColor()
{
	return AnnotationColor;
}

FString UAnnotationComponent::GetAnnotationTexturePath()
{
	return AnnotationTexturePath;
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy(UStaticMeshComponent* StaticMeshComponent)
{
	// FPrimitiveSceneProxy* PrimitiveSceneProxy = StaticMeshComponent->CreateSceneProxy();
	// FStaticMeshSceneProxy* StaticMeshSceneProxy = (FStaticMeshSceneProxy*)PrimitiveSceneProxy;
	UMaterialInterface* ProxyMaterial = AnnotationMID; // Material Instance Dynamic
	UStaticMesh* ParentStaticMesh = StaticMeshComponent->GetStaticMesh();
	if(ParentStaticMesh == NULL
		|| ParentStaticMesh->GetRenderData() == NULL
		|| ParentStaticMesh->GetRenderData()->LODResources.Num() == 0)
		// || StaticMesh->RenderData->LODResources[0].VertexBuffer.GetNumVertices() == 0)
	{
		// UE_LOG(LogTemp, Warning, TEXT("%s, ParentStaticMesh is invalid."), *StaticMeshComponent->GetName());
		return NULL;
	}

	// FPrimitiveSceneProxy* Proxy = ::new FStaticMeshSceneProxy(OwnerComponent, false);
	FPrimitiveSceneProxy* Proxy = ::new FStaticAnnotationSceneProxy(StaticMeshComponent, false, ProxyMaterial);
	return Proxy;
	// This is not recommended, but I know what I am doing.
}

// See https://github.com/EpicGames/UnrealEngine/blob/release/Engine/Source/Runtime/Engine/Private/Components/SkinnedMeshComponent.cpp:417
FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy(USkeletalMeshComponent* SkeletalMeshComponent)
{
	UMaterialInterface* ProxyMaterial = AnnotationMID; // Material Instance Dynamic

	ERHIFeatureLevel::Type SceneFeatureLevel = GetWorld()->GetFeatureLevel();

	// Ref: https://github.com/EpicGames/UnrealEngine/blob/4.19/Engine/Source/Runtime/Engine/Private/Components/SkinnedMeshComponent.cpp#L415
	FSkeletalMeshRenderData* SkelMeshRenderData = SkeletalMeshComponent->GetSkeletalMeshRenderData();

	// Only create a scene proxy for rendering if properly initialized
	if (SkelMeshRenderData &&
		SkelMeshRenderData->LODRenderData.IsValidIndex(SkeletalMeshComponent->GetPredictedLODLevel()) &&
		SkeletalMeshComponent->MeshObject) // The risk of using MeshObject
	{
		// Only create a scene proxy if the bone count being used is supported, or if we don't have a skeleton (this is the case with destructibles)
		// int32 MaxBonesPerChunk = SkelMeshResource->GetMaxBonesPerSection();
		// if (MaxBonesPerChunk <= GetFeatureLevelMaxNumberOfBones(SceneFeatureLevel))
		// {
		//	Result = ::new FSkeletalAnnotationSceneProxy(SkeletalMeshComponent, SkelMeshResource, AnnotationMID);
		// }
		// TODO: The SkeletalMeshComponent might need to be recreated
		return new FSkeletalAnnotationSceneProxy(SkeletalMeshComponent, SkelMeshRenderData, ProxyMaterial);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: The data of SkeletalMeshComponent %s is invalid."), *SkeletalMeshComponent->GetName());
		return nullptr;
	}
}


// TODO: This needs to be involked when the ParentComponent refresh its render state, otherwise it will crash the engine
FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy()
{
	// UMaterialInstanceDynamic* AnnotationMID = UMaterialInstanceDynamic::Create(AnnotationMaterial, this);
	// FColor AnnotationColor = FColor::MakeRandomColor();
	// AnnotationMID->SetVectorParameterByIndex(0, AnnotationColor);

	USceneComponent* ParentComponent = this->GetAttachParent();
	// USceneComponent* ParentComponent = this->ParentMeshInfo->GetParentMeshComponent();

	if (!IsValid(ParentComponent))
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Parent component is invalid."));
		return nullptr;
	}


	UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(ParentComponent);
	USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(ParentComponent);
	// UCableComponent* CableComponent = Cast<UCableComponent>(ParentComponent);
	if (IsValid(StaticMeshComponent))
	{
		return CreateSceneProxy(StaticMeshComponent);
	}
	else if (IsValid(SkeletalMeshComponent))
	{
		bSkeletalMesh = true;
		return CreateSceneProxy(SkeletalMeshComponent);
	}
	// else if (IsValid(CableComponent))
	// {
	// 	return CreateSceneProxy(CableComponent);
	// }
	else
	{
		//UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: The type of ParentMeshComponent : %s can not be supported."), *ParentComponent->GetClass()->GetName());
		return nullptr;
	}
	// return nullptr;
}

FBoxSphereBounds UAnnotationComponent::CalcBounds(const FTransform & LocalToWorld) const
{
	// UMeshComponent* ParentMeshComponent = ParentMeshInfo->GetParentMeshComponent();
	// if (IsValid(ParentMeshComponent))
	// {
	// 	return ParentMeshComponent->CalcBounds(LocalToWorld);
	// }
	// else
	// {
	// 	FBoxSphereBounds DefaultBounds;
	// 	return DefaultBounds;
	// }

	USceneComponent* Parent = this->GetAttachParent();
	UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(Parent);
	if (IsValid(StaticMeshComponent))
	{
		return StaticMeshComponent->CalcBounds(LocalToWorld);
	}

	USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(Parent);
	if (IsValid(SkeletalMeshComponent))
	{
		return SkeletalMeshComponent->CalcBounds(LocalToWorld);
	}

	FBoxSphereBounds DefaultBounds;
	DefaultBounds.Origin = LocalToWorld.GetLocation();
	DefaultBounds.BoxExtent = FVector::ZeroVector;
	DefaultBounds.SphereRadius = 0.f;
	return DefaultBounds;
}

// Extra overhead for the game scene
void UAnnotationComponent::TickComponent(
	float DeltaTime,
	enum ELevelTick TickType,
	FActorComponentTickFunction * ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction); 

	if (bSkeletalMesh)
	{
		MarkRenderStateDirty(); // Without it will break the SkeletalMeshComponent
	}
	/*
	// if (ParentMeshInfo->RequiresUpdate()) 
	// TODO: This sometimes miss a required update, see OWIMap. Not sure why.
	// TODO: Per-frame update is certainly wasted.
	{
		// FIXME: Update the render proxy per frame will cause jittering on the material.
		ParentMeshInfo = MakeShareable(new FParentMeshInfo(this->GetAttachParent()));
	}
	*/
}


void UAnnotationComponent::ForceUpdate()
{
	this->MarkRenderStateDirty();
}
