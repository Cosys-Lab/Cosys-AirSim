// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.


#include "Annotation/AnnotationComponent.h"
// Overwrite the material

#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Runtime/Engine/Public/Materials/Material.h"
#include "Runtime/Engine/Public/Materials/MaterialInstanceDynamic.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Engine/Classes/Components/SkeletalMeshComponent.h"
#include "Runtime/Launch/Resources/Version.h"
#include "Runtime/Engine/Public/MaterialShared.h"
#include "Runtime/Engine/Classes/Engine/Engine.h"
#include "SceneView.h"
#include "AirBlueprintLib.h"

#if ENGINE_MAJOR_VERSION >= 5
//different header files in UE
#include "Runtime/Engine/Public/StaticMeshSceneProxy.h"
#include "Runtime/Engine/Public/SkeletalMeshSceneProxy.h"
#include "Runtime/Engine/Public/SkinnedMeshSceneProxyDesc.h"
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
		this->bVerifyUsedMaterials = false;
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

// Nanite skeletal mesh annotation proxy.
// Uses Nanite's own rendering pipeline so it works correctly in annotation cameras
// (ShowFlags.Materials=false does NOT disable Nanite shading — Nanite is gated by NaniteMeshes).
// GetViewRelevance hides this proxy from the main viewport (Materials=true) so no duplicate
// copies appear alongside the original Nanite mesh.
//
// IMPORTANT: The proxy is constructed from SkeletalMeshComponent (for mesh/skin data) but
// overrides ComponentId to use AnnotationComponent's ID. This prevents a PrimitiveComponentId
// conflict with the original Nanite proxy and ensures ShowOnlyComponents/HiddenComponents
// filtering correctly identifies this proxy as belonging to the UAnnotationComponent.
class FNaniteSkeletalAnnotationSceneProxy : public Nanite::FSkinnedSceneProxy
{
public:
	FNaniteSkeletalAnnotationSceneProxy(
		const USkinnedMeshComponent* SkeletalMeshComponent,
		const UPrimitiveComponent* AnnotationComponent,
		FSkeletalMeshRenderData* InRenderData,
		UMaterialInterface* AnnotationMID
	)
		: Nanite::FSkinnedSceneProxy(
			CreateNaniteMaterialAudit(SkeletalMeshComponent),
			CreateAnnotationDesc(SkeletalMeshComponent, AnnotationComponent),
			InRenderData,
			true)
	{
		if (!IsValid(AnnotationMID))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Nanite skeletal annotation material is invalid"));
			return;
		}

		FMaterialRenderProxy* AnnotationRenderProxy = AnnotationMID->GetRenderProxy();
		for (Nanite::FSceneProxyBase::FMaterialSection& MaterialSection : GetMaterialSections())
		{
			if (!MaterialSection.bHidden)
			{
				MaterialSection.ShadingMaterialProxy = AnnotationRenderProxy;
			}
		}
		OnMaterialsUpdated();
	}

	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
	{
		// Hide from main pass and RGB cameras (Materials=true) to avoid duplicating the original mesh.
		// Annotation cameras set Materials=false — Nanite still shades normally in that mode,
		// so the annotation color is correctly applied there.
		if (View->Family->EngineShowFlags.Materials)
		{
			FPrimitiveViewRelevance ViewRelevance;
			ViewRelevance.bDrawRelevance = 0;
			return ViewRelevance;
		}
		return Nanite::FSkinnedSceneProxy::GetViewRelevance(View);
	}

private:
	static Nanite::FMaterialAudit CreateNaniteMaterialAudit(const USkinnedMeshComponent* Component)
	{
		Nanite::FMaterialAudit Audit;
		if (Component)
		{
			Nanite::AuditMaterials(Component, Audit, false);
		}
		return Audit;
	}

	// Creates a proxy desc from the skeletal mesh but overrides ComponentId to match
	// the annotation component. This ensures the proxy is correctly identified for
	// ShowOnlyComponents/HiddenComponents filtering without conflicting with the
	// original Nanite skeletal proxy (which uses SkeletalMeshComponent's ComponentId).
	static FSkinnedMeshSceneProxyDesc CreateAnnotationDesc(
		const USkinnedMeshComponent* SkeletalMeshComponent,
		const UPrimitiveComponent* AnnotationComponent)
	{
		FSkinnedMeshSceneProxyDesc Desc(SkeletalMeshComponent);
		Desc.ComponentId = AnnotationComponent->GetPrimitiveSceneId();
		return Desc;
	}
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
		ViewRelevance.bDrawRelevance = 0;
		return ViewRelevance;
	}
	else
	{
		return FSkeletalMeshSceneProxy::GetViewRelevance(View);
	}
}

// FString MeterialPath = TEXT("MaterialInstanceConstant'/UnrealCV/AnnotationColor_Inst.AnnotationColor_Inst'");
UAnnotationComponent::UAnnotationComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
	  // , ParentMeshInfo(nullptr)
{
	bSkeletalMesh = false;
	bTexture = false;
	last_foliage_type_ = EFoliageComponentType::None;

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

	// Ensure the annotation material supports Nanite skeletal mesh shading.
	// Nanite checks both MATUSAGE_Nanite and MATUSAGE_SkeletalMesh (NaniteResources.cpp:2535).
	// In the editor, CheckMaterialUsage sets the flag and triggers a one-time shader recompile if needed.
	if (AnnotationMaterial)
	{
		AnnotationMaterial->CheckMaterialUsage(MATUSAGE_Nanite);
		AnnotationMaterial->CheckMaterialUsage(MATUSAGE_SkeletalMesh);
	}

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

UAnnotationComponent::EFoliageComponentType UAnnotationComponent::GetLastDetectedFoliageType() const
{
	return last_foliage_type_;
}

UAnnotationComponent::EFoliageComponentType UAnnotationComponent::ClassifyFoliageType(const USceneComponent* Component)
{
	const UMeshComponent* MeshComponent = Cast<UMeshComponent>(Component);
	if (!IsValid(MeshComponent))
	{
		return EFoliageComponentType::None;
	}

	const FString ClassName = MeshComponent->GetClass()->GetName().ToLower();
	const FString ComponentName = MeshComponent->GetName().ToLower();
	const AActor* Owner = MeshComponent->GetOwner();
	const FString OwnerName = Owner ? Owner->GetName().ToLower() : TEXT("");
	const FString OwnerClassName = Owner ? Owner->GetClass()->GetName().ToLower() : TEXT("");

	const bool bFoliageLike =
		ClassName.Contains(TEXT("foliage")) ||
		ClassName.Contains(TEXT("vegetation")) ||
		ComponentName.Contains(TEXT("foliage")) ||
		ComponentName.Contains(TEXT("vegetation")) ||
		OwnerName.Contains(TEXT("foliage")) ||
		OwnerName.Contains(TEXT("vegetation")) ||
		OwnerClassName.Contains(TEXT("foliage")) ||
		OwnerClassName.Contains(TEXT("vegetation"));

	if (!bFoliageLike)
	{
		return EFoliageComponentType::None;
	}

	if (Cast<UStaticMeshComponent>(MeshComponent))
	{
		return EFoliageComponentType::StaticFoliage;
	}

	if (Cast<USkeletalMeshComponent>(MeshComponent))
	{
		return EFoliageComponentType::SkeletalFoliage;
	}

	const bool bInstancedSkeletalLike =
		ClassName.Contains(TEXT("instancedskeletal")) ||
		ClassName.Contains(TEXT("instancedskinned")) ||
		(ClassName.Contains(TEXT("instanced")) && (ClassName.Contains(TEXT("skeletal")) || ClassName.Contains(TEXT("skinned")))) ||
		ComponentName.Contains(TEXT("instancedskeletal")) ||
		ComponentName.Contains(TEXT("instancedskinned"));

	if (bInstancedSkeletalLike)
	{
		return EFoliageComponentType::InstancedSkeletalFoliage;
	}

	return EFoliageComponentType::None;
}

bool UAnnotationComponent::IsNaniteSkeletalMesh(const USkeletalMeshComponent* SkeletalMeshComponent)
{
	if (!IsValid(SkeletalMeshComponent))
	{
		return false;
	}
	// USkeletalMesh::IsNaniteEnabled()/NaniteSettings reflect the editor-only build *intent* and are
	// compiled out entirely in packaged/cooked builds (gated behind WITH_EDITORONLY_DATA), and
	// USkeletalMesh::HasValidNaniteData() turned out to be private. FSkeletalMeshRenderData's own
	// HasValidNaniteData() is public and reflects the actual cooked render data, and is available in
	// every build configuration, so it's used here instead.
	const FSkeletalMeshRenderData* SkelMeshRenderData = SkeletalMeshComponent->GetSkeletalMeshRenderData();
	return SkelMeshRenderData && SkelMeshRenderData->HasValidNaniteData();
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxyNaniteSkeletal(USkeletalMeshComponent* SkeletalMeshComponent)
{
	FSkeletalMeshRenderData* SkelMeshRenderData = SkeletalMeshComponent->GetSkeletalMeshRenderData();
	if (!SkelMeshRenderData || !SkelMeshRenderData->LODRenderData.IsValidIndex(SkeletalMeshComponent->GetPredictedLODLevel()))
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Nanite skeletal proxy creation skipped for %s - render data not ready (LOD level %d)"),
			*SkeletalMeshComponent->GetName(), SkeletalMeshComponent->GetPredictedLODLevel());
		return nullptr;
	}
	if (!IsValid(AnnotationMID))
	{
		return nullptr;
	}
	UE_LOG(LogTemp, Verbose, TEXT("AirSim Annotation: Creating Nanite skeletal annotation proxy for %s"), *SkeletalMeshComponent->GetName());
	// Pass 'this' (UAnnotationComponent) as the annotation component so the proxy gets the
	// correct PrimitiveComponentId, preventing conflicts with the original Nanite proxy.
	return ::new FNaniteSkeletalAnnotationSceneProxy(SkeletalMeshComponent, this, SkelMeshRenderData, AnnotationMID);
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxyNaniteInstancedSkeletal(USkinnedMeshComponent* InstancedMeshComponent)
{
	FSkeletalMeshRenderData* SkelMeshRenderData = InstancedMeshComponent->GetSkeletalMeshRenderData();
	if (!SkelMeshRenderData || SkelMeshRenderData->LODRenderData.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Nanite instanced skinned proxy creation skipped for %s - render data not ready"),
			*InstancedMeshComponent->GetName());
		return nullptr;
	}
	if (!IsValid(AnnotationMID))
	{
		return nullptr;
	}
	// MeshObject must be valid — it's set up by the component's CreateRenderState_Concurrent.
	// If null, the render state hasn't been created yet; return nullptr and rely on TickComponent to retry.
	if (!InstancedMeshComponent->MeshObject)
	{
		return nullptr;
	}
	// See the comment in IsNaniteSkeletalMesh() above: FSkeletalMeshRenderData::HasValidNaniteData()
	// (already fetched above as SkelMeshRenderData) is used instead of the editor-only/private
	// USkeletalMesh accessors so this also compiles/works in packaged builds.
	if (!SkelMeshRenderData->HasValidNaniteData())
	{
		UE_LOG(LogTemp, Verbose, TEXT("AirSim Annotation: Skipping non-Nanite instanced skinned mesh %s"), *InstancedMeshComponent->GetName());
		return nullptr;
	}
	UE_LOG(LogTemp, Verbose, TEXT("AirSim Annotation: Creating Nanite instanced skeletal annotation proxy for %s"), *InstancedMeshComponent->GetName());
	// Share the existing MeshObject (already set up by the component's CreateRenderState_Concurrent)
	// via FSkinnedMeshSceneProxyDesc. This means all instances' skinning data is available to Nanite.
	return ::new FNaniteSkeletalAnnotationSceneProxy(InstancedMeshComponent, this, SkelMeshRenderData, AnnotationMID);
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy(UStaticMeshComponent* StaticMeshComponent)
{
	UMaterialInterface* ProxyMaterial = AnnotationMID; // Material Instance Dynamic
	UStaticMesh* ParentStaticMesh = StaticMeshComponent->GetStaticMesh();
	if (ParentStaticMesh == NULL
		|| ParentStaticMesh->GetRenderData() == NULL
		|| ParentStaticMesh->GetRenderData()->LODResources.Num() == 0)
	{
		return NULL;
	}

	UE_LOG(LogTemp, VeryVerbose, TEXT("AirSim Annotation: Creating FStaticAnnotationSceneProxy for %s"), *StaticMeshComponent->GetName());
	FPrimitiveSceneProxy* Proxy = ::new FStaticAnnotationSceneProxy(StaticMeshComponent, false, ProxyMaterial);
	return Proxy;
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy(USkeletalMeshComponent* SkeletalMeshComponent)
{
	// Nanite skeletal meshes must use FNaniteSkeletalAnnotationSceneProxy.
	// FSkeletalMeshSceneProxy cannot be used with FSkeletalMeshObjectNanite because
	// Nanite does not initialize the traditional GPU skin vertex factories (only for ray tracing),
	// leading to null uniform buffer crashes in the non-Nanite rendering path.
	if (IsNaniteSkeletalMesh(SkeletalMeshComponent))
	{
		return CreateSceneProxyNaniteSkeletal(SkeletalMeshComponent);
	}

	UMaterialInterface* ProxyMaterial = AnnotationMID;
	FSkeletalMeshRenderData* SkelMeshRenderData = SkeletalMeshComponent->GetSkeletalMeshRenderData();

	if (SkelMeshRenderData
		&& SkelMeshRenderData->LODRenderData.IsValidIndex(SkeletalMeshComponent->GetPredictedLODLevel())
		&& SkeletalMeshComponent->MeshObject)
	{
		return new FSkeletalAnnotationSceneProxy(SkeletalMeshComponent, SkelMeshRenderData, ProxyMaterial);
	}

	return nullptr;
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy()
{
	USceneComponent* ParentComponent = this->GetAttachParent();
	if (!IsValid(ParentComponent))
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Parent component is invalid."));
		return nullptr;
	}

	last_foliage_type_ = ClassifyFoliageType(ParentComponent);

	UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(ParentComponent);
	USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(ParentComponent);
	// UInstancedSkinnedMeshComponent inherits from USkinnedMeshComponent but NOT USkeletalMeshComponent.
	// We detect it without including its experimental header (which causes C3837 on MSVC) by checking
	// that the component is a USkinnedMeshComponent but not a USkeletalMeshComponent.
	USkinnedMeshComponent* SkinnedMeshComponent = Cast<USkinnedMeshComponent>(ParentComponent);
	const bool bIsInstancedSkinnedMesh = IsValid(SkinnedMeshComponent) && !IsValid(SkeletalMeshComponent);

	if (IsValid(StaticMeshComponent))
	{
		bSkeletalMesh = false;
		return CreateSceneProxy(StaticMeshComponent);
	}
	else if (bIsInstancedSkinnedMesh)
	{
		FPrimitiveSceneProxy* Proxy = CreateSceneProxyNaniteInstancedSkeletal(SkinnedMeshComponent);
		bSkeletalMesh = (Proxy != nullptr);
		return Proxy;
	}
	else if (IsValid(SkeletalMeshComponent))
	{
		FPrimitiveSceneProxy* Proxy = CreateSceneProxy(SkeletalMeshComponent);
		bSkeletalMesh = (Proxy != nullptr);
		return Proxy;
	}
	else
	{
		return nullptr;
	}
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

	USkinnedMeshComponent* SkinnedMeshComponent = Cast<USkinnedMeshComponent>(Parent);
	if (IsValid(SkinnedMeshComponent))
	{
		return SkinnedMeshComponent->CalcBounds(LocalToWorld);
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
		USceneComponent* ParentComponent = this->GetAttachParent();
		// Use USkinnedMeshComponent to cover both USkeletalMeshComponent and UInstancedSkinnedMeshComponent
		USkinnedMeshComponent* SkinnedMeshComponent = Cast<USkinnedMeshComponent>(ParentComponent);
		if (SkinnedMeshComponent)
		{
			// Update render state for regular, Nanite, and instanced skeletal meshes
			// This ensures animations and deformations are properly synced
			MarkRenderStateDirty();
		}
	}
}


void UAnnotationComponent::ForceUpdate()
{
	this->MarkRenderStateDirty();
}


