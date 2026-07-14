// Weichao Qiu @ 2017
// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.
#pragma once

#include "Materials/MaterialInterface.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Runtime/Engine/Classes/Components/StaticMeshComponent.h"
#include "Runtime/Engine/Public/SkeletalRenderPublic.h"

#if ENGINE_MAJOR_VERSION >= 5
// different header files in UE
#include "Runtime/Engine/Public/StaticMeshSceneProxy.h"
#include "Runtime/Engine/Public/SkeletalMeshSceneProxy.h"
#include "Runtime/Engine/Public/NaniteSceneProxy.h"
#endif

#include "AnnotationComponent.generated.h"

// TODO: Might need to annotate every frame if there are new actors got spawned
/** A proxy component class to render annotation color
 * Should be attached to a MeshComponent to provide annotation color for AnnotationCamSensor
*/
UCLASS(meta = (BlueprintSpawnableComponent))
// class UAnnotationComponent : public UMeshComponent
// Note: if define UAnnotationComponent as a UMeshComponent, then some confusion will raise
// for example: compare the number of MeshComponent and AnnotationComponent
class AIRSIM_API UAnnotationComponent : public UPrimitiveComponent
{
	GENERATED_BODY()

public:
	UAnnotationComponent(const FObjectInitializer& ObjectInitializer);

	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;

	virtual FBoxSphereBounds CalcBounds(const FTransform & LocalToWorld) const override;

	virtual void TickComponent(float DeltaTime,
		enum ELevelTick TickType,
		FActorComponentTickFunction * ThisTickFunction) override;

	void SetAnnotationColor(FColor AnnotationColor);

	void SetAnnotationTexture(FString NewAnnotationTexturePath);
	void SetAnnotationTexture(UTexture* NewAnnotationTexture);

	FColor GetAnnotationColor();

	FString GetAnnotationTexturePath();

	virtual void OnRegister() override;

	/** Force the component to update to capture changes from the parent */
	void ForceUpdate();

public:
	// Simple classification for foliage-related mesh components.
	enum class EFoliageComponentType : uint8
	{
		None = 0,
		StaticFoliage,
		SkeletalFoliage,
		InstancedSkeletalFoliage
	};

	EFoliageComponentType GetLastDetectedFoliageType() const;

private:
	// FParentMeshInfo ParentMeshInfo;
	// TSharedPtr<class FParentMeshInfo> ParentMeshInfo;

	UPROPERTY()
	UMaterial* AnnotationMaterial;
	UMaterial* SphereMaterial;

	UPROPERTY()
	UMaterialInstanceDynamic* AnnotationMID;

	FColor AnnotationColor;
	FString AnnotationTexturePath;

	bool bSkeletalMesh; // indicate whether this is for a SkeletalMesh
	bool bTexture; // indicate if this is a texture annotation component
	EFoliageComponentType last_foliage_type_;

	static EFoliageComponentType ClassifyFoliageType(const USceneComponent* Component);
	static bool IsNaniteSkeletalMesh(const USkeletalMeshComponent* SkeletalMeshComponent);
	FPrimitiveSceneProxy* CreateSceneProxy(UStaticMeshComponent* StaticMeshComponent);
	FPrimitiveSceneProxy* CreateSceneProxy(USkeletalMeshComponent* SkeletalMeshComponent);
	FPrimitiveSceneProxy* CreateSceneProxyNaniteSkeletal(USkeletalMeshComponent* SkeletalMeshComponent);
	FPrimitiveSceneProxy* CreateSceneProxyNaniteInstancedSkeletal(USkinnedMeshComponent* InstancedMeshComponent);
};
