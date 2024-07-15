#pragma once

#include "CoreMinimal.h"
#include "Materials/Material.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/Texture2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/common_utils/Utils.hpp"
#include "common/AirSimSettings.hpp"
#include "Engine/StaticMeshActor.h"
#include "TextureShuffleActor.generated.h"

UCLASS()
class AIRSIM_API ATextureShuffleActor : public AStaticMeshActor
{
    GENERATED_BODY()

protected:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TextureShuffle)
    UMaterialInterface* DynamicMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TextureShuffle)
    TArray<UTexture2D*> SwappableTextures;

public:
    UFUNCTION(BlueprintNativeEvent)
    void SwapTexture(int tex_id = 0, int component_id = 0, int material_id = 0);

private:
    bool MaterialCacheInitialized = false;
    int NumComponents = -1;

    UPROPERTY()
    TArray<UMaterialInstanceDynamic*> DynamicMaterialInstances;
};