// Developed by CoSys-Lab, University of Antwerp. Part of Flanders Make HySLAM Project

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RandomPropSpawner.generated.h"

UCLASS()
class HYSLAM_API ARandomPropSpawner : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARandomPropSpawner();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	/** Set to true to allow nothing to spawn */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Random Spawn Parameters")
		bool AllowNone;
	/** Maximum rotation (both positive and negative) change around the Z-axis that can be randomly added. In degrees */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Random Spawn Parameters")
		float MaximumRotationOffset;
	/** Maximum position (both positive and negative) change that can be randomly added along the X-axis. In centimeters  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Random Spawn Parameters")
		float MaximumXPositionOffset;
	/** Maximum position (both positive and negative) change that can be randomly added along the Y-axis. In centimeters  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Random Spawn Parameters")
		float MaximumYPositionOffset;
};
