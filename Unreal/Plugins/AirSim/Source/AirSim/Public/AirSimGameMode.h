// Copyright (c) Cosys-Lab, University of Antwerp. 2026
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameUserSettings.h"
#include "AirSimGameMode.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API AAirSimGameMode : public AGameModeBase
{
public:
    GENERATED_BODY()

    virtual void StartPlay() override;

    AAirSimGameMode(const FObjectInitializer& ObjectInitializer);

    //private:
    //UGameUserSettings* GetGameUserSettings();
};
