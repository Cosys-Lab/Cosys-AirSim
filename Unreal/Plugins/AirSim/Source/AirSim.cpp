// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirSim.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Modules/ModuleInterface.h"

class FAirSim : public IModuleInterface
{
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
#if WITH_EDITOR
IMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl, AirSim, "AirSim")
#else
IMPLEMENT_MODULE(FAirSim, AirSim)
#endif
void FAirSim::StartupModule()
{
    //plugin startup
    UE_LOG(LogTemp, Log, TEXT("StartupModule: AirSim plugin"));
}

void FAirSim::ShutdownModule()
{
    //plugin shutdown
}