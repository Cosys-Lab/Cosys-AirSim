#pragma once

#include "Runtime/Engine/Public/EngineUtils.h"
#include "AirLib/include/vehicles/urdfbot/api/UrdfBotRpcLibServer.hpp"
#include "AirBlueprintLib.h"
#include "SimMode/SimModeWorldBase.h"
#include "api/VehicleSimApiBase.hpp"
#include "UrdfBotPawn.h"
#include "UrdfBotSimApi.h"
#include "UrdfBotApi.h"
#include "SimModeUrdfBot.generated.h"

UCLASS()
class AIRSIM_API ASimModeUrdfBot : public ASimModeBase
{
    GENERATED_BODY()

    public:
        ASimModeUrdfBot();

        virtual void BeginPlay() override;

        virtual void Tick(float DeltaSeconds) override;
        virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

        virtual bool isPaused() const override;
        virtual void pause(bool is_paused) override;
        virtual void continueForTime(double seconds) override;

        virtual bool isUrdf() override { return true; }

    protected:
        virtual void setupClockSpeed() override;
        virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;
        virtual void setupVehiclesAndCamera() override;
        virtual void getExistingVehiclePawns(TArray<AirsimVehicle*>& pawns) const override;


    private:
        typedef AUrdfBotPawn TVehiclePawn;

        void cycleVisibleCameraForward();
        void cycleVisibleCameraBackward();
        void cycleVisibleCamera(bool forward);
        void initializePauseState();

        int camera_index_ = 0;

        std::vector<std::unique_ptr<msr::airlib::VehicleSimApiBase> > vehicle_sim_apis_;

        TArray<APIPCamera*> cameras_;

        TArray<AActor*> spawned_actors_;

        std::atomic<float> current_clockspeed_;
        std::atomic<msr::airlib::TTimeDelta> pause_period_;
        std::atomic<msr::airlib::TTimePoint> pause_period_start_;

};