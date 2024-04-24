#pragma once

#include "CoreMinimal.h"
#include "ComputerVisionPawn.h"
#include "ComputerVisionPawnApi.h"
#include "PawnEvents.h"
#include "PawnSimApi.h"
#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/computervision/api/ComputerVisionApi.hpp"

class ComputerVisionPawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;
    
public:
    virtual void initialize() override;
    virtual ~ComputerVisionPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    ComputerVisionPawnSimApi(const Params& params);

    virtual void update(float delta = 0) override;
    virtual void reportState(StateReporter& reporter) override;

    virtual std::string getRecordFileLine(bool is_header_line) const override;

    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    msr::airlib::ComputerVisionApiBase* getVehicleApi() const
    {
        return vehicle_api_.get();
    }

    virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
    {
        return vehicle_api_.get();
    }

protected:
    virtual void resetImplementation() override;


private:
    std::unique_ptr<msr::airlib::ComputerVisionApiBase> vehicle_api_;
    std::unique_ptr<ComputerVisionPawnApi> pawn_api_;
    std::vector<std::string> vehicle_api_messages_;
};
