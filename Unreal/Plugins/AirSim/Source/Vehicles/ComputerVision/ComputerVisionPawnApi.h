#pragma once

#include "vehicles/computervision/api/ComputerVisionApiBase.hpp"
#include "vehicles/computervision/api/ComputerVisionApi.hpp"
#include "physics/Kinematics.hpp"
#include "ComputerVisionPawn.h"


class ComputerVisionPawnApi{
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    ComputerVisionPawnApi(AComputerVisionPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, msr::airlib::ComputerVisionApiBase* vehicle_api);

    msr::airlib::ComputerVisionApiBase::ComputerVisionState getComputerVisionState() const;

    void reset();
    void update(float delta = 0);

    virtual ~ComputerVisionPawnApi();

private:
    AComputerVisionPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::ComputerVisionApiBase* vehicle_api_;
};