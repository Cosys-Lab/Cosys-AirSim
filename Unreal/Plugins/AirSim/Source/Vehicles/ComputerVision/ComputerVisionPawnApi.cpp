#include "ComputerVisionPawnApi.h"
#include "AirBlueprintLib.h"

ComputerVisionPawnApi::ComputerVisionPawnApi(AComputerVisionPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
                                            msr::airlib::ComputerVisionApiBase* vehicle_api)
    : pawn_(pawn), pawn_kinematics_(pawn_kinematics), vehicle_api_(vehicle_api)
{
}


msr::airlib::ComputerVisionApiBase::ComputerVisionState ComputerVisionPawnApi::getComputerVisionState() const
{
    msr::airlib::ComputerVisionApiBase::ComputerVisionState state(
        *pawn_kinematics_,
        msr::airlib::ClockFactory::get()->nowNanos()
    );
    return state;
}

void ComputerVisionPawnApi::reset()
{
    vehicle_api_->reset();
}

void ComputerVisionPawnApi::update(float delta)
{
    vehicle_api_->updateComputerVisionState(getComputerVisionState());
    vehicle_api_->update(delta);
}

ComputerVisionPawnApi::~ComputerVisionPawnApi() = default;
