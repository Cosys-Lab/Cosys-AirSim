// Copyright (c) Cosys-Lab, University of Antwerp. 2026
#include "Vehicles/Multirotor/MultirotorPawnEvents.h"

MultirotorPawnEvents::ActuatorsSignal& MultirotorPawnEvents::getActuatorSignal()
{
    return actuator_signal_;
}