// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "SkidWheel.h"
#include "UObject/ConstructorHelpers.h"

USkidWheel::USkidWheel()
{
    WheelRadius = 17.667f;
    WheelWidth = 38.0f;
    WheelMass = 5.0f;
    bAffectedByHandbrake = true;
    bAffectedByBrake = true;
    bAffectedByEngine = true;
    bABSEnabled = true;
    bTractionControlEnabled = false;
    bAffectedBySteering = false;
    MaxSteerAngle = 50.f;
    AxleType = EAxleType::Undefined;
    SlipThreshold = 100.0f;
    SkidThreshold = 100.0f;
    SuspensionForceOffset = FVector(0.0f, 0.0f, 0.0f);
    SuspensionMaxRaise = 1.0f;
    SuspensionMaxDrop = 1.0f;
    SuspensionDampingRatio = 0.5f;
    SpringRate = 100;
    SuspensionSmoothing = 1;
}
