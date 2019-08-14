// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CPHuskyWheel.h"
#include "TireConfig.h"
#include "UObject/ConstructorHelpers.h"

UCPHuskyWheel::UCPHuskyWheel()
{
	ShapeRadius = 17.487f;
	ShapeWidth = 11.242f;
	Mass = 20.0f;
	DampingRate = 0.25f;
	bAffectedByHandbrake = true;
	SteerAngle = 10.0f;

	// Setup suspension forces
	SuspensionForceOffset = 0.0f;
	SuspensionMaxRaise = 0.5f;
	SuspensionMaxDrop = 0.5f;
	SuspensionNaturalFrequency = 5.0f;
	SuspensionDampingRatio = 1.05f;

	// Find the tire object and set the data for it
	static ConstructorHelpers::FObjectFinder<UTireConfig> TireData(TEXT("/AirSim/VehicleAdv/CPHusky/CPHusky_TireConfig.CPHusky_TireConfig"));
	TireConfig = TireData.Object;
}
