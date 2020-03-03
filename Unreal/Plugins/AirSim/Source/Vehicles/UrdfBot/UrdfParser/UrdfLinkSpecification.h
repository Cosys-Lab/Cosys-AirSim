#pragma once

#include "CoreMinimal.h"
#include "UrdfOrigin.h"
#include "UrdfGeometry.h"
#include "UrdfMaterialSpecification.h"
#include "common/VectorMath.hpp"

class AIRSIM_API UrdfLinkInertialSpecification
{
	public:
		UrdfOrigin Origin;
		float Mass;
		msr::airlib::VectorMathf::Matrix3x3f Inertia;
};

class AIRSIM_API UrdfLinkVisualSpecification
{
	public:
		FString Name;
		UrdfOrigin Origin;
		UrdfGeometry* Geometry;
		FString MaterialName;
};

class AIRSIM_API UrdfLinkCollisionSpecification
{
	public:
		FString Name;
		UrdfOrigin Origin;
		UrdfGeometry* Geometry;
};

class AIRSIM_API UrdfLinkSpecification
{
	public:
		FString Name;
		UrdfLinkInertialSpecification* InertialSpecification = nullptr;
		UrdfLinkVisualSpecification* VisualSpecification = nullptr;
		UrdfLinkCollisionSpecification* CollisionSpecification = nullptr;

		UrdfLinkSpecification* ParentLink;
		TArray<TPair<UrdfLinkSpecification*, class UrdfJointSpecification*> > Children;
};