#include "PInvokeWrapper.h"

//Function pointers to hold the addresses of the functions that are defined in Unity
bool(*SetPose)(AirSimPose pose, bool ignoreCollision, const char* vehicleName);
AirSimPose(*GetPose)(const char* vehicleName);
AirSimCollisionInfo(*GetCollisionInfo)(const char* vehicleName);
AirSimRCData(*GetRCData)(const char* vehicleName);
AirSimImageResponse(*GetSimImages)(AirSimImageRequest request, const char* vehicleName);
bool(*SetRotorSpeed)(int rotorIndex, RotorInfo rotorInfo, const char* vehicleName);
bool(*SetEnableApi)(bool enableApi, const char* vehicleName);
bool(*SetCarApiControls)(msr::airlib::CarApiBase::CarControls controls, const char* vehicleName);
AirSimCarState(*GetCarState)(const char* vehicleName);
AirSimCameraInfo(*GetCameraInfo)(const char* cameraName, const char* vehicleName);
bool(*SetCameraOrientation)(const char* cameraName, AirSimQuaternion orientation, const char* vehicleName);
bool(*SetSegmentationObjectId)(const char* meshName, int objectId, bool isNameRegex);
int(*GetSegmentationObjectId)(const char* meshName);
bool(*PrintLogMessage) (const char* message, const char* messageParam, const char* vehicleName, int severity);
UnityTransform(*GetTransformFromUnity)(const char* vehicleName);
bool(*Reset)(const char* vehicleName);
AirSimVector(*GetVelocity)(const char* vehicleName);
RayCastHitResult(*GetRayCastHit)(AirSimVector startVec, AirSimVector endVec, const char* vehicleName);
bool(*Pause)(const char* vehicleName, float timeScale);

void InitVehicleManager(
	bool(*setPose)(AirSimPose pose, bool ignoreCollision, const char* vehicleName),
	AirSimPose(*getPose)(const char* vehicleName),
	AirSimCollisionInfo(*getCollisionInfo)(const char* vehicleName),
	AirSimRCData(*getDroneRCData)(const char* vehicleName),
	AirSimImageResponse(*getSimImages)(AirSimImageRequest request, const char* vehicleName),
	bool(*setRotorSpeed)(int rotorIndex, RotorInfo rotorInfo, const char* vehicleName),
	bool(*setEnableApi)(bool enableApi, const char* vehicleName),
	bool(*setCarApiControls)(msr::airlib::CarApiBase::CarControls controls, const char* vehicleName),
	AirSimCarState(*getCarState)(const char* vehicleName),
	AirSimCameraInfo(*getCameraInfo)(const char* cameraName, const char* vehicleName),
	bool(*setCameraOrientation)(const char* cameraName, AirSimQuaternion orientation, const char* vehicleName),
	bool(*setSegmentationObjectId)(const char* meshName, int objectId, bool isNameRegex),
	int(*getSegmentationObjectId)(const char* meshName),
	bool(*printLogMessage) (const char* message, const char* messageParam, const char* vehicleName, int severity),
	UnityTransform(*getTransformFromUnity)(const char* vehicleName),
	bool(*reset)(const char* vehicleName),
	AirSimVector(*getVelocity)(const char* vehicleName),
	RayCastHitResult(*getRayCastHit)(AirSimVector startVec, AirSimVector endVec, const char* vehicleName),
	bool(*pause)(const char* vehicleName, float timeScale)
)
{
	SetPose = setPose;
	GetPose = getPose;
	GetCollisionInfo = getCollisionInfo;
	GetRCData = getDroneRCData;
	GetSimImages = getSimImages;
	SetRotorSpeed = setRotorSpeed;
	SetEnableApi = setEnableApi;
	SetCarApiControls = setCarApiControls;
	GetCarState = getCarState;
	GetCameraInfo = getCameraInfo;
	SetCameraOrientation = setCameraOrientation;
	SetSegmentationObjectId = setSegmentationObjectId;
	GetSegmentationObjectId = getSegmentationObjectId;
	PrintLogMessage = printLogMessage;
	GetTransformFromUnity = getTransformFromUnity;
	Reset = reset;
	GetVelocity = getVelocity;
	GetRayCastHit = getRayCastHit;
	Pause = pause;
}