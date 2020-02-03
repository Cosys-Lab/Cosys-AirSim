// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "UnrealSensorFactory.h"
#include "UnrealSensors/UnrealDistanceSensor.h"
#include "UnrealSensors/UnrealLidarSensor.h"
#include "UnrealSensors/UnrealGPULidarSensor.h"
#include "UnrealSensors/UnrealEchoSensor.h"
#include "vehicles/AirSimVehicle.h"

UnrealSensorFactory::UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform)
{
	TMap<FString, AActor*> actors;
	actors.Add(TEXT(""), actor);
	setActors(actors, ned_transform);
}

UnrealSensorFactory::UnrealSensorFactory(TMap<FString, AActor*> actors)
{
	setActors(actors, nullptr);
}

std::unique_ptr<msr::airlib::SensorBase> UnrealSensorFactory::createSensorFromSettings(
	const AirSimSettings::SensorSetting* sensor_setting) const
{
	using SensorBase = msr::airlib::SensorBase;

	FString attach_actor = FString(sensor_setting->attach_link.c_str());
	AActor* attachActor = this->actors_[attach_actor];

	switch (sensor_setting->sensor_type) {
	case SensorBase::SensorType::Distance:
		return std::unique_ptr<UnrealDistanceSensor>(new UnrealDistanceSensor(
			*static_cast<const AirSimSettings::DistanceSetting*>(sensor_setting), attachActor, ned_transform_));
	case SensorBase::SensorType::Lidar:
		return std::unique_ptr<UnrealLidarSensor>(new UnrealLidarSensor(
			*static_cast<const AirSimSettings::LidarSetting*>(sensor_setting), attachActor, ned_transform_));
	case SensorBase::SensorType::GPULidar:
		return std::unique_ptr<UnrealGPULidarSensor>(new UnrealGPULidarSensor(
			*static_cast<const AirSimSettings::GPULidarSetting*>(sensor_setting), attachActor, ned_transform_));
	case SensorBase::SensorType::Echo:
		return std::unique_ptr<UnrealEchoSensor>(new UnrealEchoSensor(
			*static_cast<const AirSimSettings::EchoSetting*>(sensor_setting), attachActor, ned_transform_));
	default:
		return msr::airlib::SensorFactory::createSensorFromSettings(sensor_setting);
	}
}

void UnrealSensorFactory::setActors(TMap<FString, AActor*> actors, const NedTransform* ned_transform)
{
	this->actors_ = actors;
	this->ned_transform_ = ned_transform;
}



