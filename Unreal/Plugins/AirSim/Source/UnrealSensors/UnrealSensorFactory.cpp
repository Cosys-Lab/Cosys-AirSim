// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "UnrealSensorFactory.h"
#include "UnrealSensors/UnrealDistanceSensor.h"
#include "UnrealSensors/UnrealLidarSensor.h"
#include "UnrealSensors/UnrealGPULidarSensor.h"
#include "UnrealSensors/UnrealEchoSensor.h"
#include "UnrealSensors/UnrealSensorTemplate.h"
#include "UnrealSensors/UnrealMarLocUwbSensor.h"
#include "UnrealSensors/UnrealWifiSensor.h"
//#include "Vehicles/AirSimVehicle.h"

UnrealSensorFactory::UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform)
{
    setActor(actor, ned_transform);
}

std::shared_ptr<msr::airlib::SensorBase> UnrealSensorFactory::createSensorFromSettings(
    const AirSimSettings::SensorSetting* sensor_setting) const
{
    using SensorBase = msr::airlib::SensorBase;

    switch (sensor_setting->sensor_type) {
    case SensorBase::SensorType::Distance:
        return std::make_shared<UnrealDistanceSensor>(
            *static_cast<const AirSimSettings::DistanceSetting*>(sensor_setting), actor_, ned_transform_);
    case SensorBase::SensorType::Lidar:
        return std::make_shared<UnrealLidarSensor>(
            *static_cast<const AirSimSettings::LidarSetting*>(sensor_setting), actor_, ned_transform_);
	case SensorBase::SensorType::GPULidar:
		return std::make_shared<UnrealGPULidarSensor>(
			*static_cast<const AirSimSettings::GPULidarSetting*>(sensor_setting), actor_, ned_transform_);
    case SensorBase::SensorType::Echo:
        return std::make_shared<UnrealEchoSensor>(
            *static_cast<const AirSimSettings::EchoSetting*>(sensor_setting), actor_, ned_transform_);
    case SensorBase::SensorType::SensorTemplate:
        return std::make_shared<UnrealSensorTemplate>(
            *static_cast<const AirSimSettings::SensorTemplateSetting*>(sensor_setting), actor_, ned_transform_);
    case SensorBase::SensorType::MarlocUwb:
        return std::make_shared<UnrealMarLocUwbSensor>(
            *static_cast<const AirSimSettings::MarLocUwbSetting*>(sensor_setting), actor_, ned_transform_);
    case SensorBase::SensorType::Wifi:
        return std::make_shared<UnrealWifiSensor>(
            *static_cast<const AirSimSettings::WifiSetting*>(sensor_setting), actor_, ned_transform_);
    default:
        return msr::airlib::SensorFactory::createSensorFromSettings(sensor_setting);
    }
}

void UnrealSensorFactory::setActor(AActor* actor, const NedTransform* ned_transform)
{
    actor_ = actor;
    ned_transform_ = ned_transform;
}
