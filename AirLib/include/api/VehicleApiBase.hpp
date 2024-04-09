// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleApiBase_hpp
#define air_VehicleApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/Common.hpp"
#include "common/Waiter.hpp"
#include "safety/SafetyEval.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/lidar/GPULidarBase.hpp"
#include "sensors/lidar/LidarBase.hpp"
#include "sensors/echo/EchoBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/distance/DistanceBase.hpp"
#include "sensors/template/SensorTemplateBase.hpp"
#include "sensors/MarLocUwb/MarLocUwbBase.hpp"
#include "sensors/wifi/WifiBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

    /*
Vehicle controller allows to obtain state from vehicle and send control commands to the vehicle.
State can include many things including sensor data, logs, estimated state from onboard computer etc.
Control commands can be low level actuation commands or high level movement commands.
The base class defines usually available methods that all vehicle controllers may implement.
Some methods may not be applicable to specific vehicle in which case an exception may be raised or call may be ignored.
*/
    class VehicleApiBase : public UpdatableObject
    {
    public:
        virtual void enableApiControl(bool is_enabled) = 0;
        virtual bool isApiControlEnabled() const = 0;
        virtual bool armDisarm(bool arm) = 0;
        virtual GeoPoint getHomeGeoPoint() const = 0;

        virtual void update(float delta = 0) override
        {
            UpdatableObject::update(delta);
        }

        virtual void cancelLastTask()
        {
            //if derived class supports async task then override this method
        }

        virtual bool isReady(std::string& message) const
        {
            unused(message);
            return true;
        }

        virtual bool canArm() const
        {
            return true;
        }

        //if vehicle supports it, call this method to send
        //kinematics and other info to somewhere (ex. log viewer, file, cloud etc)
        virtual void sendTelemetry(float last_interval = -1)
        {
            //no default action
            unused(last_interval);
        }

        //below APIs are used by FastPhysicsEngine
        virtual real_T getActuation(unsigned int actuator_index) const
        {
            unused(actuator_index);
            throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
        }
        virtual size_t getActuatorCount() const
        {
            throw VehicleCommandNotImplementedException("getActuatorCount API is not supported for this vehicle");
        }

        virtual void getStatusMessages(std::vector<std::string>& messages)
        {
            unused(messages);
            //default implementation
        }

        /*
        For RCs, there are two cases: (1) vehicle may be configured to use
        RC bound to its hardware (2) vehicle may be configured to get RC data
        supplied via API calls. Below two APIs are not symmetrical, i.e.,
        getRCData() may or may not return same thing as setRCData().
        */
        //get reading from RC bound to vehicle (if unsupported then RCData::is_valid = false)
        virtual RCData getRCData() const
        {
            static const RCData invalid_rc_data{};
            return invalid_rc_data;
        }

        //set external RC data to vehicle (if unsupported then returns false)
        virtual bool setRCData(const RCData& rc_data)
        {
            unused(rc_data);
            return false;
        }

        // Sensors APIs
        virtual const SensorCollection& getSensors() const
        {
            throw VehicleCommandNotImplementedException("getSensors API is not supported for this vehicle");
        }

        // Lidar APIs
        virtual const LidarData& getLidarData(const std::string& lidar_name) const
        {
            auto* lidar = static_cast<const LidarBase*>(findSensorByName(lidar_name, SensorBase::SensorType::Lidar));
            if (lidar == nullptr)
                throw VehicleControllerException(Utils::stringf("No lidar with name %s exist on vehicle", lidar_name.c_str()));

            return lidar->getOutput();
        }

      	virtual GPULidarData getGPULidarData(const std::string& lidar_name) const
        {
            const GPULidarBase* lidar = nullptr;

            // Find GPU lidar with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of lidars
            uint count_lidars = getSensors().size(SensorBase::SensorType::GPULidar);
            for (uint i = 0; i < count_lidars; i++)
            {
                const GPULidarBase* current_lidar = static_cast<const GPULidarBase*>(getSensors().getByType(SensorBase::SensorType::GPULidar, i));
                if (current_lidar != nullptr && (current_lidar->getName() == lidar_name || lidar_name == ""))
                {
                    lidar = current_lidar;
                    break;
                }
            }
            if (lidar == nullptr)
                throw VehicleControllerException(Utils::stringf("No GPU lidar with name %s exist on vehicle", lidar_name.c_str()));

            return lidar->getOutput();
        }

        // Echo APIs
        virtual EchoData getEchoData(const std::string& echo_name) const
        {
            const EchoBase* echo = nullptr;

            // Find echo with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of echos
            uint count_echos = getSensors().size(SensorBase::SensorType::Echo);
            for (uint i = 0; i < count_echos; i++)
            {
                const EchoBase* current_echo = static_cast<const EchoBase*>(getSensors().getByType(SensorBase::SensorType::Echo, i));
                if (current_echo != nullptr && (current_echo->getName() == echo_name || echo_name == ""))
                {
                    echo = current_echo;
                    break;
                }
            }
            if (echo == nullptr)
                throw VehicleControllerException(Utils::stringf("No echo with name %s exist on vehicle", echo_name.c_str()));

            return echo->getOutput();
        }

        virtual void setEchoData(const std::string& echo_name, const EchoData& input) const
        {
            const EchoBase* echo = nullptr;

            // Find echo with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of echos
            uint count_echos = getSensors().size(SensorBase::SensorType::Echo);
            for (uint i = 0; i < count_echos; i++)
            {
                const EchoBase* current_echo = static_cast<const EchoBase*>(getSensors().getByType(SensorBase::SensorType::Echo, i));
                if (current_echo != nullptr && (current_echo->getName() == echo_name || echo_name == ""))
                {
                    echo = current_echo;
                    break;
                }
            }
            if (echo == nullptr)
                throw VehicleControllerException(Utils::stringf("No echo with name %s exist on vehicle", echo_name.c_str()));

            echo->setInput(input);
        }

        // Echo APIs
        virtual SensorTemplateData getSensorTemplateData(const std::string& sensor_name) const
        {
            const SensorTemplateBase* sensor = nullptr;

            // Find echo with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of echos
            uint template_sensor_count = getSensors().size(SensorBase::SensorType::SensorTemplate);
            for (uint i = 0; i < template_sensor_count; i++)
            {
                const SensorTemplateBase* current_sensor = static_cast<const SensorTemplateBase*>(getSensors().getByType(SensorBase::SensorType::SensorTemplate, i));
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == ""))
                {
                    sensor = current_sensor;
                    break;
                }
            }
            if (sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No echo with name %s exist on vehicle", sensor_name.c_str()));

            return sensor->getOutput();
        }

        virtual void setSensorTemplateData(const std::string& sensor_name, const SensorTemplateData& input) const
        {
            const SensorTemplateBase* sensor = nullptr;

            // Find echo with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of echos
            uint count_echos = getSensors().size(SensorBase::SensorType::SensorTemplate);
            for (uint i = 0; i < count_echos; i++)
            {
                const SensorTemplateBase* current_sensor = static_cast<const SensorTemplateBase*>(getSensors().getByType(SensorBase::SensorType::SensorTemplate, i));
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == ""))
                {
                    sensor = current_sensor;
                    break;
                }
            }
            if (sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No echo with name %s exist on vehicle", sensor_name.c_str()));

            sensor->setInput(input);
        }

        virtual MarLocUwbSensorData getUWBSensorData(const std::string& sensor_name) const
        {
            const MarLocUwbBase* sensor = nullptr;

            // Find uwb with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of uwbs
            uint uwb_sensor_count = getSensors().size(SensorBase::SensorType::MarlocUwb);
            for (uint i = 0; i < uwb_sensor_count; i++)
            {
                const MarLocUwbBase* current_sensor = static_cast<const MarLocUwbBase*>(getSensors().getByType(SensorBase::SensorType::MarlocUwb, i));
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == ""))
                {
                    sensor = current_sensor;
                    break;
                }
            }
            if (sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No uwb sensor with name %s exist on vehicle", sensor_name.c_str()));
            return sensor->getOutput();
        }

        virtual MarLocUwbReturnMessage getUWBData(const std::string& sensor_name) const
        {
            MarLocUwbReturnMessage toReturn;                      // The entire DB (ranges and rangeArrays)
            vector<MarLocUwbRangeArray> uwbRangesArray;            // A list of the range arrays
            vector<std::string> processedRangeArrays;              // A list of all RangeArray PK's we already have
                                                                   // The PK of a range array is the tag/beacon ID
            vector<MarLocUwbRange> uwbRanges;                      // A list of the ranges
            //vector<int> processedAnchorIDs;                        // A list of all anchors PK's already processed

            const MarLocUwbBase* sensor = nullptr;

            uint uwb_sensor_count = getSensors().size(SensorBase::SensorType::MarlocUwb);
            int RangeNumber = 0;

            // Find uwb with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of uwbs

            for (uint i = 0; i < uwb_sensor_count; i++)
            {
                const MarLocUwbBase* current_sensor = static_cast<const MarLocUwbBase*>(getSensors().getByType(SensorBase::SensorType::MarlocUwb, i));
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == ""))
                {
                    sensor = current_sensor;
                    MarLocUwbSensorData output = sensor->getOutput(); // Get sensor output
                    vector<int> ranges;

                    toReturn.mura_tagId.push_back(std::to_string(sensor->getID()));
                    toReturn.mura_tagPosX.push_back(output.pose.position[0]);
                    toReturn.mura_tagPosY.push_back(output.pose.position[1]);
                    toReturn.mura_tagPosZ.push_back(output.pose.position[2]);

                    // Create a new range for all beacon/tag hits in the current sensor/anchor outputs
                    for (int itId = 0; itId < output.beaconsActiveID.size(); itId++) {
                        ranges.push_back(RangeNumber++);
                        toReturn.mur_time_stamp.push_back(output.time_stamp);
                        toReturn.mur_anchorId.push_back(output.beaconsActiveID[itId]);
                        toReturn.mur_rssi.push_back(output.beaconsActiveRssi[itId]);
                        toReturn.mur_anchorPosX.push_back(output.beaconsActivePosX[itId]/100);
                        toReturn.mur_anchorPosY.push_back(output.beaconsActivePosY[itId]/100);
                        toReturn.mur_anchorPosZ.push_back(output.beaconsActivePosZ[itId]/-100);
                        toReturn.mur_valid_range.push_back(true);
                        toReturn.mur_distance.push_back(output.beaconsActiveDistance[itId]);
                    }
                    toReturn.mura_ranges.push_back(ranges);
                }
            }
            if (sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No uwb sensor with name %s exist on vehicle", sensor_name.c_str()));

            return toReturn;
        }

        //WIFI API
        virtual WifiSensorData getWifiSensorData(const std::string& sensor_name) const
        {
            const WifiBase* sensor = nullptr;

            // Find wifi with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of wifis
            uint wifi_sensor_count = getSensors().size(SensorBase::SensorType::Wifi);
            for (uint i = 0; i < wifi_sensor_count; i++)
            {
                const WifiBase* current_sensor = static_cast<const WifiBase*>(getSensors().getByType(SensorBase::SensorType::Wifi, i));
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == ""))
                {
                    sensor = current_sensor;
                    break;
                }
            }
            if (sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No wifi sensor with name %s exist on vehicle", sensor_name.c_str()));

            return sensor->getOutput();
        }

        virtual WifiReturnMessage getWifiData(const std::string& sensor_name) const
        {
            WifiReturnMessage toReturn;                           // The entire DB (ranges and rangeArrays)
            vector<WifiRangeArray> wifiRangesArray;                // A list of the range arrays
            vector<std::string> processedRangeArrays;              // A list of all RangeArray PK's we already have
                                                                   // The PK of a range array is the tag/beacon ID
            vector<WifiRange> wifiRanges;                          // A list of the ranges
            const WifiBase* sensor = nullptr;

            uint wifi_sensor_count = getSensors().size(SensorBase::SensorType::Wifi);

            int RangeNumber = 0;

            // Find wifi sensor with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of sensors

            for (uint i = 0; i < wifi_sensor_count; i++)
            {
                const WifiBase* current_sensor = static_cast<const WifiBase*>(getSensors().getByType(SensorBase::SensorType::Wifi, i));
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == ""))
                {
                    sensor = current_sensor;
                    WifiSensorData output = sensor->getOutput(); // Get sensor output
                    vector<int> ranges;

                    toReturn.wra_tagId.push_back(std::to_string(sensor->getID()));
                    toReturn.wra_tagPosX.push_back(output.pose.position[0]);
                    toReturn.wra_tagPosY.push_back(output.pose.position[1]);
                    toReturn.wra_tagPosZ.push_back(output.pose.position[2]);

                    // Create a new range for all beacon/tag hits in the current sensor/anchor outputs
                    for (int itId = 0; itId < output.beaconsActiveID.size(); itId++) {
                        ranges.push_back(RangeNumber++);
                        toReturn.wr_time_stamp.push_back(output.time_stamp);
                        toReturn.wr_anchorId.push_back(output.beaconsActiveID[itId]);
                        toReturn.wr_rssi.push_back(output.beaconsActiveRssi[itId]);
                        toReturn.wr_anchorPosX.push_back(output.beaconsActivePosX[itId] / 100);
                        toReturn.wr_anchorPosY.push_back(output.beaconsActivePosY[itId] / 100);
                        toReturn.wr_anchorPosZ.push_back(output.beaconsActivePosZ[itId] / -100);
                        toReturn.wr_valid_range.push_back(true);
                        toReturn.wr_distance.push_back(output.beaconsActiveDistance[itId]);
                    }
                    toReturn.wra_ranges.push_back(ranges);
                }
            }
            if (sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No wifi sensor with name %s exist on vehicle", sensor_name.c_str()));

            return toReturn;
        }

        // IMU API
        virtual const ImuBase::Output& getImuData(const std::string& imu_name) const
        {
            auto* imu = static_cast<const ImuBase*>(findSensorByName(imu_name, SensorBase::SensorType::Imu));
            if (imu == nullptr)
                throw VehicleControllerException(Utils::stringf("No IMU with name %s exist on vehicle", imu_name.c_str()));

            return imu->getOutput();
        }

        // Barometer API
        virtual const BarometerBase::Output& getBarometerData(const std::string& barometer_name) const
        {
            auto* barometer = static_cast<const BarometerBase*>(findSensorByName(barometer_name, SensorBase::SensorType::Barometer));
            if (barometer == nullptr)
                throw VehicleControllerException(Utils::stringf("No barometer with name %s exist on vehicle", barometer_name.c_str()));

            return barometer->getOutput();
        }

        // Magnetometer API
        virtual const MagnetometerBase::Output& getMagnetometerData(const std::string& magnetometer_name) const
        {
            auto* magnetometer = static_cast<const MagnetometerBase*>(findSensorByName(magnetometer_name, SensorBase::SensorType::Magnetometer));
            if (magnetometer == nullptr)
                throw VehicleControllerException(Utils::stringf("No magnetometer with name %s exist on vehicle", magnetometer_name.c_str()));

            return magnetometer->getOutput();
        }

        // Gps API
        virtual const GpsBase::Output& getGpsData(const std::string& gps_name) const
        {
            auto* gps = static_cast<const GpsBase*>(findSensorByName(gps_name, SensorBase::SensorType::Gps));
            if (gps == nullptr)
                throw VehicleControllerException(Utils::stringf("No gps with name %s exist on vehicle", gps_name.c_str()));

            return gps->getOutput();
        }

        // Distance Sensor API
        virtual const DistanceSensorData& getDistanceSensorData(const std::string& distance_sensor_name) const
        {
            auto* distance_sensor = static_cast<const DistanceBase*>(findSensorByName(distance_sensor_name, SensorBase::SensorType::Distance));
            if (distance_sensor == nullptr)
                throw VehicleControllerException(Utils::stringf("No distance sensor with name %s exist on vehicle", distance_sensor_name.c_str()));

            return distance_sensor->getOutput();
        }

        virtual ~VehicleApiBase() = default;

        //exceptions
        class VehicleControllerException : public std::runtime_error
        {
        public:
            VehicleControllerException(const std::string& message)
                : runtime_error(message)
            {
            }
        };

        class VehicleCommandNotImplementedException : public VehicleControllerException
        {
        public:
            VehicleCommandNotImplementedException(const std::string& message)
                : VehicleControllerException(message)
            {
            }
        };

        class VehicleMoveException : public VehicleControllerException
        {
        public:
            VehicleMoveException(const std::string& message)
                : VehicleControllerException(message)
            {
            }
        };

    private:
        const SensorBase* findSensorByName(const std::string& sensor_name, const SensorBase::SensorType type) const
        {
            const SensorBase* sensor = nullptr;

            // Find sensor with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of sensors
            uint count_sensors = getSensors().size(type);
            for (uint i = 0; i < count_sensors; i++) {
                const SensorBase* current_sensor = getSensors().getByType(type, i);
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == "")) {
                    sensor = current_sensor;
                    break;
                }
            }

            return sensor;
        }
    };
}
} //namespace
#endif
