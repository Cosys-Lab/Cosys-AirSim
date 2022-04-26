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
class VehicleApiBase : public UpdatableObject {
public:
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() const = 0;
    virtual bool armDisarm(bool arm) = 0;
    virtual GeoPoint getHomeGeoPoint() const = 0;

    //default implementation so derived class doesn't have to call on UpdatableObject
    virtual void reset() override
    {
        UpdatableObject::reset();
    }
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
        static const RCData invalid_rc_data {};
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
    virtual LidarData getLidarData(const std::string& lidar_name) const
    {
        const LidarBase* lidar = nullptr;

        // Find lidar with the given name (for empty input name, return the first one found)
        // Not efficient but should suffice given small number of lidars
        uint count_lidars = getSensors().size(SensorBase::SensorType::Lidar);
        for (uint i = 0; i < count_lidars; i++)
        {
            const LidarBase* current_lidar = static_cast<const LidarBase*>(getSensors().getByType(SensorBase::SensorType::Lidar, i));
            if (current_lidar != nullptr && (current_lidar->getName() == lidar_name || lidar_name == ""))
            {
                lidar = current_lidar;
                break;
            }
        }
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

    virtual MarLocUwbReturnMessage2 getUWBData(const std::string& sensor_name) const
    {
        MarLocUwbReturnMessage2 toReturn;                      // The entire DB (ranges and rangeArrays)
        const MarLocUwbBase* sensor = nullptr;                 // The used sensor
        vector<MarLocUwbRangeArray> uwbRangesArray;            // A list of the range arrays
        vector<std::string> processedRangeArrays;              // A list of all RangeArray PK's we already have 
                                                               // The PK of a range array is the tag/beacon ID
        vector<MarLocUwbRange> uwbRanges;                      // A list of the ranges (incl diagnostics)
        //vector<int> processedAnchorIDs;                        // A list of all anchors PK's already processed
        


        uint uwb_sensor_count = getSensors().size(SensorBase::SensorType::MarlocUwb);

        int RangeNumber = 0;

        for (uint i = 0; i < uwb_sensor_count; i++)            // For all uwb sensors
        {                                                      // Get current sensor
            const MarLocUwbBase* current_sensor = static_cast<const MarLocUwbBase*>(getSensors().getByType(SensorBase::SensorType::MarlocUwb, i));
            if (current_sensor != nullptr && (current_sensor->getName().substr(0, 9) == sensor_name || sensor_name == "")) // If current sensor matches
            {
                sensor = current_sensor;
                MarLocUwbSensorData output = sensor->getOutput(); // Get sensor output

                // Create a new range for all beacon/tag hits in the current sensor/anchor outputs
                for (int itId = 0; itId < output.beaconsActiveID.size(); itId++) { 
                    MarLocUwbRange newRange;
                    newRange.time_stamp = output.time_stamp;
                    newRange.anchorId = sensor->getID();
                    newRange.tagId = output.beaconsActiveID[itId];
                    newRange.anchorX = output.pose.position[0];
                    newRange.anchorY = output.pose.position[1];
                    newRange.anchorZ = output.pose.position[2];
                    newRange.valid_range = 1;
                    newRange.distance = output.beaconsActiveDistance[itId];
                    newRange.rssi = output.beaconsActiveRssi[itId];
                    
                    // Test if this ID already exists in the ranges
                    // If it does not yet exist, add it
                    // if it does exist, keep the highest rssi
                    bool rangeExists = 0;
                    for (auto& uwbRange : uwbRanges) {
                        // Match PK (anchorID + tagID)
                        if (uwbRange.anchorId == newRange.anchorId && uwbRange.tagId == newRange.tagId) { // it exists
                            if (newRange.rssi < uwbRange.rssi) {
                                uwbRange.rssi = newRange.rssi;
                            }
                            rangeExists = 1;
                        }
                    }
                    if (!rangeExists) {// it does not yet exist
                        uwbRanges.push_back(newRange);
                        RangeNumber++;

                        // Test if a rangeArray for this beacon/tag already exists
                        if (count(processedRangeArrays.begin(), processedRangeArrays.end(), output.beaconsActiveID[itId]) == 0) { // If this beacon (tag) is not in the list
                            processedRangeArrays.push_back(output.beaconsActiveID[itId]); // Create a new entry in the rangeArray list
                            MarLocUwbRangeArray newUwbRangesArray;

                            newUwbRangesArray.tagId = output.beaconsActiveID[itId];
                            newUwbRangesArray.tagX = output.beaconsActivePosX[itId];
                            newUwbRangesArray.tagY = output.beaconsActivePosY[itId];
                            newUwbRangesArray.tagZ = output.beaconsActivePosZ[itId];

                            uwbRangesArray.push_back(newUwbRangesArray);
                        }

                        //Add this range to the rangeArray[ranges]
                        for (auto& rangeArray : uwbRangesArray) {  // For all rangeArrays
                            if (rangeArray.tagId == output.beaconsActiveID[itId]) { // If beacon/tag matches
                                rangeArray.ranges.push_back(RangeNumber); // 
                                //rangeArray.ranges.push_back(newRange.anchorId);
                                //for (auto& range : uwbRanges) {
                                //    if 
                                //}
                                // don't push the id but the index
                            }
                        }
                    }
                }
            }
        }
        if (sensor == nullptr)
            throw VehicleControllerException(Utils::stringf("No sensor with name %s exist on vehicle", sensor_name.c_str()));


        for (std::vector<MarLocUwbRange>::iterator it = uwbRanges.begin(); it != uwbRanges.end(); ++it) {
            toReturn.mur_time_stamp.push_back(it->time_stamp);
            toReturn.mur_anchorId.push_back(it->anchorId);
            toReturn.mur_anchorX.push_back(it->anchorX);
            toReturn.mur_anchorY.push_back(it->anchorY);
            toReturn.mur_anchorZ.push_back(it->anchorZ);
            toReturn.mur_valid_range.push_back(it->valid_range);
            toReturn.mur_distance.push_back(it->distance);
            toReturn.mur_rssi.push_back(it->rssi);
        }

        for (std::vector<MarLocUwbRangeArray>::iterator it = uwbRangesArray.begin(); it != uwbRangesArray.end(); ++it) {
            toReturn.mura_tagId.push_back(it->tagId);
            toReturn.mura_tagX.push_back(it->tagX);
            toReturn.mura_tagY.push_back(it->tagY);
            toReturn.mura_tagZ.push_back(it->tagZ);
            toReturn.mura_ranges.push_back(it->ranges);
        }
            
        //toReturn.marLocUwbRange = uwbRanges;
        //toReturn.marLocUwbRangeArray = uwbRangesArray;
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

    virtual WifiReturnMessage2 getWifiData(const std::string& sensor_name) const
    {
        WifiReturnMessage2 toReturn;                           // The entire DB (ranges and rangeArrays)
        const WifiBase* sensor = nullptr;                      // The used sensor
        vector<WifiRangeArray> wifiRangesArray;                // A list of the range arrays
        vector<std::string> processedRangeArrays;              // A list of all RangeArray PK's we already have 
                                                               // The PK of a range array is the tag/beacon ID
        vector<WifiRange> wifiRanges;                          // A list of the ranges (incl diagnostics)
        //vector<int> processedAnchorIDs;                        // A list of all anchors PK's already processed
        
        uint wifi_sensor_count = getSensors().size(SensorBase::SensorType::Wifi);

        int RangeNumber = 0;

        for (uint i = 0; i < wifi_sensor_count; i++)           // For all wifi sensors
        {                                                      // Get current sensor
            const WifiBase* current_sensor = static_cast<const WifiBase*>(getSensors().getByType(SensorBase::SensorType::Wifi, i));
            if (current_sensor != nullptr && (current_sensor->getName().substr(0, 10) == sensor_name || sensor_name == "")) // If current sensor matches
            {
                sensor = current_sensor;
                WifiSensorData output = sensor->getOutput(); // Get sensor output

                // Create a new range for all beacon/tag hits in the current sensor/anchor output
                for (int itId = 0; itId < output.beaconsActiveID.size(); itId++) {
                    WifiRange newRange;
                    newRange.time_stamp = output.time_stamp;
                    newRange.anchorId = sensor->getID();
                    newRange.tagId = output.beaconsActiveID[itId];
                    newRange.anchorX = output.pose.position[0];
                    newRange.anchorY = output.pose.position[1];
                    newRange.anchorZ = output.pose.position[2];
                    newRange.valid_range = 1;
                    newRange.distance = output.beaconsActiveDistance[itId];
                    newRange.rssi = output.beaconsActiveRssi[itId];

                    // Test if this ID already exists in the ranges
                    // If it does not yet exist, add it
                    // if it does exist, keep the highest rssi
                    bool rangeExists = 0;
                    for (auto& wifiRange : wifiRanges) {
                        // Match PK (anchorID + tagID)
                        if (wifiRange.anchorId == newRange.anchorId && wifiRange.tagId == newRange.tagId) { // it exists
                            if (newRange.rssi < wifiRange.rssi) {
                                wifiRange.rssi = newRange.rssi;
                            }
                            rangeExists = 1;
                        }
                    }
                    if (!rangeExists) {// it does not yet exist
                        wifiRanges.push_back(newRange);
                        RangeNumber++;

                        // Test if a rangeArray for this beacon/tag already exists
                        if (count(processedRangeArrays.begin(), processedRangeArrays.end(), output.beaconsActiveID[itId]) == 0) { // If this beacon (tag) is not in the list
                            processedRangeArrays.push_back(output.beaconsActiveID[itId]); // Create a new entry in the rangeArray list
                            WifiRangeArray newWifiRangesArray;

                            newWifiRangesArray.tagId = output.beaconsActiveID[itId];
                            newWifiRangesArray.tagX = output.beaconsActivePosX[itId];
                            newWifiRangesArray.tagY = output.beaconsActivePosY[itId];
                            newWifiRangesArray.tagZ = output.beaconsActivePosZ[itId];

                            wifiRangesArray.push_back(newWifiRangesArray);
                        }

                        //Add this range to the rangeArray[ranges]
                        for (auto& rangeArray : wifiRangesArray) {  // For all rangeArrays
                            if (rangeArray.tagId == output.beaconsActiveID[itId]) { // If beacon/tag matches
                                rangeArray.ranges.push_back(RangeNumber); // 
                            }
                        }
                    }
                }
            }
        }
        if (sensor == nullptr)
            throw VehicleControllerException(Utils::stringf("No sensor with name %s exist on vehicle", sensor_name.c_str()));


        for (std::vector<WifiRange>::iterator it = wifiRanges.begin(); it != wifiRanges.end(); ++it) {
            toReturn.wr_time_stamp.push_back(it->time_stamp);
            toReturn.wr_anchorId.push_back(it->anchorId);
            toReturn.wr_anchorX.push_back(it->anchorX);
            toReturn.wr_anchorY.push_back(it->anchorY);
            toReturn.wr_anchorZ.push_back(it->anchorZ);
            toReturn.wr_valid_range.push_back(it->valid_range);
            toReturn.wr_distance.push_back(it->distance);
            toReturn.wr_rssi.push_back(it->rssi);
        }

        for (std::vector<WifiRangeArray>::iterator it = wifiRangesArray.begin(); it != wifiRangesArray.end(); ++it) {
            toReturn.wra_tagId.push_back(it->tagId);
            toReturn.wra_tagX.push_back(it->tagX);
            toReturn.wra_tagY.push_back(it->tagY);
            toReturn.wra_tagZ.push_back(it->tagZ);
            toReturn.wra_ranges.push_back(it->ranges);
        }

        return toReturn;
    }

    // IMU API
    virtual ImuBase::Output getImuData(const std::string& imu_name) const
    {
        const ImuBase* imu = nullptr;

        // Find imu with the given name (for empty input name, return the first one found)
        // Not efficient but should suffice given small number of imus
        uint count_imus = getSensors().size(SensorBase::SensorType::Imu);
        for (uint i = 0; i < count_imus; i++)
        {
            const ImuBase* current_imu = static_cast<const ImuBase*>(getSensors().getByType(SensorBase::SensorType::Imu, i));
            if (current_imu != nullptr && (current_imu->getName() == imu_name || imu_name == ""))
            {
                imu = current_imu;
                break;
            }
        }
        if (imu == nullptr)
            throw VehicleControllerException(Utils::stringf("No IMU with name %s exist on vehicle", imu_name.c_str()));

        return imu->getOutput();
    }

    // Barometer API
    virtual BarometerBase::Output getBarometerData(const std::string& barometer_name) const
    {
        const BarometerBase* barometer = nullptr;

        uint count_barometers = getSensors().size(SensorBase::SensorType::Barometer);
        for (uint i = 0; i < count_barometers; i++)
        {
            const BarometerBase* current_barometer = static_cast<const BarometerBase*>(getSensors().getByType(SensorBase::SensorType::Barometer, i));
            if (current_barometer != nullptr && (current_barometer->getName() == barometer_name || barometer_name == ""))
            {
                barometer = current_barometer;
                break;
            }
        }
        if (barometer == nullptr)
            throw VehicleControllerException(Utils::stringf("No barometer with name %s exist on vehicle", barometer_name.c_str()));

        return barometer->getOutput();
    }

    // Magnetometer API
    virtual MagnetometerBase::Output getMagnetometerData(const std::string& magnetometer_name) const
    {
        const MagnetometerBase* magnetometer = nullptr;

        uint count_magnetometers = getSensors().size(SensorBase::SensorType::Magnetometer);
        for (uint i = 0; i < count_magnetometers; i++)
        {
            const MagnetometerBase* current_magnetometer = static_cast<const MagnetometerBase*>(getSensors().getByType(SensorBase::SensorType::Magnetometer, i));
            if (current_magnetometer != nullptr && (current_magnetometer->getName() == magnetometer_name || magnetometer_name == ""))
            {
                magnetometer = current_magnetometer;
                break;
            }
        }
        if (magnetometer == nullptr)
            throw VehicleControllerException(Utils::stringf("No magnetometer with name %s exist on vehicle", magnetometer_name.c_str()));

        return magnetometer->getOutput();
    }

    // Gps API
    virtual GpsBase::Output getGpsData(const std::string& gps_name) const
    {
        const GpsBase* gps = nullptr;

        uint count_gps = getSensors().size(SensorBase::SensorType::Gps);
        for (uint i = 0; i < count_gps; i++)
        {
            const GpsBase* current_gps = static_cast<const GpsBase*>(getSensors().getByType(SensorBase::SensorType::Gps, i));
            if (current_gps != nullptr && (current_gps->getName() == gps_name || gps_name == ""))
            {
                gps = current_gps;
                break;
            }
        }
        if (gps == nullptr)
            throw VehicleControllerException(Utils::stringf("No gps with name %s exist on vehicle", gps_name.c_str()));

        return gps->getOutput();
    }

    // Distance Sensor API
    virtual DistanceBase::Output getDistanceSensorData(const std::string& distance_sensor_name) const
    {
        const DistanceBase* distance_sensor = nullptr;

        uint count_distance_sensors = getSensors().size(SensorBase::SensorType::Distance);
        for (uint i = 0; i < count_distance_sensors; i++)
        {
            const DistanceBase* current_distance_sensor = static_cast<const DistanceBase*>(getSensors().getByType(SensorBase::SensorType::Distance, i));
            if (current_distance_sensor != nullptr && (current_distance_sensor->getName() == distance_sensor_name || distance_sensor_name == ""))
            {
                distance_sensor = current_distance_sensor;
                break;
            }
        }
        if (distance_sensor == nullptr)
            throw VehicleControllerException(Utils::stringf("No distance sensor with name %s exist on vehicle", distance_sensor_name.c_str()));

        return distance_sensor->getOutput();
    }

    virtual ~VehicleApiBase() = default;

    //exceptions
    class VehicleControllerException : public std::runtime_error {
    public:
        VehicleControllerException(const std::string& message)
            : runtime_error(message) {
        }
    };

    class VehicleCommandNotImplementedException : public VehicleControllerException {
    public:
        VehicleCommandNotImplementedException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };

    class VehicleMoveException : public VehicleControllerException {
    public:
        VehicleMoveException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };
};


}} //namespace
#endif
