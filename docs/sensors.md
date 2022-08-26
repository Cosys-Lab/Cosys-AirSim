# Sensors in AirSim

AirSim currently supports the following sensors.    
Each sensor is associated with a integer enum specifying its sensor type.

* Camera
* Barometer = 1
* Imu = 2
* Gps = 3
* Magnetometer = 4
* Distance Sensor = 5 
* LiDAR = 6
* Echo = 7
* GPULiDAR = 8

**Note** :  Cameras are configured differently than the other sensors and do not have an enum associated with them.    Look at [general settings](settings.md) and [image API](image_apis.md) for camera config and API. 

## Default sensors

If no sensors are specified in the `settings json`, the the following sensors are enabled by default based on the sim mode.

### Multirotor
* Imu
* Magnetometer
* Gps
* Barometer

### Car
* Gps

### ComputerVision
* None

Behind the scenes, 'createDefaultSensorSettings' method in [AirSimSettings.hpp](https://cosysgit.uantwerpen.be/sensorsimulation/airsim/-/blob/master/AirLib/include/common/AirSimSettings.hpp) which sets up the above sensors with their default parameters, depending on the sim mode specified in the `settings.json` file. 

## Configuring the default sensor list

The default sensor list can be configured in settings json:

```JSON
"DefaultSensors": {
    "Barometer": {
         "SensorType": 1,
         "Enabled" : true
    },
    "Imu": {
         "SensorType": 2,
         "Enabled" : true
    },
    "Gps": {
         "SensorType": 3,
         "Enabled" : true
    },
    "Magnetometer": {
         "SensorType": 4,
         "Enabled" : true
    },
    "Distance": {
         "SensorType": 5,
         "Enabled" : true
    },
    "Lidar": { 
         "SensorType": 6,
         "Enabled" : true,
         "NumberOfChannels": 4,
         "PointsPerSecond": 10000
    },
    "Echo": { 
         "SensorType": 7,
         "Enabled" : true,
         "NumberOfChannels": 4,
         "PointsPerSecond": 10000
    },
    "GPULidar": { 
         "SensorType": 8,
         "Enabled" : true,
         "NumberOfChannels": 4
    }
},
```

## Configuring vehicle-specific sensor list

If a vehicle provides its sensor list, it **must** provide the whole list. Selective add/remove/update of the default sensor list is **NOT** supported.   
A (vehicle specific) sensor list can be specified in the vehicle settings part of the json.
e.g.,

```
    "Vehicles": {

        "Drone1": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true,
            ...
            "Sensors": {
                "MyLidar1": { 
                    "SensorType": 6,
                    "Enabled" : true,
                    "NumberOfChannels": 16,
                    "PointsPerSecond": 10000,
                    "X": 0, "Y": 0, "Z": -1,
                    "DrawDebugPoints": true
                },
                "MyLidar2": { 
                    "SensorType": 6,
                    "Enabled" : true,
                    "NumberOfChannels": 4,
                    "PointsPerSecond": 10000,
                    "X": 0, "Y": 0, "Z": -1,
                    "DrawDebugPoints": true
                }
            }
        }
   }
```

### Sensor specific settings
Each sensor-type has its own set of settings as well.   
Please see [LiDAR](lidar.md) for example of LiDAR specific settings.
Please see [echo](echo.md) for example of Echo specific settings.
Please see [GPU LiDAR](gpulidar.md) for example of GPU LiDAR specific settings.

## Sensor APIs 
- Barometer

- Python
    ```python
    barometer_data = getBarometerData(barometer_name = "", vehicle_name = "")
    ```

- IMU

    Python
    ```python
    imu_data = getImuData(imu_name = "", vehicle_name = "")
    ```

- GPS

    Python
    ```python
    gps_data = getGpsData(gps_name = "", vehicle_name = "")
    ```

- Magnetometer

    Python
    ```python
    magnetometer_data = getMagnetometerData(magnetometer_name = "", vehicle_name = "")
    ```

- Distance sensor

    Python
    ```python
    distance_sensor_name = getDistanceSensorData(distance_sensor_name = "", vehicle_name = "")
    ```

- LiDAR   
    See [LiDAR](lidar.md) for LiDAR API.
    
- Echo   
    See [echo](echo.md) for Echo API.

- GPU LiDAR   
    See [GPU Lidar](gpulidar.md) for GPU LiDAR API.
