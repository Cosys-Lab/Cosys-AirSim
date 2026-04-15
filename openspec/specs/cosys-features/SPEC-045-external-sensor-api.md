# SPEC-045: External World Sensor Management API

**Priority:** P0
**Category:** Cosys Features
**Effort:** Small (1-2 weeks)
**Dependencies:** None

## Problem Statement

The README highlights "sensors uncoupled from vehicle placed as external world sensors" as a key feature, but the Python API has no methods to:

- List external sensors and their poses
- Move external sensors at runtime
- Add/remove external sensors dynamically
- Query external sensor data by name (without vehicle association)
- Set coordinate frames for external sensors

Sensor params have `external = false` flag but no RPC API covers external sensor lifecycle.

## Proposed Solution

### 1. External Sensor API

```python
# List all external sensors
sensors = client.simListExternalSensors()
# Returns: [
#   ExternalSensor(name="Lidar_Intersection_01", type="GPULidar",
#                  pose=Pose(...), config={...}),
#   ExternalSensor(name="Camera_Parking_01", type="Camera",
#                  pose=Pose(...), config={...}),
# ]

# Get external sensor pose
pose = client.simGetExternalSensorPose("Lidar_Intersection_01")

# Move external sensor at runtime
client.simSetExternalSensorPose("Lidar_Intersection_01", new_pose)

# Get data from external sensor
lidar_data = client.getExternalLidarData("Lidar_Intersection_01")
images = client.simGetExternalImages([ImageRequest(...)], "Camera_Parking_01")

# Add external sensor at runtime
client.simAddExternalSensor(
    name="NewCamera",
    sensor_type="Camera",
    pose=Pose(Vector3r(10, 20, 5), Quaternionr()),
    settings={
        "CaptureSettings": [{"Width": 1920, "Height": 1080, "FOV_Degrees": 90}]
    }
)

# Remove external sensor
client.simRemoveExternalSensor("NewCamera")
```

### 2. Settings Configuration

```json
{
    "ExternalSensors": {
        "IntersectionLidar": {
            "SensorType": "GPULidar",
            "External": true,
            "Position": {"X": 50, "Y": 0, "Z": 8},
            "Rotation": {"Pitch": -30, "Yaw": 0, "Roll": 0},
            "NumberOfChannels": 64,
            "Range": 100
        },
        "ParkingCamera": {
            "SensorType": "Camera",
            "External": true,
            "Position": {"X": -10, "Y": 15, "Z": 5},
            "CaptureSettings": [{"Width": 1920, "Height": 1080}]
        }
    }
}
```

## Acceptance Criteria

- [ ] List, query, and modify external sensor poses via API
- [ ] Get sensor data from external sensors (LiDAR, camera, echo)
- [ ] Add/remove external sensors at runtime
- [ ] External sensors work with annotation system
- [ ] Coordinate frame handling (NED vs world)
- [ ] Settings-based and runtime-based configuration

## Files Affected

- `AirLib/include/api/WorldSimApiBase.hpp` — external sensor methods
- `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` — implementation
- `AirLib/src/api/RpcLibServerBase.cpp` — register external sensor RPCs
- `PythonClient/cosysairsim/client.py` — external sensor API
