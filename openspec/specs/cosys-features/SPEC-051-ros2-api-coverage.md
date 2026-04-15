# SPEC-051: ROS2 Complete API Coverage

**Priority:** P1
**Category:** Cosys Features
**Effort:** Medium (3 weeks)
**Dependencies:** None

## Problem Statement

ROS2 wrapper (`ros2/src/airsim_ros_pkgs/`) covers basic sensor publishing and movement commands but misses many Python API features:

- No gimbal control services (TODO comment at `airsim_ros_wrapper.cpp:96-97`)
- No dynamic object spawning services
- No annotation layer update services
- No detection filtering services
- No wind/force control topics
- No recording control services
- No character/pedestrian topics
- No external sensor management

## Proposed Solution

### 1. Missing ROS2 Services

```yaml
# New ROS2 services to add
services:
  # Object management
  - /airsim/spawn_object (SpawnObject.srv)
  - /airsim/destroy_object (DestroyObject.srv)
  - /airsim/list_scene_objects (ListObjects.srv)

  # Annotation
  - /airsim/set_annotation_batch (SetAnnotationBatch.srv)
  - /airsim/get_annotation_objects (GetAnnotationObjects.srv)

  # Detection
  - /airsim/configure_detection (ConfigureDetection.srv)

  # Recording
  - /airsim/start_recording (StartRecording.srv)
  - /airsim/stop_recording (StopRecording.srv)

  # Character (new)
  - /airsim/set_character_controls (SetCharacterControls.srv)
  - /airsim/play_animation (PlayAnimation.srv)
  - /airsim/enter_vehicle (EnterVehicle.srv)

  # Environment
  - /airsim/set_weather (SetWeather.srv)
  - /airsim/set_wind (SetWind.srv)
  - /airsim/set_time_of_day (SetTimeOfDay.srv)

  # Physics queries
  - /airsim/raycast (Raycast.srv)
```

### 2. Missing ROS2 Topics

```yaml
topics:
  # Character state
  - /airsim/{vehicle}/character_state (CharacterState.msg)
  - /airsim/{vehicle}/character_skeleton (Skeleton.msg)

  # Detections
  - /airsim/{camera}/detections (DetectionArray.msg)

  # External sensors
  - /airsim/external/{sensor}/data (appropriate sensor msg)

  # Performance
  - /airsim/diagnostics (Diagnostics.msg)
```

### 3. Custom Message Types

```
# msg/CharacterState.msg
uint8 movement_mode
float32 speed
geometry_msgs/Pose pose
string current_animation
bool is_in_vehicle
string current_vehicle

# msg/Skeleton.msg
string[] joint_names
geometry_msgs/Pose[] joint_poses
float32[] joint_visibilities
```

## Acceptance Criteria

- [ ] All Python API methods have ROS2 equivalents
- [ ] Character control via ROS2 topics/services
- [ ] Annotation batch operations via services
- [ ] Detection data published as ROS2 topics
- [ ] Recording control via services
- [ ] Custom message types for character/skeleton data
- [ ] Integration tests for new services

## Files Affected

- `ros2/src/airsim_ros_pkgs/include/airsim_ros_wrapper.h` — new methods
- `ros2/src/airsim_ros_pkgs/src/airsim_ros_wrapper.cpp` — implementation
- New: `ros2/src/airsim_ros_pkgs/msg/` — custom messages
- New: `ros2/src/airsim_ros_pkgs/srv/` — custom services
- `ros2/src/airsim_ros_pkgs/CMakeLists.txt` — build config
