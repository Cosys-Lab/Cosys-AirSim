# SPEC-046: Recording & Playback System Enhancement

**Priority:** P1
**Category:** Cosys Features
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

The recording system (`startRecording/stopRecording/isRecording`) is basic:

- Records only to `airsim_rec.txt` CSV format
- No selective sensor recording (records everything or nothing)
- No frame-rate control
- No compression options
- No multi-vehicle synchronized recording
- No playback/replay capability
- RecordingSetting exists in settings but no API to modify at runtime

## Proposed Solution

### 1. Enhanced Recording API

```python
# Configure recording before starting
client.configureRecording(
    output_dir="/data/recordings/run_42",
    format="hdf5",               # or "rosbag2", "csv", "kitti"
    sensors=["camera_front", "lidar_top", "imu"],  # Selective sensors
    vehicles=["Car1", "Drone1"],  # Specific vehicles
    fps=30,                       # Frame rate limit
    compress=True,                # LZ4 compression for point clouds
    include_state=True,           # Vehicle pose/kinematics
    include_annotations=True,     # Segmentation/detection GT
)

# Start/stop
client.startRecording()
client.stopRecording()

# Query recording status
status = client.getRecordingStatus()
# Returns: RecordingStatus(active=True, frames=1234,
#          duration_sec=41.1, disk_usage_mb=523)
```

### 2. Playback API

```python
# Load a recording
client.loadRecording("/data/recordings/run_42")

# Playback (vehicles follow recorded trajectories)
client.startPlayback(speed=1.0)
client.pausePlayback()
client.seekPlayback(time_sec=10.5)
client.stopPlayback()

# Playback with modified parameters (re-render with different weather)
client.startPlayback(speed=1.0, override_weather={"rain": 0.5})
```

### 3. Output Formats

- **HDF5**: Structured, compressed, multi-sensor with metadata
- **ROS bag2**: Native ROS2 recording format
- **KITTI**: Standard autonomous driving format (per SPEC-007)
- **CSV**: Legacy compatibility with `airsim_rec.txt`

## Acceptance Criteria

- [ ] Selective sensor recording (choose which sensors to record)
- [ ] Multi-vehicle synchronized recording with timestamp alignment
- [ ] HDF5 export with sensor metadata and calibration
- [ ] Frame rate control (record at specific FPS)
- [ ] Compression support (LZ4 for point clouds, JPEG for images)
- [ ] Playback/replay with vehicle trajectory following
- [ ] Seek within recording
- [ ] Disk usage reporting during recording

## Files Affected

- `Unreal/Plugins/AirSim/Source/Recording/RecordingThread.h/.cpp` — enhance
- `AirLib/include/api/WorldSimApiBase.hpp` — recording config API
- `AirLib/include/common/AirSimSettings.hpp` — RecordingSetting extensions
- `PythonClient/cosysairsim/client.py` — recording/playback API
