# SPEC-002: Sensor Fusion Pipeline

**Priority:** P0
**Category:** ML/Autonomy
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

Each sensor is accessed independently with no framework for multi-modal perception:

- `getLidarData()`, `getGPULidarData()`, `getEchoData()`, `simGetImages()` are separate calls with no synchronization guarantees
- No timestamp alignment between sensors
- No point cloud ↔ image projection utilities
- No occupancy grid generation from LiDAR
- No IMU integration with position estimation
- No calibration utilities (camera intrinsics, LiDAR-camera extrinsics)

**Impact:** Researchers must write custom fusion code for every project, reducing reproducibility and wasting effort.

## Proposed Solution

### 1. Synchronized Sensor Snapshot API

Add to `client.py`:

```python
class SensorSnapshot:
    """Atomically captured multi-sensor data with aligned timestamps."""
    timestamp: float
    cameras: dict[str, ImageResponse]
    lidars: dict[str, LidarData]
    gpu_lidars: dict[str, GPULidarData]
    echoes: dict[str, EchoData]
    imu: ImuData
    gps: GpsData
    barometer: BarometerData
    magnetometer: MagnetometerData

# New API method
def getSensorSnapshot(self, sensors: list[SensorRequest]) -> SensorSnapshot:
    """Capture all requested sensors at the same simulation tick."""
```

Server-side: pause sim, capture all sensors, resume — guarantees temporal alignment.

### 2. Sensor Calibration Utilities

```python
class SensorCalibration:
    @staticmethod
    def get_camera_intrinsics(client, camera_name, vehicle_name) -> np.ndarray:
        """Return 3x3 camera intrinsic matrix K."""

    @staticmethod
    def get_lidar_to_camera_extrinsics(client, lidar_name, camera_name, vehicle_name) -> np.ndarray:
        """Return 4x4 transformation matrix from LiDAR frame to camera frame."""

    @staticmethod
    def project_lidar_to_image(points_3d: np.ndarray, K: np.ndarray, T_lidar_cam: np.ndarray) -> np.ndarray:
        """Project 3D LiDAR points onto 2D image plane."""

    @staticmethod
    def get_sensor_transform(client, sensor_name, vehicle_name) -> np.ndarray:
        """Return 4x4 sensor-to-vehicle transform."""
```

### 3. Point Cloud Processing Utilities

```python
class PointCloudUtils:
    @staticmethod
    def to_occupancy_grid(points: np.ndarray, resolution: float,
                          bounds: tuple) -> np.ndarray:
        """Convert 3D points to 2D/3D occupancy grid."""

    @staticmethod
    def to_bev(points: np.ndarray, x_range, y_range, z_range,
               resolution: float) -> np.ndarray:
        """Convert to bird's-eye-view height map."""

    @staticmethod
    def to_open3d(points: np.ndarray, colors: np.ndarray = None):
        """Convert to Open3D point cloud."""

    @staticmethod
    def to_ply(points: np.ndarray, filepath: str, colors=None, normals=None):
        """Export to PLY format."""

    @staticmethod
    def voxelize(points: np.ndarray, voxel_size: float) -> np.ndarray:
        """Voxel downsampling."""
```

### 4. Multi-Modal Dataset Recording

```python
class DatasetRecorder:
    def __init__(self, client, output_dir, sensors, format="kitti"):
        ...
    def record_frame(self) -> None:
        """Record synchronized sensor snapshot to disk."""
    def export(self, split_ratios=(0.8, 0.1, 0.1)) -> None:
        """Export recorded data with train/val/test split."""
```

## Acceptance Criteria

- [ ] `getSensorSnapshot()` captures all requested sensors within same simulation tick
- [ ] Camera intrinsics derivable from capture settings (FoV, resolution)
- [ ] LiDAR-to-camera projection tested with visual overlay verification
- [ ] Occupancy grid generation from LiDAR point clouds working
- [ ] PLY export with intensity/color data
- [ ] DatasetRecorder produces KITTI-format datasets
- [ ] All utilities have numpy-based implementations (no mandatory Open3D dependency)

## Risks

- Pausing sim for synchronized capture adds latency — make it optional
- Large sensor snapshots may exceed RPC payload limits — implement streaming

## Files Affected

- `PythonClient/cosysairsim/client.py` — add `getSensorSnapshot()`
- `AirLib/include/api/RpcLibClientBase.hpp` — add synchronized capture RPC
- `AirLib/src/api/RpcLibServerBase.cpp` — implement server-side sync
- New: `PythonClient/cosysairsim/fusion.py`
- New: `PythonClient/cosysairsim/pointcloud.py`
- New: `PythonClient/cosysairsim/calibration.py`
- New: `PythonClient/cosysairsim/dataset.py`
