# SPEC-007: Standard Dataset Export Formats

**Priority:** P1
**Category:** ML/Autonomy
**Effort:** Small (1-2 weeks)
**Dependencies:** SPEC-002

## Problem Statement

Data export is limited and non-standard:

- Point clouds exported as ASCII only (`point_cloud.py:29-39`)
- H5 files lack metadata (sensor calibration, environment state)
- No KITTI format support (the standard for autonomous driving research)
- No COCO format for detection tasks
- No ROS bag recording
- No PLY/LAS/LAZ for point clouds
- Manual train/val/test split only
- No provenance tracking or dataset versioning

## Proposed Solution

### 1. KITTI Format Exporter

```python
class KITTIExporter:
    """Export sensor data in KITTI benchmark format."""

    def export(self, recording_dir, output_dir):
        """
        Output structure:
        output_dir/
          image_2/        # Left camera RGB
          image_3/        # Right camera RGB (if stereo)
          velodyne/        # LiDAR point clouds (.bin)
          calib/           # Calibration files
          label_2/         # 3D bounding box labels
          oxts/            # GPS/IMU data
        """
```

### 2. COCO Format Exporter

```python
class COCOExporter:
    """Export detection/segmentation data in COCO format."""

    def export(self, recording_dir, output_dir, categories):
        """
        Output: COCO JSON + image directory
        Supports: bounding boxes, segmentation masks, keypoints
        """
```

### 3. Point Cloud Formats

```python
class PointCloudExporter:
    @staticmethod
    def to_ply(points, filepath, colors=None, normals=None, binary=True): ...
    @staticmethod
    def to_las(points, filepath, intensity=None, classification=None): ...
    @staticmethod
    def to_pcd(points, filepath, binary=True): ...  # PCL format
    @staticmethod
    def to_numpy(points, filepath): ...  # .npz compressed
```

### 4. Dataset Manifest

```yaml
# dataset_manifest.yaml
version: "1.0"
created: "2025-01-15T10:30:00Z"
simulator: "Cosys-AirSim"
simulator_version: "5.5.4"
checksum: "sha256:abc123..."
environment:
  map: "Blocks"
  weather: {rain: 0.0, fog: 0.1}
  time_of_day: "14:00"
sensors:
  camera_front:
    type: "rgb"
    resolution: [1920, 1080]
    fov: 90
    intrinsics: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
  lidar_top:
    type: "gpu_lidar"
    channels: 64
    range: 120.0
    points_per_second: 1300000
splits:
  train: {frames: 8000, range: [0, 8000]}
  val: {frames: 1000, range: [8000, 9000]}
  test: {frames: 1000, range: [9000, 10000]}
```

## Acceptance Criteria

- [ ] KITTI format export with calibration files verified against KITTI devkit
- [ ] COCO format export validated with pycocotools
- [ ] PLY/LAS point cloud export with intensity and labels
- [ ] Dataset manifest with full provenance (settings, environment, calibration)
- [ ] Automatic train/val/test split with stratification option
- [ ] SHA256 checksums for all data files
- [ ] Example export scripts for each format

## Files Affected

- New: `PythonClient/cosysairsim/export/kitti.py`
- New: `PythonClient/cosysairsim/export/coco.py`
- New: `PythonClient/cosysairsim/export/pointcloud.py`
- New: `PythonClient/cosysairsim/export/manifest.py`
