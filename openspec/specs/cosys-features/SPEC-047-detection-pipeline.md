# SPEC-047: Object Detection Pipeline Enhancement

**Priority:** P1
**Category:** Cosys Features
**Effort:** Small (1-2 weeks)
**Dependencies:** None

## Problem Statement

Detection APIs exist (`simGetDetections`, `simAddDetectionFilterMeshName`, `simSetDetectionFilterRadius`) but lack:

- Confidence thresholding
- Detection temporal filtering (smoothing across frames)
- 3D/2D box fusion modes
- Detection event callbacks (object enters/leaves view)
- Performance metrics (detection latency, missed detections)
- Per-class filtering
- Detection export in standard formats

## Proposed Solution

### 1. Enhanced Detection Configuration

```python
# Configure detection pipeline
client.simConfigureDetection(
    camera_name="cam0",
    vehicle_name="Drone1",
    min_area_pixels=100,          # Minimum bounding box area
    max_distance=200.0,           # Maximum detection range (m)
    class_filter=["character", "car", "motorbike"],
    temporal_smoothing=3,          # Average over N frames
    nms_threshold=0.5,            # Non-maximum suppression
    include_occluded=True,        # Include partially occluded objects
    min_visibility=0.2,           # Minimum visible fraction
)
```

### 2. Detection Events

```python
# Register callback for detection events
def on_detection(event):
    if event.type == "enter":
        print(f"{event.object_name} entered view")
    elif event.type == "exit":
        print(f"{event.object_name} left view")

client.simOnDetectionEvent(callback=on_detection,
                            camera_name="cam0",
                            vehicle_name="Drone1")
```

### 3. Detection Export

```python
# Export detections in COCO format
client.simExportDetections(
    output_path="/tmp/detections.json",
    format="coco",                 # or "yolo", "pascal_voc", "kitti"
    frames=range(0, 1000),
)
```

## Acceptance Criteria

- [ ] Class-based detection filtering
- [ ] Minimum area and maximum distance thresholds
- [ ] Temporal smoothing across frames
- [ ] Visibility/occlusion ratio per detection
- [ ] Detection export in COCO, YOLO, Pascal VOC formats
- [ ] Performance: detection processing < 2ms per frame

## Files Affected

- `Unreal/Plugins/AirSim/Source/DetectionComponent.h/.cpp` — filtering enhancements
- `AirLib/include/api/WorldSimApiBase.hpp` — detection config API
- `PythonClient/cosysairsim/client.py` — detection configuration
- New: `PythonClient/cosysairsim/export/detection.py`
