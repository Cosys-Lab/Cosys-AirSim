# SPEC-043: Character Detection, Annotation & Pose Estimation Ground Truth

**Priority:** P0
**Category:** Characters
**Effort:** Small (1-2 weeks)
**Dependencies:** SPEC-035, SPEC-037

## Problem Statement

Characters must integrate seamlessly with Cosys-AirSim's annotation and detection systems for ML training:

- Instance segmentation (unique color per character)
- 2D/3D bounding box detection
- Semantic segmentation (person class)
- Skeleton-based pose estimation ground truth
- Action/activity recognition labels
- Partial occlusion handling

## Proposed Solution

### 1. Annotation Integration

Characters automatically register with `ObjectAnnotator` on spawn:

```cpp
void CharacterPawnSimApi::initialize() {
    PawnSimApi::initialize();

    // Register with annotation system
    for (auto& annotator : WorldSimApi::getAnnotators()) {
        annotator.AnnotateNewActor(GetPawn(),
            /*component_level=*/true);  // Per-body-part annotation
    }
}
```

### 2. Detection API Extensions

```python
# Standard detection (2D/3D bounding boxes)
detections = client.simGetDetections(
    camera_name="cam0",
    image_type=ImageType.Scene,
    vehicle_name="Drone1"
)
# Returns DetectionInfo with character-specific fields:
# DetectionInfo(
#   name="Person1",
#   geo_point=...,
#   box2D=Box2D(min=(120,80), max=(230,350)),
#   box3D=Box3D(min=..., max=..., orientation=...),
#   relative_pose=...,
#   object_class="character",
#   is_occluded=False,
#   occlusion_ratio=0.15
# )
```

### 3. Pose Estimation Ground Truth

```python
# Get 3D joint positions in world space
skeleton = client.getCharacterSkeleton3D(vehicle_name="Person1")
# Returns: {
#   "head": Vector3r(10.2, 5.1, 1.72),
#   "neck": Vector3r(10.2, 5.1, 1.60),
#   "spine_03": Vector3r(10.2, 5.0, 1.45),
#   "upperarm_l": Vector3r(10.0, 4.8, 1.50),
#   "lowerarm_l": Vector3r(9.8, 4.6, 1.30),
#   "hand_l": Vector3r(9.7, 4.5, 1.20),
#   "upperarm_r": ...,
#   "thigh_l": ..., "calf_l": ..., "foot_l": ...,
#   ... (25+ joints matching COCO/MPII format)
# }

# Get 2D projected joints for a specific camera
joints_2d = client.getCharacterSkeleton2D(
    camera_name="cam0",
    vehicle_name="Person1",
    target_character="Person2"  # Which character to get joints for
)
# Returns: {
#   "head": (320, 80, 1.0),        # (x, y, visibility)
#   "neck": (318, 105, 1.0),
#   "hand_l": (280, 200, 0.5),     # Partially occluded
#   "foot_r": (330, 400, 0.0),     # Fully occluded
#   ...
# }

# Batch: get all character skeletons in one call
all_skeletons = client.getAllCharacterSkeletons2D(
    camera_name="cam0",
    vehicle_name="Drone1"
)
# Returns: {"Person1": {...}, "Person2": {...}, ...}
```

### 4. Activity Labels

```python
# Get current activity ground truth for all characters
activities = client.getCharacterActivities()
# Returns: {
#   "Person1": ActivityLabel(action="walking", speed=1.4, direction=45.0),
#   "Person2": ActivityLabel(action="sitting", speed=0.0),
#   "Person3": ActivityLabel(action="dancing", sub_action="hip_hop"),
#   "Person4": ActivityLabel(action="hiding", cover_object="Wall_03"),
#   "Person5": ActivityLabel(action="riding_motorbike", vehicle="Bike1"),
# }
```

### 5. COCO-Compatible Export

```python
# Export annotations in COCO-Keypoints format
exporter = COCOKeypointExporter(client, output_dir="/tmp/dataset")
exporter.set_keypoint_format("coco_17")  # 17-joint COCO format
exporter.record_frame()
exporter.export()
# Produces COCO JSON with keypoint annotations + images
```

## Acceptance Criteria

- [ ] Characters appear with unique colors in instance segmentation
- [ ] `simGetDetections()` returns character bounding boxes
- [ ] `getCharacterSkeleton3D()` returns 25+ joint positions in world frame
- [ ] `getCharacterSkeleton2D()` returns projected joints with occlusion labels
- [ ] Visibility/occlusion ratio computed per character
- [ ] Activity labels available for all characters
- [ ] COCO-Keypoints format export working
- [ ] Batch skeleton query for all characters in view
- [ ] Joint visibility flag (1.0=visible, 0.5=partially occluded, 0.0=hidden)

## Files Affected

- `AirLib/include/vehicles/character/api/CharacterApiBase.hpp` — skeleton/activity methods
- `Unreal/Plugins/AirSim/Source/Vehicles/Character/CharacterPawn.cpp` — bone transform extraction
- `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.cpp` — character registration
- `PythonClient/cosysairsim/client.py` — skeleton/detection API
- `PythonClient/cosysairsim/types.py` — Skeleton, ActivityLabel types
- New: `PythonClient/cosysairsim/export/coco_keypoints.py`
