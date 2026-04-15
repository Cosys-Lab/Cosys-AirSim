# SPEC-044: Annotation System Enhancement & Batch Operations

**Priority:** P0
**Category:** Cosys Features
**Effort:** Small (1-2 weeks)
**Dependencies:** None

## Problem Statement

The multi-layer annotation system is a key Cosys-Lab differentiator (RGB, Greyscale, Texture annotation types) but lacks:

- **Batch operations**: Setting annotation on many objects requires N separate API calls
- **Runtime layer creation**: Annotation layers are fixed at startup
- **Metadata queries**: No API to list which objects have which annotations
- **Performance**: Individual `simSetAnnotationObjectID/Color/Value` calls are slow for large scenes

Current API surface (`client.py:816-1010`) has 15+ annotation methods but no batching.

## Proposed Solution

### 1. Batch Annotation API

```python
# Before: 100 separate RPC calls
for obj in scene_objects:
    client.simSetAnnotationObjectColor("annotation0", obj, color)

# After: 1 RPC call
client.simSetAnnotationBatch("annotation0", {
    "Wall_01": {"color": (255, 0, 0)},
    "Wall_02": {"color": (255, 0, 0)},
    "Vehicle_01": {"color": (0, 255, 0)},
    "Person_01": {"color": (0, 0, 255)},
    # ... hundreds of objects
})
```

### 2. Annotation Query API

```python
# List all objects with annotation on a specific layer
objects = client.simGetAnnotatedObjects("annotation0")
# Returns: [{"name": "Wall_01", "color": (255,0,0), "type": "rgb"}, ...]

# Get annotation summary statistics
stats = client.simGetAnnotationStats("annotation0")
# Returns: {"total_objects": 342, "unique_colors": 28, ...}
```

### 3. Runtime Layer Management

```python
# Create new annotation layer at runtime
client.simCreateAnnotationLayer("custom_annotation", type="rgb")

# Delete annotation layer
client.simDeleteAnnotationLayer("custom_annotation")

# List available layers
layers = client.simListAnnotationLayers()
```

## Acceptance Criteria

- [ ] Batch annotation sets 100+ objects in single RPC call
- [ ] Query API lists annotated objects with their current values
- [ ] Runtime layer creation/deletion
- [ ] Performance: batch of 500 objects completes in < 100ms
- [ ] Backward compatible with existing per-object API

## Files Affected

- `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.cpp` — batch methods
- `AirLib/include/api/WorldSimApiBase.hpp` — batch annotation API
- `AirLib/src/api/RpcLibServerBase.cpp` — register batch RPCs
- `PythonClient/cosysairsim/client.py` — batch annotation methods
