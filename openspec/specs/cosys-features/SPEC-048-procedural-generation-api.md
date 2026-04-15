# SPEC-048: Procedural Generation & Dynamic World Configuration API

**Priority:** P1
**Category:** Cosys Features
**Effort:** Medium (2 weeks)
**Dependencies:** None

## Problem Statement

The README highlights "random but deterministic dynamic object types and world configuration options" with `DynamicWorldMaster` blueprints, `GroupedAI` spawners, and `RandomPropSpawner`. But these are Unreal Editor-only — no Python API for:

- Controlling procedural generation parameters at runtime
- Seed management from Python
- Object type/density configuration via API
- Triggering world regeneration between episodes
- Querying spawned dynamic objects

## Proposed Solution

### 1. World Configuration API

```python
# Configure dynamic world before episode
client.simConfigureWorld(
    seed=42,
    dynamic_objects={
        "vehicles": {"density": 0.3, "types": ["sedan", "suv", "truck"]},
        "pedestrians": {"density": 0.5, "types": ["mannequin"]},
        "props": {"density": 0.2, "types": ["traffic_cone", "barrier"]},
    },
    remove_percentage=0.1,       # Randomly remove 10% of static objects
    move_percentage=0.05,        # Randomly displace 5% of objects
    move_range=2.0,              # Max displacement distance (m)
)

# Regenerate world (between episodes)
client.simRegenerateWorld(seed=43)

# Query spawned dynamic objects
objects = client.simGetDynamicObjects()
# Returns: [DynamicObject(name="DynVehicle_01", type="sedan",
#           pose=Pose(...), spawned_by="VehicleSpawner_01"), ...]

# Modify density at runtime
client.simSetDynamicObjectDensity("pedestrians", 0.8)
```

### 2. Spawn Zone API

```python
# Define spawn zones programmatically
client.simAddSpawnZone(
    name="intersection_01",
    bounds={"min": (-50, -50), "max": (50, 50)},
    spawn_config={
        "pedestrians": {"count": 20, "behavior": "wander"},
        "vehicles": {"count": 5, "types": ["sedan"]},
    }
)
```

## Acceptance Criteria

- [ ] Seed-based world configuration from Python
- [ ] Dynamic object density control per type
- [ ] World regeneration between episodes
- [ ] Query spawned dynamic objects
- [ ] Spawn zones definable via API
- [ ] Deterministic: same seed produces same world

## Files Affected

- `AirLib/include/api/WorldSimApiBase.hpp` — world configuration API
- `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` — implementation
- `PythonClient/cosysairsim/client.py` — world configuration methods
