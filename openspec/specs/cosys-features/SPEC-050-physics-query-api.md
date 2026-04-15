# SPEC-050: Physics Query & Raycast API

**Priority:** P1
**Category:** Cosys Features
**Effort:** Small (1-2 weeks)
**Dependencies:** None

## Problem Statement

No API for physics-level spatial queries:

- No raycasting (line trace) from Python
- No sweep queries (volume trace)
- No terrain height sampling
- No ground material queries at arbitrary positions
- No obstacle distance queries
- `simGetCollisionInfo()` only returns vehicle collision, not arbitrary queries

These are essential for path planning, terrain analysis, and obstacle assessment.

## Proposed Solution

```python
# Raycast (line trace)
hit = client.simRaycast(
    start=Vector3r(0, 0, 10),
    end=Vector3r(0, 0, -10),
    ignore_actors=["Drone1"],
)
# Returns: RaycastHit(hit=True, position=Vector3r(0,0,0.5),
#          normal=Vector3r(0,0,1), distance=9.5,
#          actor_name="Ground_01", material="asphalt")

# Batch raycast (many rays, one RPC call)
hits = client.simBatchRaycast(
    origins=[Vector3r(x, y, 10) for x, y in grid],
    directions=[Vector3r(0, 0, -1)] * len(grid),
)

# Sphere sweep
hit = client.simSweep(
    start=Vector3r(0, 0, 2),
    end=Vector3r(10, 0, 2),
    radius=0.5,  # Sphere radius
)

# Terrain height at position
height = client.simGetTerrainHeight(x=10.0, y=20.0)

# Ground material at position
material = client.simGetGroundMaterial(x=10.0, y=20.0)
# Returns: "asphalt", "grass", "gravel", etc.

# Check if position is navigable (for ground vehicles/pedestrians)
navigable = client.simIsNavigable(Vector3r(10, 20, 0))
```

## Acceptance Criteria

- [ ] Single raycast with hit position, normal, distance, actor name
- [ ] Batch raycast for efficiency (100+ rays in one call)
- [ ] Sphere sweep for clearance checking
- [ ] Terrain height query at arbitrary XY position
- [ ] Ground material query
- [ ] Navigability check (navmesh query)
- [ ] Performance: batch of 1000 raycasts < 10ms

## Files Affected

- `AirLib/include/api/WorldSimApiBase.hpp` — raycast/sweep APIs
- `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` — UE line trace implementation
- `AirLib/src/api/RpcLibServerBase.cpp` — register query RPCs
- `PythonClient/cosysairsim/client.py` — physics query methods
- `PythonClient/cosysairsim/types.py` — RaycastHit type
