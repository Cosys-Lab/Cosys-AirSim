# SPEC-015: Deformable Terrain & Soft Ground Interaction

**Priority:** P2
**Category:** Physics Simulation
**Effort:** Large (6-8 weeks)
**Dependencies:** SPEC-012

## Problem Statement

All vehicle-ground interactions assume rigid surfaces:

- No tire deformation, no ground deformation tracking
- No soft ground interactions (mud, sand, snow, gravel)
- No terrain feedback forces (sinkage, resistance)
- Limits simulation of off-road vehicles, agricultural robots, planetary rovers
- No track marks or terrain modification visualization

## Proposed Solution

### 1. Bekker-Wong Terramechanics Model

```cpp
class TerrainInteractionModel {
    // Bekker pressure-sinkage: p = (k_c/b + k_phi) * z^n
    struct SoilParams {
        float k_c;          // Cohesive modulus (Pa/m^(n-1))
        float k_phi;        // Frictional modulus (Pa/m^n)
        float n;            // Sinkage exponent
        float cohesion;     // Soil cohesion (Pa)
        float friction_angle; // Internal friction angle (rad)
        float density;      // Soil density (kg/m³)
    };

    // Compute sinkage and resistance
    TerrainForces compute(float normal_load, float wheel_radius,
                          float wheel_width, float slip_ratio,
                          const SoilParams& soil);

    // Predefined soil types
    static SoilParams SAND;
    static SoilParams DRY_CLAY;
    static SoilParams WET_CLAY;
    static SoilParams SNOW_FRESH;
    static SoilParams SNOW_COMPACTED;
    static SoilParams GRAVEL;
    static SoilParams LOAM;
};
```

### 2. Terrain Modification

```cpp
class DeformableTerrain {
    // Height field representing terrain deformation
    std::vector<float> deformation_map_;
    float resolution_;

    void applyWheelTrack(const Vector3r& position, float width,
                         float depth, float heading);
    float getDeformationAt(const Vector3r& position) const;
};
```

### 3. Terrain Zone API

```python
# Define terrain zones with different soil properties
client.simSetTerrainZone(
    bounds={"min": [-100, -100], "max": [100, 100]},
    soil_type="sand",
    moisture=0.3,  # affects cohesion and friction
)
```

## Acceptance Criteria

- [ ] Bekker-Wong model for at least 5 soil types
- [ ] Sinkage and rolling resistance computed from soil parameters
- [ ] Terrain deformation tracking (height field modification)
- [ ] API to define terrain zones with different properties
- [ ] Performance: < 0.5ms per wheel per tick
- [ ] Visual track marks on deformable terrain (UE integration)

## Files Affected

- New: `AirLib/include/physics/TerrainInteraction.hpp`
- New: `AirLib/include/physics/DeformableTerrain.hpp`
- `AirLib/include/vehicles/car/` — integrate terrain model
- `AirLib/include/vehicles/skidsteer/` — integrate terrain model
- `Unreal/Plugins/AirSim/Source/` — terrain visualization
