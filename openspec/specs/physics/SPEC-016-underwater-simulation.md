# SPEC-016: Underwater Vehicle & Fluid Dynamics

**Priority:** P2
**Category:** Physics Simulation
**Effort:** Large (8-10 weeks)
**Dependencies:** SPEC-009, SPEC-010

## Problem Statement

No aquatic environment support exists:

- No buoyancy, hydrodynamic drag, or viscous forces
- No underwater vehicle models (AUV, ROV)
- No water surface interaction
- No underwater sensor modeling (sonar, pressure sensors, underwater cameras)
- Growing demand for underwater robotics simulation in ocean exploration, infrastructure inspection, environmental monitoring

## Proposed Solution

### 1. Hydrodynamics Engine

```cpp
class HydrodynamicsModel {
    struct FluidProperties {
        float density;          // kg/m³ (water: 1000, seawater: 1025)
        float dynamic_viscosity; // Pa·s
        float speed_of_sound;   // m/s
        float temperature;      // K
        float salinity;         // PSU
    };

    // Buoyancy: F_b = ρ * g * V_displaced
    Vector3r computeBuoyancy(float displaced_volume, const FluidProperties& fluid);

    // Added mass: F_am = -M_a * a (virtual mass effect)
    Vector3r computeAddedMass(const Matrix3x3r& added_mass_matrix,
                              const Vector3r& acceleration);

    // Hydrodynamic drag: F_d = 0.5 * ρ * Cd * A * v²
    Wrench computeHydrodynamicDrag(const Vector3r& velocity,
                                     const Vector3r& angular_velocity,
                                     const HydrodynamicCoefficients& coeffs,
                                     const FluidProperties& fluid);

    // Current forces (water flow)
    Vector3r computeCurrentForce(const Vector3r& current_velocity,
                                  const Vector3r& vehicle_velocity);
};
```

### 2. Underwater Vehicle Type

```cpp
class UnderwaterVehicleParams : public VehicleParams {
    float displaced_volume;
    float center_of_buoyancy_z;  // Offset from CoG
    HydrodynamicCoefficients drag_coeffs;
    Matrix3x3r added_mass;
    int num_thrusters;
    std::vector<ThrusterConfig> thrusters;
};
```

### 3. Underwater Sensors

- **Forward-Looking Sonar** (FLS): Fan-beam echo imaging
- **Side-Scan Sonar**: Terrain mapping
- **Depth/Pressure Sensor**: Absolute pressure → depth
- **Underwater Camera**: Turbidity, light absorption, color attenuation
- **DVL (Doppler Velocity Log)**: Bottom-tracking velocity

### 4. Water Surface Model

```cpp
class WaterSurface {
    // Wave model (Gerstner waves for visualization, linear for physics)
    float getHeightAt(float x, float y, float time);
    Vector3r getNormalAt(float x, float y, float time);

    // Surface crossing detection for vehicles transitioning air↔water
    bool isBelowSurface(const Vector3r& position, float time);
};
```

## Acceptance Criteria

- [ ] Buoyancy and added mass forces computed correctly
- [ ] 6-DOF hydrodynamic drag model
- [ ] At least one AUV model (e.g., BlueROV2) with thruster configuration
- [ ] Underwater camera with turbidity and color absorption
- [ ] Forward-looking sonar basic implementation
- [ ] Water current model (constant + depth-varying)
- [ ] API for underwater vehicle control
- [ ] Settings.json configuration for underwater scenarios

## Files Affected

- New: `AirLib/include/physics/HydrodynamicsModel.hpp`
- New: `AirLib/include/vehicles/underwater/`
- New: `AirLib/include/sensors/sonar/`
- New: `AirLib/include/sensors/dvl/`
- `AirLib/include/physics/FastPhysicsEngine.hpp` — add fluid medium support
- `Unreal/Plugins/AirSim/Source/` — water rendering, underwater post-processing
