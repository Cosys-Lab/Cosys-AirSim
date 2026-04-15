# SPEC-003: Programmatic Domain Randomization API

**Priority:** P0
**Category:** ML/Autonomy
**Effort:** Medium (3-4 weeks)
**Dependencies:** None

## Problem Statement

Domain randomization is critical for sim-to-real transfer, but Cosys-AirSim lacks programmatic control:

- Weather control exists (`simSetWeatherParameter`) but only for rain/snow/dust/fog
- Object spawning (`simSpawnObject`) is manual with hardcoded placements (`create_objects.py:15-32`)
- No API to randomize physics parameters (vehicle mass, inertia, drag, friction)
- No programmatic texture/material randomization beyond `simSwapTextures()`
- No lighting direction/intensity/color control API
- Seeds for dynamic objects exist but aren't controllable from Python
- No randomization presets or configuration recipes

## Proposed Solution

### 1. DomainRandomizer API

```python
class DomainRandomizer:
    def __init__(self, client, seed=None):
        self.rng = np.random.default_rng(seed)

    def randomize_weather(self, config: WeatherRandomConfig):
        """Randomize weather within specified parameter ranges."""

    def randomize_lighting(self, config: LightingRandomConfig):
        """Randomize sun direction, intensity, color temperature, ambient."""

    def randomize_textures(self, objects: list[str], texture_library: str):
        """Swap textures on specified objects from texture library."""

    def randomize_materials(self, objects: list[str], config: MaterialRandomConfig):
        """Randomize material properties (roughness, metallic, specular)."""

    def randomize_vehicle_dynamics(self, vehicle: str, config: DynamicsRandomConfig):
        """Randomize mass, inertia, drag coefficients, motor response."""

    def randomize_sensor_noise(self, vehicle: str, config: SensorNoiseConfig):
        """Randomize sensor noise parameters (bias, scale, drift)."""

    def randomize_camera_parameters(self, camera: str, config: CameraRandomConfig):
        """Randomize FoV, exposure, motion blur, chromatic aberration."""

    def randomize_object_placement(self, config: PlacementRandomConfig):
        """Spawn/move objects within specified spatial bounds."""

    def get_config_hash(self) -> str:
        """Return reproducibility hash for current randomization state."""
```

### 2. Configuration Ranges

```python
@dataclass
class WeatherRandomConfig:
    rain: tuple[float, float] = (0.0, 1.0)
    fog: tuple[float, float] = (0.0, 0.5)
    snow: tuple[float, float] = (0.0, 0.0)  # disabled by default
    dust: tuple[float, float] = (0.0, 0.3)
    wind_speed: tuple[float, float] = (0.0, 15.0)  # m/s
    wind_direction: tuple[float, float] = (0.0, 360.0)

@dataclass
class DynamicsRandomConfig:
    mass_scale: tuple[float, float] = (0.8, 1.2)  # ±20%
    inertia_scale: tuple[float, float] = (0.9, 1.1)
    drag_coefficient_scale: tuple[float, float] = (0.7, 1.3)
    motor_lag_scale: tuple[float, float] = (0.8, 1.2)
    max_thrust_scale: tuple[float, float] = (0.9, 1.1)

@dataclass
class SensorNoiseConfig:
    imu_gyro_bias_range: tuple[float, float] = (0.0, 0.01)
    imu_accel_bias_range: tuple[float, float] = (0.0, 0.005)
    gps_noise_std_range: tuple[float, float] = (0.5, 3.0)
    lidar_noise_scale_range: tuple[float, float] = (0.5, 2.0)
    camera_exposure_range: tuple[float, float] = (-2.0, 2.0)  # EV
```

### 3. New Server-Side APIs

```cpp
// AirLib additions
void setVehicleMass(const std::string& vehicle, float mass);
void setVehicleDragCoefficient(const std::string& vehicle, float cd);
void setVehicleInertia(const std::string& vehicle, const Vector3r& inertia);
void setMotorResponseLag(const std::string& vehicle, float lag_seconds);
void setSensorNoiseParams(const std::string& vehicle, const std::string& sensor,
                          const SensorNoiseParams& params);
void setLightingParameters(const LightingParams& params);
void setMaterialProperties(const std::string& object,
                           const MaterialProperties& props);
```

### 4. Randomization Presets

```python
# Built-in presets
PRESETS = {
    "mild": {...},       # Small parameter variations for fine-tuning
    "moderate": {...},   # Moderate variations for general training
    "aggressive": {...}, # Large variations for robust sim-to-real
    "photorealistic": {...}, # Lighting/material focus for visual tasks
    "dynamics_heavy": {...}, # Physics parameter focus for control tasks
}
```

## Acceptance Criteria

- [ ] Python API to randomize weather, lighting, textures, physics, sensors in one call
- [ ] Server-side APIs for runtime parameter modification (mass, drag, sensor noise)
- [ ] Seed-based reproducibility with configuration hash
- [ ] At least 5 built-in presets covering common use cases
- [ ] Episode reset randomizes selected parameters automatically
- [ ] Documentation with sim-to-real transfer guide
- [ ] Performance overhead < 5ms per randomization cycle

## Risks

- Runtime physics parameter changes may destabilize simulation — add validation bounds
- Texture randomization depends on UE material system — may need custom materials
- Some parameters (inertia) can't be changed without reinitializing physics bodies

## Files Affected

- `AirLib/include/vehicles/multirotor/MultiRotorParams.hpp` — add runtime parameter setters
- `AirLib/include/physics/FastPhysicsEngine.hpp` — add dynamic parameter modification
- `AirLib/include/sensors/` — add noise parameter setters to all sensor types
- `AirLib/include/api/WorldSimApiBase.hpp` — add lighting/material APIs
- `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` — implement UE-side APIs
- New: `PythonClient/cosysairsim/randomization.py`
