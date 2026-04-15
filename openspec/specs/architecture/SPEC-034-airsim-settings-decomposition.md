# SPEC-034: AirSimSettings Decomposition

**Priority:** P1
**Category:** Architecture
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-027

## Problem Statement

`AirSimSettings.hpp` is a 1,909-line god header containing configuration for every subsystem:

- Vehicles (line 602)
- Cameras (line 603)
- Lights (line 607)
- Beacons (line 605)
- Sensors (line 610)
- Annotations (line 581)
- Recording (line 579)
- Time-of-day (line 580)
- Subwindows (line 578)
- External cameras (line 606)
- Physics (lines 598-601)
- 30+ top-level public fields (lines 574-614)
- 60+ nested struct definitions

This creates:
- Massive compilation dependency — every file that includes settings recompiles when any setting changes
- Poor cohesion — camera settings and physics settings have no relationship
- No independent evolution — changing sensor settings requires touching the same file as vehicle settings
- Long compilation times from header-only instantiation

## Proposed Solution

### 1. Domain-Specific Settings Files

```
AirLib/include/common/settings/
  SettingsBase.hpp          # Base class with JSON parsing utilities
  VehicleSettings.hpp       # Vehicle configuration
  CameraSettings.hpp        # Camera/capture configuration
  SensorSettings.hpp        # Sensor configuration
  PhysicsSettings.hpp       # Physics engine configuration
  RenderingSettings.hpp     # Rendering/visual configuration
  RecordingSettings.hpp     # Recording configuration
  EnvironmentSettings.hpp   # Weather, time-of-day, lighting
  NetworkSettings.hpp       # RPC/API configuration
  AirSimSettings.hpp        # Composed aggregate (thin, includes above)
```

### 2. Domain Settings Classes

```cpp
// CameraSettings.hpp
struct CameraSettings {
    struct CaptureSetting {
        int width = 256;
        int height = 144;
        float fov_degrees = 90.0f;
        ImageType image_type = ImageType::Scene;
        // ... camera-specific only
    };

    std::map<std::string, CaptureSetting> cameras;

    static CameraSettings loadFromJson(const Settings& json);
    void validate(std::vector<std::string>& errors) const;
};

// SensorSettings.hpp
struct SensorSettings {
    struct LidarSetting { ... };
    struct EchoSetting { ... };
    struct ImuSetting { ... };
    // Only sensor-related settings

    static SensorSettings loadFromJson(const Settings& json);
    void validate(std::vector<std::string>& errors) const;
};
```

### 3. Thin Aggregate

```cpp
// AirSimSettings.hpp — now just a composed aggregate
class AirSimSettings {
    VehicleSettings vehicles;
    CameraSettings cameras;
    SensorSettings sensors;
    PhysicsSettings physics;
    RenderingSettings rendering;
    RecordingSettings recording;
    EnvironmentSettings environment;
    NetworkSettings network;

    static AirSimSettings load(const std::string& json_path);
    bool validate(std::vector<std::string>& errors) const;
};
```

### 4. Lazy Loading

```cpp
// Components only include what they need
// Physics engine only includes PhysicsSettings.hpp
// Camera system only includes CameraSettings.hpp
// No more rebuilding everything when sensor settings change
```

## Acceptance Criteria

- [ ] AirSimSettings.hpp reduced from 1,909 lines to < 200 lines (aggregate only)
- [ ] At least 6 domain-specific settings files
- [ ] Each domain settings has its own `loadFromJson()` and `validate()`
- [ ] Compilation dependency: changing SensorSettings doesn't recompile physics code
- [ ] Existing JSON format backward compatible (same settings.json structure)
- [ ] Each domain settings independently testable
- [ ] Include graph simplified (measure with include-what-you-use)

## Files Affected

- `AirLib/include/common/AirSimSettings.hpp` — decompose into modules
- New: `AirLib/include/common/settings/` (6-8 new files)
- All files including AirSimSettings.hpp — update includes to use specific settings
- `AirLibUnitTests/SettingsTest.hpp` — test each domain independently
