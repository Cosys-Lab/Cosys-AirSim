# SPEC-027: Type-Safe Settings with Schema Validation

**Priority:** P1
**Category:** Engineering Quality
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

`AirSimSettings.hpp` (1,909 lines) has critical configuration issues:

1. **No type safety**: Settings loaded from JSON dynamically — any type mismatch is runtime failure
2. **No range validation**: 60+ float fields with NaN defaults (lines 176-234), no bounds checking
3. **No required field validation**: Missing fields silently get defaults
4. **Typo workaround** (lines 773-781): Handles "SettingsVersion" AND "SettingdVersion" (typo) — should fail, not work around
5. **Ancient TODOs** (line 676): "Remove this workaround after we only support Unreal 4.17" — Unreal 4.17 is from 2017
6. **Monolithic struct**: 30+ top-level public fields (lines 574-614) — vehicles, cameras, lights, beacons, sensors, recording, time-of-day all in one mutable struct
7. **No settings immutability**: Once loaded, settings can be modified via public fields
8. **Duplicate TODOs** (lines 834, 840, 850, 857): Same comment repeated 4 times

## Proposed Solution

### 1. JSON Schema Definition

```json
{
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "type": "object",
    "properties": {
        "SettingsVersion": { "type": "number", "minimum": 2.0 },
        "SimMode": {
            "type": "string",
            "enum": ["Multirotor", "Car", "SkidVehicle", "ComputerVision", ""]
        },
        "PhysicsEngineName": {
            "type": "string",
            "enum": ["FastPhysicsEngine", "ExternalPhysicsEngine"]
        },
        "Vehicles": {
            "type": "object",
            "additionalProperties": { "$ref": "#/$defs/VehicleSetting" }
        }
    },
    "required": ["SettingsVersion"]
}
```

### 2. Programmatic Validation

```cpp
class SettingsValidator {
    using ValidationResult = std::pair<bool, std::vector<std::string>>;

    ValidationResult validate(const Settings& json) {
        std::vector<std::string> errors;

        // Version check (strict)
        if (!json.hasKey("SettingsVersion")) {
            errors.push_back("Missing required field: SettingsVersion");
        }

        // Range validation
        auto validateRange = [&](const std::string& path, float value,
                                  float min, float max) {
            if (value < min || value > max) {
                errors.push_back(fmt::format("{} = {} out of range [{}, {}]",
                                             path, value, min, max));
            }
        };

        // Camera settings
        validateRange("CaptureSetting.FOV_Degrees", capture.fov, 1.0f, 179.0f);
        validateRange("CaptureSetting.Width", capture.width, 1, 7680);
        validateRange("CaptureSetting.Height", capture.height, 1, 4320);

        // Sensor settings
        validateRange("LidarSetting.range", lidar.range, 0.1f, 1000.0f);
        validateRange("LidarSetting.channels", lidar.channels, 1, 256);

        return {errors.empty(), errors};
    }
};
```

### 3. Immutable Settings After Load

```cpp
class AirSimSettings {
    // Settings are immutable after load
    class Snapshot {
        friend class AirSimSettings;
        const VehicleSettings vehicles;
        const CameraSettings cameras;
        const SensorSettings sensors;
        // ... all const
    };

    static std::shared_ptr<const Snapshot> load(const std::string& json);
    static std::shared_ptr<const Snapshot> current();
};
```

### 4. Remove Dead Code

- Remove "SettingdVersion" typo workaround (line 773-781)
- Remove "Unreal 4.17 workaround" (line 676)
- Remove 4 duplicate TODO comments (lines 834-857)
- Clean up NaN sentinel defaults — use `std::optional<float>` instead

## Acceptance Criteria

- [ ] JSON schema file for settings.json validation
- [ ] Programmatic validation with clear error messages on load
- [ ] Invalid ranges rejected (not silently accepted)
- [ ] Settings immutable after initialization
- [ ] NaN sentinel values replaced with `std::optional`
- [ ] Dead code and ancient workarounds removed
- [ ] Typo workaround removed — strict field name matching
- [ ] Example settings.json validated against schema
- [ ] Migration guide for settings format changes

## Files Affected

- `AirLib/include/common/AirSimSettings.hpp` — validation, immutability, cleanup
- New: `AirLib/data/settings_schema.json`
- New: `AirLib/include/common/SettingsValidator.hpp`
- `docs/settings.md` — update with schema reference
