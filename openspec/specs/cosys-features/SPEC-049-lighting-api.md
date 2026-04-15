# SPEC-049: Comprehensive Artificial Lighting API

**Priority:** P1
**Category:** Cosys Features
**Effort:** Small (1 week)
**Dependencies:** None

## Problem Statement

Light APIs only cover visibility and intensity (`simSetWorldLightVisibility/Intensity`, `simSetVehicleLightVisibility/Intensity`). Settings support color, type, cone angle, shadow casting, but no runtime API for:

- Light color/temperature control
- Cone angle for spotlights
- Shadow casting toggle
- Light type changes (point/spot/rectangular)
- Programmatic light creation/destruction

## Proposed Solution

```python
# Full light parameter control
client.simSetLightParameters("SpotLight_01", {
    "visible": True,
    "intensity": 5000.0,       # lumens
    "color": (255, 240, 220),  # RGB
    "temperature": 4500,       # Kelvin (overrides color if set)
    "type": "spot",            # point, spot, rect
    "cone_angle": 45.0,        # degrees (spot only)
    "attenuation_radius": 50.0,
    "cast_shadows": True,
    "source_radius": 0.1,     # meters (soft shadow control)
})

# Create light at runtime
client.simCreateLight(
    name="SearchLight_01",
    type="spot",
    pose=Pose(Vector3r(0, 0, 10), Quaternionr()),
    intensity=20000,
    cone_angle=30,
    color=(255, 255, 255),
)

# Destroy runtime light
client.simDestroyLight("SearchLight_01")

# Animate light (e.g., flashing warning light)
client.simSetLightFlashing("WarningLight_01",
    frequency_hz=2.0, duty_cycle=0.5)
```

## Acceptance Criteria

- [ ] Full light parameter control via API (color, temperature, cone, shadows)
- [ ] Runtime light creation and destruction
- [ ] Light flashing/animation support
- [ ] All settings-configurable parameters exposed via API
- [ ] Works with vehicle-mounted and world-placed lights

## Files Affected

- `AirLib/include/api/WorldSimApiBase.hpp` — light parameter API
- `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` — UE light manipulation
- `PythonClient/cosysairsim/client.py` — light API methods
