# SPEC-040: Character Hiding & Cover System

**Priority:** P1
**Category:** Characters
**Effort:** Small (1-2 weeks)
**Dependencies:** SPEC-036, SPEC-037

## Problem Statement

Security, surveillance, and search-and-rescue simulations require characters that can:

- Hide behind objects (walls, vehicles, barriers, furniture)
- Peek around corners
- Take cover and emerge
- Be partially or fully occluded from sensors
- Exhibit evasive behavior

## Proposed Solution

### 1. Cover API

```python
# Character seeks nearest cover point
client.seekCover(vehicle_name="Person1")

# Hide behind specific object
client.hideAt(
    object_name="Wall_03",
    side="left",               # which side to hide behind
    vehicle_name="Person1"
)

# Peek from cover
client.peekFromCover(
    direction="left",          # peek left/right/over
    duration=2.0,              # seconds to peek
    vehicle_name="Person1"
)

# Leave cover
client.leaveCover(vehicle_name="Person1")

# Query cover state
state = client.getCoverState(vehicle_name="Person1")
# Returns: CoverState(in_cover=True, cover_object="Wall_03",
#          visibility=0.15, peek_state="none")
```

### 2. Cover Point System

```cpp
// Environment cover points (auto-detected or manually placed)
struct CoverPoint {
    FVector position;
    FVector cover_normal;     // Direction the cover faces
    float cover_height;       // How tall the cover is
    ECoverType type;          // Wall, Low_Wall, Vehicle, Pillar, etc.
    bool allows_peek_left;
    bool allows_peek_right;
    bool allows_peek_over;    // Low cover only
};

class CoverSystem {
    // Auto-detect cover points from environment geometry
    void scanForCoverPoints(float radius, const FVector& center);

    // Find nearest cover relative to threat direction
    CoverPoint findBestCover(const FVector& character_pos,
                              const FVector& threat_direction);
};
```

### 3. Visibility Estimation

```python
# How visible is a character from a given position?
visibility = client.getCharacterVisibility(
    observer_position=Vector3r(0, 0, 5),  # e.g., drone position
    vehicle_name="Person1"
)
# Returns: 0.0 (fully hidden) to 1.0 (fully exposed)
```

### 4. Animations

- `hide_crouch` — crouch behind low cover
- `hide_stand` — stand behind tall cover
- `hide_peek_left` / `hide_peek_right` — peek around corners
- `hide_peek_over` — peek over low cover
- `hide_enter` — transition from standing to cover
- `hide_leave` — transition from cover to standing

## Acceptance Criteria

- [ ] Characters can take cover behind world objects
- [ ] Peek animations (left, right, over) work from cover
- [ ] Cover reduces character visibility to sensors (LiDAR, camera)
- [ ] `getCharacterVisibility()` returns occlusion estimate
- [ ] Auto-detected cover points from environment geometry
- [ ] Multiple characters can use different cover points
- [ ] Cover API works with annotation system (hidden characters get less pixels)

## Files Affected

- New: `Unreal/Plugins/AirSim/Source/Vehicles/Character/CoverSystem.h/.cpp`
- `AirLib/include/vehicles/character/api/CharacterApiBase.hpp` — cover methods
- `PythonClient/cosysairsim/client.py` — cover API
