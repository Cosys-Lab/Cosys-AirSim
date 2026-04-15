# SPEC-041: Motorbike Vehicle Type

**Priority:** P1
**Category:** Characters
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-038

## Problem Statement

Motorbikes are common in urban environments worldwide. Autonomous systems must handle:

- Motorbike detection (different profile from cars)
- Lane-splitting behavior
- Rider + pillion passenger scenarios
- Two-wheeled vehicle dynamics (lean-based turning)

No two-wheeled vehicle exists in Cosys-AirSim.

## Proposed Solution

### 1. New Vehicle Type

```cpp
// AirSimSettings.hpp
static constexpr char const* kVehicleTypeMotorbike = "motorbike";
```

### 2. MotorbikePawn

```cpp
UCLASS()
class AIRSIM_API AMotorbikePawn : public APawn {
    GENERATED_BODY()

public:
    // Skeletal mesh for motorbike body (wheels, frame, handlebars)
    UPROPERTY(VisibleAnywhere)
    USkeletalMeshComponent* BikeMesh;

    // Physics
    UPROPERTY(VisibleAnywhere)
    UMotorbikeMovementComponent* MovementComponent;

    // Cameras (rider POV, rear mirror, side)
    TMap<FString, APIPCamera*> Cameras;

    // Lean angle for visual realism
    float CurrentLeanAngle;

    // Seat attachment points
    FVector RiderSeatOffset;
    FVector PillionSeatOffset;
};
```

### 3. Motorbike Physics Model

```cpp
class MotorbikePhysicsModel {
    struct MotorbikeParams {
        float mass = 200.0f;            // kg (typical sport bike)
        float wheelbase = 1.4f;         // meters
        float cg_height = 0.6f;         // center of gravity height
        float max_lean_angle = 45.0f;   // degrees
        float max_speed = 50.0f;        // m/s (~180 km/h)
        float max_acceleration = 5.0f;  // m/s^2
        float max_braking = 8.0f;       // m/s^2
        float steering_ratio = 15.0f;   // steering to wheel angle
    };

    // Lean-based turning (counter-steering model)
    float computeLeanAngle(float speed, float turn_radius) {
        // θ = arctan(v² / (r * g))
        return std::atan2(speed * speed, turn_radius * 9.81f);
    }

    // Two-wheel dynamics
    void update(float throttle, float steering, float brake, float dt);
};
```

### 4. Motorbike API

```python
class MotorbikeControls:
    throttle: float    # 0 to 1
    brake: float       # 0 to 1
    steering: float    # -1 (left) to 1 (right)
    lean_override: float  # Optional manual lean (-1 to 1)

class MotorbikeState:
    speed: float           # m/s
    lean_angle: float      # degrees
    gear: int
    rpm: float
    kinematics: KinematicsState
    rider_present: bool
    pillion_present: bool

# Control
client.setMotorbikeControls(MotorbikeControls(...), vehicle_name="Bike1")
state = client.getMotorbikeState(vehicle_name="Bike1")
```

### 5. Rider Attachment

When a character mounts a motorbike:
- Character mesh attached to rider seat point
- Character plays `motorbike_ride_idle` animation
- Character leans with bike (animation driven by lean angle)
- Character's cameras move with bike
- Pillion passenger plays `motorbike_pillion_idle`

### 6. Settings

```json
{
    "Vehicles": {
        "Bike1": {
            "VehicleType": "motorbike",
            "Model": "SportBike",
            "Mass": 200,
            "MaxSpeed": 50,
            "MaxLeanAngle": 45,
            "Seats": {
                "rider": {"Offset": {"X": -0.1, "Y": 0, "Z": 0.8}},
                "pillion": {"Offset": {"X": -0.5, "Y": 0, "Z": 0.75}}
            }
        }
    }
}
```

## Acceptance Criteria

- [ ] Motorbike spawns and is controllable via API
- [ ] Lean-based turning with visible lean angle
- [ ] Two-wheeled physics (can fall over at zero speed if unbalanced)
- [ ] Rider seat and pillion seat for character mounting
- [ ] Motorbike detectable by all sensor types (camera, LiDAR, echo)
- [ ] Proper annotation/segmentation (separate from rider)
- [ ] Multiple motorbike models configurable via settings
- [ ] Speed, braking, and acceleration realistic for motorcycle

## Files Affected

- `AirLib/include/common/AirSimSettings.hpp` — add motorbike type + params
- New: `AirLib/include/vehicles/motorbike/api/MotorbikeApiBase.hpp`
- New: `Unreal/Plugins/AirSim/Source/Vehicles/Motorbike/MotorbikePawn.h/.cpp`
- New: `Unreal/Plugins/AirSim/Source/Vehicles/Motorbike/MotorbikePawnSimApi.h/.cpp`
- `PythonClient/cosysairsim/client.py` — motorbike API
- `PythonClient/cosysairsim/types.py` — MotorbikeState, MotorbikeControls
