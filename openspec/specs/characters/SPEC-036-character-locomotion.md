# SPEC-036: Character Locomotion System

**Priority:** P0
**Category:** Characters
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-035

## Problem Statement

SPEC-035 establishes the character pawn and API. This spec implements the full locomotion system: walking, running, sprinting, crouching, crawling, rolling, jumping, and transitions between states.

## Proposed Solution

### 1. Locomotion State Machine

```
                    ┌─────────┐
            ┌──────►│  Idle   │◄──────┐
            │       └────┬────┘       │
            │            │            │
       stop │     move   │     stop   │
            │            ▼            │
       ┌────┴────┐  ┌─────────┐  ┌───┴─────┐
       │Crouching│  │ Walking │  │Crawling  │
       └────┬────┘  └────┬────┘  └───┬─────┘
            │            │            │
            │     sprint │            │
            │            ▼            │
            │       ┌─────────┐       │
            │       │ Running │       │
            │       └────┬────┘       │
            │            │            │
            │    sprint  │            │
            │            ▼            │
            │       ┌──────────┐      │
            │       │Sprinting │      │
            │       └──────────┘      │
            │                         │
            │       ┌─────────┐       │
            └──────►│ Rolling │◄──────┘
                    └─────────┘
```

### 2. Movement Parameters (Physically Realistic)

```cpp
struct CharacterMovementParams {
    // Speed limits (m/s) — based on biomechanics literature
    float max_walk_speed = 1.4f;       // Average human walk
    float max_run_speed = 3.5f;        // Comfortable jog
    float max_sprint_speed = 7.0f;     // Fast sprint (Bolt: 12.4)
    float max_crouch_speed = 0.8f;     // Crouched walk
    float max_crawl_speed = 0.5f;      // Army crawl
    float max_roll_speed = 2.0f;       // Tactical roll
    float max_swim_speed = 1.0f;       // Breaststroke

    // Acceleration (m/s^2)
    float walk_acceleration = 3.0f;
    float run_acceleration = 5.0f;
    float sprint_acceleration = 4.0f;
    float braking_deceleration = 8.0f;

    // Dimensions
    float standing_height = 1.75f;     // meters
    float crouching_height = 1.0f;
    float crawling_height = 0.4f;
    float capsule_radius = 0.34f;

    // Physics
    float mass = 75.0f;               // kg
    float jump_velocity = 4.2f;       // m/s (vertical)
    float max_step_height = 0.3f;     // meters
    float walkable_slope_angle = 50.0f; // degrees

    // Stamina (optional fatigue system)
    float max_stamina = 100.0f;
    float sprint_stamina_cost = 20.0f;  // per second
    float stamina_recovery_rate = 15.0f; // per second
    bool enable_stamina = false;
};
```

### 3. API Commands

```python
# Direct control
client.setCharacterControls(CharacterControls(
    move_forward=1.0,    # Full forward
    move_right=0.0,      # No strafe
    look_yaw=45.0,       # Look right 45 deg
    look_pitch=-10.0,    # Look slightly down
    mode=MovementMode.Running
), vehicle_name="Person1")

# High-level commands
client.moveToPositionAsync(10, 20, 0, speed=3.0, vehicle_name="Person1")
client.setMovementMode(MovementMode.Crawling, vehicle_name="Person1")
client.setMovementMode(MovementMode.Crouching, vehicle_name="Person1")

# Rolling (short burst, direction-based)
client.performRoll(direction="forward", vehicle_name="Person1")  # tactical roll

# Jump
client.jump(vehicle_name="Person1")

# Navmesh path following
client.followPathAsync(waypoints=[(0,0,0), (10,5,0), (20,10,0)],
                       speed=1.4, vehicle_name="Person1")
```

### 4. Unreal CharacterMovementComponent Configuration

```cpp
void ACharacterPawn::ConfigureMovement(const CharacterMovementParams& params) {
    auto* movement = GetCharacterMovement();
    movement->MaxWalkSpeed = params.max_walk_speed * 100.0f;  // cm/s
    movement->MaxAcceleration = params.walk_acceleration * 100.0f;
    movement->BrakingDecelerationWalking = params.braking_deceleration * 100.0f;
    movement->JumpZVelocity = params.jump_velocity * 100.0f;
    movement->MaxStepHeight = params.max_step_height * 100.0f;
    movement->SetWalkableFloorAngle(params.walkable_slope_angle);
    movement->GetNavAgentPropertiesRef().AgentRadius = params.capsule_radius * 100.0f;
    movement->GetNavAgentPropertiesRef().AgentHeight = params.standing_height * 100.0f;

    // Crouch config
    movement->NavAgentProps.bCanCrouch = true;
    GetCapsuleComponent()->SetCapsuleHalfHeight(params.standing_height * 50.0f);
}
```

### 5. Capsule Resizing for Crawl/Roll

```cpp
void ACharacterPawn::SetMovementMode(ECharacterMovementMode mode) {
    switch (mode) {
        case Crawling:
            // Flatten capsule for prone position
            GetCapsuleComponent()->SetCapsuleSize(
                params_.capsule_radius * 100.0f,
                params_.crawling_height * 50.0f);
            GetCharacterMovement()->MaxWalkSpeed = params_.max_crawl_speed * 100.0f;
            break;
        case Rolling:
            // Trigger roll montage, short-duration state
            PlayAnimMontage(RollMontage);
            FTimerHandle handle;
            GetWorldTimerManager().SetTimer(handle, [this](){
                SetMovementMode(previous_mode_);
            }, roll_duration_, false);
            break;
        // ...
    }
}
```

## Acceptance Criteria

- [ ] Walk at 1.4 m/s with animation blending
- [ ] Run at 3.5 m/s, sprint at 7.0 m/s
- [ ] Crouch with reduced capsule height and speed
- [ ] Crawl (prone) with flattened capsule, 0.5 m/s
- [ ] Roll as short burst (0.5-1.0s) with momentum
- [ ] Jump with realistic arc (gravity-based)
- [ ] Smooth transitions between all movement modes
- [ ] Speed configurable per-character in settings.json
- [ ] NavMesh pathfinding works for `moveToPositionAsync`
- [ ] Movement obeys collision (walls, obstacles, vehicles)

## Files Affected

- `Unreal/Plugins/AirSim/Source/Vehicles/Character/CharacterPawn.cpp` — movement config
- `AirLib/include/vehicles/character/api/CharacterApiBase.hpp` — movement commands
- `AirLib/include/common/AirSimSettings.hpp` — movement parameters
- `PythonClient/cosysairsim/client.py` — locomotion API methods
