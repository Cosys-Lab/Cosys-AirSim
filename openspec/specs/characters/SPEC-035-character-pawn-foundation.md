# SPEC-035: Character Pawn & API Foundation

**Priority:** P0
**Category:** Characters
**Effort:** Medium (2-3 weeks)
**Dependencies:** None
**PR Scope:** Single focused PR adding the core character vehicle type

## Problem Statement

Cosys-AirSim has no human character type. Training autonomous systems for real-world deployment requires:

- Pedestrian detection and avoidance
- Human-robot interaction scenarios
- Crowd simulation for urban autonomy
- Occupant simulation for vehicle interior sensing
- Security/surveillance simulation

The existing `ComputerVisionPawn` (extends `APawn`, has `UManualPoseController`, no physics) is the ideal architectural base. The existing `GroupedAI` system proves skeletal meshes + animation blueprints already work in this codebase.

## Proposed Solution

### 1. New Vehicle Type Registration

```cpp
// AirSimSettings.hpp
static constexpr char const* kVehicleTypeCharacter = "character";
```

### 2. CharacterPawn (Unreal Side)

```cpp
// Vehicles/Character/CharacterPawn.h
UCLASS()
class AIRSIM_API ACharacterPawn : public ACharacter {
    GENERATED_BODY()

public:
    ACharacterPawn();

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void NotifyHit(...) override;

    // Pawn events for collision and tick signals
    PawnEvents* getPawnEvents() { return &pawn_events_; }

    // Camera access (first-person + third-person + custom)
    const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;

    // Skeletal mesh & animation
    USkeletalMeshComponent* getCharacterMesh() const;
    UAnimInstance* getAnimInstance() const;

    // Movement
    UCharacterMovementComponent* getCharacterMovement() const;

private:
    PawnEvents pawn_events_;

    UPROPERTY()
    TMap<FString, APIPCamera*> cameras_;

    // First-person camera mounted at head height
    UPROPERTY(VisibleAnywhere)
    APIPCamera* fp_camera_;

    // Third-person camera for external view
    UPROPERTY(VisibleAnywhere)
    APIPCamera* tp_camera_;
};
```

Key design decisions:
- Extends `ACharacter` (not `APawn`) for built-in `UCharacterMovementComponent` with walking, jumping, crouching, falling, swimming support
- Built-in `UCapsuleComponent` for collision
- Built-in navmesh integration for AI-driven movement
- Skeletal mesh with animation blueprint support

### 3. CharacterApiBase (AirLib Side)

```cpp
// AirLib/include/vehicles/character/api/CharacterApiBase.hpp
class CharacterApiBase : public VehicleApiBase {
public:
    // Movement state
    enum class MovementMode : uint8_t {
        Idle, Walking, Running, Sprinting, Crouching,
        Crawling, Rolling, Jumping, Falling, Swimming, Custom
    };

    // Character-specific controls
    struct CharacterControls {
        float move_forward;       // -1 to 1
        float move_right;         // -1 to 1
        float look_yaw;           // degrees
        float look_pitch;         // degrees
        MovementMode mode;
        bool is_crouching;
    };

    // Character state
    struct CharacterState {
        MovementMode movement_mode;
        float speed;              // m/s
        float heading;            // radians
        Pose pose;
        Kinematics::State kinematics;
        bool is_grounded;
        bool is_in_vehicle;
        std::string current_vehicle;  // empty if not in vehicle
        std::string current_animation;
        uint64_t timestamp;
    };

    // Core API
    virtual void setCharacterControls(const CharacterControls& controls) = 0;
    virtual CharacterState getCharacterState() const = 0;

    // Movement commands
    virtual bool moveToPositionAsync(float x, float y, float z,
                                      float speed, float timeout_sec) = 0;
    virtual bool moveByVelocityAsync(float vx, float vy, float vz,
                                      float duration) = 0;
    virtual void setMovementMode(MovementMode mode) = 0;
    virtual void setMaxSpeed(float speed_mps) = 0;

    // Animation
    virtual bool playAnimation(const std::string& animation_name,
                                bool loop = false) = 0;
    virtual void stopAnimation() = 0;
    virtual std::vector<std::string> getAvailableAnimations() const = 0;

    // Vehicle interaction
    virtual bool enterVehicle(const std::string& vehicle_name) = 0;
    virtual bool exitVehicle() = 0;
    virtual bool isInVehicle() const = 0;
};
```

### 4. SimModeCharacter

```cpp
// SimModeCharacter.h
class ASimModeCharacter : public ASimModeBase {
protected:
    virtual bool isVehicleTypeSupported(const std::string& vehicle_type) const override {
        return vehicle_type == AirSimSettings::kVehicleTypeCharacter;
    }
    virtual std::string getVehiclePawnPathName(
        const AirSimSettings::VehicleSetting& vehicle_setting) const override;
    // ...
};
```

### 5. Python Client API

```python
class CharacterClient(CosysAirSimClientBase):
    def setCharacterControls(self, controls, vehicle_name=""):
        """Set movement controls (forward, right, look, mode)."""

    def getCharacterState(self, vehicle_name="") -> CharacterState:
        """Get current movement mode, speed, pose, animation state."""

    def moveToPositionAsync(self, x, y, z, speed, vehicle_name=""):
        """Walk/run to world position using navmesh pathfinding."""

    def setMovementMode(self, mode, vehicle_name=""):
        """Switch between idle/walking/running/crouching/crawling."""

    def playAnimation(self, animation_name, loop=False, vehicle_name=""):
        """Play named animation montage."""

    def enterVehicle(self, vehicle_name, character_name=""):
        """Character enters specified vehicle."""

    def exitVehicle(self, character_name=""):
        """Character exits current vehicle."""
```

### 6. Settings Configuration

```json
{
    "SimMode": "Character",
    "Vehicles": {
        "Person1": {
            "VehicleType": "character",
            "SkeletalMesh": "/Game/Characters/Mannequin/SK_Mannequin",
            "AnimBlueprint": "/Game/Characters/Mannequin/ABP_Mannequin",
            "DefaultMovementMode": "idle",
            "MaxWalkSpeed": 1.4,
            "MaxRunSpeed": 3.5,
            "MaxSprintSpeed": 7.0,
            "MaxCrawlSpeed": 0.5,
            "CapsuleRadius": 0.34,
            "CapsuleHalfHeight": 0.88,
            "Cameras": {
                "fp": {"Position": {"X": 0.1, "Y": 0, "Z": 0.7}},
                "tp": {"Position": {"X": -2.0, "Y": 0, "Z": 1.5}}
            },
            "Sensors": { ... }
        }
    }
}
```

## Acceptance Criteria

- [ ] `"character"` vehicle type recognized by settings parser
- [ ] `ACharacterPawn` spawns with skeletal mesh and capsule collider
- [ ] Character visible in simulation viewport
- [ ] `CharacterApiBase` has movement mode enum and state struct
- [ ] Python client can `getCharacterState()` and `setCharacterControls()`
- [ ] Character appears in annotation/segmentation renders
- [ ] Multiple characters can be spawned via settings
- [ ] Characters can be spawned at runtime via `simAddVehicle()`
- [ ] First-person and third-person cameras functional

## Risks

- `ACharacter` vs `APawn` choice affects physics behavior — ACharacter has CharacterMovementComponent which handles gravity, steps, slopes automatically
- Mixed SimMode (characters + vehicles) requires SimMode that supports both types — address in SPEC-038

## Files Affected

- `AirLib/include/common/AirSimSettings.hpp` — add `kVehicleTypeCharacter`
- New: `AirLib/include/vehicles/character/api/CharacterApiBase.hpp`
- New: `AirLib/include/vehicles/character/api/CharacterRpcLibClient.hpp`
- New: `AirLib/include/vehicles/character/api/CharacterRpcLibServer.hpp`
- New: `Unreal/Plugins/AirSim/Source/Vehicles/Character/CharacterPawn.h/.cpp`
- New: `Unreal/Plugins/AirSim/Source/Vehicles/Character/CharacterPawnSimApi.h/.cpp`
- New: `Unreal/Plugins/AirSim/Source/Vehicles/Character/SimModeCharacter.h`
- `PythonClient/cosysairsim/client.py` — add `CharacterClient` class
- `PythonClient/cosysairsim/types.py` — add `CharacterState`, `CharacterControls`
