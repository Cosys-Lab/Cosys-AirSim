# SPEC-037: Character Animation System

**Priority:** P0
**Category:** Characters
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-036

## Problem Statement

Characters need rich animations beyond locomotion: dancing, gestures, idle variations, interaction poses, and custom montages. These are essential for:

- Realistic pedestrian simulation (people waiting, talking, using phones)
- Human-robot interaction training (waving, pointing, signaling)
- Activity recognition dataset generation
- Surveillance/security scenario simulation

## Proposed Solution

### 1. Animation Blueprint Architecture

```
CharacterAnimBlueprint
├── Locomotion State Machine
│   ├── Idle (with variations: standing, looking around, checking phone)
│   ├── Walk (with direction blendspace)
│   ├── Run (with direction blendspace)
│   ├── Sprint
│   ├── Crouch Walk
│   ├── Crawl
│   ├── Roll
│   └── Jump (start, loop, land)
├── Upper Body Layer (additive)
│   ├── Wave
│   ├── Point
│   ├── Hold object
│   └── Look at target (IK)
└── Full Body Montage Slot
    ├── Dance sequences
    ├── Sit down / Stand up
    ├── Hide behind cover
    └── Custom montages
```

### 2. Animation API

```python
# Play a full-body animation montage
client.playAnimation("dance_hip_hop", loop=True, vehicle_name="Person1")
client.playAnimation("sit_down", loop=False, vehicle_name="Person1")
client.playAnimation("wave_hello", loop=False, vehicle_name="Person1")

# Stop current animation (return to locomotion state machine)
client.stopAnimation(vehicle_name="Person1")

# Upper body overlay (character can walk while waving)
client.playUpperBodyAnimation("wave", vehicle_name="Person1")

# Query available animations
anims = client.getAvailableAnimations(vehicle_name="Person1")
# Returns: ["idle", "walk", "run", "dance_hip_hop", "dance_salsa",
#           "wave_hello", "point", "sit_down", "stand_up",
#           "hide_crouch", "hide_peek", "crawl_army", "roll_tactical", ...]

# Get current animation state
state = client.getAnimationState(vehicle_name="Person1")
# Returns: AnimationState(current="walk", progress=0.65,
#          upper_body="wave", is_montage=False)

# Set animation playback speed
client.setAnimationSpeed(1.5, vehicle_name="Person1")  # 1.5x speed
```

### 3. Built-in Animation Set (Minimum Viable)

```yaml
locomotion:
  - idle_stand           # Default standing idle
  - idle_look_around     # Looking around casually
  - idle_phone           # Looking at phone
  - walk_forward         # Standard walk cycle
  - walk_backward        # Walking backwards
  - run_forward          # Jogging
  - sprint               # Fast run
  - crouch_idle          # Crouching still
  - crouch_walk          # Crouched movement
  - crawl_army           # Prone crawl
  - roll_tactical        # Combat/tactical roll
  - jump_start           # Jump takeoff
  - jump_loop            # Airborne
  - jump_land            # Landing

actions:
  - wave_hello           # Friendly wave
  - wave_stop            # Stop/halt signal
  - point_forward        # Pointing direction
  - thumbs_up            # Approval gesture
  - sit_down             # Transition to sitting
  - sit_idle             # Seated idle
  - stand_up             # Transition to standing
  - pick_up              # Bend and pick up object
  - push_button          # Interact with panel/button

dance:
  - dance_hip_hop        # Hip hop dance
  - dance_salsa          # Partner dance (solo)
  - dance_robot          # Robot dance
  - dance_victory        # Victory celebration

combat_tactical:
  - hide_crouch          # Crouch behind cover
  - hide_peek_left       # Peek left from cover
  - hide_peek_right      # Peek right from cover
  - dodge_left           # Quick dodge
  - dodge_right          # Quick dodge

vehicle_interaction:
  - enter_car_left       # Open door, sit in left side
  - enter_car_right      # Open door, sit in right side
  - exit_car_left        # Open door, step out left
  - exit_car_right       # Open door, step out right
  - mount_motorbike      # Swing leg over motorbike
  - dismount_motorbike   # Step off motorbike
```

### 4. Custom Animation Import

```python
# Users can add custom animations via UE asset paths
client.registerAnimation(
    name="my_custom_dance",
    asset_path="/Game/MyProject/Animations/CustomDance",
    category="dance",
    is_looping=True,
    vehicle_name="Person1"
)
```

### 5. Bone/Joint Access for Pose Estimation

```python
# Get skeleton bone transforms (for pose estimation ground truth)
bones = client.getCharacterBoneTransforms(vehicle_name="Person1")
# Returns: dict of bone_name -> Pose
# {"head": Pose(...), "spine_01": Pose(...), "hand_r": Pose(...), ...}

# Get 2D projected joint positions (for 2D pose estimation GT)
joints_2d = client.getCharacterJoints2D(camera_name="cam0", vehicle_name="Person1")
# Returns: dict of joint_name -> (x, y, visibility)
```

## Acceptance Criteria

- [ ] Animation blueprint with locomotion state machine + montage slot
- [ ] At least 20 built-in animations across locomotion, actions, dance, tactical
- [ ] `playAnimation()` triggers montages, `stopAnimation()` returns to state machine
- [ ] Upper body overlay allows walking + waving simultaneously
- [ ] `getAvailableAnimations()` lists all registered animations
- [ ] `getCharacterBoneTransforms()` returns skeleton pose
- [ ] `getCharacterJoints2D()` returns projected 2D joints for pose estimation GT
- [ ] Custom animation registration from UE asset paths
- [ ] Animation speed control

## Files Affected

- `Unreal/Plugins/AirSim/Source/Vehicles/Character/CharacterPawn.cpp` — animation interface
- New: `Unreal/Plugins/AirSim/Content/Characters/ABP_AirSimCharacter` (AnimBP)
- `AirLib/include/vehicles/character/api/CharacterApiBase.hpp` — animation methods
- `PythonClient/cosysairsim/client.py` — animation API
- `PythonClient/cosysairsim/types.py` — AnimationState type
