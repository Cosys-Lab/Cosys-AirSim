# SPEC-042: Autonomous Pedestrian AI & Crowd Simulation

**Priority:** P1
**Category:** Characters
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-035, SPEC-036

## Problem Statement

For realistic urban scenarios, characters need autonomous behavior beyond direct API control:

- Crowd simulation (dozens of pedestrians with realistic behavior)
- Navmesh-based pathfinding (already partially exists via GroupedAI)
- Social force model for crowd dynamics (avoiding collisions, personal space)
- Behavioral variety (some walk fast, some stop, some interact)
- Scenario scripting (pedestrian crosses road at specific time)

## Proposed Solution

### 1. Pedestrian AI Controller

```cpp
UCLASS()
class APedestrianAIController : public AAIController {
    GENERATED_BODY()

public:
    // Behavior mode
    enum class BehaviorMode {
        Patrol,         // Walk between waypoints
        Wander,         // Random walk within area
        GoTo,           // Walk to specific destination
        Follow,         // Follow another actor
        Flee,           // Run away from threat
        Idle,           // Stand still with idle animations
        Scripted,       // Follow scripted timeline
    };

    void SetBehavior(BehaviorMode mode, const FBehaviorParams& params);
    void SetWaypoints(const TArray<FVector>& waypoints, bool loop);

    // Social force model parameters
    float PersonalSpaceRadius = 1.0f;  // meters
    float AvoidanceStrength = 2.0f;
    float GroupCohesion = 0.5f;
};
```

### 2. Crowd Spawner API

```python
# Spawn a crowd of pedestrians
crowd = client.spawnCrowd(
    num_pedestrians=20,
    area_center=Vector3r(0, 0, 0),
    area_radius=50.0,
    behavior="wander",
    character_meshes=["mannequin", "casual_male", "casual_female"],
    speed_range=(1.0, 2.0),  # m/s
    seed=42
)
# Returns: ["Pedestrian_0", "Pedestrian_1", ..., "Pedestrian_19"]

# Configure individual pedestrian behavior
client.setPedestrianBehavior(
    vehicle_name="Pedestrian_5",
    behavior="patrol",
    waypoints=[(0,0,0), (10,0,0), (10,10,0), (0,10,0)],
    loop=True,
    speed=1.6
)

# Pedestrian crosses road at specific time
client.setPedestrianBehavior(
    vehicle_name="Pedestrian_3",
    behavior="scripted",
    script=[
        {"time": 0.0, "action": "idle", "position": (5, -3, 0)},
        {"time": 5.0, "action": "walk", "target": (5, 8, 0), "speed": 1.4},
        {"time": 12.0, "action": "idle"},
    ]
)

# Make pedestrian react to vehicle
client.setPedestrianBehavior(
    vehicle_name="Pedestrian_7",
    behavior="flee",
    threat_actor="Car1",
    flee_distance=10.0
)

# Destroy all crowd pedestrians
client.destroyCrowd(crowd)
```

### 3. Social Force Model

```cpp
class SocialForceModel {
    // Helbing & Molnar (1995) social force model
    FVector computeForce(const APedestrianCharacter& self,
                         const TArray<APedestrianCharacter*>& others,
                         const TArray<AActor*>& obstacles) {
        FVector force = FVector::ZeroVector;

        // Desired velocity force
        force += computeDesiredForce(self);

        // Repulsive force from other pedestrians
        for (auto* other : others) {
            force += computePedestrianRepulsion(self, *other);
        }

        // Repulsive force from obstacles/walls
        for (auto* obs : obstacles) {
            force += computeObstacleRepulsion(self, *obs);
        }

        return force;
    }
};
```

### 4. Performance: LOD for Large Crowds

```python
# Characters far from sensors get reduced processing
client.setCrowdLOD(
    full_detail_radius=30.0,     # Full animation + physics within 30m
    reduced_detail_radius=100.0, # Simplified animation 30-100m
    # Beyond 100m: no animation updates, position-only
)
```

## Acceptance Criteria

- [ ] `spawnCrowd()` spawns N pedestrians with varied appearance
- [ ] Patrol, wander, goto, follow, flee, idle, scripted behaviors
- [ ] Social force model prevents pedestrian-pedestrian overlap
- [ ] Pedestrians avoid obstacles and vehicles using navmesh
- [ ] Scripted behavior with timed actions (road crossing scenario)
- [ ] LOD system maintains > 30 FPS with 50+ pedestrians
- [ ] All pedestrians appear in sensor outputs (camera, LiDAR, annotation)
- [ ] Crowd can be spawned and destroyed at runtime

## Files Affected

- New: `Unreal/Plugins/AirSim/Source/Vehicles/Character/PedestrianAIController.h/.cpp`
- New: `Unreal/Plugins/AirSim/Source/Vehicles/Character/CrowdManager.h/.cpp`
- New: `Unreal/Plugins/AirSim/Source/Vehicles/Character/SocialForceModel.h/.cpp`
- `AirLib/include/api/WorldSimApiBase.hpp` — crowd spawning API
- `PythonClient/cosysairsim/client.py` — crowd API
