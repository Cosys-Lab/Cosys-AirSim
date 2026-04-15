# SPEC-033: Centralized State Management

**Priority:** P2
**Category:** Architecture
**Effort:** Medium (3-4 weeks)
**Dependencies:** SPEC-031

## Problem Statement

Simulation state is scattered across multiple owners with no central management:

1. **Kinematics** owned by `PawnSimApi` (member `kinematics_`, line 218)
2. **Environment** owned by `PawnSimApi` (member `environment_`, line 219)
3. **Collision** owned by `PawnSimApi` (member `state_.collision_info`, line 208)
4. **Vehicle-specific state** (rotor states, landed state) owned by firmware implementations
5. **Sensor state** owned by individual sensor objects
6. **World state** (weather, time) owned by `WorldSimApi`

Problems:
- No single source of truth for simulation state
- No state validation or consistency checking
- No state serialization for save/load
- No state rollback or undo capability
- No state diffing for efficient network synchronization
- No state snapshot for debugging/logging

## Proposed Solution

### 1. Centralized State Store

```cpp
class SimulationState {
    struct WorldState {
        float simulation_time;
        WeatherParams weather;
        TimeOfDayParams time_of_day;
        WindParams wind;
    };

    struct VehicleState {
        std::string name;
        std::string type;
        Kinematics::State kinematics;
        Environment::State environment;
        CollisionInfo collision;
        std::map<std::string, SensorState> sensors;

        // Vehicle-type-specific state
        std::variant<MultirotorState, CarState, SkidSteerState> specific;
    };

    WorldState world;
    std::map<std::string, VehicleState> vehicles;
    uint64_t version = 0;  // Monotonically increasing version number
};
```

### 2. State Operations

```cpp
class StateManager {
    SimulationState current_;
    std::vector<SimulationState> history_;  // For rollback
    size_t max_history_ = 100;

    // Atomic state update
    void update(std::function<void(SimulationState&)> mutator) {
        mutator(current_);
        current_.version++;
        if (history_.size() >= max_history_) history_.erase(history_.begin());
        history_.push_back(current_);
        notifySubscribers();
    }

    // Rollback
    bool rollback(int steps = 1);

    // Snapshot for save/load
    std::vector<uint8_t> serialize() const;
    void deserialize(const std::vector<uint8_t>& data);

    // Diff for efficient sync
    StateDiff diff(uint64_t since_version) const;

    // Subscribe to state changes
    void subscribe(std::function<void(const SimulationState&)> callback);
};
```

### 3. State Access API

```python
# Python client API
state = client.getSimulationState()
state.world.simulation_time
state.vehicles["Drone1"].kinematics.position
state.vehicles["Drone1"].sensors["Lidar1"].last_update

# Save/load simulation state
client.simSaveState("checkpoint_1")
client.simLoadState("checkpoint_1")

# State diff (efficient for continuous monitoring)
diff = client.simGetStateDiff(since_version=42)
```

### 4. State Validation

```cpp
class StateValidator {
    bool validate(const SimulationState& state) {
        // Physics consistency
        for (const auto& [name, vehicle] : state.vehicles) {
            if (vehicle.kinematics.pose.position.hasNaN()) return false;
            if (!vehicle.kinematics.pose.orientation.isApprox(
                    vehicle.kinematics.pose.orientation.normalized())) return false;
        }
        // World consistency
        if (state.world.simulation_time < 0) return false;
        return true;
    }
};
```

## Acceptance Criteria

- [ ] Single `SimulationState` struct containing all simulation state
- [ ] StateManager with atomic updates and version tracking
- [ ] State history with rollback capability (at least 10 states)
- [ ] State serialization for save/load
- [ ] State diff for efficient network synchronization
- [ ] Python API for state access, save, load
- [ ] State validation on every update (optional, debug mode)
- [ ] All existing state owners read from centralized store

## Files Affected

- New: `AirLib/include/common/SimulationState.hpp`
- New: `AirLib/include/common/StateManager.hpp`
- `Unreal/Plugins/AirSim/Source/PawnSimApi.h` — delegate state to StateManager
- `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp` — delegate state to StateManager
- `AirLib/include/api/VehicleSimApiBase.hpp` — read from StateManager
- `AirLib/include/api/WorldSimApiBase.hpp` — read/write StateManager
- `PythonClient/cosysairsim/client.py` — state access API
