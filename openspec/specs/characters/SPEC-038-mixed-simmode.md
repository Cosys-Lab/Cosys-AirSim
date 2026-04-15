# SPEC-038: Mixed SimMode (Characters + Vehicles)

**Priority:** P0
**Category:** Characters
**Effort:** Medium (2 weeks)
**Dependencies:** SPEC-035

## Problem Statement

Current SimMode architecture requires choosing ONE vehicle type per session (Car, Multirotor, SkidVehicle, ComputerVision, or Character). Real-world autonomy scenarios need mixed environments:

- Cars driving among pedestrians
- Drones observing people on the ground
- Robots navigating around humans
- Motorbikes in mixed traffic

The existing `createVehicleAtRuntime()` in `SimModeBase` already supports runtime vehicle spawning, but `isVehicleTypeSupported()` only allows one type per SimMode subclass.

## Proposed Solution

### 1. SimModeMixed — Universal SimMode

```cpp
class ASimModeMixed : public ASimModeWorldBase {
protected:
    bool isVehicleTypeSupported(const std::string& vehicle_type) const override {
        // Support ALL vehicle types
        static const std::set<std::string> supported = {
            AirSimSettings::kVehicleTypePhysXCar,
            AirSimSettings::kVehicleTypeBoxCar,
            AirSimSettings::kVehicleTypeArduRover,
            AirSimSettings::kVehicleTypeSimpleFlight,
            AirSimSettings::kVehicleTypeArduCopter,
            AirSimSettings::kVehicleTypePX4,
            AirSimSettings::kVehicleTypeCPHusky,
            AirSimSettings::kVehicleTypePioneerP3DX,
            AirSimSettings::kVehicleTypeComputerVision,
            AirSimSettings::kVehicleTypeCharacter,
            AirSimSettings::kVehicleTypeMotorbike,
        };
        return supported.count(vehicle_type) > 0;
    }

    std::string getVehiclePawnPathName(
        const AirSimSettings::VehicleSetting& vehicle_setting) const override {
        // Dispatch to correct pawn blueprint based on type
        if (vehicle_setting.vehicle_type == kVehicleTypeCharacter)
            return "Class'/Script/AirSim.CharacterPawn'";
        if (vehicle_setting.vehicle_type == kVehicleTypeMotorbike)
            return "Class'/Script/AirSim.MotorbikePawn'";
        if (vehicle_setting.vehicle_type == kVehicleTypePhysXCar)
            return "Class'/Script/AirSim.CarPawn'";
        // ... etc
    }
};
```

### 2. Settings for Mixed Mode

```json
{
    "SimMode": "Mixed",
    "Vehicles": {
        "Car1":    {"VehicleType": "physxcar", "X": 0, "Y": 0, "Z": 0},
        "Drone1":  {"VehicleType": "simpleflight", "X": 0, "Y": 5, "Z": -10},
        "Person1": {"VehicleType": "character", "X": 10, "Y": 0, "Z": 0},
        "Person2": {"VehicleType": "character", "X": 15, "Y": 3, "Z": 0},
        "Bike1":   {"VehicleType": "motorbike", "X": -5, "Y": 0, "Z": 0},
        "Husky1":  {"VehicleType": "cphusky", "X": -10, "Y": 0, "Z": 0}
    }
}
```

### 3. Runtime Spawning in Mixed Mode

```python
# Spawn additional actors during simulation
client.simAddVehicle("Person3", "character", Pose(...), "")
client.simAddVehicle("Car2", "physxcar", Pose(...), "")
client.simAddVehicle("Bike2", "motorbike", Pose(...), "")
```

### 4. API Routing

Each vehicle type uses its own API client class, but the server routes based on vehicle name:

```python
# All through same client connection
car_state = client.getCarState(vehicle_name="Car1")
char_state = client.getCharacterState(vehicle_name="Person1")
drone_state = client.getMultirotorState(vehicle_name="Drone1")
```

## Acceptance Criteria

- [ ] `"Mixed"` SimMode supports all vehicle types simultaneously
- [ ] Cars, drones, characters, and robots can coexist in one session
- [ ] Each vehicle responds to its type-specific API (car controls, character controls, etc.)
- [ ] Runtime spawning of any vehicle type via `simAddVehicle()`
- [ ] Cross-type interactions work (character walks near car, drone observes pedestrians)
- [ ] Collision between different types detected and reported
- [ ] All sensor types work on all vehicle types

## Files Affected

- New: `Unreal/Plugins/AirSim/Source/SimMode/SimModeMixed.h/.cpp`
- `AirLib/include/common/AirSimSettings.hpp` — add `kSimModeMixed`
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp` — factory for SimModeMixed
- `AirLib/src/api/RpcLibServerBase.cpp` — route API calls by vehicle type
