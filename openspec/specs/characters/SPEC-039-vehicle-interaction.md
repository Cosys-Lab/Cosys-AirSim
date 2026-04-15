# SPEC-039: Character-Vehicle Interaction (Enter/Exit)

**Priority:** P0
**Category:** Characters
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-035, SPEC-036, SPEC-038

## Problem Statement

Characters need to enter and exit vehicles (cars, motorbikes) with proper animation. This enables:

- Ride-sharing / taxi simulation
- Driver behavior training (driver as character inside vehicle)
- Passenger simulation (multiple occupants)
- First-person driving (character camera inside car)
- Autonomous vehicle handoff scenarios (human takes over from autopilot)

## Proposed Solution

### 1. Vehicle Entry/Exit State Machine

```
[Walking] ──approach──► [Near Vehicle] ──enterVehicle()──►
    ┌─────────────────────────────────────────────┐
    │  [Opening Door] → [Sitting Down] → [Seated] │
    │                                     │        │
    │              exitVehicle() ◄────────┘        │
    │  [Opening Door] → [Standing Up] → [Exiting] │
    └─────────────────────────────────────────────┘
                                    ──► [Walking]
```

### 2. Vehicle Seat System

```cpp
struct VehicleSeat {
    std::string seat_id;          // "driver", "passenger_front", "passenger_rear_left", ...
    FVector local_offset;         // Position offset relative to vehicle
    FRotator local_rotation;      // Rotation in vehicle space
    FName entry_animation;        // "enter_car_left" / "enter_car_right"
    FName exit_animation;         // "exit_car_left" / "exit_car_right"
    FName seated_pose;            // Idle seated animation
    bool is_occupied = false;
    std::string occupant_name;    // Character vehicle_name
};

struct VehicleSeatingConfig {
    std::vector<VehicleSeat> seats;

    // Presets
    static VehicleSeatingConfig Sedan();    // 4 seats
    static VehicleSeatingConfig SUV();      // 5-7 seats
    static VehicleSeatingConfig Motorbike(); // 1-2 seats
};
```

### 3. Entry/Exit API

```python
# Character approaches and enters vehicle
success = client.enterVehicle(
    vehicle_name="Car1",
    seat="driver",              # or "passenger_front", "passenger_rear_left"
    character_name="Person1"
)

# Character exits vehicle
success = client.exitVehicle(character_name="Person1")

# Query vehicle occupancy
seats = client.getVehicleSeats(vehicle_name="Car1")
# Returns: [
#   Seat(id="driver", occupied=True, occupant="Person1"),
#   Seat(id="passenger_front", occupied=False),
#   Seat(id="passenger_rear_left", occupied=False),
#   Seat(id="passenger_rear_right", occupied=False),
# ]

# First-person view from inside vehicle
# (character's fp_camera is now inside the car)
images = client.simGetImages([ImageRequest(...)], vehicle_name="Person1")

# Character can take control of vehicle they're driving
client.setDriverMode(True, character_name="Person1")
# Now Person1's controls drive Car1
```

### 4. Implementation Details

When `enterVehicle()` is called:
1. Validate character is within `entry_radius` of vehicle (default 3m)
2. Find nearest available seat
3. Play entry animation (montage)
4. On animation complete:
   - Attach character mesh to vehicle actor at seat offset
   - Disable character capsule collision
   - Disable character movement component
   - Switch character camera to vehicle interior position
   - Set `is_in_vehicle = true`, `current_vehicle = "Car1"`
5. Character now moves with vehicle

When `exitVehicle()` is called:
1. Play exit animation (montage)
2. On animation complete:
   - Detach character mesh from vehicle
   - Re-enable capsule collision at vehicle exit point
   - Re-enable character movement component
   - Set `is_in_vehicle = false`

### 5. Motorbike Specifics

```python
# Mount motorbike (swing leg over)
client.enterVehicle("Bike1", seat="rider", character_name="Person1")
# Character plays mount_motorbike animation
# Character mesh attached to motorbike, leaning pose

# Passenger seat (pillion)
client.enterVehicle("Bike1", seat="pillion", character_name="Person2")
# Second character sits behind rider

# Dismount
client.exitVehicle(character_name="Person1")
# Plays dismount animation, character stands beside bike
```

## Acceptance Criteria

- [ ] Character can enter car from either side with animation
- [ ] Character appears seated inside vehicle with correct pose
- [ ] Character's first-person camera shows interior view
- [ ] Character exits vehicle with animation, stands at exit point
- [ ] Multiple characters can occupy same vehicle (driver + passengers)
- [ ] Seated character moves with vehicle
- [ ] Character can mount/dismount motorbike
- [ ] Motorbike supports rider + pillion passenger
- [ ] Vehicle occupancy queryable via API
- [ ] `setDriverMode()` lets character control vehicle

## Files Affected

- `AirLib/include/vehicles/character/api/CharacterApiBase.hpp` — enter/exit API
- New: `AirLib/include/vehicles/VehicleSeating.hpp` — seat configuration
- `Unreal/Plugins/AirSim/Source/Vehicles/Character/CharacterPawn.cpp` — attach/detach logic
- `Unreal/Plugins/AirSim/Source/Vehicles/Car/CarPawn.h` — add seat definitions
- `PythonClient/cosysairsim/client.py` — enterVehicle/exitVehicle
- `PythonClient/cosysairsim/types.py` — Seat type
