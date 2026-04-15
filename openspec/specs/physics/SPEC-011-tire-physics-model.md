# SPEC-011: Pacejka Tire Physics Model

**Priority:** P1
**Category:** Physics Simulation
**Effort:** Medium (3-4 weeks)
**Dependencies:** None

## Problem Statement

Car physics is entirely delegated to Unreal's `ChaosWheeledVehicleMovementComponent` (`CarPawn.cpp:8`):

- No tire slip angle or slip ratio calculations in AirLib
- No Pacejka Magic Formula tire model
- Only "slippery" vs "non-slippery" physics materials (`CarPawn.cpp:24-35`)
- No longitudinal/lateral load transfer
- No load sensitivity effects on friction
- No tire deformation or thermal model
- Inadequate for validating ADAS algorithms or autonomous driving controllers

## Proposed Solution

### 1. Pacejka Magic Formula (MF-Tire 6.2)

```cpp
class PacejkaTireModel {
    // Magic Formula: Y = D * sin(C * arctan(B*x - E*(B*x - arctan(B*x))))
    struct MFCoefficients {
        float B;  // Stiffness factor
        float C;  // Shape factor
        float D;  // Peak value
        float E;  // Curvature factor
        float Sv; // Vertical shift
        float Sh; // Horizontal shift
    };

    struct TireForces {
        float Fx;  // Longitudinal force
        float Fy;  // Lateral force
        float Mz;  // Self-aligning torque
    };

    TireForces compute(float slip_angle, float slip_ratio,
                       float normal_load, float camber_angle) {
        // Lateral force (Fy) from slip angle
        float alpha = slip_angle - lateral_coeffs_.Sh;
        float Fy = lateral_coeffs_.D * std::sin(
            lateral_coeffs_.C * std::atan(
                lateral_coeffs_.B * alpha -
                lateral_coeffs_.E * (lateral_coeffs_.B * alpha -
                                     std::atan(lateral_coeffs_.B * alpha))
            )
        ) + lateral_coeffs_.Sv;

        // Longitudinal force (Fx) from slip ratio
        // ... similar formula with longitudinal coefficients

        // Combined slip (friction ellipse)
        // ...

        return {Fx, Fy, Mz};
    }

    MFCoefficients lateral_coeffs_;
    MFCoefficients longitudinal_coeffs_;
};
```

### 2. Load Transfer Model

```cpp
class LoadTransferModel {
    // Longitudinal load transfer: ΔFz = m * ax * h_cg / wheelbase
    // Lateral load transfer: ΔFz = m * ay * h_cg / track_width

    std::array<float, 4> computeWheelLoads(
        float total_mass, float acceleration_x, float acceleration_y,
        float cg_height, float wheelbase, float track_width,
        float front_weight_distribution);
};
```

### 3. Tire State Tracking

```cpp
struct TireState {
    float slip_angle;         // rad
    float slip_ratio;         // dimensionless
    float normal_load;        // N
    float temperature;        // K (optional thermal model)
    float tread_depth;        // mm (optional wear model)
    float inflation_pressure; // Pa
    PacejkaTireModel::TireForces forces;
};
```

### 4. Pre-Configured Tire Sets

```json
{
    "tire_presets": {
        "passenger_car_dry": {"B_lat": 10.0, "C_lat": 1.9, "D_lat": 1.0, ...},
        "passenger_car_wet": {"B_lat": 8.0, "C_lat": 1.7, "D_lat": 0.7, ...},
        "racing_slick": {"B_lat": 12.0, "C_lat": 2.1, "D_lat": 1.3, ...},
        "off_road": {"B_lat": 6.0, "C_lat": 1.5, "D_lat": 0.6, ...},
        "winter_tire": {"B_lat": 7.0, "C_lat": 1.6, "D_lat": 0.5, ...}
    }
}
```

## Acceptance Criteria

- [ ] Pacejka MF lateral force matching published tire data within 10%
- [ ] Combined slip (friction ellipse) model
- [ ] Load transfer under acceleration/braking/cornering
- [ ] At least 5 tire presets for common scenarios
- [ ] Tire state accessible via API (slip angle, slip ratio, forces)
- [ ] Configurable per-wheel in settings.json
- [ ] Validation against CarSim or similar reference

## Risks

- Unreal's ChaosVehicle may conflict with custom tire model — may need to bypass UE physics
- Parameter identification for specific tires requires manufacturer data
- Computational cost of 4 tires at high frequency — profile carefully

## Files Affected

- New: `AirLib/include/vehicles/car/PacejkaTireModel.hpp`
- New: `AirLib/include/vehicles/car/LoadTransferModel.hpp`
- `AirLib/include/vehicles/car/api/CarApiBase.hpp` — expose tire state
- `Unreal/Plugins/AirSim/Source/Vehicles/Car/CarPawn.cpp` — integrate tire model
- `AirLib/include/common/AirSimSettings.hpp` — tire configuration
