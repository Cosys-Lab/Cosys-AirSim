# SPEC-009: Advanced Multirotor Aerodynamics

**Priority:** P0
**Category:** Physics Simulation
**Effort:** Large (4-6 weeks)
**Dependencies:** None

## Problem Statement

The multirotor physics model has critical fidelity gaps:

1. **Linear thrust mapping** (`RotorActuator.hpp:125`): `thrust = control_signal_filtered * max_thrust` — real drones have nonlinear thrust-vs-RPM curves following blade element theory
2. **No motor dynamics** (`RotorActuator.hpp:119-128`): Control signal instantly maps to thrust with no motor lag, inertia, or electrical response time. Real motors have 50-200ms response times.
3. **No gyroscopic precession**: Spinning rotors create gyroscopic moments during attitude changes — completely absent
4. **No propeller disk theory**: Thrust doesn't change with forward speed (no thrust correction for advancing blade effects)
5. **Constant drag coefficients** (`MultiRotorParams.hpp:47`): `linear_drag_coefficient = 1.3f / 4.0f` — no Reynolds number dependency
6. **Angular drag equals linear** (`MultiRotorParams.hpp:52`): `angular_drag_coefficient = linear_drag_coefficient` — physically wrong, angular drag is typically 10-20x smaller
7. **No ground effect**: Thrust augmentation near ground surfaces not modeled
8. **No blade flapping dynamics**: No asymmetric lift effects during forward flight

## Proposed Solution

### 1. Motor Response Model

Replace linear thrust mapping with first-order motor dynamics:

```cpp
class MotorDynamics {
    float time_constant_;     // Motor response time (50-200ms)
    float current_rpm_;       // Current motor RPM
    float max_rpm_;
    float inertia_;           // Rotor moment of inertia

    float update(float commanded_rpm, float dt) {
        // First-order lag: τ * dω/dt + ω = ω_cmd
        float rpm_error = commanded_rpm - current_rpm_;
        current_rpm_ += (rpm_error / time_constant_) * dt;
        return current_rpm_;
    }
};
```

### 2. Nonlinear Thrust Model (Blade Element Theory)

```cpp
struct ThrustModel {
    // Thrust: T = C_T * ρ * n² * D⁴
    // Torque: Q = C_Q * ρ * n² * D⁵
    float c_t;       // Thrust coefficient (function of advance ratio)
    float c_q;       // Torque coefficient
    float diameter;  // Propeller diameter

    float computeThrust(float rpm, float air_density, float airspeed) {
        float n = rpm / 60.0f;
        float advance_ratio = airspeed / (n * diameter);
        float ct = c_t * (1.0f - k_advance * advance_ratio);  // BET correction
        return ct * air_density * n * n * std::pow(diameter, 4);
    }
};
```

### 3. Gyroscopic Effects

```cpp
Vector3r computeGyroscopicMoment(const RotorActuator& rotor, const Vector3r& angular_velocity) {
    // M_gyro = I_rotor * ω_rotor × ω_body
    float rotor_inertia = rotor.getInertia();
    float rotor_angular_vel = rotor.getCurrentRPM() * 2 * M_PI / 60.0f;
    Vector3r rotor_axis = rotor.getAxis();
    return rotor_inertia * rotor_angular_vel * angular_velocity.cross(rotor_axis);
}
```

### 4. Ground Effect

```cpp
float computeGroundEffectFactor(float height_above_ground, float rotor_diameter) {
    // Cheeseman-Bennett model: T_ge/T_oge = 1 / (1 - (R/(4h))²)
    float ratio = rotor_diameter / (8.0f * height_above_ground);
    return 1.0f / (1.0f - ratio * ratio);
}
```

### 5. Reynolds-Dependent Drag

```cpp
float computeDragCoefficient(float velocity, float characteristic_length, float air_density, float viscosity) {
    float Re = air_density * velocity * characteristic_length / viscosity;
    if (Re < 1e5) return 1.2f;       // Laminar
    else if (Re < 5e5) return 0.5f;  // Transition
    else return 0.2f;                 // Turbulent
}
```

## Acceptance Criteria

- [ ] Motor dynamics with configurable time constant (default 100ms)
- [ ] Nonlinear thrust curve matching published propeller data (APC Propeller DB) within 5%
- [ ] Gyroscopic precession during rapid yaw with observable pitch/roll coupling
- [ ] Ground effect increases thrust within 1 rotor diameter of ground
- [ ] Drag coefficients vary with Reynolds number
- [ ] Angular drag decoupled from linear drag with separate coefficients
- [ ] All new parameters configurable in settings.json with sensible defaults
- [ ] Unit tests comparing against MATLAB/Simulink reference model
- [ ] Backward compatibility: existing settings produce similar (not identical) behavior

## Risks

- Increased computational cost — profile and ensure <1ms per vehicle per tick
- Parameter identification complexity — provide defaults for common frames (DJI Phantom, F450)
- Stability of integration with smaller timesteps may be needed

## Files Affected

- `AirLib/include/vehicles/multirotor/RotorActuator.hpp` — major rewrite
- `AirLib/include/vehicles/multirotor/MultiRotorPhysicsBody.hpp` — add gyroscopic, ground effect
- `AirLib/include/vehicles/multirotor/MultiRotorParams.hpp` — new parameters
- `AirLib/include/physics/FastPhysicsEngine.hpp` — update drag computation (lines 269-308)
- `AirLib/include/common/AirSimSettings.hpp` — new motor/aero settings
