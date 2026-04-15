# SPEC-022: Decoupled Fixed Physics Timestep

**Priority:** P1
**Category:** Performance
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

Physics updates are tied to Unreal Engine's frame rate:

- `FastPhysicsEngine.hpp:86`: Uses `clock()->updateSince(body.last_kinematics_time)` — variable timestep
- Comment at line 116-117: "TODO: this is now being done in PawnSimApi::update" — duplicated position update logic
- No option for fixed physics timestep decoupled from rendering
- No CFL condition checking for integration stability
- Variable timestep causes inconsistent simulation results across hardware
- Verlet integration (lines 399-401) assumes reasonably constant dt

## Proposed Solution

### 1. Fixed Timestep Physics Loop

```cpp
class FixedTimestepScheduler {
    float physics_dt_;           // Fixed physics timestep (e.g., 1/240s)
    float accumulator_ = 0.0f;  // Time accumulator

    void update(float frame_dt) {
        accumulator_ += frame_dt;

        // Run physics at fixed rate
        while (accumulator_ >= physics_dt_) {
            physics_engine_->update(physics_dt_);
            accumulator_ -= physics_dt_;
        }

        // Interpolate for rendering
        float alpha = accumulator_ / physics_dt_;
        interpolateState(alpha);
    }

    void interpolateState(float alpha) {
        // Smooth rendering between physics states
        for (auto& body : bodies_) {
            body.render_position = body.prev_position * (1 - alpha)
                                 + body.curr_position * alpha;
            body.render_orientation = body.prev_orientation.slerp(alpha,
                                     body.curr_orientation);
        }
    }
};
```

### 2. Configuration

```json
{
    "PhysicsSettings": {
        "FixedTimestep": true,
        "PhysicsHz": 240,
        "MaxSubsteps": 8,
        "InterpolateRendering": true,
        "StabilityCheck": true
    }
}
```

### 3. Stability Monitoring

```cpp
class StabilityMonitor {
    void checkCFL(float dt, float max_velocity, float min_cell_size) {
        float cfl = max_velocity * dt / min_cell_size;
        if (cfl > 1.0f) {
            UE_LOG(LogAirSim, Warning,
                   TEXT("CFL condition violated: %f. Consider reducing timestep."), cfl);
        }
    }

    void checkEnergyConservation(float kinetic_energy, float potential_energy,
                                  float prev_total_energy) {
        float total = kinetic_energy + potential_energy;
        float drift = std::abs(total - prev_total_energy) / prev_total_energy;
        if (drift > 0.01f) {  // 1% drift threshold
            UE_LOG(LogAirSim, Warning,
                   TEXT("Energy conservation drift: %.2f%%"), drift * 100);
        }
    }
};
```

## Acceptance Criteria

- [ ] Fixed physics timestep decoupled from render frame rate
- [ ] Configurable physics Hz (60-1000) with default 240 Hz
- [ ] State interpolation for smooth rendering
- [ ] Maximum substeps cap to prevent spiral of death
- [ ] CFL condition warning
- [ ] Deterministic simulation: same inputs → same outputs regardless of render FPS
- [ ] Backward compatible: variable timestep remains as option

## Files Affected

- `AirLib/include/physics/FastPhysicsEngine.hpp` — fixed timestep support
- `AirLib/include/physics/World.hpp` — scheduler integration
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeWorldBase.h` — physics loop
- New: `AirLib/include/physics/FixedTimestepScheduler.hpp`
- New: `AirLib/include/physics/StabilityMonitor.hpp`
- `AirLib/include/common/AirSimSettings.hpp` — physics settings
