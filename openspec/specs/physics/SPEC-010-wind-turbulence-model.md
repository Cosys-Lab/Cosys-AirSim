# SPEC-010: Wind & Turbulence Model

**Priority:** P0
**Category:** Physics Simulation
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-009

## Problem Statement

Wind is modeled as a single static vector (`FastPhysicsEngine.hpp:473`):

- `Vector3r wind_` is constant — no gusts, no turbulence, no spatial variation
- Applied only in drag calculation (`line 287: const Vector3r relative_vel = linear_vel - wind_world`)
- No wind shear with altitude
- No Dryden or von Kármán turbulence models (standard in flight simulation, required by MIL-STD-1797)
- Completely inadequate for testing robust flight controllers
- No API to set wind profiles programmatically

## Proposed Solution

### 1. Dryden Turbulence Model (MIL-HDBK-1797)

```cpp
class DrydenTurbulenceModel {
    // Transfer functions for u, v, w components
    // Low altitude (< 1000ft): L_u = h / (0.177 + 0.000823h)^1.2
    // σ_w = 0.1 * W_20 (20ft wind speed)

    struct TurbulenceState {
        float u, v, w;           // Turbulence velocity components
        float p, q, r;           // Turbulence angular rates
    };

    TurbulenceState update(float altitude_agl, float airspeed, float dt);

private:
    // Spectral shaping filters (continuous → discrete via Tustin transform)
    FirstOrderFilter u_filter_;
    SecondOrderFilter v_filter_;
    SecondOrderFilter w_filter_;
    WhiteNoiseGenerator noise_;
};
```

### 2. Wind Profile Models

```cpp
class WindField {
    // Steady-state wind
    Vector3r base_wind_;

    // Wind shear (logarithmic profile)
    float computeWindAtAltitude(float altitude_agl) {
        // v(h) = v_ref * ln(h/z0) / ln(h_ref/z0)
        return base_speed_ * std::log(altitude_agl / roughness_length_)
                           / std::log(reference_height_ / roughness_length_);
    }

    // Discrete gusts (1-cosine model, MIL-STD-1797)
    Vector3r computeGust(float time) {
        if (time < gust_start_ || time > gust_start_ + gust_duration_) return Vector3r::Zero();
        float t_norm = (time - gust_start_) / gust_duration_;
        float factor = 0.5f * (1.0f - std::cos(2 * M_PI * t_norm));
        return gust_direction_ * gust_amplitude_ * factor;
    }

    // Total wind at a point
    Vector3r getWindAt(const Vector3r& position, float time) {
        return computeWindAtAltitude(position.z())
             + computeGust(time)
             + turbulence_.update(position.z(), airspeed_, dt);
    }
};
```

### 3. Wind Configuration API

```python
# Python API
client.simSetWindField(
    base_wind=[5.0, 2.0, 0.0],          # m/s, steady component
    turbulence_intensity="moderate",      # light/moderate/severe
    wind_shear=True,                     # logarithmic profile
    roughness_length=0.03,               # terrain roughness (m)
    gusts=[
        {"time": 10.0, "duration": 3.0, "amplitude": 8.0, "direction": [1,0,0]},
        {"time": 25.0, "duration": 2.0, "amplitude": 12.0, "direction": [0,1,0]},
    ],
    seed=42                              # reproducible turbulence
)
```

### 4. Wind Visualization (Optional)

- Debug visualization of wind vectors in viewport
- Wind speed/direction HUD display
- Wind measurement logging for post-analysis

## Acceptance Criteria

- [ ] Dryden turbulence model matching MIL-HDBK-1797 spectral characteristics
- [ ] Logarithmic wind shear profile with configurable roughness length
- [ ] 1-cosine discrete gust model with API-controlled timing
- [ ] Turbulence intensity levels (light/moderate/severe) per MIL-STD
- [ ] Wind field varies with position (at minimum with altitude)
- [ ] API to set wind field parameters at runtime
- [ ] Seed-based reproducibility
- [ ] Unit tests comparing power spectral density against reference
- [ ] Performance: < 0.1ms overhead per vehicle per tick

## Files Affected

- `AirLib/include/physics/FastPhysicsEngine.hpp` — replace `wind_` field (line 473) with WindField
- New: `AirLib/include/physics/WindField.hpp`
- New: `AirLib/include/physics/DrydenTurbulence.hpp`
- `AirLib/include/api/WorldSimApiBase.hpp` — add wind field API
- `AirLib/include/common/AirSimSettings.hpp` — wind configuration section
- `PythonClient/cosysairsim/client.py` — `simSetWindField()` method
