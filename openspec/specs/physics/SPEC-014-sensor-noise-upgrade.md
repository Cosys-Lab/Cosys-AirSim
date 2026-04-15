# SPEC-014: Comprehensive Sensor Noise Model Upgrade

**Priority:** P1
**Category:** Physics Simulation
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

Sensor noise models are incomplete:

### IMU (`ImuSimple.hpp:77-101`)
- Has ARW/VRW and bias stability (good baseline from MPU-6000 specs)
- **Missing**: Coning/sculling errors, quantization effects, scale factor errors, non-orthogonal axes, temperature effects, gravity alignment errors

### Barometer (`BarometerSimple.hpp:79-94`)
- Has uncorrelated noise + Gaussian-Markov drift
- **Missing**: Temperature correction in altitude formula, weather pressure variations, altitude formula uses constant coefficients

### Magnetometer (`MagnetometerSimple.hpp:72-97`)
- Has geographic-dependent field (IGRF dipole) + Gaussian noise + bias
- **Missing**: Soft-iron/hard-iron distortion, motor-induced magnetic fields, axis misalignment

### GPS (`GpsSimple.hpp:32-33`)
- First-order filter for accuracy convergence
- **Missing**: Satellite geometry (GDOP), multipath effects, ionospheric delay, cycle slips, signal loss

### Distance Sensor (`DistanceSimple.hpp:77-95`)
- Ray cast + Gaussian noise
- **Missing**: Beam width modeling, temperature effects, min/max range edge effects, surface angle dependency

### LiDAR (`LidarSimpleParams.hpp:26-28`)
- Noise disabled by default, linear distance scaling when enabled
- **Missing**: Nonlinear noise (range, reflectivity, incidence angle dependent), atmospheric effects, per-ray timing

## Proposed Solution

### 1. IMU Noise Model Extension

```cpp
struct ImuNoiseModelV2 : public ImuNoiseModel {
    // Existing: ARW, VRW, bias stability (keep)

    // New: Scale factor errors
    Vector3r gyro_scale_factor_error;   // ppm
    Vector3r accel_scale_factor_error;  // ppm

    // New: Cross-axis sensitivity
    Matrix3x3r gyro_misalignment;       // non-orthogonality matrix
    Matrix3x3r accel_misalignment;

    // New: Quantization
    float gyro_resolution_dps;          // deg/s per LSB
    float accel_resolution_g;           // g per LSB

    // New: Temperature sensitivity
    float gyro_temp_sensitivity;        // deg/s/K
    float accel_temp_sensitivity;       // mg/K
    float operating_temperature;        // K

    Vector3r applyNoise(const Vector3r& true_value, SensorType type, float dt);
};
```

### 2. GPS Noise Model Extension

```cpp
struct GpsNoiseModelV2 {
    // Accuracy based on satellite configuration
    float hdop;                     // Horizontal dilution of precision
    float vdop;                     // Vertical dilution of precision

    // Multipath model
    bool enable_multipath;
    float multipath_amplitude;      // m, additional error near buildings
    float multipath_time_constant;  // s, correlation time

    // Signal loss
    float signal_loss_probability;  // per second
    float reacquisition_time;       // s, time to regain fix

    // Ionospheric delay
    float iono_delay_bias;          // m
    float iono_delay_noise;         // m, standard deviation

    GeoPoint applyNoise(const GeoPoint& true_position, float dt);
};
```

### 3. LiDAR Noise Model Extension

```cpp
struct LidarNoiseModelV2 {
    // Range noise: σ_r = a + b*R + c*R² (quadratic with distance)
    float noise_constant;           // m, constant component
    float noise_linear;             // m/m, linear with range
    float noise_quadratic;          // m/m², quadratic with range

    // Angle-dependent noise
    float incidence_angle_factor;   // noise multiplier at grazing angles

    // Reflectivity-dependent noise
    float low_reflectivity_noise_scale; // extra noise for dark surfaces

    // Dropout model
    float dropout_probability_base;     // base probability of no return
    float dropout_range_factor;         // increases with range

    // Per-ray timing jitter
    float timing_jitter_ns;             // measurement timing uncertainty

    float applyNoise(float true_range, float incidence_angle,
                     float reflectivity, std::mt19937& rng);
};
```

## Acceptance Criteria

- [ ] IMU: Scale factor, cross-axis, quantization, temperature effects implemented
- [ ] GPS: GDOP-based accuracy, multipath, signal loss modeled
- [ ] LiDAR: Nonlinear range noise, incidence angle dependency, dropout model
- [ ] Magnetometer: Hard-iron + soft-iron distortion matrices
- [ ] Distance sensor: Beam width, surface angle, temperature effects
- [ ] Barometer: Temperature correction, weather-dependent pressure
- [ ] All new noise parameters configurable in settings.json
- [ ] Default parameters matching real sensor datasheets documented
- [ ] Noise models validated against real sensor data (at least qualitatively)

## Files Affected

- `AirLib/include/sensors/imu/ImuSimple.hpp` — extend noise model
- `AirLib/include/sensors/gps/GpsSimple.hpp` — new noise features
- `AirLib/include/sensors/lidar/LidarSimple.hpp` — nonlinear noise
- `AirLib/include/sensors/magnetometer/MagnetometerSimple.hpp` — iron distortion
- `AirLib/include/sensors/distance/DistanceSimple.hpp` — beam width
- `AirLib/include/sensors/barometer/BarometerSimple.hpp` — temperature correction
- `AirLib/include/common/AirSimSettings.hpp` — new noise parameters
