# SPEC-013: Sensor-Material Interaction System

**Priority:** P1
**Category:** Physics Simulation
**Effort:** Medium (3-4 weeks)
**Dependencies:** None

## Problem Statement

Sensor-material interactions are oversimplified:

- **LiDAR reflectivity**: Only Lambertian model (`GPULidarSimpleParams.hpp:37`). No angle-dependent reflectivity (Fresnel equations), no specular reflections, no multi-path effects.
- **Material database**: Loaded from undocumented `materials.csv` (line 91) with no physics basis description.
- **Echo sensor**: Fixed scattering cone (`reflection_opening_angle = 10` degrees). No surface roughness or wavelength dependency.
- **No acoustic impedance** modeling for echo sensor (sonar/radar)
- **No radar cross-section** computation for echo targets
- **Rain/fog models** not validated against real atmospheric data

## Proposed Solution

### 1. Physically-Based Reflectance Model

```cpp
class SurfaceReflectanceModel {
    // Bidirectional Reflectance Distribution Function (BRDF)
    struct MaterialBRDF {
        float diffuse_albedo;     // Lambertian component (0-1)
        float specular_albedo;    // Specular component (0-1)
        float roughness;          // Surface roughness (0-1)
        float refractive_index;   // For Fresnel calculation

        float computeReflectance(float incidence_angle, float wavelength) {
            float fresnel = computeFresnel(incidence_angle, refractive_index);
            float diffuse = diffuse_albedo * std::cos(incidence_angle) / M_PI;
            float specular = specular_albedo * fresnel *
                             computeCookTorrance(incidence_angle, roughness);
            return diffuse + specular;
        }
    };
};
```

### 2. Extended Material Database

```csv
# materials.csv v2
material_name,diffuse_906nm,specular_906nm,roughness,refractive_index,acoustic_impedance,radar_reflectivity
asphalt,0.18,0.02,0.8,1.5,3.2e6,0.15
concrete,0.25,0.05,0.6,1.5,7.4e6,0.20
vegetation,0.50,0.01,0.9,1.3,0.4e6,0.05
metal_painted,0.10,0.60,0.2,2.5,42e6,0.90
glass,0.05,0.80,0.05,1.5,13e6,0.30
water,0.01,0.70,0.1,1.33,1.5e6,0.80
snow,0.80,0.10,0.7,1.3,0.3e6,0.10
```

### 3. Atmospheric Attenuation Model

```cpp
class AtmosphericModel {
    // Beer-Lambert law: I = I_0 * exp(-α * d)
    struct AtmosphericParams {
        float visibility_km;      // Meteorological visibility
        float rain_rate_mmh;      // Rain intensity
        float humidity_percent;
        float temperature_c;
    };

    // Wavelength-dependent extinction coefficient
    float getExtinction(float wavelength_nm, const AtmosphericParams& atmo) {
        float rayleigh = computeRayleighScattering(wavelength_nm);
        float mie = computeMieScattering(visibility_km);
        float rain = computeRainAttenuation(rain_rate_mmh, wavelength_nm);
        return rayleigh + mie + rain;
    }
};
```

### 4. Echo Sensor Physics Upgrade

```cpp
class EchoPhysicsModel {
    // Acoustic/radar equation: P_r = P_t * G² * λ² * σ / ((4π)³ * R⁴)
    float computeReceivedPower(float transmitted_power, float antenna_gain,
                                float wavelength, float rcs, float range) {
        return transmitted_power * antenna_gain * antenna_gain
               * wavelength * wavelength * rcs
               / (std::pow(4 * M_PI, 3) * std::pow(range, 4));
    }

    // Angle-dependent reflection based on surface roughness
    float computeReflectionCoefficient(float incidence_angle, float roughness,
                                       float wavelength) {
        // Rayleigh criterion: smooth if h < λ/(8*cos(θ))
        float rayleigh_criterion = wavelength / (8.0f * std::cos(incidence_angle));
        if (roughness < rayleigh_criterion) {
            return computeFresnelReflection(incidence_angle);
        } else {
            return computeDiffuseScattering(incidence_angle, roughness, wavelength);
        }
    }
};
```

## Acceptance Criteria

- [ ] Fresnel-based angle-dependent reflectance for LiDAR
- [ ] Material database with at least 15 common materials
- [ ] Atmospheric attenuation validated against literature (fog, rain)
- [ ] Echo sensor uses radar/sonar equation for received power
- [ ] Surface roughness affects reflection pattern for both LiDAR and echo
- [ ] Material properties accessible via API for ML training
- [ ] Performance overhead < 10% for LiDAR/echo sensors

## Files Affected

- `AirLib/include/sensors/lidar/GPULidarSimpleParams.hpp` — new reflectance model
- `AirLib/include/sensors/echo/EchoSimpleParams.hpp` — physics-based reflection
- New: `AirLib/include/sensors/SurfaceReflectance.hpp`
- New: `AirLib/include/sensors/AtmosphericModel.hpp`
- New: `AirLib/data/materials_v2.csv`
- `Unreal/Plugins/AirSim/Source/` — pass material properties from UE surfaces
