# SPEC-012: Collision System Overhaul

**Priority:** P0
**Category:** Physics Simulation
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

The collision response system in `FastPhysicsEngine.hpp` has critical deficiencies:

1. **Rolling friction hack** (line 220): `next.twist.angular *= 0.9f` — arbitrary 10% angular velocity reduction per timestep. Not a physical model, timestep-dependent, produces different results at different frame rates.

2. **Single-point contact only** (lines 196-217): Impulse-based response assumes single contact point. Real collisions involve contact patches, multiple contact points, and articulated responses.

3. **Hardcoded ground friction** (lines 174-177): When landing, `restitution = 0` and `friction = 1` are hardcoded regardless of surface material. Comment acknowledges: "TODO: it would be better if we did this based on the material."

4. **Ground lock hack** (line 238): `next.twist.linear = Vector3r::Zero()` — completely stops all velocity when grounded. No sliding on slopes.

5. **Constant restitution** (line 161): `body.getRestitution()` returns single value regardless of impact speed or material pair.

6. **No static vs kinetic friction**: Single Coulomb friction coefficient with no distinction.

## Proposed Solution

### 1. Material-Based Contact Properties

```cpp
struct ContactMaterial {
    float static_friction;
    float kinetic_friction;
    float restitution;          // Speed-dependent
    float rolling_resistance;   // Proper rolling resistance coefficient

    float getRestitution(float impact_speed) const {
        // Restitution decreases with impact speed (energy dissipation)
        return restitution * std::exp(-speed_decay * impact_speed);
    }
};

class MaterialDatabase {
    static ContactMaterial getMaterialPair(MaterialType a, MaterialType b);
    // Predefined: concrete, asphalt, grass, gravel, metal, plastic, rubber
};
```

### 2. Proper Rolling Resistance

Replace `next.twist.angular *= 0.9f` with:

```cpp
// Rolling resistance torque: M_rr = C_rr * N * R
float rolling_torque = rolling_resistance_coeff * normal_force * wheel_radius;
Vector3r rolling_deceleration = -angular_velocity.normalized() * rolling_torque / inertia;
next.twist.angular += rolling_deceleration * dt;
```

### 3. Multi-Point Contact

```cpp
struct ContactManifold {
    std::vector<ContactPoint> contacts;  // Up to 4 contact points
    Vector3r average_normal;
    float total_penetration;
    MaterialType surface_material;
};

// Replace single-impulse with iterative solver
void resolveContacts(PhysicsBody& body, const ContactManifold& manifold, float dt) {
    for (int iter = 0; iter < solver_iterations; ++iter) {
        for (const auto& contact : manifold.contacts) {
            applyContactImpulse(body, contact, dt);
        }
    }
}
```

### 4. Slope Sliding

Replace ground lock with proper slope physics:

```cpp
void handleGroundContact(PhysicsBody& body, const Vector3r& ground_normal) {
    float slope_angle = std::acos(ground_normal.dot(Vector3r::UnitZ()));
    float friction_angle = std::atan(static_friction);

    if (slope_angle > friction_angle) {
        // Slide downhill
        Vector3r slide_dir = computeSlideDirection(ground_normal);
        float slide_accel = gravity * (std::sin(slope_angle) -
                           kinetic_friction * std::cos(slope_angle));
        next.twist.linear += slide_dir * slide_accel * dt;
    }
    // Only zero out velocity normal to ground, not tangential
    next.twist.linear -= ground_normal * ground_normal.dot(next.twist.linear);
}
```

## Acceptance Criteria

- [ ] Material database with at least 7 surface types
- [ ] Rolling resistance based on physical coefficients, not timestep-dependent hacks
- [ ] Vehicles slide on slopes above friction angle
- [ ] Impact restitution varies with speed
- [ ] Static vs kinetic friction differentiation
- [ ] Grounded vehicles can still slide tangentially
- [ ] Contact material configurable per-surface in Unreal (physics material mapping)
- [ ] Unit tests: ball bouncing on different surfaces, box sliding on slope

## Risks

- Multi-point contact solver may need iterative solution — cap iterations
- Unreal collision system may not provide all needed contact data — may need custom traces
- Backward compatibility: existing behaviors will change

## Files Affected

- `AirLib/include/physics/FastPhysicsEngine.hpp` — major rewrite of collision handling (lines 134-257)
- New: `AirLib/include/physics/ContactMaterial.hpp`
- New: `AirLib/include/physics/MaterialDatabase.hpp`
- `AirLib/include/physics/PhysicsBody.hpp` — extend CollisionInfo
- `Unreal/Plugins/AirSim/Source/PawnSimApi.cpp` — pass material info from UE
