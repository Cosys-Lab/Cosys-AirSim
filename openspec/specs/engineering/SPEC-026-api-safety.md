# SPEC-026: API Type Safety & Interface Cleanup

**Priority:** P1
**Category:** Engineering Quality
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

The public API exposes unsafe patterns:

1. **Void pointers in public interface**:
   - `void* getClient()` at `RpcLibClientBase.hpp:202`
   - `const void* getClient() const` at line 203
   - `void* getServer() const` at `RpcLibServerBase.hpp`
   - These bypass all type safety and enable undefined behavior

2. **Unsafe static_cast patterns** (`VehicleApiBase.hpp:119-325`):
   - 10+ instances of `static_cast<const SensorBase*>()` with no runtime validation
   - Silent failures if wrong sensor type is cast

3. **Massive public interfaces**:
   - `RpcLibClientBase` has 200+ public methods mixing domains (vehicle control, environment, annotation, detection, recording, cameras, meshes)
   - `MultirotorApiBase.hpp` is 371 lines of pure virtual methods — too wide an interface

4. **API version is single integer** (hardcoded to 4 in `RpcLibServerBase.cpp:96`) — no semantic versioning

5. **3 silently broken methods** returning empty defaults (`RpcLibClientBase.cpp:194-207`)

## Proposed Solution

### 1. Remove Void Pointers

```cpp
// Before
class RpcLibClientBase {
    void* getClient();
    const void* getClient() const;
};

// After: Proper pimpl pattern
class RpcLibClientBase {
private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    // No public access to implementation — only through typed methods
};
```

### 2. Domain-Separated API Interfaces

```cpp
// Split monolithic client into domain interfaces
class IVehicleControl {
    virtual void moveByVelocityAsync(...) = 0;
    virtual void moveToPositionAsync(...) = 0;
    virtual MultirotorState getMultirotorState() = 0;
    // ... vehicle control only
};

class ISensorAccess {
    virtual LidarData getLidarData(...) = 0;
    virtual std::vector<ImageResponse> simGetImages(...) = 0;
    // ... sensor data only
};

class IEnvironmentControl {
    virtual void simSetWeatherParameter(...) = 0;
    virtual void simSetTimeOfDay(...) = 0;
    // ... environment only
};

class IAnnotation {
    virtual void simSetSegmentationObjectID(...) = 0;
    // ... annotation only
};

// Client composes interfaces
class CosysAirSimClient : public IVehicleControl, public ISensorAccess,
                           public IEnvironmentControl, public IAnnotation {
    // Backward compatible: still has all methods
};
```

### 3. Semantic Versioning

```cpp
struct ApiVersion {
    int major;  // Breaking changes
    int minor;  // New features, backward compatible
    int patch;  // Bug fixes

    static ApiVersion current() { return {2, 0, 0}; }

    bool isCompatibleWith(const ApiVersion& other) const {
        return major == other.major && minor >= other.minor;
    }
};
```

### 4. Mark Unimplemented Methods

```cpp
// Instead of silently returning empty:
SensorTemplateData getSensorTemplateData(...) const {
    throw ApiNotImplementedException("getSensorTemplateData not implemented on server");
}
```

## Acceptance Criteria

- [ ] No `void*` in any public API header
- [ ] All sensor casts use `dynamic_cast` or template-based type-safe accessors
- [ ] API organized into domain interfaces (vehicle, sensor, environment, annotation)
- [ ] Semantic versioning (MAJOR.MINOR.PATCH) for API
- [ ] Unimplemented methods throw or return error result (not empty objects)
- [ ] `MultirotorApiBase` split into smaller focused interfaces
- [ ] Backward compatible: existing code compiles with deprecation warnings

## Files Affected

- `AirLib/include/api/RpcLibClientBase.hpp` — remove void*, split interfaces
- `AirLib/include/api/RpcLibServerBase.hpp` — remove void*, semantic versioning
- `AirLib/include/api/VehicleApiBase.hpp` — safe casts, domain separation
- `AirLib/include/vehicles/multirotor/api/MultirotorApiBase.hpp` — interface narrowing
- `AirLib/src/api/RpcLibClientBase.cpp` — fix unimplemented methods
- New: `AirLib/include/api/ApiVersion.hpp`
- New: `AirLib/include/api/interfaces/` — domain-separated interfaces
