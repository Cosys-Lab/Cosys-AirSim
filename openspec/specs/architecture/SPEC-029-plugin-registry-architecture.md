# SPEC-029: Plugin Registry Architecture

**Priority:** P0
**Category:** Architecture
**Effort:** Large (4-6 weeks)
**Dependencies:** SPEC-030

## Problem Statement

The codebase has no plugin/extension architecture — adding new vehicles, sensors, or firmwares requires modifying core code:

1. **Vehicle types hardcoded** (`AirSimSettings.hpp:31-40`): 7 vehicle type strings as constants. New types require editing this file.

2. **SensorFactory is a switch statement** (`SensorFactory.hpp:19-62`): Only handles IMU, Magnetometer, GPS, Barometer. LiDAR, Echo, UWB, WiFi sensors created elsewhere — no factory method.

3. **SimMode requires subclassing** (`SimModeBase.h:245-256`): `isVehicleTypeSupported()`, `getVehiclePawnPathName()` must be overridden in subclasses for each vehicle type.

4. **Firmware coupling** (`MultiRotorParamsFactory.hpp`): Firmware implementations (SimpleFlight, ArduCopter, PX4) hardcoded in factory.

5. **No discovery mechanism**: Cannot register new types from external modules.

## Proposed Solution

### 1. Type Registry Pattern

```cpp
template<typename Base, typename Key = std::string>
class TypeRegistry {
    using Creator = std::function<std::unique_ptr<Base>(const Settings&)>;
    std::unordered_map<Key, Creator> registry_;

public:
    bool registerType(const Key& key, Creator creator) {
        return registry_.emplace(key, std::move(creator)).second;
    }

    std::unique_ptr<Base> create(const Key& key, const Settings& settings) const {
        auto it = registry_.find(key);
        if (it == registry_.end()) return nullptr;
        return it->second(settings);
    }

    std::vector<Key> registeredTypes() const {
        std::vector<Key> keys;
        for (const auto& [k, _] : registry_) keys.push_back(k);
        return keys;
    }
};
```

### 2. Vehicle Registry

```cpp
class VehicleRegistry : public TypeRegistry<VehicleApiBase> {
    static VehicleRegistry& instance();
};

// Auto-registration via static initializer
#define REGISTER_VEHICLE(type_name, VehicleClass) \
    static bool _reg_##VehicleClass = \
        VehicleRegistry::instance().registerType(type_name, \
            [](const Settings& s) { return std::make_unique<VehicleClass>(s); })

// In SimpleFlight implementation:
REGISTER_VEHICLE("simpleflight", SimpleFlightApi);
REGISTER_VEHICLE("px4multirotor", PX4MultiRotorApi);
REGISTER_VEHICLE("arducoptersolo", ArduCopterApi);
```

### 3. Sensor Registry

```cpp
class SensorRegistry : public TypeRegistry<SensorBase, SensorBase::SensorType> {
    static SensorRegistry& instance();
};

// Usage in sensor implementations:
REGISTER_SENSOR(SensorBase::SensorType::Lidar, LidarSimple);
REGISTER_SENSOR(SensorBase::SensorType::GPULidar, GPULidarSimple);
REGISTER_SENSOR(SensorBase::SensorType::Echo, EchoSimple);
```

### 4. Firmware Registry

```cpp
class FirmwareRegistry : public TypeRegistry<MultirotorApiBase> {
    static FirmwareRegistry& instance();
};

REGISTER_FIRMWARE("simpleflight", SimpleFlightApi);
REGISTER_FIRMWARE("px4", PX4Api);
```

### 5. Extension Module Support

```cpp
// External modules can register types via shared library loading
class ExtensionLoader {
    void loadExtension(const std::string& path) {
        auto lib = dlopen(path.c_str(), RTLD_NOW);
        auto init = dlsym(lib, "cosysairsim_extension_init");
        if (init) ((void(*)())init)();
        // Extension's static registrations now active
    }
};
```

Settings support:
```json
{
    "Extensions": [
        {"Path": "/path/to/custom_vehicle.so"},
        {"Path": "/path/to/custom_sensor.so"}
    ]
}
```

## Acceptance Criteria

- [ ] Vehicle types registered via `REGISTER_VEHICLE` macro — no more hardcoded strings
- [ ] Sensor types registered via `REGISTER_SENSOR` — no more switch statement
- [ ] Firmware types registered via `REGISTER_FIRMWARE` — no more factory file editing
- [ ] Extension modules loadable from shared libraries
- [ ] `simGetRegisteredVehicleTypes()` API returning available types
- [ ] `simGetRegisteredSensorTypes()` API returning available sensors
- [ ] Existing vehicle/sensor types registered using the new system (backward compatible)
- [ ] Documentation for creating custom vehicle/sensor extensions
- [ ] At minimum one example external extension module

## Risks

- Static initializer ordering across translation units — mitigate with lazy initialization
- Shared library loading differs between platforms — abstract behind OS layer
- Plugin ABI stability — define stable C interface for extension points

## Files Affected

- New: `AirLib/include/common/TypeRegistry.hpp`
- New: `AirLib/include/api/VehicleRegistry.hpp`
- New: `AirLib/include/sensors/SensorRegistry.hpp`
- `AirLib/include/sensors/SensorFactory.hpp` — rewrite to use registry
- `AirLib/include/common/AirSimSettings.hpp` — remove hardcoded type strings
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.h` — use registry
- All vehicle implementations — add REGISTER_VEHICLE
- All sensor implementations — add REGISTER_SENSOR
