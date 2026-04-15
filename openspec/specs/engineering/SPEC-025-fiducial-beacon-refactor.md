# SPEC-025: FiducialBeacon & Critical Code Duplication Refactor

**Priority:** P0
**Category:** Engineering Quality
**Effort:** Small (1 week)
**Dependencies:** None

## Problem Statement

`FiducialBeacon.cpp` (7,090 lines) contains the most egregious code quality issue in the codebase:

- **Lines 52-400+**: 32+ nearly identical if-blocks handling beacon types 0-49
- Each block is ~8 lines of copy-pasted code differing only in an index number
- **Total duplicated lines**: ~400+
- A "BEGIN PASTED TEXT" comment at line 50 explicitly marks the copy-paste
- Additionally, dead/commented-out code at lines 40-46

Other major duplication sites:

1. **VehicleApiBase.hpp** (lines 119-435): 10 near-identical sensor lookup loops (Lidar, GPULidar, Echo, UWB, WiFi, SensorTemplate, MarLocUwb, etc.)
2. **RpcLibAdaptorsBase.hpp** (1,282 lines): 50+ structs with identical boilerplate (constructor, `to()`, `from()` methods)
3. **RPC client methods** in `RpcLibClientBase.cpp` (lines 72-800+): 100+ identical `pimpl_->client.call(...).as<T>()` patterns

## Proposed Solution

### 1. FiducialBeacon Refactor

```cpp
// Before: 400+ lines of copy-paste
if (beaconType == 0) {
    static ConstructorHelpers::FObjectFinder<UTexture> Finder(TEXT("path_0"));
    mesh->SetMaterial(0, CreateMaterial(Finder.Object));
}
if (beaconType == 1) {
    static ConstructorHelpers::FObjectFinder<UTexture> Finder(TEXT("path_1"));
    mesh->SetMaterial(0, CreateMaterial(Finder.Object));
}
// ... 48 more times

// After: Data-driven
static const TCHAR* BeaconTexturePaths[] = {
    TEXT("/Game/Beacons/Textures/Beacon_0"),
    TEXT("/Game/Beacons/Textures/Beacon_1"),
    // ...
    TEXT("/Game/Beacons/Textures/Beacon_49"),
};

void AFiducialBeacon::SetBeaconType(int32 beaconType) {
    check(beaconType >= 0 && beaconType < UE_ARRAY_COUNT(BeaconTexturePaths));
    UTexture* Texture = LoadObject<UTexture>(nullptr, BeaconTexturePaths[beaconType]);
    if (Texture) {
        mesh->SetMaterial(0, CreateMaterial(Texture));
    }
}
```

### 2. Generic Sensor Lookup Template

```cpp
// Before: 10 identical loops
const auto* lidar = static_cast<const LidarBase*>(
    getSensors().getByType(SensorBase::SensorType::Lidar, index));
// ... repeated for every sensor type

// After: Single template
template<typename SensorT>
const SensorT* VehicleApiBase::findSensor(const std::string& name) const {
    for (size_t i = 0; i < getSensors().size(SensorT::kSensorType); ++i) {
        auto* sensor = getSensors().getByType(SensorT::kSensorType, i);
        if (sensor->getName() == name) {
            return static_cast<const SensorT*>(sensor);
        }
    }
    return nullptr;
}

template<typename SensorT>
std::vector<const SensorT*> VehicleApiBase::findAllSensors() const {
    std::vector<const SensorT*> result;
    for (size_t i = 0; i < getSensors().size(SensorT::kSensorType); ++i) {
        result.push_back(static_cast<const SensorT*>(
            getSensors().getByType(SensorT::kSensorType, i)));
    }
    return result;
}
```

### 3. RPC Adaptor Code Generation

Either use:
- **C++ template metaprogramming** for adaptor generation
- **Code generator** (Python script) that produces adaptor structs from type definitions

```cpp
// Macro-based approach for adaptor boilerplate
#define DEFINE_ADAPTOR(AdaptorName, AirLibType, ...) \
    struct AdaptorName { \
        __VA_ARGS__ \
        AdaptorName() {} \
        AdaptorName(const msr::airlib::AirLibType& s) { from(s); } \
        msr::airlib::AirLibType to() const; \
        void from(const msr::airlib::AirLibType& s); \
    }
```

## Acceptance Criteria

- [ ] FiducialBeacon.cpp reduced from 7,090 to < 500 lines
- [ ] VehicleApiBase sensor loops consolidated to < 50 lines (from ~300)
- [ ] Dead code in FiducialBeacon.cpp (lines 40-46) removed
- [ ] "BEGIN PASTED TEXT" comment removed
- [ ] All beacon types still functional (visual verification)
- [ ] Sensor lookup template tested for all sensor types
- [ ] No behavioral changes — pure refactor

## Files Affected

- `Unreal/Plugins/AirSim/Source/Beacons/FiducialBeacon.cpp` — data-driven refactor
- `AirLib/include/api/VehicleApiBase.hpp` — template sensor lookup
- `AirLib/include/sensors/SensorBase.hpp` — add `kSensorType` static member
- `AirLib/include/api/RpcLibAdaptorsBase.hpp` — consider macro/codegen for boilerplate
