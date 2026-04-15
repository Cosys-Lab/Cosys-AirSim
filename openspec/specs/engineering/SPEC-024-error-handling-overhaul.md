# SPEC-024: Error Handling & Resilience Overhaul

**Priority:** P0
**Category:** Engineering Quality
**Effort:** Medium (3-4 weeks)
**Dependencies:** None

## Problem Statement

Error handling is inconsistent and incomplete:

1. **Zero try-catch in RPC client** (`RpcLibClientBase.cpp`): All 200+ RPC calls are naked — network failures propagate as unhandled exceptions. Example: `ping()` (lines 72-75) can throw if connection drops.

2. **Unimplemented methods return empty objects silently** (`RpcLibClientBase.cpp:194-207`):
   ```cpp
   SensorTemplateData getSensorTemplateData(...) const {
       return SensorTemplateData();  // Silent empty stub!
   }
   ```
   Callers can't distinguish "no data" from "method not implemented."

3. **Exception-only error propagation**: No `std::expected`, `Result<T>`, or error codes. Exceptions in C++ systems programming are expensive and hard to reason about.

4. **No connection resilience**: No automatic reconnection, no retry logic, no heartbeat monitoring.

5. **Settings validation**: `AirSimSettings.hpp` loads JSON with minimal validation (lines 719-766). Invalid ranges, wrong types, and missing fields accepted silently.

6. **VehicleApiBase unsafe casts**: 10+ instances of `static_cast<const SensorBase*>()` (lines 119-325) with no runtime type checking.

## Proposed Solution

### 1. Result Type for API Methods

```cpp
template<typename T>
struct ApiResult {
    std::optional<T> value;
    std::string error_message;
    enum ErrorCode { OK, NOT_CONNECTED, TIMEOUT, INVALID_PARAM, NOT_IMPLEMENTED, INTERNAL };
    ErrorCode error_code = OK;

    bool ok() const { return error_code == OK; }
    const T& get() const { return *value; }

    static ApiResult success(T val) { return {std::move(val), "", OK}; }
    static ApiResult error(ErrorCode code, std::string msg) { return {std::nullopt, msg, code}; }
};
```

### 2. RPC Client Resilience

```cpp
class ResilientRpcClient {
    // Automatic reconnection
    bool reconnect(int max_attempts = 3, int backoff_ms = 1000);

    // Heartbeat monitoring
    void startHeartbeat(int interval_ms = 5000);

    // Retry wrapper
    template<typename Func>
    auto withRetry(Func&& func, int max_retries = 2) -> decltype(func()) {
        for (int i = 0; i <= max_retries; ++i) {
            try {
                return func();
            } catch (const rpc::timeout& e) {
                if (i == max_retries) throw;
                reconnect();
            }
        }
    }

    // Connection state
    enum ConnectionState { CONNECTED, DISCONNECTED, RECONNECTING };
    ConnectionState getConnectionState() const;
    void onConnectionStateChanged(std::function<void(ConnectionState)> callback);
};
```

### 3. Safe Sensor Casts

```cpp
// Before: unsafe
auto* lidar = static_cast<const LidarBase*>(findSensorByName(name, SensorType::Lidar));

// After: type-checked
template<typename SensorT>
const SensorT* getSensor(const std::string& name) const {
    auto* base = findSensorByName(name, SensorT::sensorType());
    if (!base) return nullptr;
    return dynamic_cast<const SensorT*>(base);
}
```

### 4. Settings Validation Framework

```cpp
class SettingsValidator {
    struct ValidationResult {
        bool valid;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };

    ValidationResult validate(const AirSimSettings& settings) {
        ValidationResult result;
        // Range validation
        if (settings.time_of_day.celestial_clock_speed < 0)
            result.errors.push_back("celestial_clock_speed must be >= 0");
        // Type validation
        if (!isValidVehicleType(settings.vehicle_type))
            result.errors.push_back("Unknown vehicle type: " + settings.vehicle_type);
        // Dependency validation
        if (settings.use_px4 && settings.simple_flight)
            result.warnings.push_back("Both PX4 and SimpleFlight enabled");
        ...
        return result;
    }
};
```

### 5. Python Client Error Handling

```python
class CosysAirSimError(Exception):
    """Base exception for CosysAirSim."""

class ConnectionError(CosysAirSimError):
    """Failed to connect to simulator."""

class TimeoutError(CosysAirSimError):
    """Operation timed out."""

class InvalidParameterError(CosysAirSimError):
    """Invalid parameter passed to API."""

class NotImplementedError(CosysAirSimError):
    """Requested feature not implemented in current server version."""

# Client uses specific exceptions
def getLidarData(self, lidar_name, vehicle_name=""):
    try:
        return self._call("getLidarData", lidar_name, vehicle_name)
    except rpc.timeout:
        raise TimeoutError(f"getLidarData timed out for {lidar_name}")
```

## Acceptance Criteria

- [ ] All RPC client methods wrapped in try-catch with specific error types
- [ ] Unimplemented methods raise `NotImplementedError` instead of returning empty
- [ ] Connection resilience with automatic reconnection
- [ ] Heartbeat monitoring with configurable interval
- [ ] Settings validation runs on load with clear error messages
- [ ] `static_cast` on sensors replaced with `dynamic_cast` + null checks
- [ ] Python client has exception hierarchy with specific error types
- [ ] Error handling documented with examples in API docs

## Files Affected

- `AirLib/src/api/RpcLibClientBase.cpp` — wrap all RPC calls
- `AirLib/include/api/RpcLibClientBase.hpp` — resilience features
- `AirLib/include/api/VehicleApiBase.hpp` — safe sensor casts
- `AirLib/include/common/AirSimSettings.hpp` — validation framework
- `PythonClient/cosysairsim/client.py` — exception hierarchy
- New: `AirLib/include/api/ApiResult.hpp`
- New: `AirLib/include/common/SettingsValidator.hpp`
- New: `PythonClient/cosysairsim/exceptions.py`
