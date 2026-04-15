# SPEC-019: Multi-Vehicle Scalability

**Priority:** P0
**Category:** Performance
**Effort:** Large (4-6 weeks)
**Dependencies:** SPEC-017, SPEC-018

## Problem Statement

Performance degrades rapidly with multiple vehicles due to:

1. **Per-vehicle overhead**: Each vehicle creates independent camera objects, lights, sensors, render targets (`PawnSimApi.cpp:102-278`)
2. **Sequential sensor processing**: No parallel sensor computation across vehicles
3. **RPC per-vehicle**: Each vehicle queried independently through separate API calls
4. **Physics per-vehicle**: Each vehicle runs independent physics update (no spatial partitioning)
5. **WorkerThread starvation** (`WorkerThread.hpp:141-144`): Queue size of 1 means high-frequency calls drop tasks

## Proposed Solution

### 1. Batch API Calls

```cpp
// New batch API methods
struct BatchSensorRequest {
    std::string vehicle_name;
    std::vector<ImageRequest> image_requests;
    bool include_lidar;
    bool include_imu;
};

struct BatchSensorResponse {
    std::map<std::string, std::vector<ImageResponse>> images;
    std::map<std::string, LidarData> lidars;
    std::map<std::string, ImuData> imus;
};

// Single RPC call for all vehicles
BatchSensorResponse getBatchSensorData(const std::vector<BatchSensorRequest>& requests);
```

Python:
```python
# Before: N RPC calls
for vehicle in vehicles:
    images[vehicle] = client.simGetImages([req], vehicle_name=vehicle)

# After: 1 RPC call
batch = client.simGetBatchSensorData([
    BatchSensorRequest(vehicle, [req]) for vehicle in vehicles
])
```

### 2. Parallel Sensor Processing

```cpp
class ParallelSensorProcessor {
    ThreadPool thread_pool_;  // Fixed-size thread pool

    void updateAllSensors(std::vector<VehicleSimApi*>& vehicles, float dt) {
        std::vector<std::future<void>> futures;
        for (auto* vehicle : vehicles) {
            futures.push_back(thread_pool_.submit([vehicle, dt]() {
                vehicle->updateSensors(dt);
            }));
        }
        for (auto& f : futures) f.get();
    }
};
```

### 3. LOD for Distant Vehicles

```cpp
class VehicleLODManager {
    // Reduce sensor fidelity for vehicles far from camera/each other
    SensorQuality computeLOD(const Vehicle& vehicle, const Vector3r& observer_pos) {
        float distance = (vehicle.getPosition() - observer_pos).norm();
        if (distance < 50) return SensorQuality::Full;
        if (distance < 200) return SensorQuality::Reduced;
        return SensorQuality::Minimal;
    }

    void applySensorLOD(Vehicle& vehicle, SensorQuality quality) {
        // Full: all sensors at full resolution
        // Reduced: half camera resolution, every-other LiDAR scan
        // Minimal: no cameras, sparse LiDAR, physics only
    }
};
```

### 4. WorkerThread Queue Fix

Replace single-slot queue with proper bounded queue:

```cpp
class BoundedTaskQueue {
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    size_t max_size_;

    bool enqueue(std::function<void()> task) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (tasks_.size() >= max_size_) return false;  // Explicit backpressure
        tasks_.push(std::move(task));
        cv_.notify_one();
        return true;
    }
};
```

## Acceptance Criteria

- [ ] Batch sensor API: single RPC call retrieves data for all vehicles
- [ ] Parallel sensor processing using thread pool
- [ ] LOD system reducing overhead for distant vehicles
- [ ] WorkerThread queue supports configurable depth (default 8)
- [ ] Linear scaling: 10 vehicles at < 3x cost of 1 vehicle (for sensor-light loads)
- [ ] Benchmark: 20 vehicles with cameras at > 15 FPS
- [ ] API backward compatible (single-vehicle calls still work)

## Files Affected

- `AirLib/include/api/RpcLibClientBase.hpp` — batch API
- `AirLib/src/api/RpcLibServerBase.cpp` — batch implementation
- `AirLib/include/common/WorkerThread.hpp` — bounded queue
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp` — parallel sensor updates
- New: `AirLib/include/api/BatchSensorApi.hpp`
- New: `Unreal/Plugins/AirSim/Source/VehicleLODManager.h`
- `PythonClient/cosysairsim/client.py` — batch methods
