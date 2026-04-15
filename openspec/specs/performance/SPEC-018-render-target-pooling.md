# SPEC-018: Render Target Pooling & GPU Memory Management

**Priority:** P1
**Category:** Performance
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

Render targets are allocated per-capture with no reuse (`RenderRequest.cpp:44,50,153`):

- `FTextureRenderTargetResource` obtained per frame per camera — no pooling
- `RenderResult` objects created fresh every screenshot request (lines 21-32)
- N vehicles x M cameras = N*M simultaneous render target allocations
- GPU memory fragmentation and allocation overhead
- No explicit cleanup or lifecycle management
- Camera/light setup duplicated per vehicle (`PawnSimApi.cpp:102-278`)

## Proposed Solution

### 1. Render Target Pool

```cpp
class RenderTargetPool {
    struct PoolKey {
        int width, height;
        EPixelFormat format;
        bool operator==(const PoolKey& o) const { ... }
    };

    std::unordered_map<PoolKey, std::queue<UTextureRenderTarget2D*>> pool_;
    std::mutex mutex_;

    UTextureRenderTarget2D* acquire(int width, int height, EPixelFormat format);
    void release(UTextureRenderTarget2D* target);

    // Statistics
    size_t total_allocated() const;
    size_t currently_in_use() const;
    size_t total_gpu_memory_bytes() const;
};
```

### 2. RenderResult Recycling

```cpp
class RenderResultPool {
    std::vector<std::shared_ptr<RenderResult>> pool_;

    std::shared_ptr<RenderResult> acquire(bool pixels_as_float);
    void release(std::shared_ptr<RenderResult> result);
};
```

### 3. Per-Vehicle Camera Sharing

```cpp
class SharedCameraManager {
    // Cameras with same settings share render targets
    // Render once, distribute to multiple consumers
    std::unordered_map<CameraSettings, std::vector<CameraConsumer>> shared_cameras_;

    void registerConsumer(const CameraSettings& settings, CameraConsumer consumer);
    void renderAndDistribute();
};
```

### 4. GPU Memory Budget

```cpp
class GPUMemoryBudget {
    size_t max_gpu_memory_;  // Configurable limit

    bool canAllocate(size_t bytes) const;
    void onAllocate(size_t bytes);
    void onFree(size_t bytes);

    // Evict least-recently-used targets when over budget
    void enforceLimit();
};
```

## Acceptance Criteria

- [ ] Render targets pooled by resolution and format
- [ ] RenderResult objects recycled instead of reallocated
- [ ] GPU memory usage stable over time (no growth during long sessions)
- [ ] 10-vehicle, 3-camera scenario uses < 2x GPU memory of 1-vehicle scenario
- [ ] GPU memory usage reported via API (`simGetGPUMemoryStats()`)
- [ ] Configurable GPU memory budget in settings.json
- [ ] Performance benchmarks showing allocation reduction

## Files Affected

- `Unreal/Plugins/AirSim/Source/RenderRequest.cpp` — use pool for render targets
- `Unreal/Plugins/AirSim/Source/PawnSimApi.cpp` — shared camera management
- New: `Unreal/Plugins/AirSim/Source/RenderTargetPool.h`
- New: `Unreal/Plugins/AirSim/Source/GPUMemoryBudget.h`
