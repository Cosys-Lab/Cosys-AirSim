# SPEC-017: Zero-Copy Sensor Data Pipeline

**Priority:** P0
**Category:** Performance
**Effort:** Large (4-6 weeks)
**Dependencies:** SPEC-029

## Problem Statement

Sensor data undergoes excessive copies between UE, AirLib, RPC, and Python client:

### Image Path (4K RGB = 24.8MB per image)
1. UE render target → `RenderResult::bmp` (Unreal `FColor` array)
2. Per-pixel RGB extraction loop (`RenderRequest.cpp:114-121`) — 74.4M pointer operations for 4K
3. `bmp` → `image_data_uint8` vector copy
4. `RpcLibAdaptorsBase.hpp:565-650`: ImageResponse → msgpack serialization (full copy)
5. Network transfer
6. Python msgpack deserialization → Python list
7. `np.fromstring()` → numpy array (deprecated, uses `np.fromstring` not `np.frombuffer`)

### LiDAR Path (360K points = 1.44MB)
1. UE ray trace → point cloud vector
2. `GPULidarSimple.hpp:77-100`: `point_cloud_temp_` → `point_cloud_` copy
3. `RpcLibAdaptorsBase.hpp:668-670`: Full vector copy in adaptor constructor
4. Workaround for rpclib bug: dummy data added to empty vectors (lines 672-674)
5. msgpack serialization
6. Python deserialization → Python list → numpy conversion

### Vector Adaptation (`RpcLibAdaptorsBase.hpp:28-41`)
- Element-by-element `push_back()` without `reserve()` — O(n) reallocations
- Every struct (Vector3r, Pose, etc.) copied through intermediate adaptor types

## Proposed Solution

### 1. Shared Memory Transport for Local Connections

```cpp
class SharedMemoryTransport {
    // For same-machine connections, bypass RPC entirely
    struct SharedBuffer {
        std::atomic<uint64_t> write_seq;
        std::atomic<uint64_t> read_seq;
        size_t capacity;
        uint8_t data[];
    };

    // Ring buffer for sensor data
    SharedBuffer* image_buffers_[MAX_CAMERAS];
    SharedBuffer* lidar_buffers_[MAX_LIDARS];

    void publishImage(int camera_id, const uint8_t* data, size_t size);
    bool consumeImage(int camera_id, uint8_t* dest, size_t& size);
};
```

Python side:
```python
import multiprocessing.shared_memory as shm

class SharedMemoryClient:
    """Zero-copy sensor data access for local connections."""
    def get_image(self, camera_name) -> np.ndarray:
        buf = self._shm_buffers[camera_name]
        return np.ndarray(shape, dtype=np.uint8, buffer=buf.buf)
```

### 2. Vectorized Pixel Processing

Replace per-pixel loop (`RenderRequest.cpp:114-121`) with bulk memory operations:

```cpp
// Before: 74.4M pointer operations for 4K
for (const auto& item : bmp) {
    *ptr++ = item.R;
    *ptr++ = item.G;
    *ptr++ = item.B;
}

// After: Single memcpy with channel deinterleaving
// Or SIMD-based conversion
void convertBGRAtoRGB(const FColor* src, uint8_t* dst, size_t pixel_count) {
    #if defined(__SSE2__)
    // Process 16 pixels at a time with SSE2
    for (size_t i = 0; i < pixel_count; i += 16) {
        __m128i bgra0 = _mm_load_si128((__m128i*)(src + i));
        // Shuffle BGRA → RGB using _mm_shuffle_epi8
        ...
    }
    #else
    // Fallback: still vectorized with struct assignment
    std::memcpy(dst, src, pixel_count * 3);  // if layout matches
    #endif
}
```

### 3. Move Semantics in RPC Adaptors

```cpp
// Before
LidarData(const msr::airlib::LidarData& s) {
    point_cloud = s.point_cloud;  // COPY
}

// After
LidarData(msr::airlib::LidarData&& s) noexcept {
    point_cloud = std::move(s.point_cloud);  // MOVE
}
```

### 4. Python Client Optimizations

```python
# Before (deprecated, slow)
def string_to_uint8_array(bstr):
    return np.fromstring(bstr, np.uint8)

# After
def string_to_uint8_array(bstr):
    return np.frombuffer(bstr, dtype=np.uint8)

# Add zero-copy image decoding
def get_image_numpy(self, camera, image_type) -> np.ndarray:
    """Return image as numpy array without intermediate copies."""
    response = self._get_image_raw(camera, image_type)
    return np.frombuffer(response.image_data_uint8, dtype=np.uint8).reshape(h, w, 3)
```

### 5. Lazy Sensor Evaluation

Only compute sensor data when client requests it:

```cpp
class LazySensorOutput {
    bool dirty_ = true;
    mutable std::optional<LidarData> cached_output_;

    const LidarData& getOutput() const {
        if (dirty_) {
            cached_output_ = computeOutput();
            dirty_ = false;
        }
        return *cached_output_;
    }
};
```

## Acceptance Criteria

- [ ] Shared memory transport for local same-machine connections
- [ ] Image capture latency reduced by at least 50% for 1080p
- [ ] LiDAR data transfer uses move semantics (0 copies in RPC path)
- [ ] `np.fromstring()` replaced with `np.frombuffer()` everywhere
- [ ] Vectorized pixel conversion (SSE2/NEON or at minimum memcpy-based)
- [ ] Lazy sensor evaluation prevents unnecessary computation
- [ ] Benchmark showing throughput improvement (images/sec, points/sec)
- [ ] Backward compatible: RPC still works for remote connections

## Risks

- Shared memory requires same-machine — keep RPC as fallback
- Platform-specific SIMD — use compiler intrinsics with fallback
- rpclib may not support move semantics in all paths — may need patches

## Files Affected

- `Unreal/Plugins/AirSim/Source/RenderRequest.cpp` — vectorized pixel conversion
- `AirLib/include/api/RpcLibAdaptorsBase.hpp` — move semantics, reserve()
- `PythonClient/cosysairsim/utils.py` — np.frombuffer
- `PythonClient/cosysairsim/client.py` — shared memory client option
- New: `AirLib/include/api/SharedMemoryTransport.hpp`
- New: `PythonClient/cosysairsim/shm_client.py`
