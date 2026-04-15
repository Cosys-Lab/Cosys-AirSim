# SPEC-021: RPC Protocol & Serialization Optimization

**Priority:** P1
**Category:** Performance
**Effort:** Medium (3-4 weeks)
**Dependencies:** None

## Problem Statement

The msgpack-based RPC layer has multiple inefficiencies:

1. **No compression**: Raw sensor data serialized without compression — 4K image = 24.8MB per call
2. **Vector adaptation overhead** (`RpcLibAdaptorsBase.hpp:28-41`): Element-by-element `push_back()` without `reserve()` — O(n²) worst-case reallocations
3. **rpclib bug workarounds** (`RpcLibAdaptorsBase.hpp:672-674`): Dummy data added to empty vectors
4. **Adaptor explosion**: 1,282 lines of manual struct-to-msgpack conversions
5. **No streaming**: Large point clouds transferred as single monolithic message
6. **No request batching**: Each API call is independent RPC roundtrip
7. **Duplicate serialization**: Both `image_data_uint8` and `image_data_float` fields serialized in ImageResponse even when only one is used

## Proposed Solution

### 1. Optional Image Compression

```cpp
struct ImageResponse {
    // Add compression options
    enum Compression { NONE, PNG, JPEG, LZ4 };

    Compression compression = NONE;
    std::vector<uint8_t> compressed_data;
    int original_width, original_height, channels;

    // Only serialize the format actually used
    MSGPACK_DEFINE_MAP(compression, compressed_data, original_width, original_height, channels);
};
```

### 2. Vector Reservation Fix

```cpp
// Before: O(n²) reallocation
template <typename TSrc, typename TDest>
static void to(const std::vector<TSrc>& s, std::vector<TDest>& d) {
    d.clear();
    for (size_t i = 0; i < s.size(); ++i)
        d.push_back(s.at(i).to());
}

// After: O(n) with reservation
template <typename TSrc, typename TDest>
static void to(const std::vector<TSrc>& s, std::vector<TDest>& d) {
    d.clear();
    d.reserve(s.size());
    for (const auto& item : s)
        d.emplace_back(item.to());
}
```

### 3. Streaming for Large Data

```cpp
class StreamingSensorData {
    // Stream point clouds in chunks
    void streamLidarData(const std::string& vehicle, const std::string& sensor,
                         std::function<void(const float*, size_t)> chunk_callback,
                         size_t chunk_size = 65536);
};
```

### 4. Request Batching

```cpp
// Batch multiple RPC calls into single network roundtrip
struct BatchRequest {
    std::vector<std::pair<std::string, msgpack::object>> calls;
};

struct BatchResponse {
    std::vector<msgpack::object> results;
};
```

### 5. Selective Serialization

```cpp
// Only serialize fields client actually needs
struct ImageRequest {
    // New: specify which fields to return
    bool include_image_data = true;
    bool include_camera_info = false;
    bool include_pose = true;
    Compression compression = NONE;
    int jpeg_quality = 85;
};
```

## Acceptance Criteria

- [ ] JPEG compression reduces image transfer size by 10x (configurable quality)
- [ ] LZ4 compression for point clouds reduces transfer by 2-3x
- [ ] Vector reserve() applied in all adaptor conversion functions
- [ ] Request batching reduces roundtrip count by N for N-vehicle queries
- [ ] rpclib bug workarounds removed (or documented as permanent if unfixable)
- [ ] Selective serialization: client can request subset of response fields
- [ ] Benchmark: 30% throughput improvement for multi-camera scenarios

## Files Affected

- `AirLib/include/api/RpcLibAdaptorsBase.hpp` — reserve(), move semantics, selective fields
- `AirLib/src/api/RpcLibServerBase.cpp` — batch handling, compression
- `AirLib/include/api/RpcLibClientBase.hpp` — batch API
- `PythonClient/cosysairsim/client.py` — compression options, batching
- `PythonClient/cosysairsim/types.py` — updated ImageRequest
