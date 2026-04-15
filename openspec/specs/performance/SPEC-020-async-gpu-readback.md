# SPEC-020: Async GPU Readback Pipeline

**Priority:** P1
**Category:** Performance
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-018

## Problem Statement

GPU readback (render target → CPU memory) blocks the render thread:

- `RenderRequest.cpp:99-105`: Polling loop with 5ms timeout waiting for render completion
- Game thread blocks and logs warnings during render thread delay
- GPU LiDAR readback synchronization stalls GPU pipeline (`LidarCamera.h:1-40`)
- No multi-frame pipelining — each frame waits for previous to complete
- For N cameras, each readback serializes, creating N x latency

## Proposed Solution

### 1. Triple-Buffered Async Readback

```cpp
class AsyncGPUReadback {
    static constexpr int NUM_BUFFERS = 3;

    struct ReadbackBuffer {
        FRHIGPUTextureReadback readback;
        FRenderCommandFence fence;
        bool in_flight = false;
        uint64_t frame_number;
    };

    std::array<ReadbackBuffer, NUM_BUFFERS> buffers_;
    int write_idx_ = 0;
    int read_idx_ = 0;

    // Non-blocking: submit readback request
    void submitReadback(FTextureRenderTargetResource* rt, uint64_t frame) {
        auto& buf = buffers_[write_idx_];
        buf.readback.EnqueueCopy(rt->GetRenderTargetTexture());
        buf.fence.BeginFence();
        buf.in_flight = true;
        buf.frame_number = frame;
        write_idx_ = (write_idx_ + 1) % NUM_BUFFERS;
    }

    // Non-blocking: check if oldest readback is ready
    bool isReady() const {
        return buffers_[read_idx_].in_flight &&
               buffers_[read_idx_].fence.IsFenceComplete();
    }

    // Get completed readback data (1-2 frames old)
    const void* getData(int& width, int& height);
};
```

### 2. Camera Capture Pipeline

```cpp
class PipelinedCameraCapture {
    // Frame N: Submit readback for cameras
    // Frame N+1: Process previous frame's data while new frame renders
    // Frame N+2: Return data to client (2-frame latency, but no stalls)

    void tick() {
        // Consume any ready readbacks
        for (auto& [camera_id, readback] : readbacks_) {
            if (readback.isReady()) {
                publishResult(camera_id, readback.getData());
            }
        }

        // Submit new readbacks for this frame
        for (auto& [camera_id, camera] : cameras_) {
            readbacks_[camera_id].submitReadback(camera->getRenderTarget(), frame_);
        }
    }
};
```

### 3. GPU LiDAR Compute Shader Optimization

```cpp
// Use compute shader for GPU LiDAR ray processing
// Currently: render depth buffer → readback → CPU ray processing
// Proposed: render depth buffer → compute shader processes rays on GPU → readback only point cloud

class GPULidarCompute {
    FComputeShaderRHIRef lidar_shader_;
    FRWBufferStructured point_cloud_buffer_;

    void processRaysOnGPU(FTextureRenderTargetResource* depth_rt,
                          const LidarConfig& config) {
        // Dispatch compute shader
        // Input: depth render target + ray configuration
        // Output: structured buffer with 3D points + intensity
        // Only readback the compact point cloud, not full depth image
    }
};
```

## Acceptance Criteria

- [ ] Async readback with triple buffering (no render thread stalls)
- [ ] Camera capture latency < 2 frames (configurable)
- [ ] Parallel readback for multiple cameras (not serialized)
- [ ] GPU LiDAR point cloud generated via compute shader
- [ ] Readback bandwidth: > 500 MB/s for multiple 1080p cameras
- [ ] No `UE_LOG(Warning, "Failed: timeout waiting for screenshot")` during normal operation
- [ ] Client API includes frame timestamp to identify data freshness

## Risks

- 1-2 frame latency may affect real-time control loops — make configurable
- Compute shader approach requires UE5 RHI compatibility testing
- Triple buffering increases GPU memory usage — interact with SPEC-018 memory budget

## Files Affected

- `Unreal/Plugins/AirSim/Source/RenderRequest.cpp` — replace blocking readback
- `Unreal/Plugins/AirSim/Source/LidarCamera.h` — async readback
- New: `Unreal/Plugins/AirSim/Source/AsyncGPUReadback.h`
- New: `Unreal/Plugins/AirSim/Source/Shaders/GPULidarCompute.usf`
- `Unreal/Plugins/AirSim/Source/PIPCamera.cpp` — use pipelined capture
