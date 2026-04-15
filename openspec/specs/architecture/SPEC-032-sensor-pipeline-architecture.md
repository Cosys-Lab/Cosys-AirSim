# SPEC-032: Sensor Pipeline Architecture

**Priority:** P1
**Category:** Architecture
**Effort:** Medium (3-4 weeks)
**Dependencies:** SPEC-029

## Problem Statement

Sensors operate as independent silos with no processing pipeline:

1. **No sensor pipeline/middleware**: Each sensor independently calls `update()` in `SensorCollection`. No noise injection abstraction, no filtering, no preprocessing.

2. **Sensor factory is rigid** (`SensorFactory.hpp:19-62`): Switch statement handles only 4 basic sensors. LiDAR, Echo, UWB, WiFi created elsewhere with no factory method.

3. **No configurable processing stages**: Can't add Kalman filtering, moving average, or custom transforms to sensor output without modifying sensor code.

4. **No sensor data recording/replay**: No built-in mechanism to record raw sensor data and replay for offline testing.

5. **SensorCollection is just a container** (`SensorCollection.hpp`): Stores sensors, no processing logic. Iteration is the only operation.

## Proposed Solution

### 1. Sensor Processing Pipeline

```cpp
class ISensorProcessor {
public:
    virtual void process(SensorOutput& output) = 0;
    virtual std::string name() const = 0;
};

class SensorPipeline {
    std::vector<std::unique_ptr<ISensorProcessor>> processors_;

public:
    void addProcessor(std::unique_ptr<ISensorProcessor> proc) {
        processors_.push_back(std::move(proc));
    }

    void process(SensorOutput& output) {
        for (auto& proc : processors_) {
            proc->process(output);
        }
    }
};
```

### 2. Built-in Processors

```cpp
// Noise injection (configurable per-sensor)
class NoiseInjector : public ISensorProcessor {
    NoiseParams params_;
    void process(SensorOutput& output) override;
};

// Moving average filter
class MovingAverageFilter : public ISensorProcessor {
    int window_size_;
    void process(SensorOutput& output) override;
};

// Rate limiter
class RateLimiter : public ISensorProcessor {
    float max_hz_;
    void process(SensorOutput& output) override;
};

// Data recorder
class SensorRecorder : public ISensorProcessor {
    std::ofstream file_;
    void process(SensorOutput& output) override;
};

// Latency simulator
class LatencySimulator : public ISensorProcessor {
    float delay_ms_;
    std::queue<TimestampedOutput> buffer_;
    void process(SensorOutput& output) override;
};

// Failure injector (for robustness testing)
class FailureInjector : public ISensorProcessor {
    float failure_probability_;
    float failure_duration_;
    void process(SensorOutput& output) override;
};
```

### 3. Pipeline Configuration

```json
{
    "Sensors": {
        "Lidar1": {
            "SensorType": "GPULidar",
            "Pipeline": [
                {"Type": "NoiseInjector", "Params": {"scale": 0.02}},
                {"Type": "RateLimiter", "MaxHz": 20},
                {"Type": "SensorRecorder", "Path": "/tmp/lidar_log"},
                {"Type": "LatencySimulator", "DelayMs": 10}
            ]
        },
        "IMU": {
            "SensorType": "IMU",
            "Pipeline": [
                {"Type": "NoiseInjector"},
                {"Type": "FailureInjector", "Probability": 0.001}
            ]
        }
    }
}
```

### 4. Sensor Data Replay

```cpp
class SensorReplay {
    // Load recorded sensor data
    void load(const std::string& recording_path);

    // Replay data at original or modified rate
    void play(float speed_multiplier = 1.0f);

    // Inject replayed data into sensor pipeline
    void injectInto(SensorPipeline& pipeline);
};
```

### 5. Updated SensorFactory with Registry

```cpp
class SensorFactory {
    using SensorCreator = std::function<std::unique_ptr<SensorBase>(
        const SensorSetting&, const GroundTruth&)>;

    static std::unordered_map<SensorType, SensorCreator>& registry() {
        static std::unordered_map<SensorType, SensorCreator> r;
        return r;
    }

public:
    static void registerSensor(SensorType type, SensorCreator creator);
    static std::unique_ptr<SensorBase> create(const SensorSetting& setting,
                                               const GroundTruth& gt);
};
```

## Acceptance Criteria

- [ ] `ISensorProcessor` interface for pipeline stages
- [ ] At least 5 built-in processors (noise, filter, rate limit, recorder, latency)
- [ ] Pipeline configurable per-sensor in settings.json
- [ ] Sensor data recording with timestamps and metadata
- [ ] Sensor data replay for offline testing
- [ ] Failure injection for robustness testing
- [ ] SensorFactory uses registration instead of switch statement
- [ ] All existing sensors work with default (empty) pipeline

## Files Affected

- New: `AirLib/include/sensors/SensorPipeline.hpp`
- New: `AirLib/include/sensors/processors/`
- `AirLib/include/sensors/SensorFactory.hpp` — registry-based
- `AirLib/include/sensors/SensorCollection.hpp` — integrate pipeline
- `AirLib/include/sensors/SensorBase.hpp` — add pipeline support
- `AirLib/include/common/AirSimSettings.hpp` — pipeline configuration
