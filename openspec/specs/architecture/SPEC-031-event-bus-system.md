# SPEC-031: Event Bus & Pub/Sub System

**Priority:** P1
**Category:** Architecture
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

The current event system is minimal and tightly coupled:

1. **Basic Signal/Slot only** (`Signal.hpp:39-104`): Simple connect/emit pattern with no filtering, routing, or ordering guarantees.

2. **Only 2 events exist**: Collision and pawn tick (`PawnEvents.h`). Missing: vehicle spawn/destroy, state change, sensor update, command received, simulation lifecycle.

3. **No central event bus**: Events are point-to-point (emitter → listeners). No publish/subscribe, no event discovery.

4. **No ordering guarantees**: Multiple listeners on same signal have no guaranteed execution order (iterating `std::map` which may reorder, `Signal.hpp:92`).

5. **No async events**: All events are synchronous — handler blocks the emitter.

6. **Tight coupling**: Components call each other directly via method calls and polling instead of events. `VehicleApiBase::update()` called explicitly, state changes require explicit polling.

## Proposed Solution

### 1. Central Event Bus

```cpp
class EventBus {
public:
    static EventBus& instance();

    // Type-safe event publishing
    template<typename EventT>
    void publish(const EventT& event) {
        auto& handlers = handlers_[typeid(EventT).hash_code()];
        for (auto& [priority, handler] : handlers) {
            handler(static_cast<const void*>(&event));
        }
    }

    // Type-safe subscription with priority
    template<typename EventT>
    SubscriptionHandle subscribe(std::function<void(const EventT&)> handler,
                                  int priority = 0) {
        auto id = nextId_++;
        handlers_[typeid(EventT).hash_code()].emplace(
            priority, [handler, id](const void* e) {
                handler(*static_cast<const EventT*>(e));
            });
        return {id, typeid(EventT).hash_code()};
    }

    void unsubscribe(SubscriptionHandle handle);

private:
    std::unordered_map<size_t, std::multimap<int, std::function<void(const void*)>>> handlers_;
    std::atomic<uint64_t> nextId_{0};
};
```

### 2. Event Types

```cpp
// Simulation lifecycle
struct SimulationStartedEvent { float timestamp; };
struct SimulationPausedEvent { float timestamp; };
struct SimulationResumedEvent { float timestamp; };
struct SimulationResetEvent {};

// Vehicle lifecycle
struct VehicleSpawnedEvent { std::string vehicle_name; std::string vehicle_type; };
struct VehicleDestroyedEvent { std::string vehicle_name; };

// Vehicle state
struct VehicleStateChangedEvent {
    std::string vehicle_name;
    Kinematics::State old_state;
    Kinematics::State new_state;
};
struct CollisionEvent {
    std::string vehicle_name;
    CollisionInfo info;
};
struct LandingEvent { std::string vehicle_name; };
struct TakeoffEvent { std::string vehicle_name; };

// Sensor events
struct SensorDataReadyEvent {
    std::string vehicle_name;
    std::string sensor_name;
    SensorBase::SensorType type;
    float timestamp;
};

// API events
struct ApiCallEvent {
    std::string method;
    float timestamp;
    float duration_ms;
};

// Environment events
struct WeatherChangedEvent { WeatherParams params; };
struct TimeOfDayChangedEvent { float sun_angle; };
```

### 3. Async Event Queue

```cpp
class AsyncEventQueue {
    struct PendingEvent {
        size_t type_hash;
        std::shared_ptr<void> data;
        float timestamp;
    };

    ThreadSafeQueue<PendingEvent> queue_;
    std::thread dispatch_thread_;

    void dispatchLoop() {
        while (running_) {
            auto event = queue_.pop();
            EventBus::instance().publish(event);
        }
    }
};
```

### 4. Python Client Event Subscription

```python
class EventClient:
    def on_collision(self, callback):
        """Register callback for collision events."""

    def on_vehicle_spawned(self, callback):
        """Register callback for vehicle spawn events."""

    def on_sensor_data(self, sensor_name, callback):
        """Register callback for sensor data availability."""

    # Push-based: server pushes events to client (WebSocket or callback)
    def start_event_listener(self):
        """Start listening for events from server."""
```

## Acceptance Criteria

- [ ] Central EventBus with type-safe publish/subscribe
- [ ] At least 10 event types covering simulation, vehicle, sensor, API domains
- [ ] Priority-based handler ordering
- [ ] Async event queue for non-blocking handlers
- [ ] Python client can subscribe to events
- [ ] Existing Signal/Slot usage migrated to EventBus
- [ ] Event logging middleware for debugging
- [ ] Performance: < 1us per event dispatch (no subscribers)

## Files Affected

- New: `AirLib/include/common/EventBus.hpp`
- New: `AirLib/include/common/Events.hpp`
- `AirLib/include/common/common_utils/Signal.hpp` — deprecate in favor of EventBus
- `Unreal/Plugins/AirSim/Source/PawnSimApi.cpp` — migrate collision/tick to EventBus
- `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp` — publish lifecycle events
- `AirLib/include/api/RpcLibServerBase.hpp` — expose event subscription API
- `PythonClient/cosysairsim/client.py` — event listener
