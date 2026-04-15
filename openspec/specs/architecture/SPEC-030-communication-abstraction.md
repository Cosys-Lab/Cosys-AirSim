# SPEC-030: Communication Abstraction Layer

**Priority:** P1
**Category:** Architecture
**Effort:** Large (4-6 weeks)
**Dependencies:** None

## Problem Statement

Communication is hardcoded to msgpack-RPC via rpclib:

1. **Single protocol**: Only MessagePack-based RPC supported. No gRPC, WebSocket, REST, or shared memory alternatives.

2. **RPC library lock-in**: `RpcLibClientBase`, `RpcLibServerBase`, and `RpcLibAdaptorsBase` tightly coupled to rpclib internals (including workarounds for rpclib bugs).

3. **Adaptor explosion** (`RpcLibAdaptorsBase.hpp`, 1,282 lines): Every data type requires manual serialization structs. Adding a single new API method requires changes in 4+ files.

4. **Multiple client implementations**: Python client, C++ client, ROS2 wrapper, and Matlab client all implement their own connection logic independently.

5. **No transport alternatives**: For same-machine connections, RPC adds unnecessary serialization overhead. Shared memory would be much faster (see SPEC-017).

6. **No middleware**: No authentication, rate limiting, logging, or request tracing.

## Proposed Solution

### 1. Transport Interface

```cpp
class ITransport {
public:
    virtual ~ITransport() = default;

    virtual void start(int port) = 0;
    virtual void stop() = 0;

    using Handler = std::function<std::vector<uint8_t>(
        const std::string& method, const std::vector<uint8_t>& params)>;

    // Server side: register method handlers
    virtual void bind(const std::string& method, Handler handler) = 0;

    // Client side: call remote method
    virtual std::vector<uint8_t> call(const std::string& method,
                                       const std::vector<uint8_t>& params) = 0;
    virtual std::future<std::vector<uint8_t>> callAsync(
        const std::string& method, const std::vector<uint8_t>& params) = 0;
};
```

### 2. Transport Implementations

```cpp
// Current: MessagePack RPC (backward compatible)
class MsgpackRpcTransport : public ITransport { ... };

// New: gRPC (high-performance, language-agnostic)
class GrpcTransport : public ITransport { ... };

// New: Shared Memory (zero-copy local)
class SharedMemoryTransport : public ITransport { ... };

// New: WebSocket (browser/web clients)
class WebSocketTransport : public ITransport { ... };
```

### 3. Serialization Interface

```cpp
class ISerializer {
public:
    virtual std::vector<uint8_t> serialize(const ApiMessage& msg) = 0;
    virtual ApiMessage deserialize(const std::vector<uint8_t>& data) = 0;
};

class MsgpackSerializer : public ISerializer { ... };
class ProtobufSerializer : public ISerializer { ... };
class FlatbufSerializer : public ISerializer { ... };  // Zero-copy
```

### 4. Middleware Chain

```cpp
class IMiddleware {
public:
    virtual void onRequest(const std::string& method,
                           std::vector<uint8_t>& params) = 0;
    virtual void onResponse(const std::string& method,
                            std::vector<uint8_t>& result) = 0;
};

class LoggingMiddleware : public IMiddleware { ... };
class MetricsMiddleware : public IMiddleware { ... };
class RateLimitMiddleware : public IMiddleware { ... };
class AuthMiddleware : public IMiddleware { ... };
```

### 5. Transport Selection in Settings

```json
{
    "ApiServer": {
        "Transport": "msgpack-rpc",
        "Port": 41451,
        "Alternatives": [
            {"Transport": "grpc", "Port": 41452},
            {"Transport": "shared-memory", "Name": "cosysairsim_shm"}
        ],
        "Middleware": ["logging", "metrics"]
    }
}
```

### 6. Auto-Generated API Bindings

Define API in IDL (Interface Definition Language):

```protobuf
// api.proto or api.yaml
service VehicleControl {
    rpc MoveByVelocity(VelocityCommand) returns (ActionResult);
    rpc GetState(StateRequest) returns (VehicleState);
}

service SensorAccess {
    rpc GetImages(ImageRequest) returns (stream ImageResponse);
    rpc GetLidarData(LidarRequest) returns (LidarData);
}
```

Generate client/server stubs for: C++, Python, Matlab, ROS2.

## Acceptance Criteria

- [ ] `ITransport` interface cleanly separates transport from business logic
- [ ] MsgpackRpc transport passes all existing tests (backward compatible)
- [ ] At least one alternative transport (shared memory or gRPC) implemented
- [ ] Middleware chain supports logging and metrics
- [ ] Transport selectable via settings.json
- [ ] Python client automatically uses best available transport
- [ ] API binding generation from IDL for at least C++ and Python
- [ ] Adaptor boilerplate reduced by > 50% through code generation

## Risks

- Multiple transports increase maintenance burden — prioritize 2 max
- API IDL requires upfront design effort — start with subset
- Existing clients need migration path — maintain MsgpackRpc as default

## Files Affected

- New: `AirLib/include/api/ITransport.hpp`
- New: `AirLib/include/api/ISerializer.hpp`
- New: `AirLib/include/api/IMiddleware.hpp`
- New: `AirLib/include/api/transports/`
- `AirLib/include/api/RpcLibServerBase.hpp` — implement ITransport
- `AirLib/include/api/RpcLibClientBase.hpp` — implement ITransport
- `AirLib/include/api/RpcLibAdaptorsBase.hpp` — reduce via code generation
- New: `tools/codegen/generate_api.py`
- New: `api/cosysairsim.proto` or `api/cosysairsim.yaml`
