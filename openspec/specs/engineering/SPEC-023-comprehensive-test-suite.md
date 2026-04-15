# SPEC-023: Comprehensive Test Suite

**Priority:** P0
**Category:** Engineering Quality
**Effort:** Large (6-8 weeks)
**Dependencies:** None

## Problem Statement

Test coverage is critically insufficient:

- **Total test code**: ~460 lines across 7 test files — for a ~200K LOC project
- **Only 4 tests actually run** (`main.cpp`): QuaternionTest, CelestialTest, SettingsTest, SimpleFlightTest
- **2 tests disabled** (lines 18-20): PixhawkTest and WorkerThreadTest commented out
- **No tests for**:
  - RPC client/server communication
  - Any API methods (200+ methods in RpcLibClientBase)
  - Any Unreal integration
  - Any sensor implementation
  - Collision handling
  - Physics engine
  - Python client
  - Annotation system (1,481 lines in ObjectAnnotator.cpp — untested)
  - Configuration parsing/validation
  - Vehicle control loops
  - Error handling paths

## Proposed Solution

### 1. Test Pyramid

```
                    /\
                   /  \       E2E Tests (5%)
                  /    \      - Full simulation scenarios
                 /------\     - Multi-vehicle coordination
                /        \
               /  Integ   \   Integration Tests (25%)
              /   Tests    \  - RPC round-trip
             /              \ - Sensor → API → Client
            /----------------\
           /                  \
          /    Unit Tests       \  Unit Tests (70%)
         /                      \ - Physics models
        /------------------------\ - Sensor noise
       /                          \ - Settings parsing
      /    Static Analysis         \ - Data conversions
     /------------------------------\
```

### 2. Unit Test Coverage Targets

```cpp
// Physics tests
TEST(FastPhysicsEngine, DragForceIncreasesWithVelocity)
TEST(FastPhysicsEngine, CollisionResponseConservesEnergy)
TEST(FastPhysicsEngine, GravityProducesFreeFallAcceleration)
TEST(FastPhysicsEngine, RollingFrictionDeceleratesAngularVelocity)
TEST(FastPhysicsEngine, GroundContactPreventsDownwardMotion)

// Sensor tests
TEST(ImuSimple, NoiseHasCorrectStatisticalProperties)
TEST(ImuSimple, BiasRandomWalkDriftsBoundedly)
TEST(GpsSimple, AccuracyConvergesToSteadyState)
TEST(BarometerSimple, AltitudeDecreasesWithPressure)
TEST(MagnetometerSimple, FieldMatchesIGRFAtKnownLocation)
TEST(LidarSimple, PointCountMatchesConfiguration)
TEST(DistanceSimple, NoiseIsGaussianDistributed)

// Settings tests
TEST(AirSimSettings, InvalidJsonProducesError)
TEST(AirSimSettings, MissingFieldsGetDefaults)
TEST(AirSimSettings, UnknownVehicleTypeRejectsGracefully)
TEST(AirSimSettings, SensorConfigurationParsesCorrectly)

// API tests
TEST(VehicleApiBase, SensorLookupByNameFindsCorrectSensor)
TEST(VehicleApiBase, SensorLookupByTypeReturnsAllMatching)
TEST(VehicleApiBase, InvalidSensorNameReturnsNull)

// Data conversion tests
TEST(RpcLibAdaptors, Vector3rRoundTrips)
TEST(RpcLibAdaptors, PoseRoundTrips)
TEST(RpcLibAdaptors, ImageResponseRoundTrips)
TEST(RpcLibAdaptors, LidarDataRoundTrips)
TEST(RpcLibAdaptors, EmptyVectorHandledCorrectly)
```

### 3. Integration Tests

```cpp
// RPC round-trip tests
TEST(RpcIntegration, PingReturnsTrue)
TEST(RpcIntegration, VersionCheckSucceeds)
TEST(RpcIntegration, ImageRequestReturnsValidData)
TEST(RpcIntegration, LidarDataHasCorrectDimensions)
TEST(RpcIntegration, ConcurrentClientsHandled)
TEST(RpcIntegration, LargePayloadTransferSucceeds)
TEST(RpcIntegration, ConnectionTimeoutHandledGracefully)
```

### 4. Python Client Tests

```python
# tests/test_client.py
class TestClient:
    def test_connection_refused_raises(self):
        ...
    def test_image_conversion_roundtrip(self):
        ...
    def test_lidar_data_to_numpy(self):
        ...
    def test_type_annotations_match_runtime(self):
        ...

# tests/test_utils.py
class TestUtils:
    def test_string_to_uint8_array(self):
        ...
    def test_string_to_float_array(self):
        ...
    def test_image_reshaping(self):
        ...
```

### 5. Test Infrastructure

- **Framework**: GoogleTest for C++, pytest for Python
- **CI Integration**: Tests run on every PR (see PR-015)
- **Coverage Reporting**: gcov/lcov for C++, pytest-cov for Python
- **Mocking**: GoogleMock for C++ interfaces, unittest.mock for Python
- **Test Fixtures**: Shared setup for physics world, settings, sensor configs
- **Performance Regression**: Benchmark tests with timing assertions

## Acceptance Criteria

- [ ] C++ unit test coverage > 50% for AirLib (from current ~1%)
- [ ] Python test coverage > 70% for cosysairsim package
- [ ] All sensor types have noise model verification tests
- [ ] Physics engine has energy conservation tests
- [ ] RPC adaptor round-trip tests for every data type
- [ ] Settings parsing tests for valid, invalid, and edge-case configs
- [ ] Integration tests for RPC client-server communication
- [ ] Test fixtures reduce boilerplate and enable parallel execution
- [ ] CI runs tests on every push (Linux and Windows)
- [ ] Previously disabled tests (Pixhawk, WorkerThread) fixed or removed
- [ ] Coverage reports generated and trackable over time

## Risks

- Mocking UE classes for unit testing requires careful abstraction
- Integration tests need a running sim instance — containerize
- Test infrastructure setup is significant but pays off immediately

## Files Affected

- `AirLibUnitTests/` — major expansion
- `AirLibUnitTests/main.cpp` — enable all tests, add GoogleTest
- New: `AirLibUnitTests/PhysicsTests.cpp`
- New: `AirLibUnitTests/SensorTests.cpp`
- New: `AirLibUnitTests/SettingsTests.cpp`
- New: `AirLibUnitTests/ApiTests.cpp`
- New: `AirLibUnitTests/AdaptorTests.cpp`
- New: `PythonClient/tests/`
- New: `.github/workflows/test.yml`
- `cmake/AirLibUnitTests/CMakeLists.txt` — GoogleTest integration
