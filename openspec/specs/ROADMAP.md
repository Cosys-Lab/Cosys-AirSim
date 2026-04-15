# Cosys-AirSim Drastic Improvement Roadmap (Second Pass)

## Overview

51 specifications across 7 domains, organized into 8 implementation waves.
Each wave produces focused, reviewable PRs designed for fast iteration.

## Changes from First Pass

- **Fixed dependencies**: SPEC-022 (fixed timestep) now prerequisite for physics work;
  SPEC-028 (debt cleanup) prerequisite for SPEC-023 (test suite); removed circular
  dependency between SPEC-029 and SPEC-030
- **Added character system**: 9 new specs (SPEC-035 to SPEC-043) covering human characters,
  locomotion, animation, vehicle interaction, hiding, motorbikes, crowd simulation
- **Added Cosys-specific features**: 8 new specs (SPEC-044 to SPEC-051) covering annotation
  batching, external sensors, recording, detection, procedural generation, lighting, physics
  queries, ROS2 coverage
- **Upgraded priorities**: SPEC-022 and SPEC-028 promoted to P0

---

## Wave 0: Prereqs & Quick Wins (Weeks 1-3)

*Fix broken things first. Each PR is < 1 week, independently reviewable.*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-028 | Technical Debt & Dead Code Cleanup | Medium | Remove UE4.17 workaround, fix disabled tests, remove dead code |
| SPEC-025 | FiducialBeacon Refactor | Small | 7,090 -> ~500 lines, data-driven lookup table |
| SPEC-005 | Training Pipeline Modernization | Medium | Kill TF1/Keras2, fix np.fromstring, PyTorch examples |
| SPEC-022 | Fixed Physics Timestep | Medium | Decouple physics from render, deterministic sim |

## Wave 1: Engineering Foundation (Weeks 3-8)

*Tests and error handling before any major refactoring.*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-023 | Comprehensive Test Suite | Large | GoogleTest + pytest setup, physics/sensor/API tests |
| SPEC-024 | Error Handling Overhaul | Medium | RPC try-catch, reconnection, Result types |
| SPEC-027 | Settings Schema Validation | Medium | JSON schema, range validation, NaN cleanup |
| SPEC-029 | Plugin Registry Architecture | Large | Vehicle + sensor type registry macros |
| SPEC-044 | Annotation Batch Operations | Small | Batch set, query, runtime layer management |
| SPEC-045 | External Sensor API | Small | List, pose, add/remove external sensors |

## Wave 2: Character System (Weeks 6-14)

*The major new feature. Each spec is a focused, reviewable PR.*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-035 | Character Pawn Foundation | Medium | ACharacter pawn, CharacterApiBase, settings, Python client |
| SPEC-036 | Character Locomotion | Medium | Walk/run/sprint/crouch/crawl/roll/jump |
| SPEC-037 | Character Animations | Medium | AnimBP, 20+ built-in anims, montage API, bone access |
| SPEC-038 | Mixed SimMode | Medium | Universal SimMode supporting all vehicle types together |
| SPEC-043 | Character Detection & Annotation | Small | Segmentation, 2D/3D skeleton GT, activity labels |
| SPEC-039 | Vehicle Interaction (Enter/Exit) | Medium | Seat system, mount/dismount animations, driver mode |
| SPEC-041 | Motorbike Vehicle Type | Medium | Two-wheeled physics, lean turning, rider/pillion seats |
| SPEC-042 | Autonomous Pedestrian AI | Medium | Crowd spawning, social force, patrol/wander/flee |
| SPEC-040 | Character Hiding & Cover | Small | Cover points, peek animations, visibility query |

## Wave 3: Physics Improvements (Weeks 8-16)

*Now safe with fixed timestep (SPEC-022) and test suite (SPEC-023).*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-009 | Advanced Aerodynamics | Large | Motor dynamics, BET thrust, gyroscopic, ground effect |
| SPEC-012 | Collision System Overhaul | Medium | Material DB, rolling resistance, slope sliding |
| SPEC-010 | Wind & Turbulence | Medium | Dryden model, wind shear, gusts, API |
| SPEC-014 | Sensor Noise Upgrade | Medium | IMU scale/quantization, GPS GDOP, LiDAR nonlinear |
| SPEC-011 | Pacejka Tire Model | Medium | MF-Tire lateral/longitudinal, load transfer |
| SPEC-013 | Material Interaction | Medium | Fresnel BRDF, atmospheric attenuation, echo physics |

## Wave 4: Performance (Weeks 10-16)

*Optimize data paths and multi-vehicle scenarios.*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-017 | Zero-Copy Sensor Pipeline | Large | Shared mem, SIMD pixels, move semantics, np.frombuffer |
| SPEC-021 | RPC Optimization | Medium | Image compression, vector reserve(), batch requests |
| SPEC-018 | Render Target Pooling | Medium | RT pool, RenderResult recycling, GPU memory budget |
| SPEC-020 | Async GPU Readback | Medium | Triple-buffered readback, compute shader LiDAR |
| SPEC-019 | Multi-Vehicle Scalability | Large | Batch API, parallel sensors, LOD, WorkerThread fix |

## Wave 5: ML/Autonomy (Weeks 12-20)

*Modern RL and data infrastructure.*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-002 | Sensor Fusion Pipeline | Medium | Synchronized capture, calibration, point cloud utils |
| SPEC-001 | Modern RL Framework | Large | Gymnasium envs, continuous actions, reward registry |
| SPEC-003 | Domain Randomization API | Medium | Weather/lighting/physics/sensor randomization |
| SPEC-007 | Dataset Export (KITTI/COCO/PLY) | Small | KITTI, COCO, PLY exporters with manifests |
| SPEC-006 | Benchmarking Suite | Medium | 10+ tasks, metrics, statistical testing, baselines |
| SPEC-004 | Multi-Agent RL | Medium | PettingZoo envs, MAPPO, formation flying |

## Wave 6: Cosys-Specific Features (Weeks 14-20)

*Strengthen unique differentiators.*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-046 | Recording & Playback | Medium | Selective recording, HDF5/rosbag, playback/seek |
| SPEC-047 | Detection Pipeline | Small | Class filtering, temporal smoothing, COCO/YOLO export |
| SPEC-048 | Procedural Generation API | Medium | Seed control, density, world regen from Python |
| SPEC-049 | Lighting API | Small | Full parameter control, runtime light creation |
| SPEC-050 | Physics Query API | Small | Raycast, sweep, terrain height, navigability |
| SPEC-051 | ROS2 Complete API Coverage | Medium | Character topics, annotation services, recording |

## Wave 7: Architecture & Advanced (Weeks 18-26+)

*Longer-term architectural improvements and advanced simulation.*

| ID | Title | Effort | PR Scope |
|---|---|---|---|
| SPEC-026 | API Safety | Medium | Remove void*, domain interfaces, semantic versioning |
| SPEC-034 | Settings Decomposition | Medium | Split 1,909-line header into 6+ modules |
| SPEC-030 | Communication Abstraction | Large | ITransport interface, gRPC/shared-memory transports |
| SPEC-031 | Event Bus System | Medium | Type-safe pub/sub, lifecycle/sensor/API events |
| SPEC-032 | Sensor Pipeline Architecture | Medium | Processing stages, replay, failure injection |
| SPEC-033 | State Management | Medium | Centralized state, save/load, rollback |
| SPEC-008 | MLOps Integration | Small | W&B/TensorBoard, model validation, stress testing |
| SPEC-015 | Deformable Terrain | Large | Bekker-Wong terramechanics, track visualization |
| SPEC-016 | Underwater Simulation | Large | Buoyancy, hydrodynamics, sonar, underwater camera |

---

## Priority Distribution

```
P0 (Critical):   17 specs — Must-have for competitive simulator
P1 (Important):  17 specs — Significant research value
P2 (Desirable):   3 specs — Advanced capabilities, future-proofing
```

## Category Distribution

```
ML/Autonomy:        8 specs (SPEC-001 to SPEC-008)
Physics:            8 specs (SPEC-009 to SPEC-016)
Performance:        6 specs (SPEC-017 to SPEC-022)
Engineering:        6 specs (SPEC-023 to SPEC-028)
Architecture:       6 specs (SPEC-029 to SPEC-034)
Characters:         9 specs (SPEC-035 to SPEC-043)  [NEW]
Cosys Features:     8 specs (SPEC-044 to SPEC-051)  [NEW]
```

## Corrected Dependency Graph

```
Wave 0 (no deps, start immediately):
  SPEC-028 ──► SPEC-023 (clean code before tests)
  SPEC-025 ──► standalone
  SPEC-005 ──► SPEC-001 (training pipeline before RL)
  SPEC-022 ──► SPEC-009, SPEC-010 (fixed timestep before physics)

Wave 1:
  SPEC-023 ──► enables safe refactoring across all waves
  SPEC-029 ──► standalone (registry enables extensibility)
  SPEC-027 ──► SPEC-034 (schema before decomposition)

Wave 2 (character chain):
  SPEC-035 ──► SPEC-036 ──► SPEC-037
  SPEC-035 ──► SPEC-038 ──► SPEC-039, SPEC-041
  SPEC-035 + SPEC-037 ──► SPEC-043
  SPEC-035 + SPEC-036 ──► SPEC-042
  SPEC-036 + SPEC-037 ──► SPEC-040

Wave 3-4 (independent streams, can parallel):
  Physics: SPEC-009 → SPEC-010, SPEC-012, SPEC-014, SPEC-011, SPEC-013
  Perf:    SPEC-017, SPEC-021, SPEC-018 → SPEC-020 → SPEC-019
```

## PR Review Guidelines

Each spec is scoped to produce **one focused PR**:

1. **Max 1,500 lines changed** per PR (split larger specs across PRs)
2. **One concern per PR** (don't mix physics and UI changes)
3. **Tests included** in the same PR as the feature
4. **Backward compatible** unless explicitly noted
5. **Self-contained** — PR can be reviewed without reading other PRs
6. **< 2 weeks** to implement — if longer, split further

For specs marked "Large", split into sub-PRs:
- SPEC-009: PR-A (motor dynamics), PR-B (gyroscopic/ground effect)
- SPEC-017: PR-A (SIMD pixels + np.frombuffer), PR-B (shared memory), PR-C (move semantics)
- SPEC-019: PR-A (batch API), PR-B (parallel sensors), PR-C (LOD + queue fix)
- SPEC-029: PR-A (vehicle registry), PR-B (sensor registry), PR-C (extension loading)
- SPEC-030: PR-A (ITransport interface), PR-B (gRPC), PR-C (shared memory transport)
