# Cosys-AirSim Drastic Improvement Roadmap

## Overview

26 specifications across 5 expert domains, organized into 4 implementation waves.
Total estimated effort: 80-120 weeks of engineering work (parallelizable).

---

## Wave 0: Quick Wins & Foundation (Weeks 1-4)

*Highest ROI items that unblock everything else.*

| ID | Title | Category | Effort | Why First |
|---|---|---|---|---|
| SPEC-025 | FiducialBeacon Refactor | Engineering | Small | 7,090→500 lines, pure refactor, immediate quality win |
| SPEC-028 | Technical Debt Cleanup | Engineering | Medium | Removes dead code, ancient workarounds, establishes baseline |
| SPEC-005 | Training Pipeline Modernization | ML | Medium | Fixes deprecated TF1/Keras2, np.fromstring — blocking all ML work |
| SPEC-022 | Fixed Physics Timestep | Performance | Medium | Deterministic simulation, foundation for physics improvements |

## Wave 1: Core Infrastructure (Weeks 3-10)

*Build the architectural foundation that other specs depend on.*

| ID | Title | Category | Effort | Dependencies |
|---|---|---|---|---|
| SPEC-023 | Comprehensive Test Suite | Engineering | Large | None — needed before any refactoring |
| SPEC-024 | Error Handling Overhaul | Engineering | Medium | None |
| SPEC-029 | Plugin Registry Architecture | Architecture | Large | None |
| SPEC-009 | Advanced Aerodynamics | Physics | Large | None |
| SPEC-012 | Collision System Overhaul | Physics | Medium | None |
| SPEC-017 | Zero-Copy Sensor Pipeline | Performance | Large | SPEC-029 |

## Wave 2: Feature Expansion (Weeks 8-18)

*Major new capabilities building on Wave 1 infrastructure.*

| ID | Title | Category | Effort | Dependencies |
|---|---|---|---|---|
| SPEC-001 | Modern RL Framework | ML | Large | SPEC-002, SPEC-005 |
| SPEC-002 | Sensor Fusion Pipeline | ML | Medium | None |
| SPEC-003 | Domain Randomization API | ML | Medium | None |
| SPEC-010 | Wind & Turbulence Model | Physics | Medium | SPEC-009 |
| SPEC-011 | Tire Physics Model | Physics | Medium | None |
| SPEC-013 | Material Interaction System | Physics | Medium | None |
| SPEC-014 | Sensor Noise Upgrade | Physics | Medium | None |
| SPEC-018 | Render Target Pooling | Performance | Medium | None |
| SPEC-019 | Multi-Vehicle Scalability | Performance | Large | SPEC-017, SPEC-018 |
| SPEC-021 | RPC Optimization | Performance | Medium | None |
| SPEC-026 | API Safety | Engineering | Medium | None |
| SPEC-027 | Settings Schema Validation | Engineering | Medium | None |
| SPEC-030 | Communication Abstraction | Architecture | Large | None |
| SPEC-031 | Event Bus System | Architecture | Medium | None |
| SPEC-032 | Sensor Pipeline Architecture | Architecture | Medium | SPEC-029 |
| SPEC-034 | Settings Decomposition | Architecture | Medium | SPEC-027 |

## Wave 3: Advanced Capabilities (Weeks 16-26)

*Cutting-edge features for research leadership.*

| ID | Title | Category | Effort | Dependencies |
|---|---|---|---|---|
| SPEC-004 | Multi-Agent RL | ML | Medium | SPEC-001 |
| SPEC-006 | Benchmarking Suite | ML | Medium | SPEC-001 |
| SPEC-007 | Dataset Export Formats | ML | Small | SPEC-002 |
| SPEC-008 | MLOps Integration | ML | Small | SPEC-001, SPEC-005 |
| SPEC-015 | Deformable Terrain | Physics | Large | SPEC-012 |
| SPEC-016 | Underwater Simulation | Physics | Large | SPEC-009, SPEC-010 |
| SPEC-020 | Async GPU Readback | Performance | Medium | SPEC-018 |
| SPEC-033 | State Management | Architecture | Medium | SPEC-031 |

---

## Priority Distribution

```
P0 (Critical):   11 specs — Must-have for competitive simulator
P1 (Important):  12 specs — Significant value, enables research
P2 (Desirable):   3 specs — Advanced capabilities, future-proofing
```

## Category Distribution

```
ML/Autonomy:      8 specs (SPEC-001 to SPEC-008)
Physics:           8 specs (SPEC-009 to SPEC-016)
Performance:       6 specs (SPEC-017 to SPEC-022)
Engineering:       6 specs (SPEC-023 to SPEC-028)
Architecture:      6 specs (SPEC-029 to SPEC-034)
```

## Impact Matrix

| Improvement Area | Before | After |
|---|---|---|
| RL algorithms | DQN only (2015) | PPO, SAC, TD3, TRPO, DreamerV3 |
| Action spaces | 7 discrete actions | Continuous control with normalization |
| Sensor fusion | Manual, per-project | Built-in pipeline with sync & calibration |
| Rotor physics | Linear thrust mapping | BET with motor dynamics, ground effect |
| Wind model | Static constant vector | Dryden turbulence, wind shear, gusts |
| Tire physics | UE default only | Pacejka Magic Formula with load transfer |
| Collision | Single-point, 0.9x angular hack | Material-based, multi-point, rolling resistance |
| Image capture | 4 copies, blocking readback | Zero-copy, async, pooled |
| Multi-vehicle | Linear degradation | Batch API, LOD, parallel sensors |
| Test coverage | ~1% (460 lines) | >50% C++, >70% Python |
| Error handling | No try-catch in RPC client | Result types, reconnection, resilience |
| Extensibility | Modify core code | Plugin registry, extension modules |
| Communication | msgpack-RPC only | ITransport with gRPC, shared memory |
| Configuration | 1,909-line god header | 6+ domain-specific validated modules |
| API versioning | Integer "4" | Semantic MAJOR.MINOR.PATCH |

---

## Dependency Graph (Simplified)

```
SPEC-025 (Beacon Refactor)     ──→ standalone
SPEC-028 (Debt Cleanup)        ──→ standalone
SPEC-023 (Test Suite)          ──→ standalone, enables all refactoring
SPEC-024 (Error Handling)      ──→ standalone

SPEC-029 (Plugin Registry)    ──→ SPEC-017, SPEC-032
SPEC-027 (Schema Validation)  ──→ SPEC-034
SPEC-009 (Aerodynamics)       ──→ SPEC-010, SPEC-016
SPEC-012 (Collision)           ──→ SPEC-015
SPEC-005 (Training Pipeline)  ──→ SPEC-001, SPEC-008
SPEC-002 (Sensor Fusion)      ──→ SPEC-001, SPEC-007
SPEC-001 (RL Framework)       ──→ SPEC-004, SPEC-006, SPEC-008
SPEC-017 (Zero-Copy)          ──→ SPEC-019
SPEC-018 (Render Pool)        ──→ SPEC-019, SPEC-020
SPEC-031 (Event Bus)          ──→ SPEC-033
```
