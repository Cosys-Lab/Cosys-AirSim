# SPEC-006: Standardized Benchmarking & Metrics Suite

**Priority:** P1
**Category:** ML/Autonomy
**Effort:** Medium (2-3 weeks)
**Dependencies:** SPEC-001

## Problem Statement

No standardized benchmarks exist for evaluating algorithms in Cosys-AirSim:

- `image_benchmarker.py` only measures FPS (lines 73-94)
- No task-specific metrics (success rate, trajectory error, collision count)
- No reference trajectories or performance baselines
- No reproducibility tools (experiment serialization, environment snapshots)
- No statistical significance testing
- No comparison with other simulators

## Proposed Solution

### 1. Benchmark Task Suite

```yaml
benchmarks:
  navigation:
    - point_to_point_easy      # 50m, no obstacles
    - point_to_point_hard      # 200m, dense obstacles
    - waypoint_sequence        # 10 waypoints, mixed heights
  perception:
    - object_detection_static  # 50 objects, varied conditions
    - object_detection_dynamic # Moving objects
    - depth_estimation         # Ground-truth depth comparison
    - lidar_mapping            # Map quality vs ground truth
  control:
    - hover_stability          # Position hold in wind
    - trajectory_tracking      # Reference trajectory RMSE
    - agile_maneuvers         # Aggressive flight gates
  safety:
    - collision_avoidance      # Dynamic obstacles
    - emergency_landing        # Motor failure recovery
    - geofence_compliance     # Boundary adherence
```

### 2. Metrics Framework

```python
class BenchmarkMetrics:
    # Navigation metrics
    success_rate: float             # % episodes reaching goal
    trajectory_rmse: float          # m, vs reference path
    time_to_goal: float             # seconds
    path_efficiency: float          # optimal_length / actual_length
    smoothness: float               # jerk integral

    # Safety metrics
    collision_count: int
    min_obstacle_distance: float    # m
    geofence_violations: int

    # Control metrics
    position_hold_rmse: float       # m, during hover
    velocity_tracking_rmse: float   # m/s
    attitude_tracking_rmse: float   # rad

    # Perception metrics
    mAP: float                      # detection accuracy
    depth_abs_rel: float            # |d-d*|/d*
    map_completeness: float         # % of ground truth covered
    map_accuracy: float             # chamfer distance to GT
```

### 3. Leaderboard & Reporting

```python
class BenchmarkRunner:
    def run(self, agent, benchmark, n_episodes=100, seed=42) -> BenchmarkReport:
        ...

    def compare(self, reports: list[BenchmarkReport]) -> ComparisonTable:
        ...

    def export_report(self, report, format="html"):
        """Generate visual report with plots and tables."""
```

## Acceptance Criteria

- [ ] At least 10 benchmark tasks across navigation, perception, control, safety
- [ ] Deterministic benchmark environments with fixed seeds
- [ ] Metrics framework with automatic computation
- [ ] Statistical testing (confidence intervals, paired t-tests)
- [ ] HTML report generation with plots
- [ ] Baseline results for random and scripted agents
- [ ] Reproducibility manifest (env hash, settings, code version)

## Files Affected

- New: `PythonClient/cosysairsim/benchmarks/`
- New: `PythonClient/cosysairsim/metrics.py`
- New: `benchmarks/` (task definitions and baselines)
