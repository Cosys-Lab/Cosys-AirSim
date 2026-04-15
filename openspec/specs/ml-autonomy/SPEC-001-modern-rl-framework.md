# SPEC-001: Modern Reinforcement Learning Framework

**Priority:** P0
**Category:** ML/Autonomy
**Effort:** Large (4-6 weeks)
**Dependencies:** SPEC-002, SPEC-005

## Problem Statement

The current RL infrastructure is severely outdated and limited:

- **Only DQN** implemented (`PythonClient/reinforcement_learning/dqn_drone.py`), which is a 2015-era algorithm unsuitable for continuous control
- **Discrete action spaces only** (7 actions for drone, 6 for car) — real autonomous systems need continuous control
- **Observation space limited to depth images** (84x84x1) — no multi-modal sensor fusion
- **Hardcoded reward functions** with static waypoints (`drone_env.py:86-135`)
- **No curriculum learning**, no reward shaping utilities, no episodic memory
- Relies on `stable-baselines3` with no multi-algorithm framework

## Proposed Solution

### 1. Gymnasium-Compatible Environment Suite

Replace the outdated `airgym` with a modern `cosysairsim-gym` package:

```
cosysairsim-gym/
  envs/
    base.py                  # CosysAirSimEnv(gymnasium.Env) base
    multirotor/
      navigation.py          # Point-to-point navigation
      tracking.py            # Target tracking
      racing.py              # Drone racing gates
      inspection.py          # Infrastructure inspection
    car/
      lane_following.py       # Lane-keeping task
      parking.py             # Autonomous parking
      obstacle_avoidance.py  # Dynamic obstacle avoidance
    skidsteer/
      exploration.py         # Unknown environment exploration
  wrappers/
    observation.py           # Multi-modal observation stacking
    action.py                # Action normalization, clipping
    reward.py                # Reward shaping, curriculum
    recording.py             # Episode recording for replay
  rewards/
    registry.py              # Reward function registry
    composite.py             # Weighted composite rewards
    curiosity.py             # Intrinsic curiosity module
    curriculum.py            # Progressive difficulty
```

### 2. Continuous Action Spaces

```python
# Multirotor continuous control
self.action_space = spaces.Box(
    low=np.array([-1, -1, -1, -1]),   # roll_rate, pitch_rate, yaw_rate, thrust
    high=np.array([1, 1, 1, 1]),
    dtype=np.float32
)

# Car continuous control
self.action_space = spaces.Box(
    low=np.array([-1, -1]),   # steering, throttle_brake
    high=np.array([1, 1]),
    dtype=np.float32
)
```

### 3. Multi-Modal Observation Spaces

```python
self.observation_space = spaces.Dict({
    "rgb": spaces.Box(0, 255, (H, W, 3), np.uint8),
    "depth": spaces.Box(0, 100.0, (H, W, 1), np.float32),
    "lidar_bev": spaces.Box(-1, 1, (BEV_H, BEV_W, 1), np.float32),
    "imu": spaces.Box(-np.inf, np.inf, (6,), np.float32),
    "velocity": spaces.Box(-np.inf, np.inf, (3,), np.float32),
    "goal_relative": spaces.Box(-np.inf, np.inf, (3,), np.float32),
})
```

### 4. Reward Function Registry

```python
@reward_registry.register("navigation_v1")
class NavigationReward(RewardFunction):
    def __init__(self, distance_weight=1.0, collision_penalty=-100.0,
                 progress_weight=5.0, time_penalty=-0.01):
        ...
    def compute(self, state, action, next_state, info) -> float:
        ...
```

### 5. Algorithm-Agnostic Training Scripts

Support PPO, SAC, TD3, TRPO, DDPG, DreamerV3 out-of-box via both stable-baselines3 and CleanRL backends.

## Acceptance Criteria

- [ ] `cosysairsim-gym` package installable via pip with `gymnasium>=1.0` dependency
- [ ] At least 3 multirotor + 2 car + 1 skidsteer pre-defined tasks with continuous action spaces
- [ ] Multi-modal observation wrapper combining camera, LiDAR, IMU data
- [ ] Reward function registry with at least 5 built-in reward functions
- [ ] Training scripts for PPO and SAC with documented hyperparameters
- [ ] Vectorized environments (`gymnasium.vector.AsyncVectorEnv`) for parallel training
- [ ] Episode recording and replay capability
- [ ] Benchmark results on at least 2 tasks showing convergence

## Risks

- API latency may bottleneck training loop — mitigate with vectorized envs and async data collection
- Gymnasium API may evolve — pin version and provide compatibility layer
- Multi-modal observations increase memory — implement lazy loading and frame stacking

## Files Affected

- `PythonClient/reinforcement_learning/` — complete rewrite
- `PythonClient/cosysairsim/client.py` — add synchronized sensor reads
- New package: `PythonClient/cosysairsim-gym/`
