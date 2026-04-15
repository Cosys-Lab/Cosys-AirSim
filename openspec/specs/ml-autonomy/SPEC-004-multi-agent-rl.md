# SPEC-004: Multi-Agent Reinforcement Learning Framework

**Priority:** P1
**Category:** ML/Autonomy
**Effort:** Medium (3-4 weeks)
**Dependencies:** SPEC-001

## Problem Statement

Multi-vehicle examples exist (`multi_agent_drone.py`, `multi_agent_car.py`) but only support sequential/manual control:

- No Gymnasium-compatible multi-agent environment wrapper
- No independent agents with shared or independent observation/action spaces
- No communication mechanism between agents
- No centralized training / decentralized execution (CTDE) framework
- Scalability unclear beyond 2-3 agents
- No standard multi-agent benchmarks

## Proposed Solution

### 1. PettingZoo-Compatible Multi-Agent Environments

```python
from pettingzoo import ParallelEnv

class MultiDroneNavigation(ParallelEnv):
    """PettingZoo parallel environment for cooperative multi-drone navigation."""
    metadata = {"name": "cosysairsim_multidrone_v0"}

    def __init__(self, num_agents=3, task="formation", **kwargs):
        self.possible_agents = [f"drone_{i}" for i in range(num_agents)]
        ...

    def observation_space(self, agent) -> gymnasium.spaces.Space:
        ...

    def action_space(self, agent) -> gymnasium.spaces.Space:
        ...

    def reset(self, seed=None, options=None):
        ...

    def step(self, actions: dict[str, np.ndarray]):
        ...
```

### 2. Pre-Defined Multi-Agent Tasks

- **Formation Flying**: N drones maintain relative positions while navigating
- **Cooperative Search**: Agents search an area, sharing map coverage
- **Collision Avoidance**: Dense traffic scenarios with collision penalties
- **Pursuit-Evasion**: Adversarial multi-agent games
- **Collaborative Transport**: Multiple agents carrying a shared payload

### 3. Communication Framework

```python
class AgentCommunication:
    def broadcast(self, agent_id: str, message: np.ndarray):
        """Broadcast fixed-size message to all agents."""

    def send(self, from_agent: str, to_agent: str, message: np.ndarray):
        """Send message to specific agent."""

    def receive(self, agent_id: str) -> list[tuple[str, np.ndarray]]:
        """Receive all pending messages for agent."""
```

### 4. Scalability Mechanisms

- Agent observation sharing via shared memory
- Vectorized multi-agent environments
- Configurable observation radius (agents only observe nearby agents)
- Agent pooling for scenarios with many homogeneous agents

## Acceptance Criteria

- [ ] PettingZoo ParallelEnv interface implemented for at least 3 tasks
- [ ] Support for 2-16 agents without degraded frame rate
- [ ] Communication API with configurable bandwidth/range
- [ ] MAPPO training script demonstrating formation flying
- [ ] Heterogeneous agent support (mix drones + cars)
- [ ] Centralized critic with decentralized actors pattern example

## Files Affected

- New: `PythonClient/cosysairsim-gym/envs/multiagent/`
- New: `PythonClient/cosysairsim-gym/communication.py`
- `PythonClient/cosysairsim/client.py` — batch API calls for multi-vehicle efficiency
