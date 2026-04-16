# PR-002 Replace deprecated NumPy APIs

status: backlog
priority: P0

## Objective
Replace deprecated or removed NumPy APIs to keep Python client compatible with modern NumPy.

## Scope
- Replace `np.fromstring()` with `np.frombuffer()`.
- Replace `np.sctypes` checks with robust `isinstance` checks.

## Acceptance Criteria
- No runtime use of `np.fromstring` or `np.sctypes` in Python client.
- Python smoke examples run on NumPy 2.x.

## Risks
- Binary payload decoding regressions if dtype/shape handling changes.

## Dependencies
- Coordinate with PR-012 for typing updates.
