# PR-012 Add Python type annotations to public API

status: backlog
priority: P1

## Objective
Increase developer ergonomics and safety with first-class type hints.

## Scope
- Add type hints to key public methods in `client.py`.
- Type annotate core objects in `types.py` and selected utilities.
- Add `py.typed` marker for distributed typing.

## Acceptance Criteria
- Type checker runs in CI with baseline pass.
- Public API signatures include meaningful type hints.

## Risks
- Initial annotation debt may require incremental rollout.

## Dependencies
- PR-015.
