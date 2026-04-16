# PR-014 Remove mutable Python defaults

status: backlog
priority: P1

## Objective
Fix Python object initialization bugs caused by shared mutable defaults.

## Scope
- Replace mutable function defaults with `None` + local initialization.
- Replace class-level mutable default fields with instance-safe initialization.

## Acceptance Criteria
- No mutable default args remain in package modules.
- Unit tests validate independent instance state.

## Risks
- Serialization behavior may reveal latent assumptions.

## Dependencies
- PR-015.
