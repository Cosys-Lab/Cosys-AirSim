# PR-009 Generic sensor lookup in VehicleApiBase

status: backlog
priority: P1

## Objective
Eliminate duplicated sensor lookup logic via a generic helper.

## Scope
- Introduce template/helper for lookup by name and sensor type.
- Refactor repetitive methods to use helper.

## Acceptance Criteria
- Repeated lookup blocks removed from `VehicleApiBase.hpp`.
- Behavior and error messages remain equivalent.

## Risks
- Template errors can impact compile diagnostics.

## Dependencies
- None.
