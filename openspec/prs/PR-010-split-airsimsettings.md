# PR-010 Split AirSimSettings god header

status: backlog
priority: P1

## Objective
Reduce compile-time and coupling by splitting monolithic settings header.

## Scope
- Extract settings data structures into dedicated header.
- Move parsing-heavy implementation to source file(s) where possible.
- Keep public API stable.

## Acceptance Criteria
- `AirSimSettings.hpp` reduced substantially in size.
- Project builds without include cycles or behavior changes.

## Risks
- Include-order regressions.

## Dependencies
- PR-006 recommended.
