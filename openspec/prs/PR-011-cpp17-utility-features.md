# PR-011 Adopt C++17 utility features

status: backlog
priority: P2

## Objective
Use standard C++17 language/library features for cleaner utility code.

## Scope
- Replace custom clamp helper with `std::clamp`.
- Add structured bindings where map iteration is verbose.
- Add `[[nodiscard]]` and `[[maybe_unused]]` in selected APIs.

## Acceptance Criteria
- Utility modules compile with reduced custom helper footprint.
- No behavior change in covered code paths.

## Risks
- Attribute changes may expose pre-existing warnings.

## Dependencies
- PR-006.
