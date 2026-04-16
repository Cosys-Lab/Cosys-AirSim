# PR-008 Apply make_unique and make_shared

status: backlog
priority: P1

## Objective
Improve exception safety and clarity by replacing direct `new` in smart pointer construction.

## Scope
- Replace `shared_ptr<T>(new T(...))` with `std::make_shared<T>(...)`.
- Replace reset/new patterns with `std::make_unique<T>(...)` where appropriate.

## Acceptance Criteria
- No direct `new` inside smart pointer factory contexts in targeted files.
- Existing tests/build pass.

## Risks
- Rare constructor visibility edge cases.

## Dependencies
- None.
