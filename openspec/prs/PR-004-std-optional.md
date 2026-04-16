# PR-004 Replace custom optional with std::optional

status: backlog
priority: P1

## Objective
Remove legacy optional polyfill and use standard C++17 `std::optional`.

## Scope
- Migrate include and type usage from custom optional header.
- Remove obsolete optional implementation file once unreferenced.

## Acceptance Criteria
- Build succeeds with `std::optional` only.
- No include path references custom optional header.

## Risks
- Namespace mismatches in template-heavy headers.

## Dependencies
- PR-006 (explicit C++ standard enforcement).
