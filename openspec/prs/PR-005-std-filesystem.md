# PR-005 Migrate FileSystem to std::filesystem

status: backlog
priority: P1

## Objective
Modernize path and file utilities by adopting `std::filesystem`.

## Scope
- Replace custom path manipulation helpers where feasible.
- Remove deprecated codecvt conversion paths.

## Acceptance Criteria
- Filesystem operations compile and run on Linux/macOS/Windows.
- No deprecated `codecvt` usage in migrated paths.

## Risks
- Cross-platform path semantics can differ.

## Dependencies
- PR-006.
