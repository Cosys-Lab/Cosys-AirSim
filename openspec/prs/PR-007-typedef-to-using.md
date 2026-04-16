# PR-007 Replace typedef with using

status: backlog
priority: P2

## Objective
Improve readability and modern C++ style by replacing legacy `typedef` declarations.

## Scope
- Convert project-owned `typedef` declarations to `using` aliases.
- Keep API behavior unchanged.

## Acceptance Criteria
- No project-owned `typedef` remains in targeted modules.
- Build output unchanged functionally.

## Risks
- Large mechanical diff can increase merge conflicts.

## Dependencies
- None.
