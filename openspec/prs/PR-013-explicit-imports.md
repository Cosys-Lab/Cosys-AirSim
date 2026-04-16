# PR-013 Replace wildcard imports and define __all__

status: backlog
priority: P1

## Objective
Define explicit Python API boundaries and reduce namespace pollution.

## Scope
- Remove wildcard imports in package modules.
- Define explicit exports via `__all__`.

## Acceptance Criteria
- No wildcard imports in `cosysairsim` package.
- Public import surface remains documented and stable.

## Risks
- Downstream code may rely on accidental exports.

## Dependencies
- PR-012 recommended.
