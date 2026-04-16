# PR-001 Remove polyfill.io from MkDocs

status: backlog
priority: P0

## Objective
Remove `polyfill.io` script usage from docs configuration to eliminate a known supply chain risk.

## Scope
- Update `mkdocs.yml` to remove external `polyfill.io` script.
- Ensure docs build still succeeds.

## Acceptance Criteria
- No `polyfill.io` reference exists in repository.
- `mkdocs build` completes.

## Risks
- Minor rendering differences in legacy browsers.

## Dependencies
- None.
