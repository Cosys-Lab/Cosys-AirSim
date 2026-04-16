# PR-020 Upgrade docs stack to Material + mkdocstrings

status: backlog
priority: P2

## Objective
Improve docs usability and maintainability with modern MkDocs tooling.

## Scope
- Migrate to Material theme.
- Add mkdocstrings for Python API references.
- Add docs build dependency file for reproducibility.

## Acceptance Criteria
- `mkdocs build` succeeds with updated theme/plugins.
- Generated API reference pages are accessible in navigation.

## Risks
- Theme migration may require navigation tweaks.

## Dependencies
- PR-001 should land first.
