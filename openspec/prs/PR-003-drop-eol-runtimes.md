# PR-003 Drop EOL Python and Ubuntu runner

status: backlog
priority: P0

## Objective
Raise baseline runtime versions to currently supported platforms.

## Scope
- Set `requires-python >=3.9` in Python packaging metadata.
- Remove unsupported `ubuntu-20.04` from CI matrix.
- Update README/doc references.

## Acceptance Criteria
- Metadata and CI reflect supported versions only.
- CI passes on maintained runner images.

## Risks
- Users pinned to old environments need migration guidance.

## Dependencies
- None.
