# OpenSpec PR Planning

This folder tracks modernization work as specification-first PRs.

## Conventions

- IDs: `PR-001` ... `PR-020`
- Status values: `backlog`, `ready`, `in_progress`, `blocked`, `review`, `done`
- Priority values: `P0`, `P1`, `P2`
- Each spec file contains: objective, scope, acceptance criteria, risks, dependencies

## Workflow

1. Move item from `backlog` to `ready` in `tracker.yaml`.
2. Open a git branch: `pr/<id>-<slug>`.
3. Implement only the scope in the corresponding spec.
4. Update status to `review` with PR URL.
5. Mark as `done` after merge.
