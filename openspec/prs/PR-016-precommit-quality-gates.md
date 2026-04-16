# PR-016 Add pre-commit and formatting policy

status: backlog
priority: P1

## Objective
Prevent style drift by enforcing local and CI quality checks.

## Scope
- Add `.pre-commit-config.yaml`.
- Configure Python lint/format and C++ formatting checks.
- Document contributor setup.

## Acceptance Criteria
- `pre-commit run --all-files` produces stable results.
- CI validates hook-equivalent checks.

## Risks
- Initial formatting churn may create large diffs.

## Dependencies
- PR-015 recommended.
