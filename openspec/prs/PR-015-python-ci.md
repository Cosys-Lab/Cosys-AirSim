# PR-015 Add Python CI workflow

status: backlog
priority: P0

## Objective
Establish reliable CI for Python packaging, linting, typing, and tests.

## Scope
- Add GitHub Actions workflow for Python matrix.
- Run formatter/linter, type checker, and tests.
- Validate wheel/sdist build.

## Acceptance Criteria
- CI status checks run on pull requests.
- Failures clearly identify lint/type/test/build stages.

## Risks
- Existing code quality debt may initially fail checks.

## Dependencies
- None.
