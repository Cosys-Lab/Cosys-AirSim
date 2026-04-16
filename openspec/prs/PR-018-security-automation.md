# PR-018 Add Dependabot and CodeQL

status: backlog
priority: P1

## Objective
Automate dependency updates and static security scanning.

## Scope
- Add Dependabot config for GitHub Actions, Python, Docker.
- Add CodeQL workflow for C++ and Python.

## Acceptance Criteria
- Dependabot opens update PRs on schedule.
- CodeQL scan runs successfully on pull requests.

## Risks
- False positives may require baseline triage.

## Dependencies
- None.
