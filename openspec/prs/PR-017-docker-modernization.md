# PR-017 Modernize Dockerfiles

status: backlog
priority: P1

## Objective
Improve reproducibility, image size, and security posture of Docker images.

## Scope
- Add multi-stage build where appropriate.
- Clean apt caches and reduce layers.
- Add `.dockerignore` and image metadata labels.

## Acceptance Criteria
- Docker builds remain functional.
- Image size and vulnerability scan baseline improve.

## Risks
- Build cache behavior changes can affect local workflows.

## Dependencies
- None.
