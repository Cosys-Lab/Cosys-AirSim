# PR-019 Clean ROS2 package metadata and CMake

status: backlog
priority: P1

## Objective
Align ROS2 packages with modern ament practices and remove stale metadata.

## Scope
- Remove ROS1-era dependencies from ROS2 package manifests.
- Deduplicate package.xml entries and align versioning.
- Remove hardcoded architecture-specific include/library paths.

## Acceptance Criteria
- `colcon build` succeeds in supported ROS2 distro.
- Package manifests pass linting without duplicate/stale entries.

## Risks
- ROS distribution-specific behavior differences.

## Dependencies
- None.
