# PR-006 Enforce C++ standard in CMake

status: backlog
priority: P1

## Objective
Ensure all compilers enforce the intended C++ standard consistently.

## Scope
- Add `CMAKE_CXX_STANDARD_REQUIRED ON`.
- Add `CMAKE_CXX_EXTENSIONS OFF`.
- Align comments/documentation with actual required standard.

## Acceptance Criteria
- CMake configure shows strict C++ standard settings.
- Build fails fast when compiler lacks required standard support.

## Risks
- Older local toolchains may stop compiling.

## Dependencies
- None.
