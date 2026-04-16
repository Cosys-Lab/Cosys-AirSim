# Modernization Roadmap

## Wave 1 (P0 stability and security)

- PR-001 Remove polyfill.io from MkDocs
- PR-002 Replace deprecated NumPy APIs
- PR-003 Drop EOL Python and Ubuntu runner
- PR-015 Add Python CI workflow

## Wave 2 (core code modernization)

- PR-006 Enforce C++ standard in CMake
- PR-004 Replace custom optional with std::optional
- PR-005 Migrate FileSystem to std::filesystem
- PR-008 Apply make_unique and make_shared
- PR-009 Generic sensor lookup in VehicleApiBase
- PR-010 Split AirSimSettings god header

## Wave 3 (quality + ecosystem)

- PR-012 Add Python type annotations to public API
- PR-013 Replace wildcard imports and define __all__
- PR-014 Remove mutable Python defaults
- PR-016 Add pre-commit and formatting policy
- PR-017 Modernize Dockerfiles
- PR-018 Add Dependabot and CodeQL
- PR-019 Clean ROS2 package metadata and CMake
- PR-020 Upgrade docs stack to Material + mkdocstrings
- PR-007 Replace typedef with using
- PR-011 Adopt C++17 utility features
