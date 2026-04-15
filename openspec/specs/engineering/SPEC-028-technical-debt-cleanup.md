# SPEC-028: Technical Debt & Dead Code Cleanup

**Priority:** P1
**Category:** Engineering Quality
**Effort:** Medium (2-3 weeks)
**Dependencies:** None

## Problem Statement

Accumulated technical debt across the codebase:

### Dead Code
1. **Disabled tests** (`AirLibUnitTests/main.cpp:18-20`): PixhawkTest and WorkerThreadTest commented out — fix or remove
2. **Commented code** in `FiducialBeacon.cpp:40-46`: Dead if-block mixed with working code
3. **Unimplemented stubs** (`RpcLibClientBase.cpp:194-207`): 3 methods return empty objects with no TODO or warning

### Ancient Workarounds
4. **rpclib bug** (`RpcLibAdaptorsBase.hpp:672-674`): 4 instances of workaround for rpclib issue #152 — check if fixed upstream
5. **Unreal 4.17** (`AirSimSettings.hpp:676`): "Remove this workaround after we only support Unreal 4.17" — we're on UE 5.5
6. **Linux crash** (`AirSimSettings.hpp:676`): saveJSonString crash workaround — verify if still needed

### Duplicate Comments
7. **4x repeated TODO** (`AirSimSettings.hpp:834,840,850,857`): "we should be selecting remote if available else keyboard" copied 4 times

### Build System Issues
8. **CMake GLOB_RECURSE** (`cmake/AirLib/CMakeLists.txt`): Anti-pattern for build systems — new files not detected without re-running cmake
9. **Suppressed warnings** (`AirSim.Build.cs:88`): `_SCL_SECURE_NO_WARNINGS=1` — investigate and fix actual warnings
10. **STRICT_MODE macros** throughout code — indicates compiler portability issues

### Dependency Issues
11. **MavLinkMessages.hpp**: 11,090 lines in single file — likely auto-generated, verify freshness
12. **rpclib vendored** at 2.3.1 — check for security updates
13. **No version manifests** for any dependency

## Proposed Solution

### 1. Audit & Action Plan

For each item, categorize as:
- **DELETE**: Dead code, obsolete workarounds confirmed unnecessary
- **FIX**: Broken features, workarounds for bugs that have been fixed
- **DOCUMENT**: Workarounds still needed, with explanation of why
- **UPGRADE**: Dependencies needing updates

### 2. Priority Actions

```
DELETE:
  - FiducialBeacon.cpp:40-46 (commented code)
  - AirSimSettings.hpp:676 (UE 4.17 workaround - verify first)
  - AirSimSettings.hpp:834,840,850,857 (duplicate comments)
  - AirSimSettings.hpp:773-781 (typo workaround)

FIX:
  - AirLibUnitTests/main.cpp:18-20 (either fix tests or delete them)
  - RpcLibClientBase.cpp:194-207 (throw NotImplemented or implement)
  - CMake GLOB_RECURSE → explicit file lists

DOCUMENT:
  - rpclib issue #152 workarounds (if still needed)

UPGRADE:
  - rpclib 2.3.1 → check for newer version
  - Investigate warning suppression macros
```

### 3. Dependency Manifest

Create `DEPENDENCIES.md`:
```markdown
| Dependency | Version | Location | License | Last Checked |
|-----------|---------|----------|---------|-------------|
| rpclib | 2.3.1 | external/rpclib/ | MIT | 2024-XX-XX |
| Eigen | 3.x | AirLib/deps/ | MPL2 | 2024-XX-XX |
| MavLink | X.X | AirLib/deps/MavLinkCom/ | MIT | 2024-XX-XX |
| msgpack-c | X.X | via rpclib | Boost | 2024-XX-XX |
```

### 4. Compiler Warning Cleanup

```cmake
# Replace warning suppression with actual fixes
# Remove: _SCL_SECURE_NO_WARNINGS=1
# Add proper warning flags
target_compile_options(AirLib PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)
```

## Acceptance Criteria

- [ ] All dead/commented code removed or restored with justification
- [ ] Ancient workarounds (UE 4.17, etc.) removed after verification
- [ ] Duplicate TODO comments consolidated to single instance
- [ ] Disabled tests either fixed and enabled, or removed with issue filed
- [ ] Unimplemented stubs marked with `[[deprecated]]` or throw `NotImplemented`
- [ ] DEPENDENCIES.md created with versions and license info
- [ ] rpclib bug workarounds documented or removed
- [ ] CMake uses explicit file lists instead of GLOB_RECURSE
- [ ] Warning suppression macros removed, actual warnings fixed
- [ ] Zero new compiler warnings with -Wall -Wextra

## Files Affected

- `AirLibUnitTests/main.cpp` — fix or remove disabled tests
- `Unreal/Plugins/AirSim/Source/Beacons/FiducialBeacon.cpp` — remove dead code
- `AirLib/include/common/AirSimSettings.hpp` — remove workarounds
- `AirLib/include/api/RpcLibAdaptorsBase.hpp` — document/remove rpclib workarounds
- `AirLib/src/api/RpcLibClientBase.cpp` — fix unimplemented stubs
- `cmake/AirLib/CMakeLists.txt` — explicit file lists
- `Unreal/Plugins/AirSim/Source/AirSim.Build.cs` — fix warning suppression
- New: `DEPENDENCIES.md`
