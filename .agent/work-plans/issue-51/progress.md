---
issue: 51
---

# Issue #51 — Build on ROS 2 Rolling: port mru_transform, add a rolling branch and CI

## Issue Review
**Status**: complete
**When**: 2026-11-23 09:04 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**Issue**: #51

### Scope Assessment

**Well-scoped?** Yes — two packages, one fix per commit, backport by hand; the owner's rule keeps the warnings out. The cause list is a text search and is incomplete (below), but every addition is a build or declaration fix in the same two packages, so the scope does not grow in kind.
**Right repo?** Yes — mru_transform is where the code and the CI belong.
**Serves**: none — mru_transform has no goals or design document of its own (no `docs/`, no `PRINCIPLES.md`); it is the root-level port of rolker/unh_marine_autonomy#415, whose goal is the project building on ROS 2 Rolling and Jazzy.
**Design picture?** No change — no design document exists and the port changes no behaviour.
**Dependencies**: `marine_vertical_datum` (ported; hosted core install may not be ready); `geodesy` from geographic_info at `f70b81a` or later; rolker/unh_marine_autonomy needs a `rolling` ref for CI.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| A change includes its consequences | Action needed | The repository already has a CI scaffold the issue does not mention: `upstream.repos` (geographic_info at `ros2`, unh_marine_autonomy at `jazzy`) and `.agents/ci_local_upstream_extra.sh`, which prunes every unh_marine_autonomy package except `marine_vertical_datum`. Both were written for industrial_ci / `ci_local.sh`. The new workflow must either consume them (and move `geographic_info` to `f70b81a`, and unh_marine_autonomy to the right ref per branch) or retire them in the same change; leaving them would leave a second, stale dependency list. |
| Verify before claiming | Action needed | The issue says "`cmake_minimum_required(VERSION 3.8)`". Only `mru_transform_interfaces/CMakeLists.txt` has 3.8 (line 1); `mru_transform/CMakeLists.txt` has 3.14.4. The listing, and the follow-up issue drafted from it, must name only the interfaces package. |
| Only what's needed | Watch | Cause 1 and cause 8 are the same four calls (`ament_target_dependencies(... geodesy)` in the library, `nav_sat_fix_to_velocity`, `chart_datum_node`, `test_lifecycle_reconfigure`), and every one names only `geodesy`. One fix per commit means one commit for them, replacing the call with `target_include_directories(... ${geodesy_INCLUDE_DIRS})` and `${geodesy_LIBRARIES}` (per #415 cause 8). For the library the include directory must be PUBLIC, since two public headers include `geodesy/ecef.h`. |
| Test what breaks | Watch | The nodes and launch files cannot be exercised on Jazzy here; the `Done when` already says to state what was not checked there. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0001 / 0017 | No | No change to how the parts fit; the project has no design document. |
| 0011 | No | Workspace adapter contract; nothing is changed in the workspace. |
| 0013 | Yes | The issue's records go in `.agent/work-plans/issue-51/progress.md` via the standard script. |

### Consequences

- `upstream.repos` and `.agents/ci_local_upstream_extra.sh` (see the table): update or retire with the new workflow.
- The `.github/workflows/ci.yml` header comment, as in marine_ais, should say the same file is carried on both branches.

### Recommendations

- Add to the cause list, cause 9 (`package.xml` element order): `mru_transform_interfaces/package.xml` puts `<test_depend>` after `<member_of_group>`, which `ament_xmllint` rejects on Rolling (#415 cause 9). It shows as a failing lint test once the package builds.
- Add to cause 11 (undeclared dependencies), found by reading CMake and sources against `package.xml`: `mru_transform` uses `std_srvs` (`find_package`, `src`, `test_map_origin`) without declaring it; `mru_transform_interfaces` names `geographic_msgs` and `builtin_interfaces` in `rosidl_generate_interfaces` and `find_package(geographic_msgs)` without declaring them, and has no `<buildtool_depend>rosidl_default_generators</buildtool_depend>`; `mru_transform` has `ament_lint_auto` in CMake tests but no `<test_depend>ament_lint_auto</test_depend>` or `ament_lint_common`; the launch files import `launch`, `launch_ros` and `lifecycle_msgs.msg` with no `<exec_depend>`. Run `rosdep install` without `-r` to confirm each before declaring it, and declare only what the check or a build shows missing.
- Correct cause 2's counts: the `.h` includes in tests are in three files (`test_twist_rotation.cpp`, `test_water_line_offset.cpp`, `test_water_line_lever_arm.cpp`), not four. The removed-header list is otherwise complete: Rolling's `tf2/` has only `.hpp` for `tf2/utils` and `tf2/LinearMath/{Matrix3x3,Quaternion,Vector3}`; `tf2_geometry_msgs` is already `.hpp`. The 12 deprecated `tf2_ros/*.h` includes check out (`mru_transform.hpp` 3, `chart_datum_node.hpp` 3, `sea_surface_estimator.hpp` 3, `map_frame.hpp` 1, `water_line_lever_arm.hpp` 1, `test_water_line_lever_arm.cpp` 1).
- Say that causes 10 (GCC 15 template bodies in `gz4d_geo.h`), 12 (GDAL) and 13 (Nav2 lifecycle header) do not apply: nothing here includes `gz4d`, GDAL or Nav2 (PROJ and yaml-cpp moved into `marine_vertical_datum`, #41), so only a build can add to the list. `nav_sat_fix_to_velocity.hpp` also includes `geodesy/geodesics.h` and `wgs84.h`; check they exist in Jazzy's 1.0.6 (the CI job on `f70b81a` covers this either way).
- Remove the dead commented-out copy of the `CMakeLists.txt` at the bottom of `mru_transform_interfaces/CMakeLists.txt` only if touching that file for the 3.8 line anyway; otherwise leave it, as it is not part of the port.

### Actions
- [ ] The repository already has a CI scaffold the issue does not mention: `upstream.repos` (geographic_info at `ros2`, unh_marine_autonomy at `jazzy`) and `.agents/ci_local_upstream_extra.sh`, which prunes every unh_marine_autonomy package except `marine_vertical_datum`. Both were written for industrial_ci / `ci_local.sh`. The new workflow must either consume them (and move `geographic_info` to `f70b81a`, and unh_marine_autonomy to the right ref per branch) or retire them in the same change; leaving them would leave a second, stale dependency list.
- [ ] The issue says "`cmake_minimum_required(VERSION 3.8)`". Only `mru_transform_interfaces/CMakeLists.txt` has 3.8 (line 1); `mru_transform/CMakeLists.txt` has 3.14.4. The listing, and the follow-up issue drafted from it, must name only the interfaces package.
- [ ] Add to the cause list, cause 9 (`package.xml` element order): `mru_transform_interfaces/package.xml` puts `<test_depend>` after `<member_of_group>`, which `ament_xmllint` rejects on Rolling (#415 cause 9). It shows as a failing lint test once the package builds.
- [ ] Add to cause 11 (undeclared dependencies), found by reading CMake and sources against `package.xml`: `mru_transform` uses `std_srvs` (`find_package`, `src`, `test_map_origin`) without declaring it; `mru_transform_interfaces` names `geographic_msgs` and `builtin_interfaces` in `rosidl_generate_interfaces` and `find_package(geographic_msgs)` without declaring them, and has no `<buildtool_depend>rosidl_default_generators</buildtool_depend>`; `mru_transform` has `ament_lint_auto` in CMake tests but no `<test_depend>ament_lint_auto</test_depend>` or `ament_lint_common`; the launch files import `launch`, `launch_ros` and `lifecycle_msgs.msg` with no `<exec_depend>`. Run `rosdep install` without `-r` to confirm each before declaring it, and declare only what the check or a build shows missing.
- [ ] Correct cause 2's counts: the `.h` includes in tests are in three files (`test_twist_rotation.cpp`, `test_water_line_offset.cpp`, `test_water_line_lever_arm.cpp`), not four. The removed-header list is otherwise complete: Rolling's `tf2/` has only `.hpp` for `tf2/utils` and `tf2/LinearMath/{Matrix3x3,Quaternion,Vector3}`; `tf2_geometry_msgs` is already `.hpp`. The 12 deprecated `tf2_ros/*.h` includes check out (`mru_transform.hpp` 3, `chart_datum_node.hpp` 3, `sea_surface_estimator.hpp` 3, `map_frame.hpp` 1, `water_line_lever_arm.hpp` 1, `test_water_line_lever_arm.cpp` 1).
- [ ] Say that causes 10 (GCC 15 template bodies in `gz4d_geo.h`), 12 (GDAL) and 13 (Nav2 lifecycle header) do not apply: nothing here includes `gz4d`, GDAL or Nav2 (PROJ and yaml-cpp moved into `marine_vertical_datum`, #41), so only a build can add to the list. `nav_sat_fix_to_velocity.hpp` also includes `geodesy/geodesics.h` and `wgs84.h`; check they exist in Jazzy's 1.0.6 (the CI job on `f70b81a` covers this either way).
- [ ] Remove the dead commented-out copy of the `CMakeLists.txt` at the bottom of `mru_transform_interfaces/CMakeLists.txt` only if touching that file for the 3.8 line anyway; otherwise leave it, as it is not part of the port.

## Checkpoint
**Status**: complete
**When**: 2026-11-23 09:04 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the Issue Review found the issue well-scoped; its seven actions correct or extend the cause list inside the same two packages and none changes scope)
**After**: issue-actions
**Decision**: proceed

Read: the Issue Review of 51 (well-scoped, right repository, no design change). Its actions go to the planner as inputs:

- Taken into the plan: the existing CI scaffold (`upstream.repos`, `.agents/ci_local_upstream_extra.sh`) is either used by the new workflow or retired in the same change; cause 9 (`<test_depend>` after `<member_of_group>` in `mru_transform_interfaces/package.xml`); cause 11 (undeclared keys: `std_srvs`, `geographic_msgs`, `builtin_interfaces`, `rosidl_default_generators`, `ament_lint_auto`/`ament_lint_common`, `launch`, `launch_ros`, `lifecycle_msgs`), each confirmed by `rosdep` or a build before it is declared; causes 1 and 8 as one commit, with the `geodesy` include directory PUBLIC on the library; the corrected counts (three test files with removed tf2 headers; `cmake_minimum_required` 3.8 only in `mru_transform_interfaces`).
- Not taken: removing the dead commented-out CMake text in `mru_transform_interfaces/CMakeLists.txt`. It is not a Rolling fix; by the owner's rule a port makes no changes it does not need.
- Host decision, listed for the owner's veto: the undeclared ROS keys are declared in this port (manifest lines only, one commit per package), not deferred to a follow-up issue, because the new workflow's `rosdep install` without `-r` is the first check this repository has ever had of its manifests and they are needed for it to mean anything.
