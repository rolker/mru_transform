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

## Plan Authored
**Status**: complete
**When**: 2026-11-23 09:22 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Plan**: `.agent/work-plans/issue-51/plan.md` at `de4e3c5`

Eight commits on `rolling`, planned from a Rolling baseline build and a scratch copy that builds and passes (97 tests, 0 failures, no intermittent failure in 100 to 300 runs per binary): geodesy CMake and 1.0.7 floor (causes 1, 8), tf2 `.hpp` includes (2), a new cause found by the build in the `tf2_ros` listener and broadcaster node constructors (with a mutation-checked test), manifest order (9), undeclared dependencies per package (11), the CI workflow, and retiring the old `upstream.repos` scaffold. Backport is the same eight commits by cherry-pick.

## Plan Review
**Status**: complete
**When**: 2026-11-23 09:27 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: needs-work

**Issue**: #51 — Build on ROS 2 Rolling: port mru_transform, add a rolling branch and CI
**Plan**: `.agent/work-plans/issue-51/plan.md` at `de4e3c5`
**Branch**: `feature/issue-51`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Eight commits, two packages plus CI; every change is a build, manifest or CI fix. The `ament_lint_common` hold-back is sound: CMake never names it (`ament_lint_auto_find_test_dependencies` only picks up linters the manifest declares), so it is not an undeclared key; declaring it would add 293 failing tests, a behaviour change, not a port fix. |
| Issue alignment | Good | Every Issue Review action is taken or declined per the Checkpoint. Serves: "None: mru_transform has no goals or design document; it is the root-level port of rolker/unh_marine_autonomy#415 (the project building on ROS 2 Rolling and Jazzy)." Not flagged (a project is not asked to keep goals). |
| File targeting | Good | Listener/broadcaster constructions checked in both packages and all tests: only `src/mru_transform.cpp:93,99` take a node; `sea_surface_estimator.hpp:191` and `chart_datum_node.hpp:256` use the one-argument `TransformListener(*tf_buffer_)` (the same overload on both distros, own node by design, per their comments) and `TransformBroadcaster(*this)`. No other construction in `test/` or `tide_copier`. |
| Consequences | Needs work | One launch dependency missed (finding 5); retirement of `upstream.repos` affects nothing else (cube_bathymetry's industrial_ci reads its own `upstream.repos`, industrial_ci does not recurse into a dependency's). |
| Principle alignment | Needs work | Test what breaks: the new test's negative clause is domain-global and can fail from another repository's test (finding 1); the dropped `spin_thread=false` is not covered (finding 2). |
| ADR compliance | Good | ADR-0013 only; records on `rolling`, not backported. |
| ROS conventions | Good | Rolling form `TransformListener(buf, *node_, false)` → `NodeInterfaces` from `Node&` (implicit, `node_interfaces.hpp:150`), same default `DynamicListenerQoS`/`StaticListenerQoS` and QoS-override options as Jazzy's `NodeT&&` constructor, `static_only=false`, subscriptions created through the node's parameters/topics interfaces (so the same `qos_overrides./tf...` parameters); the buffer keeps `node_->get_clock()`, so `use_sim_time` is unchanged. `TransformBroadcaster(*node_)`: Rolling takes `RequiredInterfaces` from `Node&`; Jazzy's `NodeT&&` calls `get_node_*_interface(node)`, which accepts a `Node&` (rclcpp `jazzy` `test_get_node_interfaces.cpp:84`), and this package already compiles `TransformBroadcaster(*this)` on Jazzy. |

### Findings

1. **[Principle alignment — Test what breaks]** (medium) The planned `TransformListenerSubscribesFromTheNodeItself` asserts that no `transform_listener_impl*` node subscribes to `/tf` in the graph. That check is domain-wide, and `test_map_origin` runs on `ROS_DOMAIN_ID=87` with `LOCALHOST` discovery, the same domain as rolker/cube_bathymetry's `test_node_lifecycle` (`cube_bathymetry/CMakeLists.txt:473`), whose live node constructs `TransformListener(*tf_buffer_, this)` (`cube_bathymetry_node.hpp:522`), which on Rolling becomes the bool overload and creates a `transform_listener_impl_<hex>` node. In a whole-workspace `colcon test` (packages run in parallel) the new test can then fail with nothing wrong in mru_transform. The positive clause (a `/tf` subscription whose node is the node under test, in its per-test namespace) already fails for the `node_.get()` mutation on its own. Resolution: drop the negative clause, or restrict it to something this process owns; and note the shared domain 87 in the follow-up issue.
2. **[Principle alignment — Test what breaks]** (medium) The plan says the helper picks `*node` or the `shared_ptr` but does not say it passes the third argument `false`, and the test cannot tell: `TransformListener(buf, *node_)` (default `spin_thread = true`) subscribes from the node itself and passes the test, yet adds a dedicated executor thread and callback group and sets `buffer.setUsingDedicatedThread(true)`, which changes `canTransform`/`lookupTransform` timeout behaviour in the navigation path. Resolution: state that the helper takes and forwards `spin_thread = false`, and add an assertion that fails if it is dropped (e.g. `isUsingDedicatedThread()` on the buffer via a test accessor, or the node's callback-group count unchanged by the listener). Checked here: the reference form with `false` on Rolling leaves `isUsingDedicatedThread()` false.
3. **[ROS conventions]** (low) Make the helper's listener type a template parameter (e.g. `template<class ListenerT = tf2_ros::TransformListener>`), detecting `typename ListenerT::RequiredInterfaces` with `std::void_t`, so both `if constexpr` branches are dependent. With a non-dependent `tf2_ros::TransformListener` the discarded branch is still parsed and its non-dependent calls resolved at definition time; through `std::make_shared` that happens to compile (checked with GCC 15 against `/opt/ros/rolling` headers and a Jazzy-shaped mock), but a direct `new`/constructor call in the discarded branch would be a hard error on Rolling. Dependent branches remove the question on both distros and compilers.
4. **[File targeting — CI]** (low) The workflow description leaves out details the two models carry and an implementer would otherwise guess: `shell: bash` on the step that uses `set -euo pipefail` (the container's default `sh` is dash); `bash -c 'source /opt/ros/<distro>/setup.bash && …'` on the colcon build, test and test-result steps (no step sources ROS in a `ros:*-ros-core` container otherwise); `permissions: contents: read`; `DEBIAN_FRONTEND=noninteractive` on apt and rosdep. Resolution: embed the YAML in the plan, or list these.
5. **[Consequences — cause 11]** (low) `mru_transform/launch/ixblue_example.launch.xml` is a ROS 2 XML launch file (`exec=`, `$(find-pkg-share …)`), which needs `launch_xml` at run time; step 6 declares `launch` and `launch_ros` only. Add `<exec_depend>launch_xml</exec_depend>` (it is in `ros-core`, so CI cannot show it missing, as with the others). `launch/test_posmv.launch` is a ROS 1 file (`<rosparam>`, `type=`); leave it and add it to the follow-up list.
6. **[Consequences — step 8]** (low) The stated reason that `upstream.repos` "pins `geographic_info` at `ros2` (a branch without the `1.0.7` fix)" is wrong today: `gh api repos/ros-geographic-info/geographic_info/compare/f70b81a…ros2` reports `identical`, and `geodesy/package.xml` on `ros2` is 1.0.7. Retiring is still right (nothing reads the files; `ros2` is a moving branch; unh_marine_autonomy is pinned to `jazzy` for both distros), but correct the commit message and PR text.

### Summary

The plan is sound and evidenced; the Rolling listener/broadcaster fix maps to the same node, QoS and threading as the original on both distros, and each commit builds on Jazzy alone (by reading) while 1 to 3 build on Rolling only together, which the backport cherry-picks in order anyway. The new test needs two adjustments before it guards the one behavioural change properly (findings 1 and 2); the rest are small corrections.

### Recommended Actions

- [ ] Drop or scope the domain-wide `transform_listener_impl*` assertion in the new test; note the shared `ROS_DOMAIN_ID=87` with cube_bathymetry in the follow-up issue.
- [ ] State that the listener helper forwards `spin_thread = false`, and assert no dedicated listener thread in the test.
- [ ] Make the helper's listener type a template parameter so both branches are dependent.
- [ ] Embed the workflow YAML in the plan or list `shell: bash`, the per-step ROS sourcing, `permissions` and `DEBIAN_FRONTEND`.
- [ ] Add `<exec_depend>launch_xml</exec_depend>` to `mru_transform/package.xml` in step 6; list `launch/test_posmv.launch` (ROS 1) in the follow-up.
- [ ] Correct step 8's reason about the `ros2` branch.

## Checkpoint
**Status**: complete
**When**: 2026-11-23 09:28 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the Plan Review's verdict is needs-work on six findings, each with its resolution stated exactly and none changing scope; handing them to the implementer as binding corrections gives the same result as a plan revision without another plan and review round)
**After**: plan
**Decision**: proceed

Read: `plan.md` at `de4e3c5` and the Plan Review of it (needs-work; six findings; the seven points the host asked about confirmed except the new test, partly wrong). The findings are binding corrections that override the plan where they differ:

- F1 (medium): the new test's negative clause (no `transform_listener_impl*` node on `/tf`) is domain-wide and can fail because of another package's test on the same `ROS_DOMAIN_ID`. Keep the positive clause (a `/tf` subscription whose node is the node under test, in its own namespace); drop the negative clause or restrict it to what this process owns.
- F2 (medium): the helper must take and forward `spin_thread = false`, as the original call did, and a test must fail if it is dropped (for example the buffer's `isUsingDedicatedThread()` staying false, or the node's callback-group count unchanged by the listener).
- F3 (low): make the helper's listener type a template parameter and detect `typename ListenerT::RequiredInterfaces` with `std::void_t`, so both `if constexpr` branches are dependent.
- F4 (low): the workflow needs `shell: bash` where `set -euo pipefail` is used, `bash -c 'source /opt/ros/<distro>/setup.bash && ...'` on the colcon steps, `permissions: contents: read`, and `DEBIAN_FRONTEND=noninteractive` on apt and rosdep, as the two model workflows have.
- F5 (low): declare `launch_xml` as an `exec_depend` of `mru_transform` (it has an XML launch file); leave the ROS 1 file `launch/test_posmv.launch` and add it to the follow-up list.
- F6 (low): the reason given for retiring `upstream.repos` is wrong (geographic_info's `ros2` branch equals `f70b81a` today); retire it for the true reasons (nothing reads it; a moving branch; one upstream pinned to `jazzy` for both distros).

The planner's exception for `ament_lint_common` stands: the review found it sound (CMake never names it, so it is not an undeclared key).
