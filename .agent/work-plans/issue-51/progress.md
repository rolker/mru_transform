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

## Implementation
**Status**: complete
**When**: 2026-11-23 09:46 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-51 at `4fab763`

The plan at `de4e3c5` (eight steps), with the Plan Review's corrections F1 to F6 applied. One fix per commit, in backport order:

1. `fbb003e` geodesy: `ament_target_dependencies` (4 calls) replaced by `${geodesy_INCLUDE_DIRS}` / `${geodesy_LIBRARIES}` (library include PUBLIC, library in its plain link list; the other three PRIVATE); `find_package(geodesy 1.0.7)` and `<depend version_gte="1.0.7">` (#415 causes 1, 8). Behaviour: none; the same include directories and the same `geoconv` library reach the same targets.
2. `12541ed` tf2 `.h` to `.hpp`, 14 includes in 8 files (#415 cause 2). Behaviour: none; Jazzy's `.h` forward to the `.hpp`.
3. `dab9463` tf2_ros constructors (new #415 cause, 14): `TransformBroadcaster(*node_)`; `makeTransformListener<ListenerT>` selects `*node` when `ListenerT::RequiredInterfaces` exists (`std::void_t`, both branches dependent) and the `shared_ptr` otherwise, forwarding `spin_thread = false`. New test `TransformListenerSubscribesFromTheNodeItself` in `test_map_origin`: a `/tf` subscription from the node under test (per-test namespace; no domain-wide clause, F1), and no callback group added by the listener (F2). **The one change to executable behaviour**; intended to be identical (same node, default QoS and override parameters, no spin thread, same buffer clock), protected by that test.
4. `89e3030` `member_of_group` after `test_depend` in `mru_transform_interfaces/package.xml` (#415 cause 9); `ament_xmllint` now valid. Behaviour: none.
5. `3821227` interfaces manifest: `rosidl_default_generators` (buildtool), `builtin_interfaces`, `geographic_msgs` (#415 cause 11). Behaviour: none.
6. `639dd93` mru_transform manifest: `std_srvs`, exec `launch`, `launch_ros`, `launch_xml` (F5), test `ament_lint_auto`; `ament_lint_common` deliberately not declared (#415 cause 11). Behaviour: none.
7. `4252c64` `.github/workflows/ci.yml`: Jazzy and Rolling jobs, geodesy at `f70b81a`, sparse `marine_vertical_datum`, paths-only rosdep without `-r`, `shell: bash`, per-step ROS sourcing, `permissions: contents: read`, `DEBIAN_FRONTEND` (F4).
8. `4fab763` retire `upstream.repos` and `.agents/ci_local_upstream_extra.sh`, for the true reasons (F6).

Verification (Rolling only, this machine):
- Clean `./build.sh`: 2 packages finished, 0 errors (warnings only the known, listed ones). `./test.sh` (`ROS_DOMAIN_ID=42` inherited; the two node tests set 87 and 89 in CMake): `Summary: 97 tests, 0 errors, 0 failures, 0 skipped`.
- Rolling CI rehearsal (scratch workspace laid out as the job, `env -i`, only `/opt/ros/rolling` sourced, the job's commands, rosdep `--simulate`): colcon list gives geodesy, mru_transform, mru_transform_interfaces, marine_vertical_datum; build 4 packages finished; `Summary: 97 tests, 0 errors, 0 failures, 0 skipped`.
- Mutations on a scratch copy (ldd confirmed the scratch `libmru_transform.so`): (a) `node_.get()`: the new test fails ("no /tf subscription from /map_origin_test_.../mru_transform; subscribers seen: //transform_listener_impl_57ba169c8ee0"), and the full binary segfaults at teardown of the other node tests; (b) `spin_thread` dropped: the new test fails on the callback-group count (2 vs 1), full binary also segfaults. Restored: 22/22 pass.
- Repeat runs, separate processes, 0 failures: twist_rotation, water_line_offset, water_line_lever_arm, nav_source_selection 300 each (domains 42 to 45); subscribe_once 100 (46); map_origin 150 (87, its own); lifecycle_reconfigure 100 (89, its own).

Not done: no Jazzy build or test (not installed here; the PR's Jazzy CI job is the first evidence, in particular for the helper's Jazzy branch and the new test); the workflow itself has not run; no vehicle run, bag replay or simulation.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-23 09:53 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-51 at `3c5cd7d`
**Base**: origin/rolling
**Depth**: Standard (reason: 17 files, +663 -102, a new CI workflow and the one behaviour change in a navigation-path node; no security or cross-layer trigger for Deep)
**Must-fix**: 0 | **Suggestions**: 3
**Round**: 1 | **Ship**: recommended — no must-fix findings; remaining suggestions can be applied or tracked

Listener and broadcaster (the one change to executable behaviour), checked line by line against `/opt/ros/rolling/include/tf2_ros/tf2_ros/transform_listener.hpp` and geometry2 `jazzy` `transform_listener.hpp` / `transform_broadcaster.hpp`: on Rolling `(buffer, *node, false)` selects the `RequiredInterfaces` constructor (implicit `NodeInterfaces(Node&)`), same node, `spin_thread=false`, the same default `DynamicListenerQoS`/`StaticListenerQoS` and QoS-override options as Jazzy's; on Jazzy `RequiredInterfaces` does not exist, so the `else` branch is the original expression (`NodeT&&` deduced as `const shared_ptr<Node>&`; `shared_ptr`'s `operator bool` is explicit, so the simplified constructor is not a candidate). The buffer clock is unchanged (`node_->get_clock()`), so `use_sim_time` is unchanged. `TransformBroadcaster(*node_)`: same defaults on both; Jazzy's `get_node_*_interface` accept a reference (already used as `*this` by two nodes in this package). Same behaviour on Rolling (built and tested) and on Jazzy by reading. The new test fails for both wrong forms (`node.get()`: no /tf subscriber on the node; `spin_thread` dropped: callback groups 1 to 2), as the implementer's mutations and an independent scratch compile showed.

Not findings, by checking: `ament_target_dependencies` on Jazzy (ament_cmake `jazzy`) added include dirs PUBLIC without `$<BUILD_INTERFACE>` and without `SYSTEM` unless asked, and linked `_LIBRARIES` plain, so the replacement exports the same absolute paths as before (Gemini's must-fix and SYSTEM points are false positives); `sea_surface_estimator`/`chart_datum_node` use the one-argument listener on purpose; `launch_xml` and the dropped negative test clause are Plan Review F5 and F1. Workflow read step by step against a bare `ros:*-ros-core` container and the marine_ais and unh_marine_autonomy models: tools installed before use, `shell: bash` only where bash is needed, ROS sourced in each colcon step, rosdep needs only `ROS_DISTRO` (set by the image), closure of 4 packages, no `-r`, guard as in the models; geodesy at the same tree builds with tests on Jazzy in unh_marine_autonomy's Jazzy job (core.repos `ros2` == `f70b81a`). No system key missing from either manifest (system keys come in through marine_vertical_datum and geodesy).

### Findings
- [ ] (suggestion) Declare `geometry_msgs` in mru_transform_interfaces (three srv files use `geometry_msgs/PointStamped`; it is in neither the manifest nor `rosidl_generate_interfaces` DEPENDENCIES and arrives only through geographic_msgs) — `mru_transform_interfaces/package.xml:14`, `mru_transform_interfaces/CMakeLists.txt:22`
- [ ] (suggestion) Declare `<test_depend>rcutils</test_depend>`: test_map_origin includes `rcutils/logging.h` directly (pre-existing, same completeness class as cause 11) — `mru_transform/package.xml:36`, `mru_transform/test/test_map_origin.cpp:18`
- [ ] (suggestion) The new test's per-test namespace is the same in every process, so two concurrent runs of test_map_origin on one machine (domain 87, LOCALHOST) can let one run's correct subscriber satisfy the other's `found` check (a false pass, not a flaky failure; CI jobs are isolated); add a per-process suffix (e.g. pid) to the namespace, or track in the follow-up issue — `mru_transform/test/test_map_origin.cpp:200`

## Checkpoint
**Status**: complete
**When**: 2026-11-23 09:54 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the pre-push review approved round 1 with no must-fix; this repository has never had CI and Jazzy cannot be built on the development machine, so the pushed branch's first run is the first Jazzy evidence, and of the workflow itself)
**After**: publish
**Decision**: publish

Read: the Local Review (Pre-Push) at `3c5cd7d`, verdict approved, ship recommended, three suggestions open: declare `geometry_msgs` in `mru_transform_interfaces` (`package.xml:14`, `CMakeLists.txt:22`); declare `<test_depend>rcutils</test_depend>` in `mru_transform` (`package.xml:36`); the new test's per-test namespace is the same in every process, so two concurrent runs on one machine could give a false pass (`test/test_map_origin.cpp:200`). They stay open and go through triage after publish, so the first CI run starts now. This is a project issue, so the host runs no full pre-commit pass. The pull request says `Part of #51`.

## Integrated Review
**Status**: complete
**When**: 2026-11-23 09:59 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**PR**: #52 at `a1ecb86`
**Sources**: 4 (Local Review (Pre-Push) @ `3c5cd7d` (covers `a1ecb86` via bookkeeping), Gemini and Codex in that round @ `3c5cd7d`, CI rollup run 45199381330); GitHub: 0 reviews, 0 inline comments, 0 conversation comments, no Copilot
**Cross-source confirmations**: 1
**CI**: all-pass

CI, first run of the workflow (run 45199381330, both jobs at `a1ecb86`):
- `build-and-test (jazzy)`: pass. rosdep "#All required rosdeps installed successfully" (no system key missing from the closure); build "Summary: 4 packages finished [1min 1s]" (geodesy, marine_vertical_datum, mru_transform_interfaces, mru_transform); test "Summary: 97 tests, 0 errors, 0 failures, 0 skipped". The helper's Jazzy branch compiles under GCC 13, and the new test passes: "7: [       OK ] MapOriginNodeTest.TransformListenerSubscribesFromTheNodeItself (6 ms)". Only compiler warnings are geodesy's own `geodesics.h:234` -Wparentheses.
- `build-and-test (rolling)`: pass. Same rosdep line; "Summary: 4 packages finished [1min 3s]"; "Summary: 97 tests, 0 errors, 0 failures, 0 skipped"; the same new-test OK line. Warnings: the deprecated `tf2_ros/*.h` header `#warning`s (owner-settled, follow-up issue) and the geodesy one.
- `[ERROR]` log lines in both test logs are the expected output of negative tests (TF_NAN_INPUT, rejected buffer durations), not failures.

Suggestion 1 checked against `rosidl_generate_interfaces.cmake` (Rolling): a DEPENDENCIES entry must be `_FOUND` (it is today only because geographic_msgs' config finds geometry_msgs transitively, which is also how `builtin_interfaces` works with no `find_package`). Adding it changes nothing a consumer sees: geometry_msgs is already in geographic_msgs' recursive interface dependencies and its exported dependencies, so the generated code, the typesupport link set and the exported CMake dependencies stay the same; only the manifest and the CMake become explicit.

### Findings
- [x] (suggestion, Local Review @ `3c5cd7d`) Declare `geometry_msgs` in mru_transform_interfaces: add `<depend>geometry_msgs</depend>` to `package.xml`, `find_package(geometry_msgs REQUIRED)` and `geometry_msgs` in `rosidl_generate_interfaces(... DEPENDENCIES ...)`; same on both distros, no change to generated interfaces or exports — `mru_transform_interfaces/package.xml:14`, `mru_transform_interfaces/CMakeLists.txt:22`
- [x] (suggestion, Local Review @ `3c5cd7d`) Add `<test_depend>rcutils</test_depend>`: test_map_origin includes `rcutils/logging.h` directly and gets it only through rclcpp (CI passes because rclcpp brings it) — `mru_transform/package.xml:36`, `mru_transform/test/test_map_origin.cpp:18`
- [x] (cross-confirmed: Local Review + Codex @ `3c5cd7d`) The per-test namespace in `make_node` is the same in every process, so two concurrent runs of test_map_origin on one host (domain 87, LOCALHOST) can let one run's subscriber satisfy the other's `found` check (a false pass), and can also cross-serve `reset_map_frame` calls between runs; append a per-process suffix (e.g. `"_" + std::to_string(getpid())` with `<unistd.h>`) to the namespace in `make_node`, which fixes every test in the fixture on both distros, or move it to the follow-up issue. CI is unaffected (jobs are isolated) — `mru_transform/test/test_map_origin.cpp:200`

### False positives
- (Codex) Set `persist-credentials: false` on checkout — both jobs run with `permissions: contents: read` on a public repository, so the persisted token grants nothing anonymous access does not; the unpinned code fetched is the owner's own unh_marine_autonomy and geographic_info is pinned by SHA.
- (Gemini) Guard the sparse checkout of marine_vertical_datum with a `package.xml` check — if the folder vanished, colcon would not list it and `rosdep install` without `-r` fails on the unresolved `marine_vertical_datum` key, so the job still fails loudly at the dependency step; it cannot pass silently.
- (Gemini) `Test results` with `if: always()` errors when `../ws` is missing — that happens only when an earlier step already failed the job, so it adds a second red step but never changes the verdict.
- (Gemini) Unquoted `$paths` and `actions/checkout@v6` by major tag — colcon paths in the container are fixed, space-free `src/...` paths, and the GitHub-owned checkout action by major tag matches the marine_ais and unh_marine_autonomy model workflows; neither can produce a wrong result here.
- (Gemini, Codex; already ruled in Local Review (Pre-Push) @ `3c5cd7d`) geodesy include/link export paths, missing SYSTEM, `sea_surface_estimator` listener audit, `launch_xml`, and the dropped hidden-listener negative check — ruled not findings there with evidence (ament_target_dependencies on Jazzy exported the same paths; one-argument listener on purpose; Plan Review F1/F5); nothing at `a1ecb86` changes that.

## Checkpoint
**Status**: complete
**When**: 2026-11-23 09:59 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (three valid suggestions are open and each has a small, named fix; merging past open findings is the owner's answer only)
**After**: findings
**Decision**: address

Read: the Integrated Review of pull request 52 at `a1ecb86`: three valid suggestions, four false positives, CI all-pass on the workflow's first run (Jazzy and Rolling each: 4 packages built, 97 tests, 0 errors, 0 failures; the new listener test passing on both; rosdep without `-r` satisfied). All three are addressed: (1) `geometry_msgs` declared in `mru_transform_interfaces` (`package.xml`, `find_package`, `rosidl_generate_interfaces` DEPENDENCIES); (2) `<test_depend>rcutils</test_depend>` in `mru_transform`; (3) a per-process suffix on the test namespace in `make_node` (`test/test_map_origin.cpp:200`), so two runs on one machine cannot answer each other.

## Implementation
**Status**: complete
**When**: 2026-11-23 10:09 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**PR**: #52 at `0783b0a`
**Addressed**: Integrated Review at `a1ecb86` (2026-11-23 09:59 -05:00)
**Commits**: 7fb49ec, ddcc9f5, 0783b0a

### Actions
- [x] Declared `geometry_msgs` in mru_transform_interfaces (manifest `<depend>`, `find_package`, `DEPENDENCIES`) in 7fb49ec — `mru_transform_interfaces/package.xml:16`, `mru_transform_interfaces/CMakeLists.txt:11,26`. All four srv files use `geometry_msgs/PointStamped` (the finding said three). Installed CMake exports compared before and after: every `*Export.cmake` target file is byte-identical (the generator targets' `INTERFACE_LINK_LIBRARIES` already carried `geometry_msgs::*` through `geographic_msgs`); the only difference is `ament_cmake_export_dependencies-extras.cmake`, whose direct list gains `geometry_msgs` after `geographic_msgs`. Consumers already reached `geometry_msgs` there, since `geographic_msgs` exports it, so the set they find is unchanged.
- [x] Added `<test_depend>rcutils</test_depend>` in ddcc9f5, manifest only — `mru_transform/package.xml:37`. CMake was left unchanged on purpose: `rclcpp::rclcpp`'s exported `INTERFACE_LINK_LIBRARIES` carries `rcutils::rcutils`, and rclcpp's public headers include rcutils, so the link cannot go away without breaking rclcpp itself.
- [x] Per-process namespace suffix `"_" + std::to_string(getpid())` (`<unistd.h>`) in `make_node`, plus the matching comment on the listener test, in 0783b0a — `mru_transform/test/test_map_origin.cpp:200`. Concurrent runs on one domain (143, LOCALHOST): 30 pairs and 25 rounds of 4 processes, 0 failures. Mutation check (scratch copy, listener built `std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_.get())`, `ldd` showed the scratch `libmru_transform.so`): the new test fails ("no /tf subscription ... subscribers seen: //transform_listener_impl_..."). With a decoy node `mru_transform` subscribed to `/tf` in the old, un-suffixed namespace, the old test falsely passed against the mutated library (230 ms) and the new test still failed. Scratch removed; the real tree was never mutated.

No non-test source changed: navigation code (position and orientation transforms) untouched.

### Verification (Rolling only; Jazzy is CI's)
- `./build.sh`: 2 packages finished. `ROS_DOMAIN_ID=143 ./test.sh`: `Summary: 97 tests, 0 errors, 0 failures, 0 skipped`.
- `test_map_origin` 150 times as separate processes (domain 143): 0 failed.
- Not verified here: the Jazzy build and tests of these three commits (CI on the pushed head is the only check).

## Local Review
**Status**: complete
**When**: 2026-11-23 10:16 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Dispatch**: resumed (agent a2f11c65de0e0135e, resume 1 of 3)
**Verdict**: approved

**PR**: #52 at `4b70556`
**Depth**: Standard (reason: narrow check per handoff: round 1 had no must-fix, so the owner's standing rule gives a check of that review's three suggestions and the fix diff `a1ecb86..4b70556`; no cross-model reviewers)
**Must-fix**: 0 | **Suggestions**: 0

Checked: `git diff a1ecb86..4b70556 --stat` touches only `mru_transform/package.xml`, `mru_transform/test/test_map_origin.cpp`, `mru_transform_interfaces/{package.xml,CMakeLists.txt}` and the records; no non-test source of the navigation path. The three Local Review (Pre-Push) suggestions are resolved as asked:
- `7fb49ec` geometry_msgs in the interfaces manifest, `find_package` and `DEPENDENCIES`. No consumer-visible change: geometry_msgs already reached consumers through geographic_msgs' exported dependencies and the generator targets' link interface; the fix pass's byte-identical `*Export.cmake` comparison is consistent with that; only the direct export-dependency list gains a name that was already found transitively.
- `ddcc9f5` `<test_depend>rcutils</test_depend>`; CMake unchanged is right (`rclcpp::rclcpp` carries `rcutils::rcutils`).
- `0783b0a` namespace `map_origin_test_<TestName>_<pid>`: a valid ROS name token (starts with a letter; gtest names are identifiers; digits only after `_`). Nothing else in the fixture used the fixed name: the listener test reads `node->get_namespace()` and the `reset_map_frame` client is relative to the node, so both follow the suffix.

`--gtest_repeat` hang: not reproduced here at `4b70556`. In-process runs of test_map_origin passed: `--gtest_repeat=300` on the full binary (domain 63, 41 s, 300/300 PASSED 22 tests), and `--gtest_repeat=300` on the listener and reset tests on domains 63 and 87 (300/300 each). Nothing in `0783b0a` touches init/shutdown or process lifetime, so the change gives no mechanism for a hang. CI and `colcon test` run each gtest binary once (ctest, no repeat), so it does not matter for CI.

CI on `4b70556` (run 45202593814, both jobs pass):
- build-and-test (jazzy), pass, 3m10s: "#All required rosdeps installed successfully"; "Summary: 4 packages finished [1min 28s]"; "7: [       OK ] MapOriginNodeTest.TransformListenerSubscribesFromTheNodeItself (11 ms)"; "Summary: 97 tests, 0 errors, 0 failures, 0 skipped".
- build-and-test (rolling), pass, 3m9s: "#All required rosdeps installed successfully"; "Summary: 4 packages finished [1min 19s]"; "7: [       OK ] MapOriginNodeTest.TransformListenerSubscribesFromTheNodeItself (9 ms)"; "Summary: 97 tests, 0 errors, 0 failures, 0 skipped".

Re-ran locally (Rolling): `./build.sh` (2 packages finished) and `ROS_DOMAIN_ID=63 ./test.sh`: "Summary: 97 tests, 0 errors, 0 failures, 0 skipped".

### Findings
- [ ] No issues found. LGTM.
