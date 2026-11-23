# Plan: Build on ROS 2 Rolling: port mru_transform, add a rolling branch and CI

## Issue

https://github.com/rolker/mru_transform/issues/51 (part of rolker/unh_marine_autonomy#415; causes 1 to 13 in its comments)

## Context

Planned from a build, not from the issue's text search. Baseline on `jazzy`/`rolling` (both at `03871aa`), Rolling underlay plus the hosted core install, after a clean `./build.sh --continue-on-error`:

- `mru_transform_interfaces` builds (CMake 3.8 deprecation warning only).
- `mru_transform` fails at configure: `CMake Error at CMakeLists.txt:68 (ament_target_dependencies): Unknown CMake command "ament_target_dependencies".` `./test.sh` then runs 0 tests (nothing built to test).

A scratch copy (outside the repository) with the fixes below, the walk through the failures in order:

1. `ament_target_dependencies` (4 calls) gone. Replacing one of them in the library's plain `target_link_libraries` signature first fails with "all-keyword or all-plain" (`geodesy_LIBRARIES` goes into the library's existing plain list, not a new `PUBLIC` call).
2. Then the compile fails on `tf2/*.h` (14 includes, 8 files) once the CMake is fixed.
3. Then a **cause the issue does not list**: `tf2_ros::TransformListener(*tf_buffer_, node_, false)` with a `shared_ptr<Node>` does not compile on Rolling (its constructor takes `NodeInterfaces`, built from a node reference; Jazzy's takes a pointer-like node, `node->get_node_base_interface()`), and so does `TransformBroadcaster(node_)`. See step 3 for why the obvious fix is wrong.
4. After that the package builds with 0 errors. Final scratch result, in a clean `/opt/ros/rolling`-only environment (no underlay, no core install; geodesy from the `f70b81a` clone, `marine_vertical_datum` from a sparse clone of unh_marine_autonomy `rolling`), `colcon build --packages-up-to` of the four packages then `colcon test` without sourcing install: `Summary: 97 tests, 0 errors, 0 failures, 0 skipped` (7 gtest binaries; the 97th is the new test of step 3).

Repeat runs (each gtest binary as separate processes, ROS domain and discovery range as CMake sets them for the two that set them): 100 runs of `test_lifecycle_reconfigure` and `test_subscribe_once`, 300 of each of the other five, 0 failures (counts in the host report). No intermittent failure. Two of the tests prove negatives on live DDS traffic, so the first CI run on a runner is the real check (see Open Questions).

## Approach

One fix per commit on `rolling`, in this order, each in a form that also builds on Jazzy. Commits 1 to 3 do not build alone (each fixes the next first diagnostic); the branch builds from commit 3.

1. **Cause 1 + 8: geodesy.** `mru_transform/CMakeLists.txt`: `find_package(geodesy 1.0.7 REQUIRED)` with the comment `ais_layer` carries; the four `ament_target_dependencies(... geodesy)` calls (library, `nav_sat_fix_to_velocity`, `chart_datum_node`, `test_lifecycle_reconfigure`) become `target_include_directories(<t> <vis> ${geodesy_INCLUDE_DIRS})` plus `${geodesy_LIBRARIES}`. The library's include directory is `PUBLIC` (`map_frame.hpp`, `nodes/chart_datum_node.hpp` include `geodesy/ecef.h`); its `${geodesy_LIBRARIES}` goes in the existing plain `target_link_libraries` list (plain is transitive, which is what `ament_target_dependencies` did); the three others are `PRIVATE`. `package.xml`: `<depend version_gte="1.0.7">geodesy</depend>`. The 1.0.7 floor also covers `geodesy/geodesics.h` (used by `nav_sat_fix_to_velocity.hpp`), which Jazzy's 1.0.6 does not have either.
2. **Cause 2: removed tf2 `.h` names.** `tf2/utils.h` and `tf2/LinearMath/{Matrix3x3,Quaternion,Vector3}.h` to `.hpp` in `src/mru_transform.cpp`, `include/mru_transform/{twist_rotation_utils,water_line_offset,water_line_lever_arm}.hpp`, `include/mru_transform/nodes/sea_surface_estimator.hpp`, and the three tests `test_twist_rotation.cpp`, `test_water_line_offset.cpp`, `test_water_line_lever_arm.cpp`. Jazzy's `.h` files are one-line forwarders to the `.hpp`, so the code compiled is identical.
3. **New cause (propose #415 cause 14): tf2_ros node constructors.** In `src/mru_transform.cpp`: `TransformBroadcaster(*node_)` (the reference form `chart_datum_node` and `sea_surface_estimator` already use with `*this`; Jazzy's `get_node_*_interface` accept a reference, per rclcpp's own `test_get_node_interfaces`). For the listener, a local helper in an anonymous namespace selects by `TransformListener::RequiredInterfaces` (a nested type only Rolling has): `*node` where it exists, the `shared_ptr` otherwise (`if constexpr` in a function template, so the discarded branch is never instantiated). **Do not use `node_.get()`**: it compiles on Rolling, because the pointer converts to the `bool spin_thread` of the simplified constructor, and the listener then subscribes to `/tf` from a node of its own (`transform_listener_impl_<hex>`) with its own spin thread. Same commit: `test_map_origin.cpp` `TransformListenerSubscribesFromTheNodeItself` (polls `get_subscriptions_info_by_topic("/tf")` for a subscription from the node under test and for none from a `transform_listener_impl*` node). Mutation-checked on the scratch copy: with `node_.get()` it fails after the 10 s poll, naming the extra node.
4. **Cause 9: manifest order.** `mru_transform_interfaces/package.xml`: move `<member_of_group>` after the `<test_depend>` lines. Confirmed with `ament_xmllint` (line 19: `Element 'test_depend': This element is not expected`); not a failing colcon test today, because that package's CMake never calls `ament_lint_auto`, so the check is by hand here.
5. **Cause 11, `mru_transform_interfaces`.** `package.xml`: `<buildtool_depend>rosidl_default_generators</buildtool_depend>`, `<depend>geographic_msgs</depend>`, `<depend>builtin_interfaces</depend>`. Evidence: `rosidl_generate_interfaces` names both and `find_package(geographic_msgs REQUIRED)`; the srv files use `geographic_msgs`; `geographic_msgs` is **not** in `ros-core`'s apt closure (the others are), so this package alone with `rosdep install` over its path would fail to build.
6. **Cause 11, `mru_transform`.** `package.xml`: `<depend>std_srvs</depend>` (`find_package(std_srvs REQUIRED)`, used in `src` and `test_map_origin`), `<exec_depend>launch</exec_depend>` and `<exec_depend>launch_ros</exec_depend>` (four launch files import `launch`, `launch_ros`; `lifecycle_msgs` is already a `<depend>`), `<test_depend>ament_lint_auto</test_depend>` (CMake does `find_package(ament_lint_auto REQUIRED)`). **Not** `ament_lint_common`: with it the package gains 348 lint tests of which 293 fail (copyright 39, cpplint 210, flake8 13, lint_cmake 2, uncrustify 24; cppcheck skips all 35 on Rolling), measured on a second scratch copy; that is a decision for the follow-up issue, not a port. Evidence for 5 and 6: every key resolves with `rosdep resolve` on (jazzy, noble) and (rolling, resolute) (`ros-<distro>-std-srvs`, `-launch`, `-launch-ros`, `-ament-lint-auto`, `-geographic-msgs`, `-builtin-interfaces`, `-rosidl-default-generators`); all except `geographic_msgs` are already pulled by `ros-core`, so CI cannot show them missing and this is a manifest-honesty fix (the owner-approved Checkpoint decision), not a build fix. No system (non-ROS) key is missing: no Boost, GDAL, PROJ or yaml-cpp include in either package.
7. **CI (separate commit).** New `.github/workflows/ci.yml`, drafted and syntax-checked (YAML parse) in the scratch area; modelled on marine_ais's and unh_marine_autonomy's. Name `CI`; `on` push and pull request to `[rolling, jazzy]`; header comment says the same file is carried on both branches. Two jobs, `ros:jazzy-ros-core` and `ros:rolling-ros-core` on `ubuntu-24.04`, the Rolling job `if: (github.base_ref || github.ref_name) != 'jazzy'`. Each: apt update + `ros-dev-tools git`; `actions/checkout@v6`; rosdep init (guarded) + update; workspace `../ws/src` with a symlink to the checkout; clone geographic_info at `f70b81a438172cd7a066dc1b18314d70e0eb6389` with `COLCON_IGNORE` in its `geographic_msgs` and `geographic_info`; sparse clone of unh_marine_autonomy (`--depth 1 --filter=blob:none --sparse -b <jazzy|rolling>`, `sparse-checkout set marine_vertical_datum`, so colcon sees one package; the folder is identical on the two branches today); `rosdep install --ignore-src -y --from-paths $(colcon list --base-paths src --paths-only --packages-up-to mru_transform_interfaces mru_transform)` with `set -euo pipefail` and a non-empty check, no `-r`; `colcon build --packages-up-to mru_transform_interfaces mru_transform`; `colcon test --packages-select mru_transform_interfaces mru_transform --event-handlers console_direct+ --return-code-on-test-failure` without sourcing `install/`; `colcon test-result --verbose` with `if: always()`. Verified here with the same commands in a clean `/opt/ros/rolling`-only environment on a workspace laid out like CI (colcon list gives exactly `geodesy`, `marine_vertical_datum`, `mru_transform_interfaces`, `mru_transform`). System keys pulled in: `pkg-config`, `proj` (libproj-dev), `yaml-cpp` (libyaml-cpp-dev) by `marine_vertical_datum`; `python3-pyproj`, `python3-catkin-pkg` by `geodesy`; all resolve on noble and resolute.
8. **Retire the old scaffold (separate commit).** Delete `upstream.repos` and `.agents/ci_local_upstream_extra.sh` (and the then-empty `.agents/`). Reason: written for industrial_ci / the workspace's container `ci_local.sh`; the new workflow does not read them, `ci_local.sh` is no longer in the workspace (container runner declined, `docs/fork_parity.md` row 39), and `upstream.repos` pins `geographic_info` at `ros2` (a branch without the `1.0.7` fix) and unh_marine_autonomy at `jazzy` for both distros, so keeping it leaves a second, wrong dependency list. The CI workflow header carries the dependency pins.

Duration estimate, per job, from marine_ais's last run (Jazzy job 2 m 02 s: apt and tools 40 s, rosdep install 21 s, build 33 s, test 3 s) and this package's measured cost (clean four-package build 93 s CPU, 23 s wall on 20 cores; tests 11 s): about 4 minutes, range 3 to 6; the jobs run in parallel. unh_marine_autonomy's 15-package run takes about 7.

## Files to Change

| File | Change |
|------|--------|
| `mru_transform/CMakeLists.txt` | step 1 |
| `mru_transform/package.xml` | steps 1, 6 |
| `mru_transform/src/mru_transform.cpp` | steps 2, 3 |
| `mru_transform/include/mru_transform/{twist_rotation_utils,water_line_offset,water_line_lever_arm}.hpp`, `.../nodes/sea_surface_estimator.hpp` | step 2 |
| `mru_transform/test/{test_twist_rotation,test_water_line_offset,test_water_line_lever_arm}.cpp` | step 2 |
| `mru_transform/test/test_map_origin.cpp` | step 3 (new test) |
| `mru_transform_interfaces/package.xml` | steps 4, 5 |
| `.github/workflows/ci.yml` | step 7 (new) |
| `upstream.repos`, `.agents/ci_local_upstream_extra.sh` | step 8 (deleted) |

## Prior Art

rolker/unh_marine_autonomy `rolling` (`marine_sidescan_mosaic`, `marine_survey_index`, `bathymetry_layer`): `target_include_directories(... ${geodesy_INCLUDE_DIRS})` plus `${geodesy_LIBRARIES}`, floor as `ais_layer`. Taken; the library uses the plain list because its `target_link_libraries` is plain (sidescan's is keyword form; converting this one is a bigger edit). rolker/marine_ais and unh_marine_autonomy CI workflows: taken (geodesy clone, `COLCON_IGNORE`, paths-only rosdep, no `install/` sourcing, skip expression). Differences: a single build step including geodesy's tests (as unh_marine_autonomy), a sparse checkout instead of `.repos` (that repository's workflow has no `.repos` need for a single package). Rejected: `vcs import upstream.repos` (clones whole repos; the old prune script existed only to undo that); the `.get()` listener form (silent bool conversion); `ament_lint_common` (293 failures). No prior art for the TransformListener split: `marine_autonomy/tf2_utils.h` passes a `shared_ptr` and `cube_bathymetry`/`coverage_assessment` pass `this`; neither is covered by a test on Rolling (see Open Questions).

## Serves

None: mru_transform has no goals or design document; it is the root-level port of rolker/unh_marine_autonomy#415 (the project building on ROS 2 Rolling and Jazzy).

## Design Picture

No change: the project has no design document, and the port changes no behaviour.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | The old CI scaffold is retired in the same change (step 8); the manifest fixes go with the CI that first checks them (rosdep without `-r`). |
| Only what's needed | Warnings stay (list below); `ament_lint_common`, the dead CMake text and the 3.8 minimum are not touched; the only addition beyond the issue's causes is the listener/broadcaster fix, which the build forces. |
| Test what breaks | The listener fix has a mutation-checked test; CMake and header renames are covered by the build. |
| Verify before claiming | Baseline and every fix built and run here; Jazzy claims below are from reading upstream, marked as such. |
| Know whether it works | See Control path: unit tests only. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 | Yes | Entries in `.agent/work-plans/issue-51/progress.md` through the standard script; carried in the PR to `rolling`, not backported. |
| 0001 / 0017 | No | No design document, no change to the picture. |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `ament_target_dependencies` in `mru_transform` | the other consumers of the library's headers pick up geodesy through the PUBLIC include and link | Yes (step 1) |
| the old CI scaffold | nothing reads it | Yes (step 8) |
| the listener construction | other repos with the same call (below) | No: follow-up in #415 |

## Jazzy facts, by reading (nothing here was built on Jazzy; Ubuntu 26.04, no Docker)

- tf2 `jazzy` (geometry2, `tf2_ros` 0.36.23): `tf2/utils.hpp`, `tf2/LinearMath/{Matrix3x3,Quaternion,Vector3}.hpp`, `tf2_geometry_msgs/tf2_geometry_msgs.hpp` exist; `tf2/LinearMath/Vector3.h` is a one-line `#include <tf2/LinearMath/Vector3.hpp>`. `tf2_ros/{buffer,transform_listener,transform_broadcaster}.hpp` exist too (the deprecation follow-up can be done on both distros).
- `tf2_ros/transform_listener.hpp` on `jazzy`: node constructor is `template<class NodeT> TransformListener(BufferCore&, NodeT&& node, bool spin_thread = true, ...)` calling `node->get_node_base_interface()`, so a pointer-like node; a four-`SharedPtr` interface constructor; no `RequiredInterfaces`. `transform_broadcaster.hpp`: `TransformBroadcaster(NodeT&& node, ...)` via `rclcpp::node_interfaces::get_node_parameters_interface(node)`; rclcpp `jazzy` `test_get_node_interfaces.cpp` calls `get_node_topics_interface(node_reference)`.
- geodesy released for Jazzy is 1.0.6-2 (rosdistro `jazzy/distribution.yaml`, geographic_info-release): include dir has only `utm.h`, `wgs84.h`: no `ecef.h`, **no `geodesics.h`**. Tag 1.0.7 and `f70b81a` have `ecef.h ellipsoid.h geodesics.h utm.h wgs84.h wgs84_ellipsoid.h`. geodesy exports `ament_export_include_directories(include)` and `ament_export_libraries(geoconv)`, hence `geodesy_INCLUDE_DIRS` and `geodesy_LIBRARIES`; at `f70b81a` it needs `geographic_msgs::validation`, which geographic_msgs 1.0.6 on Jazzy defines (read at tag 1.0.6). So released `geographic_msgs` 1.0.6 is enough, as marine_ais's Jazzy job (green on Jazzy) already shows.
- `${<msgs>_TARGETS}`, `rclcpp::rclcpp`, `tf2::tf2`, `tf2_ros::tf2_ros`, `marine_vertical_datum::marine_vertical_datum` are used by this repository today on Jazzy; no new target form is introduced.
- unh_marine_autonomy `jazzy` and `rolling` `marine_vertical_datum` folders are byte-identical today (`diff -rq`); its `package.xml` declares `pkg-config`, `proj`, `yaml-cpp`.
- Not read or settled: that the new test passes on Jazzy's DDS timing, and the `else` branch of the listener helper compiling against Jazzy's rclcpp (it is the unchanged original expression, so it should); the Jazzy job is the check, and it runs on the PR to `rolling` before any backport.

## Left alone, to be tracked (one follow-up issue)

- `tf2_ros/{buffer,transform_listener,transform_broadcaster}.h` deprecated for `.hpp` (12 includes): `include/mru_transform/mru_transform.hpp:19-21`, `include/mru_transform/nodes/chart_datum_node.hpp:48-50`, `include/mru_transform/nodes/sea_surface_estimator.hpp:16-18`, `include/mru_transform/map_frame.hpp:12`, `include/mru_transform/water_line_lever_arm.hpp:9`, `test/test_water_line_lever_arm.cpp:19`. Warning once per translation unit that includes them (7 buffer, 7 broadcaster, 6 listener).
- Free `rclcpp::spin_some(node)` (deprecated): `test/test_map_origin.cpp:466` and `:479`. (The `exec.spin_some()` calls in the other tests are the executor member and do not warn.)
- `cmake_minimum_required(VERSION 3.8)`: `mru_transform_interfaces/CMakeLists.txt:1` only (CMake 4.2 deprecation warning at configure); `mru_transform` is 3.14.4. The dead commented-out copy of that CMake file, `mru_transform_interfaces/CMakeLists.txt:33-51`, is left (not taken from the Issue Review).
- `-Wunused-parameter` on `request` in `MRUTransform::resetMapFrameService`, `src/mru_transform.cpp:433` (before step 3; not a Rolling effect).
- `ament_lint_common` not declared in `mru_transform`: enabling it adds 293 failing lint tests (step 6). `mru_transform_interfaces` calls no `ament_lint_auto`, so `ament_xmllint` never runs on its manifest (cause 9 has no test).
- geodesy's own warning at `geodesy/geodesics.h:234` (`-Wparentheses`), upstream, not ours.
- `.agents/README.md` does not exist in this repository (only the script step 8 retires).

## Backport to `jazzy` (by hand, after the `rolling` PR merges)

Cherry-pick, in order, steps 1 to 8 (each is one commit). Not backported: the `.agent/work-plans/issue-51/` records (settled). `rolling` and `jazzy` are both at `03871aa` today, so every commit applies cleanly unless `jazzy` moves first (the files touched are `mru_transform/CMakeLists.txt`, `package.xml`, `src/mru_transform.cpp`, headers and tests; a conflicting #49-style change would show there). The CI commit is the same file on both branches; on `jazzy` the Rolling job is skipped by its `if`. Step 3's test and helper cannot be shown to pass on Jazzy from here; the Jazzy job on the `rolling` PR is the first evidence.

## Control path

`mru_transform` is in the vehicle's navigation path, not itself a controller. Per its README it consumes the MRU position, orientation and velocity topics and publishes `earth`, `map`, `odom`, `base_link` (and the level/north-up variants) on `/tf` plus the odometry topic; `sea_surface_estimator` and `chart_datum_node` add `map_tide`/`chart_datum` frames. Everything that navigates or steers from `map`/`odom`/`base_link` consumes these. `.agents/README.md` does not exist here, so this comes from `README.md` and the code.

Behaviour-neutral arguments: step 1 passes the same include directories and the same static `geoconv` library that `ament_target_dependencies` passed (and the `PUBLIC`/plain-list choice keeps consumers' view identical); step 2 changes include names whose Jazzy `.h` is a forwarder to the `.hpp`; steps 4 to 6 and 8 touch manifests and CI, not the built artefacts. Step 3 is the one change to executable behaviour and is the only place a mistake would be silent (listener on a node of its own: a second spin thread, different QoS-override parameters, a node in the graph); the new test catches exactly that, and `ros2 node list` on the built node shows one node (`/mru_transform`) with the `/tf` publisher and subscription on it.

What would break it: a wrong overload choice in the listener (above); a different `geodesy` than `f70b81a` or later (a version floor now stops 1.0.6 at configure); a different `marine_vertical_datum` branch than the one the vehicle runs.

Not provided by this port: any run against a vehicle, a bag replay, or a simulation; any Jazzy run from here; any run of `chart_datum_node`, `sea_surface_estimator`, `tide_copier` or the launch files outside their unit tests (the launch files are not even imported by a test); any Nav2 or downstream consumer on Rolling. The owner's validation before a Rolling deployment is a sim or bag run with the transforms compared against the Jazzy build.

## Pull request wording

One PR into `rolling` (not the default branch, so a close keyword is inert). Title: `Port to ROS 2 Rolling: build fixes, manifests and CI (#51)`. Body: what was built and tested, what was not (Jazzy locally, vehicle, sim, launch files); `Part of #51`; "the owner closes #51 after the backport to `jazzy`"; the follow-up list above (or the issue number once drafted); the cause-14 note.

## Open Questions

- Add the TransformListener/Broadcaster finding to #415 as cause 14, and check the same call in `cube_bathymetry` (`cube_bathymetry_node.hpp:522`, `coverage_assessment_node.hpp:548`, both `TransformListener(*tf_buffer_, this)`: on Rolling that compiles and silently becomes the bool overload) and `marine_autonomy/include/marine_autonomy/tf2_utils.h:50` (`shared_ptr`; fails to compile when instantiated). Recommendation: yes, host does it.
- CI uses the `rolling`/`jazzy` branches of unh_marine_autonomy, not pinned SHAs (accepted precedent in that repository's own CI). Recommendation: keep branches.
- Retire vs keep the scaffold (step 8). Recommendation: retire; unh_marine_autonomy and cube_bathymetry keep their own `upstream.repos`, so the same question is open there.
- `test_lifecycle_reconfigure` and `test_map_origin` prove negatives on live DDS with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on fixed domains; a container runner may behave differently from this machine. If the first CI run fails there, treat it as a test-environment finding, not a port finding.
- `ament_lint_common` and an `ament_lint_auto` call in `mru_transform_interfaces`: follow-up, not the port.

## Estimated Scope

Single PR into `rolling` (8 commits), then a second PR to `jazzy` by cherry-pick.
