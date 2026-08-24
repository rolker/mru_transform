# Plan: sea_surface_estimator: cleanup -> configure throws ParameterAlreadyDeclaredException

## Issue

https://github.com/rolker/mru_transform/issues/34

## Context

`rclcpp`'s `declare_parameter()` throws `ParameterAlreadyDeclaredException` if a
parameter name is already declared on the node. All four `rclcpp_lifecycle::LifecycleNode`
subclasses in this package declare their parameters unconditionally in
`on_configure()` and never release them in `on_cleanup()`, so a
`configure -> cleanup -> configure` cycle throws out of the second `on_configure`
call:

- `sea_surface_estimator.cpp` — 7 `declare_parameter()` calls, `on_cleanup` resets
  TF/lever-arm/buffer state but never undeclares.
- `chart_datum_node.cpp` — 11 `declare_parameter()` calls, `on_cleanup` resets
  PROJ/TF/config state but never undeclares.
- `nav_sat_fix_to_velocity.cpp` — 2 `declare_parameter()` calls, **no `on_cleanup`
  override at all**.
- `tide_copier.cpp` — 4 `declare_parameter()` calls, `on_cleanup` is a bare
  passthrough to the base class.

(Per the `## Issue Review` entry in this file's `progress.md`: `mru_transform_node.cpp`
is a plain `rclcpp::Node`, not a `LifecycleNode`, and is correctly out of scope.)

This blocks #33's lever-arm cache invalidation from ever being exercised in the
field on `sea_surface_estimator`, because the transition that would trigger it
(`cleanup` -> `configure`) never completes. `tide_copier` feeds `map_tide` into
every downstream sounding, so its lifecycle correctness has the same field
stakes even though it wasn't named in the issue's original fix sketch.

Scope for this PR is settled (per the operator): all four affected lifecycle
nodes, one PR, atomic commits inside.

## Approach

### Design decision 1 — `undeclare_parameter()` in `on_cleanup`, not `has_parameter()` guards

Two ways to make re-declaration safe:

- **(a) Guard the declare with `has_parameter()`** — skip `declare_parameter()`
  (and just `get_parameter()`) if already declared. Cheap, but the parameter set
  never shrinks: `ros2 param list` on an *unconfigured* node (fresh process,
  post-cleanup, or after a *failed* configure that still declared some params)
  keeps showing every parameter from the last successful configure, with its
  last resolved value. That is misleading in the field — an operator inspecting
  an unconfigured node's parameters sees stale configuration state that looks
  current.
- **(b) `undeclare_parameter()` every declared name in `on_cleanup`** — the
  node's declared-parameter set follows the lifecycle state: populated only
  while configured/inactive/active, empty in `unconfigured`. This matches how
  the managed-node lifecycle is documented to behave (`cleanup` undoes what
  `configure` did) and is what ADR-0008 (follow ROS 2 conventions) points at.

**Decision: (b), applied consistently across all four nodes.** Each
`on_cleanup` gets one `undeclare_parameter(name)` call per parameter declared
in `on_configure`, mirroring the declare list by name so a future parameter
addition is a paired two-line diff (declare + undeclare) rather than requiring
a separate tracked-names data structure to stay in sync. No new shared
abstraction (e.g. a templated "parameter scope" helper) — the four files
already differ stylistically (`declare_parameter` + `get_parameter` vs.
`declare_parameter<T>(...)`'s return-value form in `tide_copier`), and a
one-line-per-parameter mirror is the smallest, most directly-reviewable diff
for a defect of this shape.

`undeclare_parameter()` throws if the name isn't declared, so each cleanup
guards with `has_parameter()` first — this makes `on_cleanup` itself safe to
call defensively (e.g. a `configure` that FAILED partway through, before all
declares ran, followed by a `cleanup`) without deciding here whether that
partial-configure path is currently reachable per node.

### Design decision 2 — regression fixture: in-process construction, not `launch_testing`

The package has no lifecycle test harness today. Building one here (rather than
deferring) is cheaper than building it per-node later, and the operator has
called this in-scope and non-deferrable.

Two ways to exercise the FSM:

- **`launch_testing`** — spawn the real executable as a subprocess, drive it
  through `ros2 lifecycle` service calls, and assert on exit code / log output.
  Heavier (new test infra, process spin-up, service-call round trips per
  transition) and coarser (an uncaught C++ exception surfaces as a process
  crash / non-zero exit, not a typed assertion on which exception or which
  transition failed).
- **In-process construction** — instantiate the concrete `LifecycleNode`
  subclass directly in a gtest binary and call its inherited `configure()` /
  `cleanup()` transition methods. Matches the package's existing test
  convention: `test/test_subscribe_once.cpp` already includes
  `mru_transform/orientation_sensor.hpp` directly and constructs the class
  under test in-process (with `rclcpp::init()`/`shutdown()` in
  `SetUp()`/`TearDown()`). Fast, and `EXPECT_NO_THROW(node->configure())` gives
  a precise assertion at exactly the transition that used to throw.

**Decision: in-process construction**, consistent with the existing harness
convention in this package.

**Prerequisite refactor**: all four node classes are currently defined inline
in their `nodes/*.cpp` files, which also contain `int main()` — not
`#include`-able from a test binary without a duplicate `main()` link
collision. Extract each class into a header under
`include/mru_transform/nodes/` (mechanical, no behavior change beyond the
lifecycle fix itself), leaving `nodes/*.cpp` as `#include` + `int main()`.
This mirrors how `orientation_sensor.hpp` / `datum_config.hpp` are already
split from their consumers for testability.

### Steps (one atomic commit per node; each commit is self-testing)

1. **`sea_surface_estimator`** — extract `SeaSurfaceEstimator` into
   `include/mru_transform/nodes/sea_surface_estimator.hpp`; add the 7
   `undeclare_parameter()` calls (guarded by `has_parameter()`) to
   `on_cleanup`; create `test/test_lifecycle_reconfigure.cpp` with a
   `LifecycleReconfigureTest` fixture (the shared `SetUp`/`TearDown` calling
   `rclcpp::init`/`shutdown`, per the `test_subscribe_once.cpp` pattern) and
   its first case: `TEST_F(LifecycleReconfigureTest, SeaSurfaceEstimator)`
   asserting `configure()` -> `cleanup()` -> `configure()` all succeed
   (`EXPECT_NO_THROW`, and each returns to the expected lifecycle state).
   Register the new `ament_add_gtest` target in `CMakeLists.txt`.
2. **`chart_datum_node`** — same pattern: extract to
   `include/mru_transform/nodes/chart_datum_node.hpp`, add the 11
   `undeclare_parameter()` calls to `on_cleanup`, append a
   `ChartDatumNode` `TEST_F` case to the shared fixture file, extend the
   test target's `target_link_libraries`/include dirs to match
   `chart_datum_node`'s (datum_config, PkgConfig::PROJ, geodesy, etc.).
3. **`nav_sat_fix_to_velocity`** — extract to
   `include/mru_transform/nodes/nav_sat_fix_to_velocity.hpp`; this node has
   **no `on_cleanup` override today** — add one (undeclare the 2 params,
   reset the publisher/subscription for symmetry with the other three
   nodes' cleanup — resource teardown, not just parameters, since a
   `cleanup` that leaves live pub/sub wired is its own latent gap once
   we're touching this callback anyway). Append the `TEST_F` case.
4. **`tide_copier`** — extract to
   `include/mru_transform/nodes/tide_copier.hpp`; add the 4
   `undeclare_parameter()` calls to the existing (currently-empty)
   `on_cleanup`. Append the `TEST_F` case, completing the shared fixture
   file (all four nodes covered).

## Files to Change

| File | Change |
|------|--------|
| `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp` | New — `SeaSurfaceEstimator` class, extracted from the `.cpp`, with `on_cleanup` undeclaring its 7 params |
| `mru_transform/nodes/sea_surface_estimator.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp` | New — `ChartDatumNode` class, `on_cleanup` undeclaring its 11 params |
| `mru_transform/nodes/chart_datum_node.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/include/mru_transform/nodes/nav_sat_fix_to_velocity.hpp` | New — `NavSatFixToVelocity` class, new `on_cleanup` override (undeclare 2 params + reset pub/sub) |
| `mru_transform/nodes/nav_sat_fix_to_velocity.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/include/mru_transform/nodes/tide_copier.hpp` | New — `TideCopier` class, `on_cleanup` undeclaring its 4 params |
| `mru_transform/nodes/tide_copier.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/test/test_lifecycle_reconfigure.cpp` | New — shared `configure -> cleanup -> configure` fixture, one `TEST_F` per node, built up across the four commits |
| `mru_transform/CMakeLists.txt` | Register `test_lifecycle_reconfigure` gtest target (introduced in commit 1, link libs/includes extended in commits 2-4 as each node's dependencies are pulled in) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | The exact failure mode (`configure -> cleanup -> configure` throwing) gets a direct regression test per node, not just a manual/eyeball check. |
| A change includes its consequences | Sibling audit from the Issue Review (`chart_datum_node`, `nav_sat_fix_to_velocity`, and the review-added `tide_copier`) is fully covered in this PR — no follow-up issue needed for the parameter-redeclaration bug shape. |
| Improve incrementally | Four atomic, per-node commits — each buildable and independently revertible — rather than one large mixed commit. |
| Capture decisions, not just implementations | This plan records both real design choices (undeclare vs. guard; in-process vs. launch_testing) with reasoning, per the operator's ask. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — Follow ROS 2 Official Conventions | Yes | The fix restores standard managed-node `configure`/`cleanup` symmetry (cleanup undoes configure) rather than deviating from it; no new ADR needed. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `sea_surface_estimator.cpp`'s `on_cleanup` | The lever-arm cache invalidation from #33, which this PR is what makes reachable in the field | Yes — no code change needed beyond the fix itself; #33's logic already runs inside `on_configure` and will now actually execute on a second pass |
| Node class location (inline `.cpp` -> header) | Nothing external — headers are new files under the package's existing `include/mru_transform/` public include path, but these four are node-internal implementation classes, not part of the package's exported library/API (`mru_transform` and `datum_config` are the exported targets) | Yes — no `ament_export_*` changes needed since these headers aren't part of an exported target |
| `nav_sat_fix_to_velocity.cpp` gaining its first `on_cleanup` | Its `on_activate`/`on_deactivate` are also absent (not overridden, base-class no-ops) — left as-is; adding cleanup is the minimum fix for this issue's bug shape, and the base no-op activate/deactivate isn't broken | No — not part of this issue; flagged here so it isn't mistaken for an oversight |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): None — `README.md`'s parameter tables
  and lifecycle-adjacent language (e.g. the `water_line_frame`/`datum_config_path`
  rows) describe configure-time behavior and are unaffected; no externally-visible
  behavior changes, only reconfigure robustness.
- **Agent-instruction candidates** (proposals only): The `declare_parameter()` /
  `undeclare_parameter()` pairing requirement for `LifecycleNode` subclasses is a
  reusable ROS 2 pattern this package didn't have written down anywhere. Once this
  PR lands, `.agent/knowledge/ros2_development_patterns.md` could gain a short
  note ("a `LifecycleNode`'s `on_cleanup` must undeclare everything its
  `on_configure` declares, or a `cleanup -> configure` cycle throws") — proposed,
  not applied here; operator's call whether it's worth the cross-repo promotion.

## Open Questions

- None blocking implementation — the two named design decisions (undeclare vs.
  guard; in-process vs. launch_testing) are settled above.

## Estimated Scope

Single PR, four atomic commits (one per node), each commit extending the shared
`test_lifecycle_reconfigure.cpp` fixture with that node's regression case.
