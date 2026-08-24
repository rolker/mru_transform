# Plan: sea_surface_estimator: cleanup -> configure throws ParameterAlreadyDeclaredException

## Issue

https://github.com/rolker/mru_transform/issues/34

## Context

`rclcpp`'s `declare_parameter()` throws `ParameterAlreadyDeclaredException` if a
parameter name is already declared on the node. All four `rclcpp_lifecycle::LifecycleNode`
subclasses in this package declare their parameters unconditionally in
`on_configure()`, so a `configure -> cleanup -> configure` cycle throws out of
the second `on_configure` call:

- `sea_surface_estimator.cpp` — 7 `declare_parameter()` calls, `on_cleanup` resets
  TF/lever-arm/buffer state.
- `chart_datum_node.cpp` — 11 `declare_parameter()` calls, `on_cleanup` resets
  PROJ/TF/config state.
- `nav_sat_fix_to_velocity.cpp` — 2 `declare_parameter()` calls, **no `on_cleanup`
  override at all**.
- `tide_copier.cpp` — 4 `declare_parameter()` calls, `on_cleanup` is a bare
  passthrough to the base class.

(Per the `## Issue Review` entry in this file's `progress.md`: `mru_transform_node.cpp`
is a plain `rclcpp::Node`, not a `LifecycleNode`, and is correctly out of scope.)

This blocks #32's lever-arm cache invalidation from ever being exercised in the
field on `sea_surface_estimator`, because the transition that would trigger it
(`cleanup` -> `configure`) never completes. `tide_copier` feeds `map_tide` into
every downstream sounding, so its lifecycle correctness has the same field
stakes even though it wasn't named in the issue's original fix sketch.

Scope for this PR is settled (per the operator): all four affected lifecycle
nodes, one PR, atomic commits inside.

## Approach

### Design decision 1 — `has_parameter()` guards at the declare site, not `undeclare_parameter()` in `on_cleanup`

Two ways to make re-declaration safe:

- **(a) Guard the declare with `has_parameter()`** — skip `declare_parameter()`
  when the name is already declared, and `get_parameter()` unconditionally.
- **(b) `undeclare_parameter()` every declared name in `on_cleanup`** — the
  node's declared-parameter set follows the lifecycle state.

**Decision: (a), applied consistently across all four nodes.** Three reasons,
in order of weight:

1. **(b) breaks the workflow this issue exists to enable.** The motivating
   use is reconfiguring a boat between survey lines: `ros2 param set`, then
   `cleanup` -> `configure` to pick the new value up. Undeclaring on cleanup
   discards that value, so the next `configure` re-reads the launch override
   and the operator's change is silently lost. Nor can the value be staged
   while unconfigured — `set` on an undeclared parameter is rejected. With
   the guard, the second `on_configure` skips the declare and `get_parameter`
   returns the operator's value: the node genuinely reconfigures.
2. **(b) cannot fix the failed-`configure` path, and (a) can.**
   `chart_datum_node` returns `CallbackReturn::FAILURE` after 8 of its 11
   declares (the `publish_rate` / `recalc_interval` validation at
   `chart_datum_node.cpp:94-103`). A failed `configure` returns the FSM to
   `unconfigured` **without** invoking `on_cleanup`, and `cleanup` is not a
   legal transition from `unconfigured` — so an undeclare-in-cleanup can
   never run on this path and the operator's corrected retry still throws.
   A declare-site guard fixes it, because the guard is on the path that
   re-runs. This is not theoretical: correcting a bad `publish_rate` and
   re-configuring is exactly what an operator does with the node already
   launched.
3. **The guard is the prevailing convention, upstream and in-house.** nav2
   ships `nav2_util::declare_parameter_if_not_declared` for precisely this
   case. Every lifecycle node in this workspace that has solved this bug used
   a guard: `udp_bridge`'s documented `declareIfMissing()`, `s57_grids`'
   `grid_publisher.cpp` and `s57_grids_catalog.cpp`, `helm_manager`,
   `marine_control/control_server`, `manda_coverage`. `undeclare_parameter()`
   appears nowhere in the workspace. ADR-0008 (follow ROS 2 conventions)
   therefore points at (a), not (b).

The cost of (a) is that `ros2 param list` on an unconfigured node that has
been configured before still lists the parameters, at their last values. That
is the price of keeping the operator's value, and it is what every in-house
precedent already does.

**Form:** inline `if (!has_parameter(name)) { declare_parameter(name, default); }
get_parameter(name, member);`, matching `s57_grids/grid_publisher.cpp` verbatim
rather than introducing a new shared helper. `tide_copier` currently uses the
`member_ = declare_parameter<T>(name, member_)` return-value form; that form has
no guarded equivalent, so those four lines convert to the declare/get pair.

### Design decision 2 — what `on_cleanup` should release

`on_cleanup` must undo what `on_configure` did. The in-house model is
`s57_grids/grid_publisher.cpp`'s `on_cleanup`, which releases every resource its
`on_configure` created — services, timers, TF listener and buffer, the catalog.

The four nodes here are **not** symmetric today and none of them matches that
model (this corrects a wrong claim in the previous revision of this plan, which
asserted the other three already released pub/sub — they do not):

| Node | `on_cleanup` today | Missing |
|---|---|---|
| `sea_surface_estimator` | TF listener/buffer, lever-arm cache, odom buffer | broadcaster, `tide_estimate` publisher, odom subscription |
| `chart_datum_node` | PROJ, config, TF trio, cached validity | 3 publishers, both timers (defensively) |
| `nav_sat_fix_to_velocity` | *(no override)* | everything, incl. `last_navsatfix_` |
| `tide_copier` | *(bare passthrough)* | publisher, subscription |

**Decision: bring all four to the grid_publisher target state** — every
publisher, subscription, timer, broadcaster and cached-state member created or
populated under a configuration is released in `on_cleanup`. Subscriptions are
released before the publishers their callbacks write to.

`nav_sat_fix_to_velocity` needs `last_navsatfix_` cleared specifically: it is
the previous fix used to difference positions into a velocity. Retained across
a cleanup, the first fix after a re-configure differences against a
pre-cleanup fix and reports a velocity averaged over the cleanup gap —
whenever that gap is shorter than `maximum_interval_` (default **2 s**, i.e.
routinely for a quick reconfigure, and trivially under sim/bag time). Clearing
it makes the first post-reconfigure sample seed the state instead of producing
a fabricated velocity.

### Design decision 3 — publishers must be lifecycle publishers (live bug, fixed here)

Found by the plan review while checking decision 2, and adopted into scope by
the operator:

`tide_copier` and `nav_sat_fix_to_velocity` hold their publishers as
`rclcpp::Publisher<T>::SharedPtr`, not
`rclcpp_lifecycle::LifecyclePublisher<T>::SharedPtr`.
`rclcpp::Publisher::publish` is a **non-virtual** template that
`LifecyclePublisher` merely hides, so publishing through a base-class pointer
compiles and **bypasses the activation gate entirely**. Neither node's callback
checks lifecycle state either (`sea_surface_estimator`'s does).

The consequence is live today: a `tide_copier` that has been deactivated — or
cleaned up, since it never released its subscription — keeps copying `map_tide`
into `/tf`. `map_tide` is the tide applied to every sounding, so the tide feeding
the survey is currently unaffected by its node's lifecycle state. That is the
opposite of what a managed node promises an operator.

**Fix, in this PR** (it is inside the same callbacks and members the parameter
fix already rewrites, and shipping the parameter fix while leaving this would
mean touching these files twice):

- change both member types to `rclcpp_lifecycle::LifecyclePublisher<T>::SharedPtr`,
- release the subscription (and publisher) in `on_cleanup` per decision 2,
- guard both callbacks on
  `get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE`,
  matching `sea_surface_estimator::odometry_callback`.

`nav_sat_fix_to_velocity`'s guard returns **before** touching `last_navsatfix_`,
so an inactive interval leaves a stale fix behind rather than seeding from one;
the `maximum_interval_` check then rejects the first sample after re-activation,
which is the correct outcome.

### Design decision 4 — regression fixture: in-process construction, not `launch_testing`

The package has no lifecycle test harness today. Building one here (rather than
deferring) is cheaper than building it per-node later, and the operator has
called this in-scope and non-deferrable.

- **`launch_testing`** — spawn the real executable, drive it through
  `ros2 lifecycle` service calls. Heavier (process spin-up, service round trips
  per transition) and coarser: an uncaught C++ exception surfaces as a non-zero
  exit, not as an assertion naming the transition that failed.
- **In-process construction** — instantiate the concrete `LifecycleNode`
  subclass in a gtest binary and call its inherited `configure()` / `cleanup()`.
  Fast, and gives a typed assertion at exactly the transition that used to
  throw, plus direct `get_parameter()` access to assert the *value-survival*
  semantics decision 1 turns on.

**Decision: in-process construction.** What it does not cover, and is accepted:
the real executables' `main()`, and the `ros2 lifecycle set` service path the
operator actually drives. (The package's other gtest, `test_subscribe_once.cpp`,
also constructs its subject in-process, but it builds a plain helper class, not
a node with a `main()` — so it is a precedent for the harness style, not for
this specific extraction.)

**Prerequisite refactor**: all four node classes are defined inline in their
`nodes/*.cpp` files alongside `int main()`, so a test binary cannot `#include`
them without a duplicate-`main` link collision. Extract each class into
`include/mru_transform/nodes/<name>.hpp`, leaving `nodes/<name>.cpp` as
`#include` + `main()`. Each constructor also gains a
`const rclcpp::NodeOptions & options = rclcpp::NodeOptions()` parameter, which
the failed-configure test needs to inject a bad `publish_rate` override before
the first `configure`.

These headers are genuinely installed: `install(DIRECTORY include/ ...)` ships
everything under `include/mru_transform/` into the installed include tree. They
are simply not part of an exported CMake target, so no `ament_export_*` change
is required. (This corrects a second wrong claim in the previous revision, which
said they would not be installed, and a third: `orientation_sensor.hpp` is a
declaration header with `src/orientation_sensor.cpp` compiled into the exported
library — it is not precedent for a header-only node class.)

### Design decision 5 — `tide_copier` has no build target

`tide_copier.cpp` has **no `add_executable`** in `CMakeLists.txt` and is not
installed, yet `launch/tide_copier_launch.py` launches `executable='tide_copier'`
— which fails at launch with "executable not found". The file compiles for the
first time in this PR (the test binary must build it). Add the missing
`add_executable` + `install(TARGETS ...)` alongside the fix rather than leaving
a node that is tested but unshippable.

### Steps (two commits per node: mechanical move, then behaviour + test)

Each node is split so the large moves are verifiable by inspection:

1. **`sea_surface_estimator` (move)** — extract the class verbatim into
   `include/mru_transform/nodes/sea_surface_estimator.hpp` (+ `NodeOptions`
   ctor arg); `nodes/sea_surface_estimator.cpp` becomes `#include` + `main()`.
   No behaviour change.
2. **`sea_surface_estimator` (fix + test)** — guard the 7 declares; extend
   `on_cleanup` per decision 2. Add `test/test_lifecycle_reconfigure.cpp` with
   the `LifecycleReconfigureTest` fixture (`rclcpp::init`/`shutdown` in
   `SetUp`/`TearDown`, per `test_subscribe_once.cpp`) and its first cases:
   reconfigure cycle, and parameter-value survival across the cycle. Register
   the `ament_add_gtest` target.
3. **`chart_datum_node` (move)** — same mechanical extraction (558 lines).
4. **`chart_datum_node` (fix + test)** — guard the 11 declares; extend
   `on_cleanup`; add the reconfigure, value-survival **and failed-configure**
   cases (`publish_rate:=0` -> `FAILURE` -> corrected `configure` must reach
   `inactive` without throwing). Extend the test target's links (datum_config,
   `PkgConfig::PROJ`, geodesy).
5. **`nav_sat_fix_to_velocity` (move)** — extract (85 lines).
6. **`nav_sat_fix_to_velocity` (fix + test)** — guard the 2 declares; add the
   node's first `on_cleanup` (params kept, pub/sub released, `last_navsatfix_`
   cleared); `LifecyclePublisher` + lifecycle guard per decision 3. Add its
   test cases, including one that the retained-fix bug is gone.
7. **`tide_copier` (move)** — extract (73 lines) and add the missing
   `add_executable`/`install` per decision 5.
8. **`tide_copier` (fix + test)** — guard the 4 declares (converting the
   return-value declare form); release pub/sub in `on_cleanup`;
   `LifecyclePublisher` + lifecycle guard per decision 3. Add its test cases.

## Files to Change

| File | Change |
|------|--------|
| `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp` | New — `SeaSurfaceEstimator`, guarded declares, fuller `on_cleanup` |
| `mru_transform/nodes/sea_surface_estimator.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp` | New — `ChartDatumNode`, guarded declares, fuller `on_cleanup` |
| `mru_transform/nodes/chart_datum_node.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/include/mru_transform/nodes/nav_sat_fix_to_velocity.hpp` | New — `NavSatFixToVelocity`, guarded declares, first `on_cleanup`, `LifecyclePublisher` + active-state guard |
| `mru_transform/nodes/nav_sat_fix_to_velocity.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/include/mru_transform/nodes/tide_copier.hpp` | New — `TideCopier`, guarded declares, real `on_cleanup`, `LifecyclePublisher` + active-state guard |
| `mru_transform/nodes/tide_copier.cpp` | Reduced to `#include` + `main()` |
| `mru_transform/test/test_lifecycle_reconfigure.cpp` | New — reconfigure / value-survival / failed-configure cases, one group per node |
| `mru_transform/CMakeLists.txt` | Register `test_lifecycle_reconfigure`; add the missing `tide_copier` executable + install |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | The failure mode (`configure -> cleanup -> configure`) gets a direct regression test per node — plus the two paths a naive fix leaves broken: value survival and failed-configure retry. |
| A change includes its consequences | The guard keeps the node re-configur**able**, not merely re-configur**ing**; cleanup is brought to a consistent target state across all four nodes; the lifecycle-gate bug found in the same callbacks is fixed rather than left to rot; the un-buildable `tide_copier` target is fixed rather than shipped as a tested-but-unlaunchable node. |
| Improve incrementally | Eight atomic commits — a no-op move and a reviewable fix+test per node — rather than four commits that each bury a behaviour change inside a 400-line move. |
| Capture decisions, not just implementations | Five design decisions recorded with their reasoning, including where the previous revision of this plan was wrong and why. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — Follow ROS 2 Official Conventions | Yes | The declare-if-not-declared guard is the upstream (nav2) and in-house convention; the lifecycle-publisher fix restores the activation gate a managed node is supposed to honour. No new ADR needed. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `sea_surface_estimator`'s `on_configure`/`on_cleanup` | #32's lever-arm cache invalidation, which this PR makes reachable in the field | Yes — no code change needed; `setWaterLineFrame()` already runs in `on_configure` and now actually executes on a second pass |
| Node class location (inline `.cpp` -> header) | These headers are installed by `install(DIRECTORY include/ ...)`, but belong to no exported target | Yes — no `ament_export_*` change needed |
| `tide_copier`/`nav_sat_fix_to_velocity` publisher types | Their callbacks must check lifecycle state, since `LifecyclePublisher::publish` only warns (and drops) when inactive | Yes — decision 3 |
| `tide_copier` gaining a build target | Nothing downstream; the launch file already expects the executable to exist | Yes — decision 5 |
| `nav_sat_fix_to_velocity` gaining its first `on_cleanup` | Its `on_activate`/`on_deactivate` remain unoverridden base no-ops. With the publisher now a `LifecyclePublisher` and the callback state-guarded, the base no-ops are correct — no override needed | Yes — deliberately not adding empty overrides |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): None — `README.md`'s parameter tables
  describe configure-time behaviour and are unaffected. Nothing externally
  visible changes except that a deactivated `tide_copier` / `nav_sat_fix_to_velocity`
  now correctly stops publishing, which is the documented lifecycle contract
  rather than a documented behaviour of these nodes.
- **Agent-instruction candidates** (proposals only): the declare-if-not-declared
  guard for `LifecycleNode` subclasses, and the non-virtual `publish` trap
  behind holding a lifecycle publisher as `rclcpp::Publisher`, are both reusable
  ROS 2 patterns not written down anywhere in this workspace.
  `.agent/knowledge/ros2_development_patterns.md` could gain a short note on
  each — proposed, not applied here; operator's call.

## Open Questions

- None blocking implementation — decisions 1-5 are settled above (1, 3 and 5 by
  the operator following the plan review).

## Estimated Scope

Single PR, eight atomic commits (a mechanical extraction and a fix+test pair per
node), all sharing one `test_lifecycle_reconfigure.cpp` fixture.
