---
issue: 34
---

# Issue #34 — `SeaSurfaceEstimator::on_configure` re-declares parameters on cleanup->configure, throwing `ParameterAlreadyDeclaredException`

## Issue Review
**Status**: complete
**When**: 2026-08-23 22:57 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #34
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Verification

Read `mru_transform/nodes/sea_surface_estimator.cpp` directly (not taking the
issue body's claim on faith, per the injected-context caveat):

- `on_configure` (lines 26-75) calls `declare_parameter()` seven times:
  `sea_surface_frame`, `minimum_buffer_duration`, `maximum_buffer_duration`,
  `water_line_frame`, `chart_datum_frame`, `mhhw_frame`, `tide_range_margin`.
- `on_cleanup` (lines 87-97) resets `tf_listener_`/`tf_buffer_`, the lever-arm
  cache, and buffered odometry state, but never calls `undeclare_parameter()`
  or guards the declarations with `has_parameter()`.
- `rclcpp`'s `NodeParameters::declare_parameter()` throws
  `rclcpp::exceptions::ParameterAlreadyDeclaredException` when a parameter
  name is already declared on the node — this is standard, well-documented
  `rclcpp` behavior, not something specific to this package. So a
  `cleanup` -> `configure` cycle does throw out of `on_configure` exactly as
  described. **Claim confirmed against source, not just asserted.**
- The launch file (`launch/sea_surface_estimator_launch.py`) drives this node
  as a managed `LifecycleNode`; nothing in the launch wiring prevents an
  operator or a supervising process from issuing `cleanup` then `configure`
  in the field, so the failure mode is reachable, not theoretical.

**Sibling audit (fix sketch names `chart_datum_node` and
`nav_sat_fix_to_velocity`) — checked both, plus one the sketch omits:**

- `chart_datum_node.cpp`: 11 `declare_parameter()` calls in `on_configure`,
  `on_cleanup` present but does not undeclare — same bug shape, correctly
  named in the fix sketch.
- `nav_sat_fix_to_velocity.cpp`: 2 `declare_parameter()` calls
  (`map_frame`, `maximum_interval_seconds`) in `on_configure`; **no
  `on_cleanup` override at all** — same bug shape (the params are declared
  once and never released across a cleanup cycle), correctly named in the
  fix sketch.
- `tide_copier.cpp`: 4 `declare_parameter()` calls (`input_map_frame`,
  `input_map_tide_frame`, `output_map_frame`, `output_map_tide_frame`) in
  `on_configure`; has an `on_cleanup` override that also does not
  undeclare. **Same bug shape, but this file is not named in the issue's
  fix-sketch audit list.** Flagged as a Recommendation below rather than
  blocking — the issue's primary scope (sea_surface_estimator) stands on
  its own regardless.
- `mru_transform_node.cpp` is a plain `rclcpp::Node`, not a
  `LifecycleNode` — not affected by this class of bug, correctly excluded.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | Action needed | Issue body flags the reconfigure path as untested and notes "there is no such fixture in the package yet," but stops short of committing to build one. AGENTS.md Quality Standard ("fix it completely: add the test... check the lifecycle transition") requires the harness be added as part of this fix, not deferred — a parameter-lifecycle regression like this is exactly the kind of bug a "looks fixed" patch can silently un-fix later without a configure->cleanup->configure test. |
| Improve incrementally | OK | Scoping the fix to `sea_surface_estimator` alone (mirroring how #33 kept the tide fix reviewable on its own) is the right call — a repo-wide sweep across all lifecycle nodes in one PR would be harder to review. |
| A change includes its consequences | Watch | The fix sketch explicitly proposes auditing sibling nodes but only lists two of the three affected ones (misses `tide_copier`). Worth completing the audit scope, whether as part of this issue's follow-up note or a fresh issue, so the gap doesn't get lost. |
| Capture decisions, not just implementations | OK | The issue already documents why this was deliberately deferred from #33 and why it blocks #32's lever-arm cache invalidation from being exercised in the field — good context for future readers. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0008 — Follow ROS 2 Official Conventions | Yes | Correct handling of the lifecycle `configure`/`cleanup` transition pair is a core ROS 2 managed-node convention; the fix brings the node into conformance rather than deviating from it. No new ADR needed. |

### Dependencies

None blocking. Builds on the already-merged #32/#33 (lever-arm cache
invalidation + tide fix); this issue is what makes that cache-invalidation
logic reachable in the field, per the issue body.

### Actions
- [ ] Implementation should add a configure->cleanup->configure lifecycle-harness test (there being no such fixture yet is a reason to build one, not to skip the test) rather than shipping the parameter-declaration fix untested.
- [ ] Recommend extending the "audit the package's other lifecycle nodes" follow-up to include `tide_copier.cpp`, not just `chart_datum_node` and `nav_sat_fix_to_velocity` — it has the identical declare-without-undeclare shape.

## Plan Authored
**Status**: complete
**When**: 2026-08-23 23:07 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-34/plan.md` at `99e0795`
**Branch**: feature/issue-34 at `99e0795`
**Phases**: single (4 atomic commits, one per lifecycle node, each extending a shared regression fixture)

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-08-23 23:15 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-34/plan.md` at `99e0795`
**PR**: PR-less (`--issue` mode; branch `feature/issue-34`)
**Verdict**: changes-requested

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Four nodes + one shared fixture in one PR is the operator's settled call and is right-sized; the prerequisite extraction is the only scope question and it is justified (see findings 5-6). |
| Issue alignment | Good | Covers all four affected `LifecycleNode`s and both Issue-Review actions (regression fixture in scope; `tide_copier` added). |
| File targeting | Needs work | Complete for the chosen approach, but `nodes/*.cpp` resource teardown (publishers/subscriptions) is under-specified for three of the four nodes. |
| Consequences | Concern | Misses two reachable consequences: the failed-`configure` path (finding 3) and the operator's `ros2 param set` reconfigure workflow the issue itself cites as the motivation (finding 2). |
| Documentation & instruction impact | Good | Present and non-silent; instruction item correctly framed as a candidate. Its wording changes if finding 1 changes the decision. |
| Principle alignment | Needs work | "A change includes its consequences" — the fix as specified leaves the node re-configurable but not re-configurABLE (params revert to launch overrides). |
| ADR compliance | Concern | ADR-0008 (follow ROS 2 conventions) is cited in support of `undeclare_parameter()`, but the prevailing convention upstream and in this workspace is the `has_parameter()` guard (finding 1). |
| ROS conventions | Concern | See findings 1-4. |

### Findings
- [ ] (must-fix) Design decision 1 inverts the actual convention: nav2 ships `nav2_util::declare_parameter_if_not_declared` for exactly this case, and every lifecycle node in this workspace that solved this bug used a `has_parameter()` guard (`udp_bridge`'s documented `declareIfMissing()` wrapper, `s57_grids/grid_publisher.cpp`, `s57_grids_catalog.cpp`, `helm_manager`, `marine_control/control_server`, `manda_coverage`); `undeclare_parameter()` appears nowhere in the workspace. Re-decide, or re-justify against that evidence rather than against ADR-0008 in the abstract — `plan.md:39,57`
- [ ] (must-fix) Undeclaring breaks the reconfigure workflow the issue is filed to enable: `on_cleanup` drops the parameter, so a value the operator set with `ros2 param set` is discarded and the next `configure` re-reads the launch override; while unconfigured no value can be staged either (undeclared params reject `set`). With the guard, the second `on_configure` skips the declare and `get_parameter` picks up the operator's value — "reconfigure a boat between lines" actually reconfigures — `plan.md:39-58`
- [ ] (must-fix) The failed-`configure` path is left broken and is reachable: `chart_datum_node` returns `FAILURE` after 8 of its 11 declares (the `publish_rate`/`recalc_interval` validation), the FSM sends a failed configure straight back to `unconfigured` without invoking `on_cleanup`, and `cleanup` is not a legal transition from `unconfigured` — so the planned undeclares can never run and the next `configure` still throws. The plan's `has_parameter()`-guard rationale at `plan.md:70-73` describes that unreachable `FAILURE`-then-`cleanup` transition. A declare-site guard fixes this path; undeclare-in-cleanup cannot. Add a regression case: `publish_rate:=0` -> `FAILURE` -> valid `configure` must not throw — `plan.md:70-73`
- [ ] (must-fix) `nav_sat_fix_to_velocity`'s new `on_cleanup` needs `last_navsatfix_` cleared, not just parameters and pub/sub: the retained fix makes the first message after a re-configure compute a velocity across the cleanup gap whenever that gap is under `maximum_interval_` (default 2 s — routine for a quick cycle, and for bag/sim time) — `plan.md:127-133`
- [ ] (must-fix) The "symmetry with the other three nodes' cleanup" justification is factually wrong and should not drive the design: none of the other three release pub/sub (`sea_surface_estimator` resets TF + caches only; `chart_datum_node` resets timers/TF/PROJ but not its three publishers; `tide_copier` resets nothing). Decide the target state explicitly — `s57_grids/grid_publisher.cpp`'s `on_cleanup` is the in-house model that releases everything `on_configure` created — and apply it to all four rather than to one — `plan.md:129-133`
- [ ] (must-fix) Found while checking finding 5, field-relevant and squarely inside the callbacks this PR rewrites: `tide_copier` and `nav_sat_fix_to_velocity` hold their publishers as `rclcpp::Publisher<T>::SharedPtr`. `rclcpp::Publisher::publish` is a non-virtual template (jazzy `publisher.hpp:242,296`) that `LifecyclePublisher` only hides, so publishing through the base pointer bypasses the activation gate; neither callback checks lifecycle state (`sea_surface_estimator`'s does). A deactivated or cleaned-up `tide_copier` therefore keeps copying `map_tide` into `/tf` — the tide feeding every sounding. Fix here (member type -> `LifecyclePublisher`, release the subscription in `on_cleanup`) or file it with a note in the PR; do not leave it undecided — `plan.md:127-137`
- [ ] (suggestion) Split each node's commit into a pure mechanical class move and a separate fix+test commit. The extraction itself is justified (an in-process typed assertion needs the class declaration; no smaller route exists), but a 400-line and a 558-line whole-file move bundled with the behaviour change is what makes this PR hard to review — a no-op move commit is verifiable by inspection at zero cost — `plan.md:100-108,109-137`
- [ ] (suggestion) Two claims in the extraction rationale need correcting: `orientation_sensor.hpp` is a declaration header with `src/orientation_sensor.cpp` compiled into the exported library, not a header-only class, so it is not precedent for header-only node classes; and `install(DIRECTORY include/ ...)` does ship anything under `include/mru_transform/` into the installed include tree, so "not part of the exported API" is true only in the `ament_export_targets` sense — `plan.md:106,175`
- [ ] (suggestion) Keep the `launch_testing` rejection but restate its grounds: the substantive reasons (typed assertion at the throwing transition, no process/service round trip) hold; "consistency with the existing test" is thin, since `test_subscribe_once` constructs a plain helper class, not a node with a `main()`. Note what in-process does not cover — the real executable and the `ros2 lifecycle set` service path the operator uses — `plan.md:74-99`
- [ ] (suggestion) Pin the chosen semantics in the fixture, not just the absence of a throw: assert what `has_parameter`/`get_parameter` report after `cleanup`, and (with the guard approach) that a value set between `cleanup` and `configure` survives into the reconfigured node — that is the behaviour the field workflow depends on — `plan.md:109-137`
- [ ] (suggestion) This repo has no `.agents/README.md`; noted as a gap per AGENTS.md, not work for this PR.

## Implementation
**Status**: complete
**When**: 2026-08-23 23:41 -04:00
**By**: Claude Opus

**Plan**: `.agent/work-plans/issue-34/plan.md` at `9c9c2b2` (revised for the changes-requested review)
**Branch**: feature/issue-34 at `fc393ec`
**Commits**: 9 (plan revision + 4 mechanical extractions/fix pairs + test hardening)

### Plan revision (step 1, committed before implementing)

`9c9c2b2` rewrites the plan against every must-fix in `## Plan Review`:

- **Decision 1 flipped to `has_parameter()` guards** at the declare site.
  Recorded reasoning: undeclaring discards a value set with `ros2 param set`
  (and nothing can be staged while unconfigured), so the reconfigure workflow
  the issue exists to enable would still fail; it cannot fix the
  failed-configure path; and the guard is the upstream (nav2) and in-house
  convention (`udp_bridge`, `s57_grids` ×2, `helm_manager`,
  `marine_control/control_server`, `manda_coverage`).
- **Decision 2** names the `on_cleanup` target state explicitly, with
  `s57_grids/grid_publisher.cpp` as the in-house model, and tabulates what each
  of the four nodes actually released — correcting the previous claim that the
  other three already released pub/sub. None of them did.
- **Decision 3** adds the lifecycle-publisher bug to scope with its reasoning.
- **Decision 5** (new) records that `tide_copier` had no build target at all.
- Steps split into a mechanical move + a fix/test commit per node; the
  `orientation_sensor.hpp` and `install(DIRECTORY include/ ...)` claims corrected.

### What was built

Four `LifecycleNode`s, each extracted to `include/mru_transform/nodes/*.hpp`
(verbatim move + a defaulted `NodeOptions` ctor arg) and then fixed:

| Node | Declares guarded | `on_cleanup` now releases | Lifecycle gate |
|---|---|---|---|
| `sea_surface_estimator` | 7 | + odom subscription, `tide_estimate` pub, TF broadcaster, `logged_lever_arm_` | already had one |
| `chart_datum_node` | 11 | + 3 latched publishers, both timers | n/a (timer-driven) |
| `nav_sat_fix_to_velocity` | 2 | first `on_cleanup`: pub, sub, **`last_navsatfix_`** | added |
| `tide_copier` | 4 | first real `on_cleanup`: pub, sub | added |

Publishers in `tide_copier` and `nav_sat_fix_to_velocity` changed from
`rclcpp::Publisher<T>::SharedPtr` to `LifecyclePublisher<T>::SharedPtr`, and
both callbacks now return unless `PRIMARY_STATE_ACTIVE`.
`nav_sat_fix_to_velocity`'s guard returns *before* touching `last_navsatfix_`,
so the stale fix is rejected by the `maximum_interval_` check on re-activation
rather than seeding a gap-spanning velocity.

`chart_datum_node.cpp`'s file-scope `namespace fs = std::filesystem` alias was
dropped in favour of qualified use: `install(DIRECTORY include/ ...)` ships the
new header, and a bare `fs` alias would leak into every includer.

`tide_copier` gained the `add_executable`/`install` it never had —
`launch/tide_copier_launch.py` launches `executable='tide_copier'`, which was
never built, so that launch file could only ever have failed. This PR is the
first time `tide_copier.cpp` compiles.

### Findings worth carrying forward

- **`rclcpp_lifecycle` CATCHES exceptions thrown by transition callbacks.**
  Verified empirically by re-introducing the unguarded declares: the node logs
  `Caught exception in callback for transition 10 / Original error: parameter
  'sea_surface_frame' has already been declared` and reports ERROR — the
  exception never escapes `configure()`. `ASSERT_NO_THROW` alone therefore does
  **not** detect this bug; under the regression the parameter-survival test
  passed while every re-configure was in fact failing. Commit `fc393ec` adds a
  lifecycle-state assertion after every re-configure in the fixture, which is
  what actually catches it. Anyone testing a lifecycle transition in this
  workspace should assert the resulting state, not just the absence of a throw.
- **The deactivated-`tide_copier` bug is real and reproduced**, not merely
  argued: with the pre-fix publisher type and no state check, the new test
  fails three times over — "a configured-but-inactive node copied the tide into
  /tf", "a deactivated node kept copying", "a cleaned-up node kept copying".

### Verification

Negative check first (the fix's own regression test earning its place):
with the guards stripped from `sea_surface_estimator`, both its tests fail; with
the pre-fix publisher/gate/cleanup restored in `tide_copier`,
`TideCopierRespectsLifecycleState` fails on all three lifecycle phases. Both
were then restored and the suite re-run green.

- `./platforms_ws/build.sh mru_transform` — **pass** (no new warnings; the two
  pre-existing ones are in vendored `geodesy` and untouched `mru_transform.cpp`)
- `./platforms_ws/test.sh mru_transform` — **82 tests, 0 errors, 0 failures,
  0 skipped** (70 before this PR; +12 in `test_lifecycle_reconfigure`)
- `test_lifecycle_reconfigure`: 12/12 pass in 2.9 s — 4 reconfigure cycles,
  4 parameter-survival, `ChartDatumNodeReconfigureUsesTheNewValue`,
  `ChartDatumNodeRecoversFromFailedConfigure`, and the two live pub/sub
  lifecycle-gate tests.

This repo has no CI; the local run above is the only gate.

### Actions
- [ ] Not pushed — the host performs pushes. PR body should carry the
      lifecycle-publisher fix (decision 3) and the `tide_copier` build target
      (decision 5) as scope beyond the issue's original text.
- [ ] Agent-instruction candidates (operator's call, not applied):
      `.agent/knowledge/ros2_development_patterns.md` could gain notes on the
      declare-if-not-declared guard, on `rclcpp::Publisher::publish` being
      non-virtual (so holding a lifecycle publisher by the base type silently
      bypasses the activation gate), and on lifecycle transition tests needing
      a state assertion because the FSM swallows callback exceptions.
- [ ] This repo has no `.agents/README.md` (noted by the plan review); still a
      gap, still not work for this PR.


## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-23 23:50 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-34 at `7437a3f`
**Mode**: pre-push
**Depth**: Deep (reason: 2296 insertions / 12 files, lifecycle + resource-teardown semantics across four managed nodes)
**Must-fix**: 3 | **Suggestions**: 7
**Round**: 1 | **Ship**: continue — three must-fixes at round 1, one a resource leak on the retry-after-FAILURE path this PR's own test blesses as the supported recovery.

### Independently verified (not taken from the Implementation entry)

- Header extractions are **pure**. Diffed the removed `.cpp` lines against the added `.hpp` lines for all four moves: the only deltas are include guards, the defaulted `NodeOptions` ctor arg, `explicit` on `ChartDatumNode`, and the `namespace fs` alias replaced by qualified `std::filesystem`. No behaviour hid inside the 400/558-line moves.
- **`rclcpp_lifecycle` does swallow transition-callback exceptions.** Restored the unguarded declares (`git show e5dae40:...sea_surface_estimator.hpp`): the node logs `Caught exception in callback for transition 10 / Original error: parameter 'sea_surface_frame' has already been declared`, `configure()` does **not** throw, and the two sea_surface tests fail on the *state* assertions at `test_lifecycle_reconfigure.cpp:59` and `:152`. The `ASSERT_NO_THROW`-is-not-enough claim is correct.
- **Pre-fix `tide_copier` fails `TideCopierRespectsLifecycleState` three times** ("configured-but-inactive", "deactivated", "cleaned-up"), plus `TideCopierReconfigures` and `TideCopierKeepsOperatorParameter`. Both mutations restored; tree verified clean.
- **Clean rebuild from scratch: 0 errors.** 3 warnings, all in untouched `src/mru_transform.cpp` and vendored `geodesy/geodesics.h` — none in changed files.
- **82 tests, 0 failures**, `test_lifecycle_reconfigure` = 12 cases (so 70 before). Package suite runs in ~8.7 s.
- `tide_copier` builds and installs to `lib/mru_transform/tide_copier`, matching `executable='tide_copier'` in `launch/tide_copier_launch.py`; `tf2_msgs` was already a `<depend>`. The launch file works for the first time.
- Every publishing path is gated: `sea_surface_estimator` and both new gates check `PRIMARY_STATE_ACTIVE` at the top of the sole publish path; `chart_datum_node` publishes only from timers created in `on_activate` and reset in both `on_deactivate` and `on_cleanup`. `get_current_state()` is safe from a callback (the returned `State::id()` takes the same recursive mutex a transition holds) and cannot alter ACTIVE behaviour.

### Findings
- [x] (must-fix) PROJ context + both pipelines leak on the `FAILURE` return after `setup_proj()` succeeded; the retry `configure` overwrites the pointers and leaks again each time — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:176-195`
- [x] (must-fix) Comment claims the ACTIVE guard makes the pre-deactivation fix "too old for the maximum_interval_ check" — true only when the inactive gap exceeds 2 s; a deactivate→activate inside 2 s does publish a velocity across the muted gap. Add an `on_deactivate` that clears `last_navsatfix_`, or correct the comment — `mru_transform/include/mru_transform/nodes/nav_sat_fix_to_velocity.hpp:67-72`
- [x] (must-fix) `odometry_buffer_.begin()` is dereferenced after a prune loop that can empty the map: a negative `maximum_buffer_duration` erases the just-inserted sample and the deref is UB. Neither buffer duration is validated, unlike `tide_range_margin_` here and `publish_rate`/`recalc_interval` in the sibling node. The guard sweep is what makes a runtime `ros2 param set` reach this — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:183-184`
- [x] (suggestion) Deactivate leaves stale state that re-activation acts on: `odometry_buffer_` survives (30 s default window, so the first post-activation average spans the deactivation and reaches `map_tide`), and `chart_datum_node`'s validity flags/cached datum survive while `on_activate`'s `recalc_callback()` returns early on the TF lookup that a just-rebuilt buffer usually fails. Pre-existing gates — worth a follow-up issue, not scope creep here — `sea_surface_estimator.hpp:111`, `chart_datum_node.hpp:236` (deferred: filed as mru_transform#37; the analogous `nav_sat_fix_to_velocity` case was a must-fix and is fixed here)
- [x] (suggestion) No test pins the new `on_cleanup` releases for two nodes: deleting `odometry_subscription_.reset()`/`tide_estimate_pub_.reset()` or `chart_datum`'s `publish_timer_.reset(); recalc_timer_.reset();` leaves all 12 tests green — `mru_transform/test/test_lifecycle_reconfigure.cpp`
- [x] (suggestion) Tests use absolute `/tf`, `/fix`, `/velocity` on the default domain and prove negatives with `EXPECT_TRUE(copies.empty())`; unrelated `/tf` traffic on the dev host fails them, and `get_subscription_count() >= 2` can be satisfied by an external subscriber, making a negative check vacuous. Set `ROS_LOCALHOST_ONLY=1` + a unique `ROS_DOMAIN_ID` via `set_tests_properties`, or namespace the topics — `mru_transform/test/test_lifecycle_reconfigure.cpp:373-436`
- [x] (suggestion) Negative assertions use fixed 300-500 ms `spin_for` windows while the positives get 5 s `spin_until` budgets — on a loaded box the negatives can pass vacuously. A round-trip sentinel would make them deterministic — `mru_transform/test/test_lifecycle_reconfigure.cpp:287,322,415` (deferred: filed as mru_transform#38; the isolation half of the same weakness is fixed here, so the negatives are no longer breakable or satisfiable by outside traffic)
- [x] (suggestion) `lifecycle_msgs` is included directly by all four installed headers and the test but is not a `<depend>` / `find_package` — it compiles only through `rclcpp_lifecycle`'s transitive export (REP-149) — `mru_transform/package.xml`, `mru_transform/CMakeLists.txt`
- [x] (suggestion) `install(DIRECTORY include/ ...)` now ships four global-namespace node classes as public API, and `chart_datum_node.hpp` pulls `proj.h`, which is not exported — so the headers ship but cannot be consumed downstream. Either `PATTERN "nodes" EXCLUDE` from the install or put the classes in `namespace mru_transform` (the "no `ament_export_*` change needed" conclusion from the plan review stands; this is the residual wart) — `mru_transform/CMakeLists.txt:100-103`
- [x] (suggestion) Three `on_cleanup` comments assert that resetting a subscription/timer means no callback can be in flight; `shared_ptr::reset()` provides no such synchronization. It holds only under the single-threaded executor each `main()` uses — worth naming that precondition now that the ctors take `NodeOptions` and the headers are installed — `tide_copier.hpp:48`, `sea_surface_estimator.hpp:113`, `chart_datum_node.hpp:245`

### Specialists
- Static analysis: **not available** — this repo has no CI and no pre-commit config, and `ament_lint_auto_find_test_dependencies()` finds no linters (only `ament_cmake_gtest` is a `test_depend`; mru_transform#35). The clean build + full test run above is the only gate, and it was run rather than quoted.
- Claude Adversarial: 2 passes (Lens A logic/correctness, Lens B systemic/lifecycle). All three must-fixes are cross-pass confirmed and were re-verified against source by the lead.
- Copilot Adversarial: off (not requested).
- Local Adversarial: **skipped** — the local Ollama server is down (llama-server killed); not retried per instruction.
- Governance: ADR-0008 pass (the `has_parameter()` guard is the nav2 + in-house convention; the `LifecyclePublisher` change restores the activation gate a managed node owes its operator). Commit identity correct on all 14 commits. Atomic commits honoured. Doc impact: `README.md` carries no node inventory, so the new `tide_copier` target owes no README change — the plan's "no stale docs" call is correct. Repo still has no `.agents/README.md` and no root `AGENTS.md` (ADR-0017) — pre-existing gaps, not this PR's work.
- Plan drift: none material. The two small extractions were combined into one commit (plan said eight, actual nine including the plan revision); every plan-listed file changed and no unplanned file did.

## Implementation
**Status**: complete
**When**: 2026-08-24 00:12 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-34 at `fa896a0`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-23 23:50 -04:00, branch at `7437a3f`) — 3 must-fix + 7 suggestions
**Commits**: `cac8498`, `9bcecf9`, `a529e69`, `77e7542`, `45be3e8`, `f5544f5`, `d6dd5d1`, `fa896a0`

### Decisions worth recording

- **Must-fix 2 (`last_navsatfix_`) — clear on deactivate, don't correct the
  comment.** The review offered either. Deactivation is the operator muting the
  node, and an interval it was muted for must not come back out as motion; the
  ACTIVE guard alone leaves the pre-deactivation fix inside the 2 s
  `maximum_interval_` window, so a quick cycle published a velocity averaged
  across the mute. The comment now points at what actually gives the guarantee
  rather than at the age check, which a sub-2 s cycle passes.
- **Must-fix 3 — validate at configure AND check the invariant at the deref.**
  `on_configure` now fails the transition on a non-positive
  `maximum_buffer_duration` (which prunes away the sample just inserted) and on
  a `minimum_buffer_duration` at or above it (a window that can never be long
  enough, so the node would silently never publish), and clamps a negative
  minimum to zero exactly as `tide_range_margin` is clamped. FAILURE rather
  than clamping for the two unusable cases, matching the sibling's
  `publish_rate` / `recalc_interval` treatment: configure is retryable and the
  parameters survive the failure, so the corrected value is what the retry
  reads. The callback still checks `odometry_buffer_.empty()` — the deref was
  undefined behaviour, not a missing publication, so the invariant is checked
  rather than assumed.
- **`chart_datum_node`'s `on_cleanup` timer resets are not pinned by a test and
  cannot be.** Every legal route to `cleanup` passes through `on_deactivate`,
  which already resets both timers, or never activated at all. They stay as
  defensive teardown; the *publishers* released in the same callback are what
  the new test pins.

### Actions
- [x] PROJ context + both pipelines leak on the `FAILURE` return after `setup_proj()` — `chart_datum_node.hpp:176-195` — `cac8498`
- [x] Comment claims a guarantee the ACTIVE guard does not give; deactivate→activate inside 2 s published across the muted gap — `nav_sat_fix_to_velocity.hpp:67-72` — `77e7542`
- [x] `odometry_buffer_.begin()` dereferenced after a prune that can empty the map; neither buffer duration validated — `sea_surface_estimator.hpp:183-184` — `a529e69`
- [x] Deactivate leaves stale state that re-activation acts on (`odometry_buffer_`, `chart_datum_node`'s validity flags) — (deferred: the review classed it a follow-up, not this PR's scope; filed as [mru_transform#37](https://github.com/rolker/mru_transform/issues/37). The one case the review called a must-fix — `nav_sat_fix_to_velocity` — is fixed above.)
- [x] No test pins the new `on_cleanup` releases for two nodes — `test_lifecycle_reconfigure.cpp` — `45be3e8`
- [x] Tests use absolute `/tf`, `/fix`, `/velocity` on the default domain — `test_lifecycle_reconfigure.cpp:373-436` — `9bcecf9`
- [x] Negative assertions use fixed 300-500 ms spin windows — (deferred: filed as [mru_transform#38](https://github.com/rolker/mru_transform/issues/38). The isolation half of the same weakness is fixed in `9bcecf9`, so the negatives can no longer be broken or vacuously satisfied by outside traffic; making them deterministic means restructuring each negative phase into a sentinel round trip, which is its own change.)
- [x] `lifecycle_msgs` included but not depended on — `package.xml`, `CMakeLists.txt` — `f5544f5`
- [x] `install(DIRECTORY include/ ...)` ships four node classes that cannot be consumed — `CMakeLists.txt:100-103` — `d6dd5d1` (chose `PATTERN "nodes" EXCLUDE`: they are a test seam, not API)
- [x] Three `on_cleanup` comments assert a synchronization `shared_ptr::reset()` does not provide — `fa896a0` (verified all four `nodes/*.cpp` call `rclcpp::spin()`, i.e. a single-threaded executor)

### Verification (run, not quoted)

This repo has no CI, no pre-commit config and no registered linters
(mru_transform#35), so this local run is the only gate.

- **Clean rebuild** (`build/` + `install/` for the package removed first) —
  0 errors. The only warning outside vendored `geodesy` is the pre-existing
  unused-parameter in untouched `src/mru_transform.cpp`; none in changed files.
- `./platforms_ws/test.sh mru_transform` — **85 tests, 0 errors, 0 failures,
  0 skipped** (82 before this round; +3 in `test_lifecycle_reconfigure`, now
  15 cases in 3.4 s — comfortably inside the test's 60 s ctest timeout).
- **Every new negative check re-confirmed against the bug it targets**, by
  mutation and restore:
  - removing the new `on_deactivate` from `nav_sat_fix_to_velocity` →
    `NavSatFixToVelocityRespectsLifecycleState` fails on "a velocity was
    computed across the deactivation gap";
  - deleting `odometry_subscription_.reset()` / `tide_estimate_pub_.reset()`
    and `chart_datum_node`'s three publisher resets → both new cleanup cases
    fail, reporting the endpoint counts they saw;
  - the re-namespaced pre-existing negatives still pass, and the suite was
    re-run green after each restore (`git status` clean).
- **Header install verified empirically**: removed the stale installed
  `nodes/` directory, rebuilt, confirmed it is not re-created.
- The PROJ leak has no test: reaching it needs a real VDatum grid directory
  plus a malformed datum config, and the leak is invisible to the process
  under test. Reviewed by reading `setup_proj` / `cleanup_proj` instead.

### Not pushed
The host performs pushes. PR body should still carry the lifecycle-publisher
fix and the `tide_copier` build target as scope beyond the issue's text, and
can now also point at the two follow-ups filed from this round
([#37](https://github.com/rolker/mru_transform/issues/37),
[#38](https://github.com/rolker/mru_transform/issues/38)).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 00:41 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-34 at `9d151d7`
**Mode**: pre-push
**Depth**: Deep (reason: lifecycle + resource-teardown semantics across four managed nodes; this round reviews the 445-line fix pass `7437a3f..HEAD`)
**Must-fix**: 3 | **Suggestions**: 10
**Round**: 2 | **Ship**: continue — two of the three are prose corrections, but the third is a newly-proven correctness bug (a FINALIZED `chart_datum_node` still publishing) that changes behaviour rather than tidying round 1's work, so the guard deserves one independent read.

### Independently verified this round (run, not quoted)

- **Build + tests run here.** Clean rebuild (package `build/`+`install/` removed first): 0 errors; the only warnings are the pre-existing unused-parameter in untouched `src/mru_transform.cpp` and vendored `geodesy`. `./platforms_ws/test.sh mru_transform` — **85 tests, 0 errors, 0 failures, 0 skipped**. Matches the fix pass's numbers.
- **All three round-2 regression checks are load-bearing**, by mutate-and-restore: deleting `on_deactivate` from `nav_sat_fix_to_velocity` fails `NavSatFixToVelocityRespectsLifecycleState`; blanking the `on_cleanup` endpoint resets fails both new cleanup cases; deleting the buffer-duration validation fails `SeaSurfaceEstimatorRejectsUnusableBufferDurations`. Tree verified clean after each restore, suite re-run green.
- **The test isolation is real and load-bearing**, by experiment: with a competing publisher pushing `out/map` transforms onto `/mru_transform_lifecycle_test/tf` on the default domain, `TideCopierRespectsLifecycleState` **FAILS** un-isolated and **PASSES** under `ROS_DOMAIN_ID=89` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. `set_tests_properties(... ENVIRONMENT ...)` clobbers nothing — confirmed in the generated `CTestTestfile.cmake`, where it sits alongside ament's LABELS/TIMEOUT/WORKING_DIRECTORY.
- **The PROJ leak fix is complete, correctly placed, and idempotent.** The datum-config `catch` is the *only* FAILURE return after `setup_proj()` succeeds (the `publish_rate`/`recalc_interval` returns at `:126`/`:131` precede it). `cleanup_proj()` is null-guarded and re-nulls every pointer, so it is safe against the later `on_cleanup` (`:264`) and the destructor (`:64`). Nothing else is allocated at that point — `tf_buffer_`, `tf_listener_` and the three publishers all come after. `datum_entries_.clear()` is load-bearing, not decoration: `datum_entries_ = load_datum_config(...)` leaves the previous contents intact when it throws.
- **The fix pass's reason for leaving it untested is correct.** `setup_proj()` allocates only when `vdatum_grid_dir` *and* `geoid_grid` are set *and* real `*_mllw.gtx` files are found, so no in-process test can reach the leak without a VDatum grid tree. Accepted, not a gap.
- **`on_deactivate`'s clear covers the window it claims — and on firmer ground than the comment says.** The seed sentinel is `rclcpp::Time(last_navsatfix_.header.stamp).nanoseconds() != 0`, so a default-constructed `NavSatFix` is a true "no reference" marker rather than a time-based heuristic. That holds under bag/sim time starting at zero, which a `maximum_interval_`-based argument would not.
- **The validation cannot reject a configuration that used to work in this fleet.** No `.yaml`, `.py` or `.xml` anywhere under `layers/main` sets either buffer-duration parameter (grepped independently), and there is no `on_set_parameters_callback` on the node — so a mid-mission `ros2 param set` cannot reach the validation at all. It bites only at a configure the operator initiates. (One boundary exception: see must-fix 3.)
- **`min >= max` as a hard FAILURE is the right call.** Both rejected pairs previously produced a node that was up and silent — the failure mode that reaches soundings. Configure is retryable and the parameters survive the failure (the new test pins exactly that), and it matches the sibling's `publish_rate`/`recalc_interval` treatment.
- **The `nodes/` install exclusion is safe.** Those four headers are *new files in this branch*, so no external consumer can exist; no `mru_transform/nodes/` include appears anywhere under `layers/main` outside the package; the installed tree still ships the 12 pre-existing headers and no `nodes/`.
- Commit identity correct on all 24 commits; working tree clean.

### Findings
- [ ] (must-fix) `shutdown` from `active` skips both `on_deactivate` and `on_cleanup` — only `on_shutdown` runs, and no node overrides it — so `publish_timer_`/`recalc_timer_` survive and a **FINALIZED** node keeps publishing. Proven with a throwaway case: state `4` (finalized) and **3 `datum_source` messages received after `shutdown()`**. `tf_broadcaster_` is a plain `tf2_ros::TransformBroadcaster` with no activation gate at all, so `map -> chart_datum` keeps going out on `/tf` from a finalized node — the same defect class this PR fixes in `tide_copier`, on the frame every sounding is reduced against. Round 1's "every publishing path is gated" clearance for this node rested on an incomplete transition enumeration. Fix: a `PRIMARY_STATE_ACTIVE` guard at the top of `publish_callback`, matching the package's other three nodes (it also covers the ERROR path in suggestion 8) — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:531-570`
- [ ] (must-fix) The new configure-failure modes are undocumented, against this README's own convention: the `sea_surface_estimator` table still gives only defaults, while `datum_config_path` (`:95`) documents "A malformed file fails `on_configure`" and `tide_range_margin` (`:172`) documents "Negative values are clamped to 0". Retuning the averaging window between lines is the workflow #34 exists to enable, and the operator will hit this. The row also needs the operational framing, because "the boat refuses to configure" is not what happens: `launch_ros`'s `LifecycleTransition` matches neither `inactive` nor `errorprocessing` on a `FAILURE`, so nothing chains, nothing logs "stopping transitions", `respawn` never fires — the process stays up in `unconfigured`, publishes no `map_tide`, and leaves one ERROR line. Say so, and point at `ros2 lifecycle get`. Note `plan.md:275-279` currently asserts the opposite ("README's parameter tables ... are unaffected") — `mru_transform/README.md:168-169`
- [ ] (must-fix) The rationale for rejecting `maximum_buffer_duration == 0` is factually wrong, and the same wrong claim is repeated in the test. The prune is `begin()->first < now - max`; at `max == 0` the cutoff *equals* `now` and the just-inserted key is exactly `now`, so `now < now` is false and the sample is kept — only a strictly *negative* maximum empties the buffer. `(min=0, max=0)` was therefore a working "publish the instantaneous height, no smoothing" configuration and is now a hard configure FAILURE. Rejecting it is defensible policy (a zero-length window is not an averaging window, and this node exists to average) — but the comment must say that, instead of a mechanism that does not occur — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:55-57`, `mru_transform/test/test_lifecycle_reconfigure.cpp:197-200`
- [ ] (suggestion) The isolation comment overclaims for the collision that actually matters in this workspace: two concurrent runs of *this same test* (two agent worktrees running `colcon test` at once) share both the hardcoded `ROS_DOMAIN_ID=89` and the namespace, and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` explicitly permits same-host peers — so `tf_pub->get_subscription_count() >= 2` can be satisfied by the other process and the negatives pass vacuously. Derive the domain per run, or scope the claim to unrelated traffic — `mru_transform/CMakeLists.txt:457-464`, `mru_transform/test/test_lifecycle_reconfigure.cpp:43-56`
- [ ] (suggestion) `TransformListener(*tf_buffer_)` is the one-arg form, which creates its **own** internal node with default options — root namespace, no remap rules — so in `sea_surface_estimator` and `chart_datum_node` the TF listener subscribes to the global `/tf` and `/tf_static` regardless of the fixture's namespacing, and it runs its own spin thread. Nothing asserts on TF today so nothing flakes, but both the isolation comment's "every node it creates" and the new "every `main()` uses a single-threaded executor" comments are looser than stated — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:144-152`, `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:255-260`
- [ ] (suggestion) `ament_add_gtest` leaves `TIMEOUT` at 60 s while the file now budgets roughly eight 15 s discovery waits; I measured a two-case failure taking 30 s on its own. A regression can turn into an opaque ctest timeout instead of a named failure. Pass `TIMEOUT 180` — `mru_transform/CMakeLists.txt:435`
- [ ] (suggestion) `ROS_LOCALHOST_ONLY=0` is the deprecated mechanism `ROS_AUTOMATIC_DISCOVERY_RANGE` replaced; setting it at all emits two rcl deprecation WARNs per run (observed). Drop it — `mru_transform/CMakeLists.txt:463`
- [ ] (suggestion) No `on_error` on `chart_datum_node`: an exception out of `on_configure`/`on_activate` after `setup_proj()` succeeded routes `errorprocessing -> unconfigured` (the default `on_error` returns SUCCESS) without `on_cleanup`, leaking the PROJ context by the very argument the new catch-block comment makes. `on_activate` is worse — a throwing `create_wall_timer` leaks the context, both pipelines, and the TF/publisher members. A three-line `on_error` calling `cleanup_proj()` closes the residual of the class this round fixed — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:186-206`
- [ ] (suggestion) `ROS_DISCOVERY_SERVER`, `ROS_STATIC_PEERS` and `FASTRTPS_DEFAULT_PROFILES_FILE` / `CYCLONEDDS_URI` are inherited and can re-join the test to a wider graph regardless of the discovery range; empty overrides in the same ENVIRONMENT list are cheap. Worth a word that none of this applies when the gtest binary is run directly — a normal debugging step — `mru_transform/CMakeLists.txt:457-461`
- [ ] (suggestion) The re-configure leg waits for subscription re-discovery but not for the re-created `velocity_publisher_` to match the peer's subscription; the single volatile-QoS velocity can be dropped, failing the assertion for a reason unrelated to the code under test. One more `spin_until` on `get_publisher_count()` before the publish — `mru_transform/test/test_lifecycle_reconfigure.cpp:486-497`
- [ ] (suggestion) Plan drift — three statements the plan now contradicts, on exactly the points a reviewer checks: `:186` and `:269` assert the node headers are installed (reversed by `d6dd5d1`), `:272` says `nav_sat_fix_to_velocity`'s `on_deactivate` is deliberately not overridden (added in `77e7542`), and `:275-279` says the README is unaffected (see must-fix 2) — `.agent/work-plans/issue-34/plan.md`
- [ ] (suggestion) A negative `minimum_buffer_duration` clamps the member but leaves `ros2 param get` reporting the negative value, so an operator diagnosing on the water sees a number the node is not using. Consistent with `tide_range_margin`, but the block now gives three different answers to a bad duration inside fifteen lines. Either `set_parameter()` the clamped value back or document the divergence in the README row — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:68-74`
- [ ] (suggestion) `${lifecycle_msgs_TARGETS}` is indented two spaces where every sibling in the same `target_link_libraries` block uses four; this repo has no linter to catch it (mru_transform#35) — `mru_transform/CMakeLists.txt:437`

### Specialists
- Static analysis: **not available** — no CI, no pre-commit, and `ament_lint_auto_find_test_dependencies()` finds no registered linters (mru_transform#35). The clean build and full test run above are the only gate, and both were run here rather than quoted.
- Claude Adversarial: 2 passes (Lens A logic/correctness, Lens B systemic/lifecycle). Must-fix 1 is Lens B's, confirmed by a throwaway probe before promotion; must-fix 2 is cross-confirmed by both lenses and the governance read; must-fix 3 is Lens A's, confirmed against the prune comparison in source.
- Copilot Adversarial: off (not requested).
- Local Adversarial: **skipped** — the Ollama server is in fact up, but the calibrated review model `qwen3.5:35b` is not present locally (only `qwen3.5:4b-q8_0` and `llama3.2:1b`), so the specialist would self-skip as "model not pulled". Not substituted with a smaller model.
- Governance: ADR-0008 pass. Doc-impact **Missing** — see must-fix 2; the README does document `sea_surface_estimator`'s parameters, so round 1's "no README change owed" (which was about `tide_copier`'s build target) does not carry to this round's validation change. Commit identity correct on all 24 commits; atomic commits honoured. Repo still has no `.agents/README.md` and no root `AGENTS.md` (ADR-0017) — pre-existing gaps, not this PR's work.
- Plan drift: three stale statements, listed above as a suggestion.

### Actions
- [ ] Not pushed, no PR opened — the operator gates both.
