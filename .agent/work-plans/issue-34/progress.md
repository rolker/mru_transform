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
- [x] (must-fix) `shutdown` from `active` skips both `on_deactivate` and `on_cleanup` — only `on_shutdown` runs, and no node overrides it — so `publish_timer_`/`recalc_timer_` survive and a **FINALIZED** node keeps publishing. Proven with a throwaway case: state `4` (finalized) and **3 `datum_source` messages received after `shutdown()`**. `tf_broadcaster_` is a plain `tf2_ros::TransformBroadcaster` with no activation gate at all, so `map -> chart_datum` keeps going out on `/tf` from a finalized node — the same defect class this PR fixes in `tide_copier`, on the frame every sounding is reduced against. Round 1's "every publishing path is gated" clearance for this node rested on an incomplete transition enumeration. Fix: a `PRIMARY_STATE_ACTIVE` guard at the top of `publish_callback`, matching the package's other three nodes (it also covers the ERROR path in suggestion 8) — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:531-570`
- [x] (must-fix) The new configure-failure modes are undocumented, against this README's own convention: the `sea_surface_estimator` table still gives only defaults, while `datum_config_path` (`:95`) documents "A malformed file fails `on_configure`" and `tide_range_margin` (`:172`) documents "Negative values are clamped to 0". Retuning the averaging window between lines is the workflow #34 exists to enable, and the operator will hit this. The row also needs the operational framing, because "the boat refuses to configure" is not what happens: `launch_ros`'s `LifecycleTransition` matches neither `inactive` nor `errorprocessing` on a `FAILURE`, so nothing chains, nothing logs "stopping transitions", `respawn` never fires — the process stays up in `unconfigured`, publishes no `map_tide`, and leaves one ERROR line. Say so, and point at `ros2 lifecycle get`. Note `plan.md:275-279` currently asserts the opposite ("README's parameter tables ... are unaffected") — `mru_transform/README.md:168-169`
- [x] (must-fix) The rationale for rejecting `maximum_buffer_duration == 0` is factually wrong, and the same wrong claim is repeated in the test. The prune is `begin()->first < now - max`; at `max == 0` the cutoff *equals* `now` and the just-inserted key is exactly `now`, so `now < now` is false and the sample is kept — only a strictly *negative* maximum empties the buffer. `(min=0, max=0)` was therefore a working "publish the instantaneous height, no smoothing" configuration and is now a hard configure FAILURE. Rejecting it is defensible policy (a zero-length window is not an averaging window, and this node exists to average) — but the comment must say that, instead of a mechanism that does not occur — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:55-57`, `mru_transform/test/test_lifecycle_reconfigure.cpp:197-200`
- [x] (suggestion) The isolation comment overclaims for the collision that actually matters in this workspace: two concurrent runs of *this same test* (two agent worktrees running `colcon test` at once) share both the hardcoded `ROS_DOMAIN_ID=89` and the namespace, and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` explicitly permits same-host peers — so `tf_pub->get_subscription_count() >= 2` can be satisfied by the other process and the negatives pass vacuously. Derive the domain per run, or scope the claim to unrelated traffic — `mru_transform/CMakeLists.txt:457-464`, `mru_transform/test/test_lifecycle_reconfigure.cpp:43-56`
- [x] (suggestion) `TransformListener(*tf_buffer_)` is the one-arg form, which creates its **own** internal node with default options — root namespace, no remap rules — so in `sea_surface_estimator` and `chart_datum_node` the TF listener subscribes to the global `/tf` and `/tf_static` regardless of the fixture's namespacing, and it runs its own spin thread. Nothing asserts on TF today so nothing flakes, but both the isolation comment's "every node it creates" and the new "every `main()` uses a single-threaded executor" comments are looser than stated — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:144-152`, `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:255-260`
- [x] (suggestion) `ament_add_gtest` leaves `TIMEOUT` at 60 s while the file now budgets roughly eight 15 s discovery waits; I measured a two-case failure taking 30 s on its own. A regression can turn into an opaque ctest timeout instead of a named failure. Pass `TIMEOUT 180` — `mru_transform/CMakeLists.txt:435`
- [x] (suggestion) `ROS_LOCALHOST_ONLY=0` is the deprecated mechanism `ROS_AUTOMATIC_DISCOVERY_RANGE` replaced; setting it at all emits two rcl deprecation WARNs per run (observed). Drop it — `mru_transform/CMakeLists.txt:463`
- [x] (suggestion) No `on_error` on `chart_datum_node`: an exception out of `on_configure`/`on_activate` after `setup_proj()` succeeded routes `errorprocessing -> unconfigured` (the default `on_error` returns SUCCESS) without `on_cleanup`, leaking the PROJ context by the very argument the new catch-block comment makes. `on_activate` is worse — a throwing `create_wall_timer` leaks the context, both pipelines, and the TF/publisher members. A three-line `on_error` calling `cleanup_proj()` closes the residual of the class this round fixed — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:186-206`
- [x] (suggestion) `ROS_DISCOVERY_SERVER`, `ROS_STATIC_PEERS` and `FASTRTPS_DEFAULT_PROFILES_FILE` / `CYCLONEDDS_URI` are inherited and can re-join the test to a wider graph regardless of the discovery range; empty overrides in the same ENVIRONMENT list are cheap. Worth a word that none of this applies when the gtest binary is run directly — a normal debugging step — `mru_transform/CMakeLists.txt:457-461`
- [x] (suggestion) The re-configure leg waits for subscription re-discovery but not for the re-created `velocity_publisher_` to match the peer's subscription; the single volatile-QoS velocity can be dropped, failing the assertion for a reason unrelated to the code under test. One more `spin_until` on `get_publisher_count()` before the publish — `mru_transform/test/test_lifecycle_reconfigure.cpp:486-497`
- [x] (suggestion) Plan drift — three statements the plan now contradicts, on exactly the points a reviewer checks: `:186` and `:269` assert the node headers are installed (reversed by `d6dd5d1`), `:272` says `nav_sat_fix_to_velocity`'s `on_deactivate` is deliberately not overridden (added in `77e7542`), and `:275-279` says the README is unaffected (see must-fix 2) — `.agent/work-plans/issue-34/plan.md`
- [x] (suggestion) A negative `minimum_buffer_duration` clamps the member but leaves `ros2 param get` reporting the negative value, so an operator diagnosing on the water sees a number the node is not using. Consistent with `tide_range_margin`, but the block now gives three different answers to a bad duration inside fifteen lines. Either `set_parameter()` the clamped value back or document the divergence in the README row — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:68-74`
- [x] (suggestion) `${lifecycle_msgs_TARGETS}` is indented two spaces where every sibling in the same `target_link_libraries` block uses four; this repo has no linter to catch it (mru_transform#35) — `mru_transform/CMakeLists.txt:437`

### Specialists
- Static analysis: **not available** — no CI, no pre-commit, and `ament_lint_auto_find_test_dependencies()` finds no registered linters (mru_transform#35). The clean build and full test run above are the only gate, and both were run here rather than quoted.
- Claude Adversarial: 2 passes (Lens A logic/correctness, Lens B systemic/lifecycle). Must-fix 1 is Lens B's, confirmed by a throwaway probe before promotion; must-fix 2 is cross-confirmed by both lenses and the governance read; must-fix 3 is Lens A's, confirmed against the prune comparison in source.
- Copilot Adversarial: off (not requested).
- Local Adversarial: **skipped** — the Ollama server is in fact up, but the calibrated review model `qwen3.5:35b` is not present locally (only `qwen3.5:4b-q8_0` and `llama3.2:1b`), so the specialist would self-skip as "model not pulled". Not substituted with a smaller model.
- Governance: ADR-0008 pass. Doc-impact **Missing** — see must-fix 2; the README does document `sea_surface_estimator`'s parameters, so round 1's "no README change owed" (which was about `tide_copier`'s build target) does not carry to this round's validation change. Commit identity correct on all 24 commits; atomic commits honoured. Repo still has no `.agents/README.md` and no root `AGENTS.md` (ADR-0017) — pre-existing gaps, not this PR's work.
- Plan drift: three stale statements, listed above as a suggestion.

### Actions
- [ ] Not pushed, no PR opened — the operator gates both.

## Implementation
**Status**: complete
**When**: 2026-08-24 09:10 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-34 at `306108a`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-24 00:41 -04:00, branch at `9d151d7`) — 3 must-fix + 10 suggestions
**Commits**: `02cbcde`, `09ce07a`, `e7d6b22`, `822d149`, `11ae6f3`, `306108a`

### Decisions worth recording

- **Must-fix 1 — the guard, and the enumeration re-done for all four nodes.**
  `publish_callback` now returns unless the node is `PRIMARY_STATE_ACTIVE`,
  matching the other three. The review's diagnosis is confirmed against source:
  `shutdown` from `active` runs `on_shutdown` only, no node overrides it, so
  both timers survive into `finalized`; `tf_broadcaster_` is a plain
  `tf2_ros::TransformBroadcaster` with no activation gate at all. The single
  check covers the TF branches and the `datum_source` publish together, and
  also covers `errorprocessing`, which is not ACTIVE either.

  **The enumeration was re-run for all four nodes rather than trusting round
  1's clearance.** Every publish site in the package was re-listed from source
  (`grep` for `create_wall_timer` / `->publish(` / `sendTransform`), and each
  traced to its entry point:
  - `chart_datum_node` — the only node with timers. Both are created in
    `on_activate`; `publish_callback` was the sole ungated publish path. Fixed.
  - `tide_copier` (`tf_callback`), `sea_surface_estimator`
    (`odometry_callback`), `nav_sat_fix_to_velocity` (`navsat_callback`) —
    every publish in each is reached only from that one subscription callback,
    and each callback checks `PRIMARY_STATE_ACTIVE` at its top. Their
    subscriptions do survive `shutdown`-from-`active` exactly as
    `chart_datum_node`'s timers did, so messages keep *arriving* at a finalized
    node — but the gate is inside the callback, so nothing goes out. The
    incomplete-enumeration failure mode was specific to the timer node.
  - `sea_surface_estimator` publishes twice (`tide_estimate`, then the TF)
    deep inside `odometry_callback`; both are downstream of the same gate.

- **Must-fix 1's test is a real regression test, not a throwaway.**
  `ChartDatumNodeStopsPublishingWhenFinalized`: configure → activate at 20 Hz,
  wait for `datum_source` traffic, `shutdown()`, assert state `4`, drain 300 ms,
  then assert an empty 700 ms (fourteen publish periods) window. Mutation-checked
  by deleting the guard and rebuilding: it fails with **14 datum_source messages
  published by a FINALIZED node**. `datum_source` is the observable half because
  it is the one branch that does not need a resolved datum — the TF branches
  need an `earth → base_link` lookup this in-process fixture cannot supply, and
  the listener is not namespaceable (see the suggestion-2 note). Both sit behind
  the same single check, and the test comment says so.

- **Must-fix 3 — the mechanism claim was wrong, the policy stands.** Verified
  in source: the prune erases while `begin()->first < now - maximum`, so at
  `maximum == 0` the cutoff equals `now`, the just-inserted key is exactly
  `now`, `now < now` is false, and the sample is **kept**. Only a strictly
  negative maximum empties the buffer. `(0, 0)` was a working "no smoothing"
  setting. The comment, the log message, the test comment and the README now
  separate the two rejections: negative maximum = correctness bug (the UB
  deref); `minimum >= maximum` including `(0, 0)` = **policy**, because an
  unsmoothed single-sample tide on the path feeding every sounding is not a
  configuration anyone should be running.

  Two consequential details. The check was rewritten from `!(max > 0.0)` to
  `!(max >= 0.0)` — deliberately a negated `>=` so a **NaN maximum still
  fails it**, which the old form also did; `(0, 0)` is now caught one line
  later by the `minimum >= maximum` branch instead, so no input that used to be
  rejected has become acceptable. And the `(0, 0)` boundary is pinned by a new
  case inside `SeaSurfaceEstimatorRejectsUnusableBufferDurations`, mutation-
  checked by relaxing `>=` to `>` (it fails, reporting state 2). It is pinned
  *because* it is a policy call: if the policy is ever revisited, that case has
  to be revisited with it.

- **Must-fix 2 — the README says the failure is quiet.** The
  `sea_surface_estimator` parameter rows now document the configure failures the
  way `datum_config_path` and `tide_range_margin` are documented, plus a
  "Buffer durations that fail `on_configure`" section. It states plainly that a
  configure FAILURE is **not an interlock**: `launch_ros`'s
  `LifecycleTransition` matches neither `inactive` nor `errorprocessing`, so
  nothing chains, nothing announces that transitions stopped, `respawn` never
  fires, and the process sits up in `unconfigured` publishing no tide behind one
  ERROR line — check `ros2 lifecycle get`. `chart_datum_node`'s
  `publish_rate` / `recalc_interval` rows (pre-existing validation, verified
  against `jazzy`) gained the same one-line note and are named in that section.

- **All ten suggestions taken; none deferred, none filed onward.** Each was
  cheap and self-contained. Two were taken as the *comment-correcting* half of
  the choice the review offered rather than the mechanism-changing half:
  suggestion 1 scopes the isolation claim instead of deriving a per-run domain
  (a per-run domain needs a wrapper the ctest ENVIRONMENT property cannot
  express), and suggestion 2 names what the one-argument `TransformListener`
  does instead of switching to the node-attached form, which would change how
  the buffer is filled. Suggestion 5's `on_error` was taken in the fuller form:
  the `on_cleanup` teardown is factored into
  `release_everything_on_configure_created()` and run from both, so a throwing
  `on_activate` no longer strands the TF members and the three publishers
  alongside the PROJ context.

### Actions
- [x] (must-fix) A FINALIZED `chart_datum_node` keeps publishing — timers survive `shutdown` from `active` — `chart_datum_node.hpp:531-570` — `02cbcde` (guard + `ChartDatumNodeStopsPublishingWhenFinalized`, mutation-checked; enumeration re-run on all four nodes)
- [x] (must-fix) New configure failures undocumented; a configure FAILURE is a quiet failure, not an interlock — `README.md` — `e7d6b22`
- [x] (must-fix) The `maximum_buffer_duration == 0` rationale is factually wrong; state the policy instead — `sea_surface_estimator.hpp:55-57`, `test_lifecycle_reconfigure.cpp:197-200` — `09ce07a`
- [x] (suggestion) Isolation comment overclaims for two concurrent runs of this same test — `CMakeLists.txt:457-464`, `test_lifecycle_reconfigure.cpp:43-56` — `822d149` (scoped the claim; also says it does not apply when the gtest binary is run directly)
- [x] (suggestion) One-argument `TransformListener` makes its own root-namespace node and thread — `sea_surface_estimator.hpp:144-152`, `chart_datum_node.hpp:255-260` — `822d149` (named at both construction sites and in the two `on_cleanup` "single-threaded executor" comments)
- [x] (suggestion) `TIMEOUT` left at 60 s while the file budgets eight 15 s waits — `CMakeLists.txt:435` — `822d149` (`TIMEOUT 180`; confirmed in the generated `CTestTestfile.cmake`)
- [x] (suggestion) `ROS_LOCALHOST_ONLY=0` is deprecated and emits two WARNs per run — `CMakeLists.txt:463` — `822d149` (dropped; deprecation warnings in the test log now number 0)
- [x] (suggestion) No `on_error` on `chart_datum_node`; the error path leaks the PROJ context — `chart_datum_node.hpp:186-206` — `11ae6f3`
- [x] (suggestion) Inherited `ROS_DISCOVERY_SERVER` / `ROS_STATIC_PEERS` / DDS profile vars can re-join a wider graph — `CMakeLists.txt:457-461` — `822d149` (blanked in the same ENVIRONMENT list)
- [x] (suggestion) Re-configure leg does not wait for the re-created `velocity_publisher_` to match — `test_lifecycle_reconfigure.cpp:486-497` — `822d149`
- [x] (suggestion) Plan drift on three points — `.agent/work-plans/issue-34/plan.md` — `306108a` (marked revised-during-implementation with the commit that reversed each, rather than rewritten)
- [x] (suggestion) A clamped negative `minimum_buffer_duration` still reports the negative value to `ros2 param get` — `sea_surface_estimator.hpp:68-74` — `e7d6b22` (documented the divergence in the README row; kept consistent with `tide_range_margin` rather than adding a third behaviour)
- [x] (suggestion) `${lifecycle_msgs_TARGETS}` indented two spaces — `CMakeLists.txt:437` — `822d149`

### Verification (run, not quoted)

This repo has no CI, no pre-commit config and no registered linters
(mru_transform#35), so this local run is the only gate. It was run fresh after
the last commit, not carried over from the fix rounds.

- **Clean rebuild** — `platforms_ws/build/mru_transform` and
  `platforms_ws/install/mru_transform` removed first, then rebuilt: **0 errors**.
  Warnings are the pre-existing unused-parameter in untouched
  `src/mru_transform.cpp` and vendored `geodesy/geodesics.h`; none in any
  changed file.
- `./platforms_ws/test.sh mru_transform` — **86 tests, 0 errors, 0 failures,
  0 skipped** (85 before this round; +1 = `ChartDatumNodeStopsPublishingWhenFinalized`.
  The `(0, 0)` policy case is a third block inside the existing
  `SeaSurfaceEstimatorRejectsUnusableBufferDurations`, so it adds no case count).
  Suite runs in ~10 s, far inside the new 180 s ctest timeout.
- **Both new checks mutation-verified, and the tree verified clean afterwards**:
  - removing the `PRIMARY_STATE_ACTIVE` guard from `publish_callback` →
    `ChartDatumNodeStopsPublishingWhenFinalized` fails, "a FINALIZED node
    published 14 datum_source message(s)"; restored, suite green.
  - relaxing `minimum >= maximum` to `minimum > maximum` → the `(0, 0)` block
    fails ("(0, 0) was accepted"), alongside the `(30, 30)` block; restored,
    suite green.
- **Generated `CTestTestfile.cmake` inspected**: `TIMEOUT "180"` and the six-var
  `ENVIRONMENT` list are both present, sitting alongside ament's `LABELS`,
  `REQUIRED_FILES` and `WORKING_DIRECTORY` — nothing clobbered.
- **Deprecation noise gone**: `ROS_LOCALHOST_ONLY is deprecated` appears **0**
  times in the test log (two per run before).
- **Header install still excludes `nodes/`** after the clean rebuild: the
  installed `include/mru_transform/mru_transform/` carries the 12 pre-existing
  headers and no `nodes/` directory.
- The PROJ leak and the new `on_error` path have no test, for the reason round 2
  accepted: `setup_proj()` allocates only with a real VDatum grid tree present,
  which no in-process test can supply. Reviewed by reading instead — every reset
  in `release_everything_on_configure_created()` is null-safe and re-nulls, so
  running it from both `on_cleanup` and `on_error` is idempotent, and
  `cleanup_proj()` was already verified null-guarded in round 2.

### Not pushed
The host performs pushes; no PR opened. The PR body should still carry the
lifecycle-publisher fix and the `tide_copier` build target as scope beyond the
issue's text, the two follow-ups filed in round 1
([#37](https://github.com/rolker/mru_transform/issues/37),
[#38](https://github.com/rolker/mru_transform/issues/38)), and — new this round
— the fact that `(min=0, max=0)` was a working configuration that this branch
deliberately refuses, which is an operator-visible policy change rather than a
bug fix.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 10:13 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-34 at `253e1f6`
**Mode**: pre-push
**Depth**: Deep (reason: lifecycle + teardown semantics across four managed nodes; this round reviews the 491-line fix pass `9d151d7..HEAD`)
**Must-fix**: 4 | **Suggestions**: 5
**Round**: 3 | **Ship**: continue — the must-fix count did not fall (3 → 3 → 4), and three of the four are the same unfinished thought: the branch established that a transition can skip its teardown, then closed only part of what follows from that. One (the false publish-gate rationale) was proven by experiment, not argument, and it shows two of the guards this PR adds are pinned by no test at all.

### Independently verified this round (run here, not quoted)

- **Clean rebuild + full suite, run fresh at `253e1f6`.** Package `build/` and `install/` removed first: **0 errors**; the only warnings are the pre-existing unused-parameter in untouched `src/mru_transform.cpp` and vendored `geodesy`. `./platforms_ws/test.sh mru_transform` — **86 tests, 0 errors, 0 failures, 0 skipped** in ~10 s. Matches the fix pass's numbers. `ROS_LOCALHOST_ONLY is deprecated` appears **0** times. This repo has no CI, no pre-commit and no registered linters (mru_transform#35), so this run is the whole gate.
- **The finalized-node gate covers every publish path in `chart_datum_node`.** All five outward calls (`sendTransform` ×2, `mllw_pub_`, `mhhw_pub_`, `datum_source_pub_`) sit below the single `PRIMARY_STATE_ACTIVE` check at the top of `publish_callback`; `recalc_callback` publishes nothing. `tf_broadcaster_` is inside the gated region, so the `map -> chart_datum` branch is covered.
- **`ChartDatumNodeStopsPublishingWhenFinalized` is load-bearing**, by mutate-and-restore: deleting the guard → *"a FINALIZED node published 14 datum_source message(s)"*. Not vacuous — the `sources.size() >= 2` precondition rules out a pass-because-the-timer-never-ran. Restored, tree clean, suite green.
- **The `(0, 0)` case is load-bearing too**: relaxing `>=` to `>` fails with *"(0, 0) was accepted"* alongside the `(30, 30)` block. Restored, verified clean.
- **The enumeration re-run holds.** Re-listed every outward path in all four nodes from source. `tide_copier` → one publish in `tf_callback`; `nav_sat_fix_to_velocity` → one publish in `navsatfix_callback`; `sea_surface_estimator` → `tide_estimate_pub_` + `sendTransform`, both in `odometry_callback`; each gates at the top of that one callback. No services, no parameter callbacks, no other timers. `chart_datum_node` is the only node with wall timers. **The conclusion that only the timer node was affected is correct.**
- **The `(0, 0)` policy rejection changes nothing but the rationale.** Worked the accept/reject set out on both forms and confirmed it against a compiled probe. `!(max > 0.0)` rejected `max < 0`, `max == 0` and NaN; `!(max >= 0.0)` rejects `max < 0` and NaN, and `max == 0` then falls to the `minimum >= maximum` branch — where the negative-minimum clamp (which runs *first*) guarantees `minimum >= 0`, so `max == 0` is still rejected for every minimum. **The accepted set is bit-identical between `9d151d7` and `253e1f6`.** Nothing previously rejected became acceptable; nothing became newly rejected, not even the intended `(0,0)` — it was already refused a line earlier. (The `(0,0)`-was-working claim is relative to `jazzy`, and is correct there.)
- **The shared teardown is safe from both paths and idempotent.** Every reset in `release_everything_on_configure_created()` is null-safe and re-nulls; `cleanup_proj()` null-checks all three PROJ handles. Order is right: timers before `cleanup_proj()` (nothing can enter dead PROJ state), `tf_listener_` before `tf_buffer_` (the listener's dedicated thread writes into the buffer and its destructor joins that thread). Nothing `on_configure` or `on_activate` allocates escapes it — checked member by member. `on_error` is reachable only from `errorprocessing`, entered from a transition callback on the single executor thread, so no concurrent dispatch; it exits to `unconfigured`, and "everything `on_configure` created" is exactly the resource set consistent with that state. A `FAILURE` return (as opposed to a throw) does not enter `errorprocessing`, and all three `FAILURE` returns precede the first allocation, so nothing is stranded on them — the comment scopes itself correctly.
- Commit identity correct on all **32** commits; working tree verified clean after every mutation.

### Findings
- [x] (must-fix) **The rationale that justifies all four lifecycle guards is false, and two of the guards are pinned by no test.** Three files now say `LifecyclePublisher`'s gate is "a non-virtual hide that `publish()` bypasses". It is not: `lifecycle_publisher.hpp:84-92` declares `virtual void publish(const MessageT &)` and it returns early on `!is_activated()`; every publisher member here is declared as `LifecyclePublisher<T>::SharedPtr`, so that gated override is what the call site reaches. **Proven by experiment, both directions:** (a) deleting *only* the state guard from `tide_copier::tf_callback` leaves **86/86 passing** — `TideCopierRespectsLifecycleState` does not detect its removal, because round 1 mutated the publisher *type* and the guard together and never separated them; (b) the branch's own new test shows `datum_source_pub_` — also a `LifecyclePublisher` — emitting 14 messages from a FINALIZED node, because `shutdown` from `active` never runs `on_deactivate`, so `SimpleManagedEntity::activated_` stays `true` into `finalized`. **That** is why the guards are load-bearing, and the comments say something else. On `jazzy` both `tide_copier` and `nav_sat_fix_to_velocity` held plain `rclcpp::Publisher` (`git show origin/jazzy:...`), so the claim was true of the old code and went stale when this branch changed the type. A maintainer who checks it, finds the gate does fire on deactivate, and concludes the guard is redundant would delete a check that is load-bearing for `shutdown`-from-`active` — and no test would stop them. Fix the three comments to the real mechanism, and extend the finalized test to at least `tide_copier` — `mru_transform/include/mru_transform/nodes/tide_copier.hpp:64-67`, `nav_sat_fix_to_velocity.hpp:76-79`, `chart_datum_node.hpp:572-574`
- [x] (must-fix) **The validation this round rewrote lets non-finite values through, and the comment it added claims otherwise.** The comment states "(A non-finite maximum is rejected by the same check…)". Only NaN and `-inf` are: `!(+inf >= 0.0)` is false, so `maximum_buffer_duration = .inf` configures cleanly — and `rclcpp::Duration::from_seconds(inf)` casts `inf` to `int64_t` (UB; `INT64_MIN` on x86-64), after which `now - Duration(INT64_MIN)` **throws `std::overflow_error`** out of `odometry_callback`, uncaught, out of `rclcpp::spin`, killing the tide node on its first odometry message. Verified by compiling and running it against jazzy's `rclcpp`: `Duration::from_seconds(+inf).nanoseconds() = -9223372036854775808`, then `THREW: std::overflow_error: time subtraction leads to int64_t overflow`. The same gap is on the minimum in mirror image: `NaN < 0.0` is false (no clamp) and `NaN >= maximum` is false, so a NaN minimum **passes both checks**, and `buffer_duration.seconds() < NaN` is false forever — the node publishes an unsmoothed single-sample `tide_estimate` and `map_tide` from the first message, which is precisely the configuration the `(0,0)` branch was added this round to refuse. Not a regression (the old form had the same hole), but the comment is new and the branch shows it knows the idiom — `lake_datum` gets an explicit `std::isinf` normalization. `std::isfinite()` on both, and say so in the README row — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:62-64`, `:80`, `:88`, `README.md`
- [x] (must-fix) **`shutdown` still releases nothing; the gate treats the symptom.** No node in the package overrides `on_shutdown`, so `shutdown` from `active` *or* `inactive` never runs `on_cleanup`. The new gate stops fresh publications, but the three `transient_local` publishers on `chart_datum_node` (and `tide_estimate` on `sea_surface_estimator`) are never released, so for the rest of the process's life a **late-joining** subscriber still receives `datum_source: vdatum` and a latched MLLW offset from a finalized node — exactly the property round 2 pinned for the cleanup path in `ChartDatumNodeCleanupReleasesItsPublishers` ("a cleaned-up node still latches a datum for late subscribers"), which `shutdown` walks straight past. The new test cannot see it: its subscriber is created *before* the shutdown, so it exercises the fresh-publish path, not the durable history. `recalc_timer_` survives the same way and `recalc_callback` has no gate, so a finalized node keeps doing `earth -> base_link` lookups, PROJ queries, and `RCLCPP_INFO("Datum at (…) [source: …]")` into the operator's console forever. One `on_shutdown` override per node calling the existing release helper closes all of it. (If the operator prefers to hold the line on scope, this is legitimate follow-up-issue material alongside #37/#38 — the live-publishing half is already fixed — but it should be a recorded decision, not an omission.) — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:226-230`, `:494`, `sea_surface_estimator.hpp:155-157`
- [x] (must-fix) **`on_error` went to one node and not to the one that publishes `map_tide`.** `sea_surface_estimator` has the identical member set and construction order and no `on_error`. A throw at or after `create_publisher`/`create_subscription` routes `errorprocessing -> unconfigured` with `tf_buffer_` and `tf_listener_` still set, and `cleanup` is not legal from `unconfigured`. The supported recovery is another configure — whose first act is `tf_buffer_ = std::make_shared<Buffer>(...)`, dropping the last reference to the old Buffer and destroying it while the **old** `tf_listener_` (not reassigned for another six lines) is still alive and writing into it from its dedicated thread through a raw `tf2::BufferCore &`. That is the exact use-after-free this file's own `on_cleanup` comment describes and orders against; the error path has no ordering at all. Same shape without the UAF on `tide_copier` and `nav_sat_fix_to_velocity` (shared_ptrs a retry overwrites). Fix as `chart_datum_node` did: factor the `on_cleanup` body and run it from both — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:147-153`, `:169-200`
- [x] (suggestion) `~ChartDatumNode` calls `cleanup_proj()` in the destructor **body**, but `publish_timer_`/`recalc_timer_` are members destroyed only *after* the body returns — reproducing the ordering hazard the helper's own comment names ("a timer outliving the PROJ context it calls into would be a use-after-free"). Benign under the single-threaded executor, a UAF under a composed multi-threaded one, and the new test pins the fact that a node can sit in `finalized` with both timers armed — the state a destructor is most likely called from. Call `release_everything_on_configure_created()` instead — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:62-65`
- [x] (suggestion) `ChartDatumNodeStopsPublishingWhenFinalized` goes straight from `activate()` to a 5 s `sources.size() >= 2` wait, so DDS discovery of the peer's `datum_source` subscription has to complete inside that budget. Every other live case in the file waits up to 15 s for endpoint match *first* — including the `get_publisher_count()` wait this very delta added at `:578-585`. Add the matching `spin_until(..., source_sub->get_publisher_count() > 0, 15s)` before it — `mru_transform/test/test_lifecycle_reconfigure.cpp:465-469`
- [x] (suggestion) The `(0, 0)` policy is narrower than the rationale it is documented with. `minimum_buffer_duration = 0` with any positive maximum is accepted, and on the first odometry message `0.0 < 0.0` is false — so the node's first published `tide_estimate` and first `map_tide` are the same unsmoothed single sample the policy refuses, latched on a `transient_local` topic. Worse, the README row added this round tells operators a negative minimum "is clamped to 0", steering the documented recovery from one bad value straight into that bucket. Either reject `minimum <= 0` (and clamp to something non-zero), or narrow the rationale to what is actually enforced — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:88-105`, `README.md:168`
- [x] (suggestion) The `on_cleanup` comment still says "the **installed** headers now take NodeOptions" — `d6dd5d1` excluded `nodes/` from the install, so they are a source-tree test seam, not installed API. The substance survives (a composed user must keep to a single-threaded executor); the word is stale — `mru_transform/include/mru_transform/nodes/sea_surface_estimator.hpp:178`
- [x] (suggestion) Pre-existing, surfaced by the same lens and worth a follow-up rather than this PR: `chart_datum_node`'s `publish_rate`/`recalc_interval` checks are `<= 0.0`, so `publish_rate = inf` passes and yields `create_wall_timer(1.0/inf)` — a **zero-period timer**, the core-pegging failure the check exists to stop — and NaN passes into an out-of-range float→integral cast (UB). The rows added to the README this round ("Must be > 0; `on_configure` fails otherwise") are accurate about what is enforced, so this is not a doc defect — `mru_transform/include/mru_transform/nodes/chart_datum_node.hpp:121-132`

### Specialists
- Static analysis: **not available** — no CI, no pre-commit, `ament_lint_auto_find_test_dependencies()` finds no registered linters (mru_transform#35). The clean build and full test run above are the only gate and were run here.
- Claude Adversarial: 2 passes (Lens A logic/correctness, Lens B systemic/lifecycle). Must-fix 1 and must-fix 2 are **cross-confirmed by both lenses independently**, and both were then proven here by experiment rather than accepted on argument. Must-fix 3 is Lens B's, with the recalc half raised by both. Must-fix 4 split the lenses (B: use-after-free, must-fix; A: leak-only, suggestion) — adjudicated here against `tf2_ros/transform_listener.hpp`, which holds the buffer by raw reference, so B's mechanism stands.
- Copilot Adversarial: off (not requested).
- Local Adversarial: **skipped** — round 2 established the calibrated model `qwen3.5:35b` is not pulled on this host; not substituted with a smaller model.
- Governance: **Documentation Accuracy — Concern** (must-fix 1 and 2: claims about `rclcpp_lifecycle` and about non-finite rejection that source does not support; AGENTS.md "never document from assumptions"). **Quality Standard — Concern** (must-fix 3 and 4: a fix applied to one node and one transition where the same argument covers four and two). ADR-0018 N/A pre-push. Commit identity correct on 32/32; atomic commits honoured. README carries the parameter consequences of this round's changes. Repo still has no `.agents/README.md` and no root `AGENTS.md` (ADR-0017) — pre-existing, not this PR's work.
- Plan drift: **none this round.** The three stale statements round 2 flagged are marked revised-during-implementation with the commit that reversed each, which is the right treatment.

### Convergence — the plain answer asked for
Three rounds, three real findings each, and the count is flat-to-rising rather than falling. That is not review noise: must-fix 1 was demonstrated by deleting a guard and watching all 86 tests still pass, and must-fix 2 by compiling the overflow and watching it throw. Neither was manufactured to justify the round.

The pattern is diagnostic, though. Round 2 found that a transition can skip its teardown; the fix pass closed the publishing half on the one node with timers. Must-fix 3 and 4 are the rest of that same thought — the other transitions (`shutdown`) and the other nodes. Until it is closed at the cause rather than per-symptom, each round will keep finding the next instance.

**Recommendation**: one more scoped pass, not another open-ended round. Fix 1 and 2 (both are claims this branch itself makes, both cheap), then either fix 3 and 4 with `on_shutdown` + `on_error` overrides calling the release helpers — which closes the class — or file them as a follow-up beside #37/#38 as a recorded decision. If that pass lands and the suite stays green, round 4 should be a confirmation read, not a fourth full review.

### Actions
- [ ] Not pushed, no PR opened — the operator gates both.

## Implementation
**Status**: complete
**When**: 2026-08-24 10:49 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-34 at `7cc2c60`
**Addressed**: `## Local Review (Pre-Push)` (2026-08-24 10:13 -04:00, branch at `253e1f6`) — 4 must-fix + 5 suggestions
**Commits**: `a963cf2`, `7681b09`, `4459329`, `3e3ac03`, `4301d18`, `ba18cee`, `8bc0360`, `7cc2c60`

Operator decision for this pass: **close all four must-fixes in this branch**,
including the two the review offered to let me file. Nothing was filed onward.

### The exit-path enumeration — every path, all four nodes

This is the thing the review asked for, and it is the point of the pass: every
one of the four must-fixes was an instance of *something `on_configure` or
`on_activate` created is not released on some exit path*. Enumerated from
source, not from the previous rounds' summaries. **✓ = closed and how; NEW =
added this pass.**

| Exit path | `chart_datum_node` | `sea_surface_estimator` | `tide_copier` | `nav_sat_fix_to_velocity` |
|---|---|---|---|---|
| `deactivate` (active→inactive) | ✓ `on_deactivate` resets both timers — the only thing `on_activate` creates | ✓ nothing to release: `on_activate` allocates nothing (stale `odometry_buffer_` across the gap is the **deferred** #37) | ✓ `on_activate` allocates nothing | ✓ `on_deactivate` clears `last_navsatfix_` |
| `cleanup` (inactive→unconfigured) | ✓ `release_everything_on_configure_created()` | ✓ same helper (**factored out this pass**) | ✓ same helper (**factored out this pass**) | ✓ same helper (**factored out this pass**) |
| `shutdown` from **unconfigured** | **NEW** `on_shutdown` → helper (null-safe, nothing allocated yet) | **NEW** | **NEW** | **NEW** |
| `shutdown` from **inactive** | **NEW** `on_shutdown` → helper | **NEW** | **NEW** | **NEW** |
| `shutdown` from **active** | **NEW** `on_shutdown` → helper (this is the path that skips *both* `on_deactivate` and `on_cleanup`) | **NEW** | **NEW** | **NEW** |
| `error` (throw → `errorprocessing`) | ✓ `on_error` → helper (round 2) | **NEW** `on_error` → helper — the use-after-free case | **NEW** | **NEW** |
| `FAILURE` **return** (no `errorprocessing`, no teardown callback ever runs) | ✓ audited: the two timer-period returns (`:143`, `:149`) precede the first allocation; the datum-config `catch` (`:222`) is after `setup_proj()` and releases explicitly | ✓ audited: all three returns (`:112`, `:120`, `:138`) precede the first allocation at `:179` | ✓ no `FAILURE` return exists | ✓ no `FAILURE` return exists |
| destructor | **NEW** calls the helper, not just `cleanup_proj()` — member subobjects (both timers) are destroyed only *after* the body returns, so the old form freed the PROJ context while the timers that call into it were still armed | ⚠ no destructor added, and reverse-order member destruction gives only *half* the helper's ordering (corrected round 4): `tf_buffer_` `:590` before `tf_listener_` `:591`, so the listener is torn down first — that half holds. But `odometry_subscription_` is declared at `:547`, ahead of the broadcaster `:588`, `tide_estimate_pub_` `:589` and the TF pair, so it is destroyed **last** — everything its callback dereferences is gone before the subscription is. Benign under the single-threaded executor every `main()` here uses (no callback can be in flight during destruction); a hazard only under a composed multi-threaded executor, the same residual that made `~ChartDatumNode` call the helper | ⚠ two `shared_ptr`s, but the declaration order is the **inverse** of the helper's (corrected round 4): `tf_subscription_` `:151` before `tf_publisher_` `:152`, so reverse-order destruction frees the publisher the callback publishes through *first*. Benign for the same reason as `sea_surface_estimator` above | ✓ correct by declaration (the only one of the three that is): `velocity_publisher_` `:172-173` before `navsat_subscription_` `:174`, so the subscription is destroyed first — the helper's order, for free |

Every allocation site was re-listed from source and checked against the helper,
member by member: `chart_datum_node` (PROJ context + both pipelines,
`datum_entries_`, TF buffer/listener/broadcaster, three latched publishers, two
wall timers), `sea_surface_estimator` (broadcaster, TF buffer/listener, latched
`tide_estimate`, odom subscription, plus the lever-arm cache and the odometry
buffer), `tide_copier` and `nav_sat_fix_to_velocity` (one publisher + one
subscription each, plus `last_navsatfix_`). Nothing is outside a helper.

**With that closed, the class is closed at the cause.** The four state guards in
the callbacks are now the *second* gate rather than the fix, and the comments
say so — see must-fix 1.

### Must-fix 1 — the rationale was false; corrected, and the gap it exposed is pinned

Verified in `/opt/ros/jazzy/include/rclcpp_lifecycle/.../lifecycle_publisher.hpp`:
all three `publish()` overloads are declared `virtual` and each returns early on
`!is_activated()`. The review is right and the old comment was wrong. It was
*true of `jazzy`*, where `tide_copier` and `nav_sat_fix_to_velocity` held plain
`rclcpp::Publisher`s, and it went stale when this branch changed the member type.
Corrected in all five places it appeared: `tide_copier.hpp`,
`nav_sat_fix_to_velocity.hpp`, `chart_datum_node.hpp`, and the two test-file
comments that repeated it.

**The mutation experiments, re-run per node rather than generalized.** The
review's finding was demonstrated on `tide_copier`; it does not hold uniformly,
and the difference is what the new test had to target. Deleting *only* the state
guard, one node at a time, rebuilding, and running the suite:

- `nav_sat_fix_to_velocity` — **already pinned.** `NavSatFixToVelocityRespectsLifecycleState`
  fails ("the first fix cannot yield a velocity"), because the guard returns
  *before* `last_navsatfix_` is updated, so an inactive fix must not become the
  reference — something no publisher gate can do. Not a gap; the comment now
  names this as the reason the check is kept.
- `sea_surface_estimator` — **not pinned; this was the real gap.** 86/86 passed
  with the guard gone. It is also the node where the guard is the *only* gate
  that exists: `transform_broadcaster_` is a plain `tf2_ros::TransformBroadcaster`
  with no activation gate whatsoever, and subscriptions are not lifecycle-gated,
  so a configured-but-inactive node receives odometry and broadcasts `map_tide`
  from it. New case `SeaSurfaceEstimatorDoesNotBroadcastWhenInactive`
  (`minimum_buffer_duration:=0` so one message decides it). **Mutation-checked:**
  with only the guard removed it fails alone —
  *"a configured-but-inactive node broadcast 1 sea surface transform(s)"* — and
  no other case fails. Tree verified clean afterwards.
- `tide_copier` and `chart_datum_node` — **honestly, their guards are now
  redundant and no test can isolate them.** Both publish only through
  `LifecyclePublisher`s (`chart_datum_node`'s `tf_broadcaster_` is the exception,
  and it sits inside the same gated region), so the publisher's own gate covers
  `inactive`; and now that `on_shutdown` releases the subscription/timers, the
  `shutdown`-from-`active` path that made them load-bearing no longer reaches
  the callback at all. They are kept as one-line defence in depth — the member
  type could change back, and a composed multi-threaded executor could dispatch
  a timer created in `on_activate` before the transition completes — and the
  comments state exactly that rather than claiming a mechanism. **This is stated
  plainly rather than papered over with a test that would only re-pin
  `on_shutdown`.**

### Must-fix 2 — non-finite bounds

Both directions confirmed against the review's account. `+inf` passed
`!(max >= 0.0)`; the comment added last round claiming that check covered
non-finite maxima was wrong. `std::isfinite()` now gates **both** parameters,
before every other check, with the negative-maximum test simplified back to
`max < 0.0` (finiteness having already run). Two new cases inside
`SeaSurfaceEstimatorRejectsUnusableBufferDurations`, one per bound:
`maximum_buffer_duration:=.inf` and `minimum_buffer_duration:=.nan`.
**Mutation-checked** by disabling the finiteness branch: both report
(*"an infinite maximum_buffer_duration was accepted"*, *"a NaN
minimum_buffer_duration was accepted"*), that case fails alone, restored clean.
README documents it as a third rejection class with the operational
consequence — a boat that configures, activates, and then **dies on its first
odometry message**.

### Must-fix 3 — `shutdown`

`on_shutdown` added to all four nodes, each calling that node's existing
(or newly factored) release helper. Four new cases via one shared
`expect_shutdown_releases_endpoints()` helper, all going `configure → activate →
shutdown` so they take the worst path. **Mutation-checked** by stripping all
four overrides at once: exactly the four target cases fail
(`ChartDatumNodeShutdownReleasesItsPublishers`,
`SeaSurfaceEstimatorShutdownReleasesItsEndpoints`,
`TideCopierShutdownReleasesItsEndpoints`,
`NavSatFixToVelocityShutdownReleasesItsEndpoints`) and nothing else does — each
one failing on its own node's missing override. Restored, tree clean, suite
green. Note the consequence the review predicted and this confirms:
`ChartDatumNodeStopsPublishingWhenFinalized` now passes under *either* fix
alone, so it pins the pair; the four new endpoint cases are what pin
`on_shutdown` specifically.

### Must-fix 4 — `on_error`

Added to `sea_surface_estimator`, `tide_copier` and `nav_sat_fix_to_velocity`,
each calling the same helper `on_cleanup` uses (`chart_datum_node` had one from
round 2). The `sea_surface_estimator` comment records the mechanism the review
adjudicated: a retry-configure reassigns `tf_buffer_` first, destroying the
Buffer while the **old** `tf_listener_` — not reassigned for another few lines —
is still writing into it from its own thread through a raw `tf2::BufferCore &`.
That is a use-after-free, not a leak.

**No test, for the reason round 2 accepted for the same path on
`chart_datum_node` and re-verified here**: nothing in these `on_configure`s can
be made to *throw* from an in-process fixture. Every parameter is declared
before the first allocation, so a type-mismatch override throws too early to
strand anything; the topic names are fixed, so no remap can make
`create_publisher`/`create_subscription` fail. Reviewed by reading instead: each
helper is null-safe, re-nulls what it releases, and is therefore idempotent
across `on_cleanup` → `on_error` → destructor in any order.

### Suggestions — all five taken, none deferred, none filed onward

- **Destructor ordering** (`~ChartDatumNode`) — now calls the release helper.
- **Discovery wait** — `ChartDatumNodeStopsPublishingWhenFinalized` waits up to
  15 s for `source_sub->get_publisher_count() > 0` before spending its 5 s
  traffic budget, matching every other live case in the file.
- **`(0, 0)` rationale narrower than enforced** — took the *narrow-the-rationale*
  half, not the reject-`minimum <= 0` half, which would expand the policy and
  the branch. Comment, log message, test comment and README now say what is
  actually enforced: `minimum = 0` with a positive maximum **is** accepted and
  its first estimate is unsmoothed too, but the window then fills, so that is a
  start-up transient; `(0, 0)` is unsmoothed *permanently*. The README's
  clamped-to-0 row now says so rather than steering an operator into it.
- **Stale "installed headers"** — corrected: `nodes/` is excluded from the
  install, so they are a source-tree test seam that happens to take `NodeOptions`.
- **`publish_rate = inf`** — the review flagged this as follow-up material. Taken
  here instead: it is the same defect class as must-fix 2, two lines, and the
  failure it prevents is a **zero-period timer pegging a core** on the datum
  node. `std::isfinite()` on `publish_rate` and `recalc_interval`, new case
  `ChartDatumNodeRejectsNonFiniteTimerPeriods`, **mutation-checked** by
  reverting to the bare `<= 0.0` form (fails alone, restored clean), README rows
  updated.

### Verification (run fresh, not carried over)

No CI, no pre-commit, and `ament_lint_auto_find_test_dependencies()` finds no
registered linters (mru_transform#35), so this local run is the entire gate. Run
after the last commit.

- **Clean rebuild** — `platforms_ws/build/mru_transform` and
  `platforms_ws/install/mru_transform` removed first: **0 errors**, and **0
  warnings in any changed file** (the only warnings are the pre-existing
  unused-parameter in untouched `src/mru_transform.cpp` and vendored
  `geodesy/geodesics.h`).
- `./platforms_ws/test.sh mru_transform` — **92 tests, 0 errors, 0 failures, 0
  skipped** in ~11 s (86 before this pass; +6 = one guard case, four shutdown
  cases, one non-finite-timer case. The two non-finite buffer-duration cases are
  blocks inside the existing `SeaSurfaceEstimatorRejectsUnusableBufferDurations`,
  so they add no case count).
- **Every new regression test mutation-checked against the specific bug it
  targets**, each failing alone, with the tree verified clean after each
  restore — detailed per finding above.
- `ROS_LOCALHOST_ONLY is deprecated` still appears **0** times in the test log.
- **Header install still excludes `nodes/`** after the clean rebuild: the
  installed `include/mru_transform/mru_transform/` carries the 12 pre-existing
  headers and no `nodes/` directory.
- Working tree clean; commit identity correct on all 40 commits.

### Actions
- [x] (must-fix) The rationale justifying all four lifecycle guards is false, and two guards are pinned by no test — `tide_copier.hpp`, `nav_sat_fix_to_velocity.hpp`, `chart_datum_node.hpp`, `test_lifecycle_reconfigure.cpp` — `a963cf2` (comments corrected in all five places; `SeaSurfaceEstimatorDoesNotBroadcastWhenInactive` added and mutation-checked; per-node re-verification recorded above)
- [x] (must-fix) Non-finite buffer durations pass validation and kill the node; the comment claims otherwise — `sea_surface_estimator.hpp`, `README.md`, `test_lifecycle_reconfigure.cpp` — `7681b09` (`std::isfinite()` on both bounds, one pinned case per bound, mutation-checked)
- [x] (must-fix) `shutdown` releases nothing — `all four node headers`, `test_lifecycle_reconfigure.cpp` — `4459329` (`on_shutdown` on all four calling the release helper; four endpoint cases, mutation-checked together, exactly four failures)
- [x] (must-fix) `on_error` missing from the node that publishes `map_tide` — `sea_surface_estimator.hpp`, `tide_copier.hpp`, `nav_sat_fix_to_velocity.hpp` — `3e3ac03` (added to all three; no test, for the reason round 2 accepted and this pass re-verified)
- [x] (suggestion) `~ChartDatumNode` frees PROJ before the timers that call into it are destroyed — `chart_datum_node.hpp:62-65` — `4301d18`
- [x] (suggestion) `ChartDatumNodeStopsPublishingWhenFinalized` skips the endpoint-match wait every other live case does — `test_lifecycle_reconfigure.cpp` — `ba18cee`
- [x] (suggestion) The `(0, 0)` policy is narrower than the rationale it carries — `sea_surface_estimator.hpp`, `README.md:168`, `test_lifecycle_reconfigure.cpp` — `8bc0360` (rationale narrowed to what is enforced; `minimum <= 0` deliberately NOT rejected — that would expand the policy)
- [x] (suggestion) Stale "installed headers" wording — `sea_surface_estimator.hpp` — `ba18cee`
- [x] (suggestion) `publish_rate = inf` yields a zero-period timer; NaN is UB — `chart_datum_node.hpp:121-132` — `ba18cee` (taken here rather than filed: same class, two lines, core-pegging failure)

### Not pushed
The host performs pushes; no PR opened. The PR body should carry, beyond the
issue's text: the lifecycle-publisher fix, the `tide_copier` build target, the
two round-1 follow-ups ([#37](https://github.com/rolker/mru_transform/issues/37),
[#38](https://github.com/rolker/mru_transform/issues/38)), the operator-visible
policy change that `(min=0, max=0)` is now refused, and — new this pass — that
`maximum_buffer_duration`/`minimum_buffer_duration`/`publish_rate`/
`recalc_interval` now reject non-finite values that previously configured
cleanly.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-24 11:00 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-34 at `97948ba`
**Mode**: pre-push
**Depth**: Deep (scoped as round 3 recommended: a **confirmation read** of the fix pass, not a fourth full review)
**Must-fix**: 2 | **Suggestions**: 1
**Round**: 4 | **Ship**: recommended — both must-fixes are mechanical (a wrong test name in a comment; one claim to correct, with a working test already written and proven below). Must-fix count fell 3 → 3 → 4 → 2 and nothing found is a design or correctness concern. Fix these two and ship; do not run a fifth round.

### The gate, run here

No CI, no pre-commit, no registered linters (mru_transform#35), so this run is the whole gate.

- **Clean rebuild** (`platforms_ws/build/mru_transform` + `install/mru_transform` removed first): **0 errors**. Three warnings, all outside changed files: `src/mru_transform.cpp:341` unused-parameter (untouched) and vendored `geodesy/geodesics.h:234` ×2. **0 warnings in any changed file** — the fix pass's claim confirmed.
- `./platforms_ws/test.sh mru_transform` — **92 tests, 0 errors, 0 failures, 0 skipped** in ~10.6 s. Matches the reported 92 (86 before).
- `ROS_LOCALHOST_ONLY is deprecated`: **0** occurrences.
- Installed `include/mru_transform/mru_transform/` carries the 12 pre-existing headers and **no `nodes/`** — the install exclusion survives a clean rebuild.
- 42 commits, all authored `Claude Code Agent <roland+claude-code@ccom.unh.edu>`; working tree verified clean after every mutation below.

### The exit-path enumeration — audited cell by cell against source

**The table holds, with one row overclaiming.** Verified independently, not read off the entry:

- **`deactivate`** ✓ — `chart_datum_node::on_deactivate` resets both timers (the only things `on_activate` creates); `sea_surface_estimator`/`tide_copier` `on_activate` bodies are a bare `return LifecycleNode::on_activate(state)`, so there is nothing to release; `nav_sat_fix_to_velocity::on_deactivate` clears `last_navsatfix_`.
- **`cleanup` / `shutdown` (all three source states) / `error`** ✓ — all four nodes now route to one `release_everything_on_configure_created()`. Every reset in each helper is null-safe and re-nulls, so one `on_shutdown` override is genuinely correct from `unconfigured`, `inactive` and `active`. Checked each helper member-by-member against its node's `on_configure`/`on_activate` allocation list: nothing allocated is outside a helper.
- **`FAILURE`-return row — line numbers checked, not accepted.** `chart_datum_node`: `:143` and `:149` (the `publish_rate` / `recalc_interval` checks) both precede the first allocation, which is `setup_proj()` at `:194`; the datum-config `catch` at `:222` is the only return after it and calls `cleanup_proj()` + `vdatum_enabled_ = false` + `datum_entries_.clear()` before returning. `sea_surface_estimator`: `:112`, `:120`, `:138` all precede the first allocation at `:179`. `tide_copier` and `nav_sat_fix_to_velocity` have no `FAILURE` return at all (grep-confirmed). **Row is true.**
- **`destructor` row — two of four cells overclaim.** See the suggestion below.

### The two claims against round 3 — both verified by mutation, both stand

Round 3's `tide_copier` result does **not** generalize, exactly as the pass says:

- **`nav_sat_fix_to_velocity`'s guard is already pinned.** Deleted *only* the `PRIMARY_STATE_ACTIVE` check (`:132-134`), rebuilt, ran the suite: **`NavSatFixToVelocityRespectsLifecycleState` fails, alone** (92 tests, 1 failing case). It is pinned because the guard returns *before* `last_navsatfix_ = *msg` — something no publisher gate can do. Restored, tree clean.
- **`sea_surface_estimator` was the real gap, and is now pinned.** Deleted only its guard (`:251-252`): **`SeaSurfaceEstimatorDoesNotBroadcastWhenInactive` fails, alone**. Confirmed at source that `transform_broadcaster_` is a plain `std::shared_ptr<tf2_ros::TransformBroadcaster>` (`sea_surface_estimator.hpp:588`) with no activation gate whatsoever — so the state check is the *only* gate this node has. Restored, tree clean.

### The honesty claim about `tide_copier` / `chart_datum_node` — true, and worth having on the record

**Confirmed: no in-process test can isolate either guard any more.** `tide_copier::tf_callback` has exactly one side effect, `tf_publisher_->publish()` through a `LifecyclePublisher` whose `publish()` is virtual and gates on `is_activated()` — so with the guard removed the observable behaviour in `inactive`/`unconfigured` is identical. `chart_datum_node::publish_callback` is reachable only from `publish_timer_`, which is created in `on_activate` and now released by `on_deactivate`, `on_cleanup`, `on_shutdown` **and** `on_error`, so no non-active state can dispatch it at all; its `tf_broadcaster_` sits inside that same gated region. Declining to manufacture a test that would only re-pin `on_shutdown` was the right call, and stating it plainly was better than a green box.

### Findings
- [x] (must-fix) The comment justifying the guard names a test that **does not exist**: `NavSatFixToVelocityIgnoresFixesWhileInactive` appears nowhere in the repo (grepped whole tree). The test that actually pins it — verified by mutation above — is `NavSatFixToVelocityRespectsLifecycleState`. `tide_copier.hpp:83-84` gets the same construct right, naming `SeaSurfaceEstimatorDoesNotBroadcastWhenInactive`, which does exist. A maintainer following this comment to check the guard is load-bearing finds nothing and is left where round 3's must-fix 1 left them. One word — `mru_transform/include/mru_transform/nodes/nav_sat_fix_to_velocity.hpp:128`
- [x] (must-fix) **`on_error` IS testable in-process on `chart_datum_node`; the claim is over-generalized.** The pass's reasoning is scoped to `on_configure` bodies and is correct there — in all four nodes every `declare_parameter` precedes every allocation, so a type-mismatch override throws too early to strand anything, and the topic names are fixed. But the branch's own `on_error` comment names `on_activate` as the worse case, and that path *is* reachable: `publish_rate = 1e-10` is finite and > 0, so it passes `on_configure`'s new validation, and `1.0/1e-10 = 1e10 s` exceeds `std::chrono::nanoseconds::max()` (~9.22e9 s), so `rclcpp`'s `safe_cast_to_period_in_ns` throws `std::invalid_argument` out of `create_wall_timer`. **Proven, not argued**: a throwaway case (configure → activate with that override) produced `Caught exception in callback for transition 13`, then `on_error`'s own log line, state `1` (unconfigured), and both latched publishers released — the case **passed**, 93/93. This is the one node whose `on_error` release includes the PROJ context and both pipelines, i.e. the case the round-3 finding was actually about. Either add the ~25-line case (working code exists; it was run here) or narrow the recorded claim to "`on_configure` cannot be made to throw, and `on_activate` allocates nothing in the other three nodes" — but the current blanket "no test" is not accurate. Round 2's acceptance was likewise about `on_configure`, so it does not carry — `mru_transform/test/test_lifecycle_reconfigure.cpp`, `chart_datum_node.hpp:298-315`
- [x] (suggestion) **The `destructor` row's two `✓ nothing needed` cells are justified by an ordering the member declarations give only half of.** The helpers enforce two orderings — subscription before the members its callback dereferences, and listener before buffer — but reverse-order member destruction only reproduces the second. `sea_surface_estimator`: `odometry_subscription_` is declared at `:547`, before `transform_broadcaster_` `:588`, `tide_estimate_pub_` `:589`, `tf_buffer_` `:590`, `tf_listener_` `:591` — so the subscription is destroyed **last**, after every member its callback touches (the cited listener-before-buffer half is correct). `tide_copier`: `tf_subscription_` `:151` before `tf_publisher_` `:152` — the publisher is destroyed first, the exact inversion of the helper's stated order. Only `nav_sat_fix_to_velocity` is right by declaration (`velocity_publisher_` `:172`, `navsat_subscription_` `:174`). This is the same residual that made `~ChartDatumNode` call the helper — benign under the single-threaded executor every `main()` uses, a UAF under a composed multi-threaded one — so it is a consistency/truth gap, not a live bug. Either give those two nodes the same one-line destructor, or correct the two cells to say what the member order actually gives — `sea_surface_estimator.hpp:547,588-591`, `tide_copier.hpp:151-152`

### Specialists
- Static analysis: **not available** — no CI, no pre-commit, `ament_lint_auto_find_test_dependencies()` finds no registered linters (mru_transform#35). The clean build + full suite above is the gate and was run here.
- Claude Adversarial: **not dispatched** — this round was scoped as a confirmation read of a specific set of claims, per round 3's own recommendation. Everything reported was verified by the lead directly against source or by experiment.
- Copilot Adversarial: **not run** (instructed).
- Local Adversarial: **skipped** — the calibrated model `qwen3.5:35b` is not pulled on this host (established round 2); not substituted.
- Governance: **Documentation Accuracy — Concern**, both must-fixes (a comment naming a test that does not exist; a testability claim broader than what holds). Both are the same class this branch has been closing, and both are cheap. Quality Standard: satisfied — the exit-path class is closed at the cause and the enumeration is honest. Commit identity correct on 42/42. Repo still has no `.agents/README.md` and no root `AGENTS.md` (ADR-0017) — pre-existing, not this PR's work.
- Plan drift: none this round; the reversed plan statements remain marked with the commit that reversed each.

### Convergence — the plain answer
The branch is done. The enumeration is the deliverable it was asked to be: I checked every cell against source and every `FAILURE`-return line number rather than accepting them, and the only row that overclaims is the destructor row, in a direction that is benign under this package's executors. Both round-3 corrections are real and I reproduced both by mutation. The one thing the pass got wrong is a claim about testability, not about the code — and the missing test is 25 lines that already exist and pass.

Nothing found here questions the design, the fix, or the tests. Fix the two must-fixes and ship it.

### Actions
- [ ] Not pushed, no PR opened — the operator gates both.

## Implementation
**Status**: complete
**When**: 2026-08-24 11:10 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-34 at `8853344`
**Addressed**: `## Local Review (Pre-Push)` (round 4, 2026-08-24 11:00 -04:00, branch at `97948ba`) — 2 must-fix, 1 suggestion, verdict `Ship: recommended`
**Commits**: `eef23cb`, `f0f6f80`, `8853344`

All three findings actioned; none deferred. This was the last pass — no round 5.

### Actions
- [x] (must-fix) Comment named a test that does not exist — `include/mru_transform/nodes/nav_sat_fix_to_velocity.hpp:128` — `eef23cb`
- [x] (must-fix) The `on_error`-is-untestable claim was over-generalized; the case now exists — `test/test_lifecycle_reconfigure.cpp:553-611` — `f0f6f80`
- [x] (suggestion) The `destructor` row overclaimed in two cells — `.agent/work-plans/issue-34/progress.md` (round-3 enumeration table) — `8853344`

### What each fix actually was

**1 — the wrong test name.** `NavSatFixToVelocityIgnoresFixesWhileInactive` →
`NavSatFixToVelocityRespectsLifecycleState`, the name the mutation check in round
4 proved is what pins the guard. **Checked that no other comment cites a test that
does not exist**: extracted all 37 `TEST_F` fixture+case names from `test/`, then
grepped every comment in `include/` and `src/` for identifiers of that shape. Four
citations exist in total, and the other three all resolve —
`SeaSurfaceEstimatorDoesNotBroadcastWhenInactive` (`tide_copier.hpp:84`),
`ChartDatumNodeRecoversFromFailedConfigure` (`chart_datum_node.hpp:216`) and
`ChartDatumNodeStopsPublishingWhenFinalized` (`test_lifecycle_reconfigure.cpp:182`).
The one bad citation was the only one.

**2 — `on_error` is now pinned on the node whose error path releases the PROJ
context.** Took the case rather than narrowing the claim, and the reviewer's lever
worked exactly as described. `ChartDatumNodeErrorPathReleasesItsPublishers`:
`publish_rate = 1e-10` passes `on_configure`'s finiteness/positivity check, so the
node reaches `inactive` with all three latched publishers up (asserted, 3 endpoints
observed from a peer); `activate` then asks `create_wall_timer` for a `1e10 s`
period, `safe_cast_to_period_in_ns` throws, and `rclcpp_lifecycle` catches it —
so, as everywhere else in this file, the test asserts on the resulting **state**
(`unconfigured`) and not on a throw, then on the publisher count falling to 0.

Mutation-checked against the specific bug it targets: deleting *only* the
`release_everything_on_configure_created()` call from `chart_datum_node::on_error`
(leaving the override and its log line in place) makes
`ChartDatumNodeErrorPathReleasesItsPublishers` **the sole failing case** — 1 gtest
failure across the whole suite, with the intended message ("the error path left 3
latched publisher(s) up"). Restored; tree verified clean.

No live claim of untestability needed narrowing: the "no test" statements are in
the round-2/round-3 history entries above, which are the record of what was
believed then and are not rewritten. This entry is the correction. The `on_error`
comments in the four headers never asserted untestability — they describe the
mechanism, and `chart_datum_node.hpp:298-315` names `on_activate` as the worse
case, which is precisely the path now covered.

**3 — the destructor row, corrected rather than coded around.** Re-derived the
declaration order from source before touching the table, and the reviewer is right
in all three cells:

- `sea_surface_estimator` — `odometry_subscription_` `:547` precedes
  `transform_broadcaster_` `:588`, `tide_estimate_pub_` `:589`, `tf_buffer_` `:590`,
  `tf_listener_` `:591`, so reverse-order destruction destroys the subscription
  **last**, after everything its callback dereferences. The listener-before-buffer
  half the old cell cited is correct; the subscription-first half is not.
- `tide_copier` — `tf_subscription_` `:151` before `tf_publisher_` `:152`: the
  publisher goes first, the exact inversion of the helper's order.
- `nav_sat_fix_to_velocity` — `velocity_publisher_` `:172-173` before
  `navsat_subscription_` `:174`: correct by declaration, the only one of the three.

Corrected the table rather than reordering members, per the suggestion's own
preferred branch. The reorder is not worthwhile here: this is the same residual
already recorded for `~ChartDatumNode` — benign under the single-threaded executor
every `main()` in this package uses, since no callback can be in flight while the
node is being destroyed — and shuffling member declarations across three headers to
buy nothing under the executors that actually run is churn on a branch that is
otherwise ready to ship. The cells now say what the member order gives and mark the
gap `⚠` with its blast radius, so a future composed multi-threaded deployment finds
the truth rather than a `✓`.

### Noted, not chased — a validation gap worth its own issue

`publish_rate = 1e-10` passing `on_configure` and then throwing out of
`on_activate` is a real gap in the round-3 validation, not only a test lever. The
check rejects non-finite and non-positive rates, but not rates so small that
`1.0 / publish_rate_` overflows `std::chrono::nanoseconds` (~9.22e9 s, i.e. any
`publish_rate_` below ~1.08e-10). The operator-visible failure is worse than a
`FAILURE` return: the node configures cleanly, then `activate` dumps a caught-
exception trace and drops it back to `unconfigured`. `recalc_interval_` has the
symmetric problem at the top end (any value above ~9.22e9 s). A one-line upper/lower
bound in the same `on_configure` block would convert both into the same clean ERROR
+ `FAILURE` the other bad values already get.

**Recommend filing as a follow-up** beside #35/#37/#38 — deliberately not opened
here (issue-first policy: the operator decides), and deliberately not fixed on this
branch, which the round-4 review closed. Note that fixing it must not silently
delete the new test: the case would need a different lever (a `recalc_interval_`
above the ceiling, or a parameter set after configure).

### Verification — this run is the whole gate

No CI, no pre-commit, no registered linters on this repo (mru_transform#35), so the
local run is the only gate, per the round-4 entry.

- **Clean rebuild** (`platforms_ws/build/mru_transform` + `install/mru_transform`
  removed first): 0 errors, and **0 warnings in any changed file**. The only
  warnings are the three pre-existing ones outside the diff —
  `src/mru_transform.cpp:341` unused-parameter and vendored
  `geodesy/geodesics.h:234` ×2.
- **Full suite**: `93 tests, 0 errors, 0 failures, 0 skipped` (92 before this pass;
  the new `on_error` case is the 93rd).
- Mutation check as described under fix 2 — the new case fails alone, then passes
  again once restored.
- Working tree verified clean after every mutation; all commits authored
  `Claude Code Agent <roland+claude-code@ccom.unh.edu>`.
- Not pushed and no PR opened — the operator gates both.
