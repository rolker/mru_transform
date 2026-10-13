---
issue: 49
---

# Issue #49 — The map origin cannot be pre-defined: map is anchored at the first position fix, so it differs per restart and per platform


## Issue Review
**Status**: complete
**When**: 2026-10-12 15:18 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #49
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: needs-more-detail

### Actions
- [ ] Hold implementation: owner's standing rule is design thinking first for shared packages; issue itself lists four open design questions (origin source/altitude, reset_map_frame semantics, far-from-origin bound, relation to #48).
- [ ] Decide with the owner whether this lands inside the #48 map-frame design session or first on its own; the origin parameter is the natural input to any #48 direction, but the far-origin bound check overlaps #48's stopgap direction 5.
- [ ] Settle reset_map_frame semantics (return to configured origin / refuse / re-anchor) and document it; today it just drops mapFrame_ and re-anchors silently (mru_transform.cpp resetMapFrameService).
- [ ] Require that a configured origin has a defined validity bound (d^2/2R height error) and a warning or refusal beyond it.
- [ ] If implemented: add tests (configured origin used instead of first fix, reset behaviour, unset = current behaviour), and update README parameter docs and .agents/README.md verified-parameter table in the same PR (new parameter(s)).
- [ ] Note for downstream: map_tide / chart_datum_node / sea_surface_estimator consume the map origin altitude (0 today); a configured ellipsoid height changes their inputs, so check them if altitude is configurable.


## Plan Authored
**Status**: complete
**When**: 2026-10-12 15:40 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-49/plan.md` at `b8887be`
**Branch**: feature/issue-49 at `b8887be`
**Phases**: single

### Open questions
- [ ] Default for map_origin_max_height_error: proposed 0.5 m (~2.5 km); owner to confirm.
- [ ] Log-only announcement of the active origin (no topic) is enough? Proposed yes.
- [ ] Repo has no .agents/README.md (no verified-parameter table): create as a separate task; README is the sole parameter doc here.


## Plan Review
**Status**: complete
**When**: 2026-10-12 16:05 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-49/plan.md` at `b8887be`
**PR**: PR-less
**Verdict**: changes-requested

Owner decisions 1-4 (2026-10-12): the plan matches all four; no departures found. Checked against source: `updatePosition`/`resetMapFrameService` in `mru_transform.cpp`, `map_frame.cpp`, `mru_transform_node.cpp`, existing parameter names, `CMakeLists.txt` test registration, and every launch/config in `layers/main` that starts `mru_transform_node`.

### Findings
- [ ] (must-fix) Startup failure mode is overstated: `mru_transform_node.cpp` has no try/catch, so a constructor throw is an uncaught exception, i.e. `std::terminate`/SIGABRT (possible core dump, no `rclcpp::shutdown`), not a clean "non-zero exit". Plan should change `main` to catch, `RCLCPP_FATAL`, `rclcpp::shutdown()`, `return 1`, and validate before creating publishers/services. `core_launch.py` (bizzyboat) starts the node without `respawn`, so launch will report the exit; say so in README. — `plan.md:51-53`
- [ ] (must-fix) Follow-up list is incomplete and omits the field platform. Nodes started via `mru_transform_node` in `layers/main`: `bizzyboat_project11/launch/core_launch.py` and `bizzyboat_sim_core_launch.py` (+ `config/bizzyboat.yaml` `/**/mru_transform:` block), `ben_project11/launch/ben_core_launch.py` and `sim_local_cw4.launch`, `echoboat_project11/launch/echo_launch.py`, `echo_helm/launch/echo_helm_launch.py`, `molab_hardware/launch/mobile_lab_launch.py`, `lr30_project11/launch/backseat_launch.py`, `simulation_ws/.../vrx_project11/launch/wamv_launch.py`, and `marine_simulation/launch/demo_boats_launch.py:451` (the ODD demo; there the origin must be one shared value across boats whose map_frame is namespaced via `boat.frame('map')`). Note launch-inline dicts need a Python float (`43.0`). Also record that no caller of `reset_map_frame` exists in `layers/main` outside this repo (grep done), so the "others not checked" caveat can be closed. — `plan.md:141-150`
- [ ] (must-fix) The throttled WARN wiring in `updatePosition` has no test; only the pure predicate is tested, so removing the `RCLCPP_WARN_THROTTLE` call or reading the wrong threshold parameter passes. Add a node-level case capturing the log (`rcutils_logging_set_output_handler`) with a fix beyond the threshold and a non-default `map_origin_max_height_error` honoured. Also the node-level tests (construct, TF read) need rclcpp spin or the accessor; no existing test constructs `MRUTransform` as a fixture (only `test_twist_rotation` references the type), so make `mapOrigin()` the primary check and drop the "if flaky" hedge. — `plan.md:74-85`
- [ ] (suggestion) d^2/2R definition: say explicitly d = hypot(x, y) of the fix in `map` (tangent-plane horizontal distance, before sensor offset; altitude ignored, which is correct since z is not used), R = mean radius. The local radius of curvature varies by ~0.3% to ~1% with latitude/azimuth, so the warn is an estimate, not a bound; state "approximate". Check of the formula: 3570 m gives 1.000 m, consistent with #48 (2 m at 5 km, 8 m at 10 km, 31 m at 20 km). — `plan.md:60-66`
- [ ] (suggestion) Threshold default 0.5 m (~2.5 km): the 2026 field failures in #48 / unh_echoboats_project11#473 were at 10-12 km (d^2/2R about 8-11 m), i.e. 16-20x above the default. The estimator's rejection limit is tidal range x `tide_range_margin` (default 2.0), so real harm is site dependent. 0.5 m will warn on any survey more than 2.5 km from a first-fix anchor (e.g. dock start, far end of a lake) for the whole run at one WARN per 10 s. Not wrong as a conservative default, but the plan should record this evidence for the owner's confirmation and either word the WARN with the remedy (restart near the work area, or set the origin; `reset_map_frame` re-anchors at the current fix) or set a higher default. Keep it a documented owner decision. — `plan.md:49,153`
- [ ] (suggestion) Parameter naming: existing convention is spelled-out `maximum_`/`minimum_` (`maximum_buffer_duration`, `maximum_interval_seconds`, `minimum_buffer_duration`). `map_origin_max_height_error` abbreviates; consider `map_maximum_height_error` (the bound applies to any map anchor, first-fix included, so the `map_origin_` prefix is also misleading). `map_origin_latitude`/`_longitude` fit existing `map_frame` naming. — `plan.md:49`
- [ ] (suggestion) Read-once parameters are silently ignored when changed at runtime (`ros2 param set` succeeds). The Quality Standard disfavours silent no-ops; a small `add_on_set_parameters_callback` rejecting changes to the three names ("restart required") is cheap and testable. — `plan.md:44-45`
- [ ] (suggestion) `reset_map_frame` before any fix has anchored discards an unconsumed configured origin; the message "no longer applies" would then be literally true but the origin was never used. Decide and test it (current plan text applies only "when an origin had been configured and still active", which is ambiguous for this case). — `plan.md:57-60`
- [ ] (suggestion) ADR row mislabelled: ADR-0010 lives in `unh_marine_autonomy` (`0010-geospatial-world-model.md`, D6 = vertical-datum library location), not "vertical datum split in this repo". Reword to "no ADR triggered; altitude 0 preserves chart_datum_node/sea_surface_estimator inputs". Verified those two consume only the TF `map` frame and never `updatePosition` state, so they are unchanged. — `plan.md:122`
- [ ] (suggestion) Test (a)/(d): make the first fix differ from the configured origin by more than the assertion tolerance (tens of metres at least), and assert exact altitude 0 in the ECEF comparison, so a bug that anchors at the fix cannot pass.

### Dimension summary
Scope Good; Issue alignment Good (all four checkpoint decisions and review-issue actions covered; `.agents/README.md` genuinely absent, gap noted correctly); File targeting Needs work (follow-ups); Consequences Needs work; Docs impact Good; Principle alignment Good; ADR compliance Needs work (wording only); ROS conventions Needs work (naming, startup failure, runtime param change).

## Implementation
**Status**: complete
**When**: 2026-10-12
**By**: Claude Code Agent (Claude Sonnet 5.5)

**Branch**: feature/issue-49 (not pushed)

### What was built
- `map_origin.hpp`: `validateMapOrigin`, `validateMaxHeightError`, `flatPlaneHeightError` (d^2/2R, R = 6371008.8 m), `exceedsFlatPlaneBound` (hypot of x, y in map).
- Parameters `map_origin_latitude` / `map_origin_longitude` (typed double, no default; altitude exactly 0) and `map_maximum_height_error` (default 0.5 m), validated at the top of the constructor before publishers/services, read once; an on-set callback rejects runtime sets.
- `mru_transform_node` main: catch, `RCLCPP_FATAL`, `rclcpp::shutdown()`, exit 1 (checked by hand: half-set origin and integer-typed latitude both exit 1).
- `reset_map_frame`: after anchoring re-anchors at the current fix and drops the configured origin; before any fix keeps it. Messages say so. Anchor state guarded by `state_mu_`; `mapOrigin()` accessor.
- Throttled WARN (10 s) with distance, estimated error, limit and remedy; also for first-fix anchors.
- README: parameters, Frames sentence, reset semantics (both cases), WARN, startup failure.

### Tests
`test_map_origin`: 17 (5 validation, 3 flat-plane, 9 node-level). Other packages' xunit unchanged and green: lifecycle 26, water_line_lever_arm 14, water_line_offset 12, nav_source_selection 9, twist_rotation 6, subscribe_once 1. 0 failures. No ament linters are declared in package.xml, so none ran; the new code compiles without new warnings under -Wall -Wextra -Wpedantic. Mutations checked (each failed the named tests): WARN removed, threshold hard-coded to 0.5, configured origin ignored, origin dropped early on reset, origin not cleared on reset.

### Deviations
- `RCLCPP_WARN_THROTTLE` uses a static per-call-site timer, so one far-WARN per test process per 10 s: fixture gives non-WARN tests a large threshold and the WARN test orders negative before positive (recorded in plan.md). Positive case uses a first-fix anchor; the configured-origin path is covered by the negative case only.
- Integer-typed origin test asserts only that the constructor throws (rclcpp's exception type).

### Gaps and follow-ups
- This repo has no `.agents/README.md` (only `.agents/ci_local_upstream_extra.sh`), so no verified-parameter table was updated; creating the guide is a separate task.
- Platform/launch follow-ups in other repos are listed in plan.md (unchanged; no other repo touched).

## Implementation

### Address review findings

Fixes for the pre-push review findings 1-10. Commits are on feature/issue-49, not pushed.

1. Node built inside the try, FATAL via `rclcpp::get_logger("mru_transform")` when it does not exist (fafcd0a). Not unit-testable; the typo'd-origin path was run by hand (FATAL line, exit 1).
2. Integer or double accepted for `map_origin_latitude/longitude` and `map_maximum_height_error` via dynamic typing; other types refused with `std::invalid_argument` naming the parameter (f2128f9). Tests: IntegerOriginAndThresholdAreAcceptedAndUsedExactly, NonNumericTypesAreRefusedWithTheParameterName. Run by hand: `latitude:=43` starts and keeps running. With dynamic typing switched off, the integer test fails (mutation check).
3. WARN text shortened, no URL, says a restart or `reset_map_frame` shifts the map frame (7a50388).
4. Decision factored into `FarFromOriginWarning` plus `MRUTransform::shouldWarnFarFromOrigin` (instance state, injected time) (7a50388). Tests: DefaultThresholdIsHalfAMetre, FarWarnDecisionHonoursTheConfiguredThreshold, FarWarnRepeatsEveryTenSecondsWhileExceeded, FarFixLogsOneWarn... Removing the throttle fails the repeat and wiring tests (mutation check).
5. configured_origin_ comment corrected (7a50388).
6. Validation-ordering claim narrowed in code (f2128f9) and README (0837937).
7. Single-threaded assumption stated at state_mu_ and in the node main (7a50388, fafcd0a).
8. README notes a bad origin takes only this node down (0837937).
9. test_map_origin uses ROS_DOMAIN_ID 87 (6d2a2a6).
10. `exceedsFlatPlaneBound` uses x^2+y^2 directly (7a50388).

Plan updated (18dd60c). Full `./platforms_ws/test.sh mru_transform`: 96 tests, 0 failures (test_map_origin 21, lifecycle_reconfigure 26, nav_source_selection 9, subscribe_once 1, twist_rotation 6, water_line_lever_arm 14, water_line_offset 12).

Not verified: a failure of the `rclcpp::Node` constructor itself (hard to provoke; a bad remap fails earlier, in `rclcpp::init`).
