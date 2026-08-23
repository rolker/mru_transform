---
issue: 32
---

# Issue #32 — Field import: mru_transform (2026-08-21)

## Local Review
**Status**: complete
**When**: 2026-08-23 00:17 -04:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**PR**: #33 at `58b7f97`
**Mode**: post-PR
**Depth**: Deep (reason: 279 changed lines, 200+ threshold)
**Must-fix**: 6 | **Suggestions**: 11

### Findings
- [x] (must-fix) Lockstep claim is false: an undeclared parameter override is a SILENT no-op, not a startup failure — verified empirically against the installed pre-fix `sea_surface_estimator`, which configured cleanly with `water_line_frame` set. Correct the wording in issue #32, this PR body, and `unh_echoboats_project11` PR #453 — `PR#33 body` (deferred: the false claim appears only in the issue and PR bodies, which the host is editing; `grep` finds no such wording in any repo file)
- [x] (must-fix) NaN quaternion defeats the degenerate guard (`q.length2() <= 0.0` is false for NaN) and propagates NaN through the average into latched `tide_estimate` and the broadcast `map_tide`; `is_out_of_range` cannot reject NaN — `include/mru_transform/water_line_offset.hpp:33`
- [x] (must-fix) When `water_line_frame` is configured but the lookup fails, the node still publishes and latches (transient_local) a tide it has just logged as wrong, and a late-resolving TF steps the whole rolling window by the full lever arm mid-line — `nodes/sea_surface_estimator.cpp:117-134`
- [x] (must-fix) Lever-arm cache is not reset in `on_cleanup` and is not keyed on `water_line_frame_`, so a cleanup/re-configure keeps the old frame's lever arm silently (cross-confirmed: Copilot, Lens A, Lens B) — `nodes/sea_surface_estimator.cpp:156-172`
- [x] (must-fix) The node-level logic is untested — lookup argument order (the sign convention), the `z +=` application, `have_offset` gating, cache reuse/invalidation, empty `child_frame_id`. A `tf2_ros::Buffer` + `setTransform` fixture pins it in ~a dozen lines — `test/`
- [x] (must-fix) Documentation consequence missing: `README.md` gains no `water_line_frame` entry and no `sea_surface_estimator` parameter section, and the changed meaning of `map_tide`/`tide_estimate` is undocumented — `README.md`
- [x] (suggestion) `AttitudeTermIsNegligibleForBizzyButNotForALargeLeverArm` asserts one-sided bounds that also pass for the scalar-offset implementation it claims to distinguish; use `EXPECT_NEAR` against the closed form — `test/test_water_line_offset.cpp:96-106`
- [x] (suggestion) `have_offset` is decided from the newest message's `child_frame_id` but applied to every buffered sample — `nodes/sea_surface_estimator.cpp:114-125`
- [x] (suggestion) An empty `child_frame_id` produces `Cannot look up '' -> ...` every 10 s, pointing at TF instead of the odometry publisher — `nodes/sea_surface_estimator.cpp:166`
- [x] (suggestion) On persistent lookup failure a `tf2::TransformException` is constructed and thrown per odometry message (10-50 Hz) while only the log is throttled; rate-limit the retry — `nodes/sea_surface_estimator.cpp:168`
- [x] (suggestion) `TimePointZero` plus a permanent cache silently freezes the first value if `water_line_frame` is ever pointed at a non-static frame; the comment asserts staticness but nothing enforces it — `nodes/sea_surface_estimator.cpp:152-156`
- [x] (suggestion) Error log reports `vehicle -> water_line` while `lookupTransform(vehicle, water_line)` is source-into-target; repo convention elsewhere is source-first (Copilot) — `nodes/sea_surface_estimator.cpp:196`
- [x] (suggestion) On a metre-scale platform the correction can newly trip `is_out_of_range`; note also that `tide_estimate` publishes unconditionally while `map_tide` is suppressed, so topic and frame can disagree — `nodes/sea_surface_estimator.cpp:203`
- [x] (suggestion) Pre-existing, adjacent, two-line fix: `on_cleanup` resets `tf_buffer_` before `tf_listener_`, a use-after-free against the listener's spin thread; this diff adds a second buffer consumer — `nodes/sea_surface_estimator.cpp:85-86`
- [x] (suggestion) Pre-existing: `on_configure` re-declares parameters with no undeclare, so any cleanup -> configure cycle throws `ParameterAlreadyDeclaredException`; this diff adds a fifth declaration. File as its own issue — `nodes/sea_surface_estimator.cpp:25-44` (deferred: filed as rolker/mru_transform#34 rather than expanded into this PR)
- [x] (suggestion) Pre-existing repo gap: `ament_lint_auto_find_test_dependencies()` registers zero linters (no `ament_lint_common` test_depend), and the repo has no `.github/workflows` and no `.pre-commit-config.yaml`; merge verification must be a full-scope `ci_local.sh` attestation per ADR-0018. File as its own issue — `mru_transform/package.xml` (deferred: filed as rolker/mru_transform#35)
- [x] (suggestion) Governance gaps, not this PR's job: no `.agents/README.md` and no root `AGENTS.md` (ADR-0017) in this repo (deferred: filed as rolker/mru_transform#36)

## Implementation
**Status**: complete
**When**: 2026-08-23 00:55 -04:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-32 at `392e270`
**PR**: #33
**Addressed**: `## Local Review` (post-PR, changes-requested) — 2026-08-23 00:17 -04:00, PR #33 at `58b7f97`
**Commits**: 781da70 7a5ffac 9c192a5 392e270 

All six must-fixes and all eleven suggestions are handled: thirteen fixed here,
four consciously deferred (one to the host, three to their own issues).

### Verification

This repo has no CI, so local verification is the only gate.
`colcon build --symlink-install` + `colcon test` against the platforms layer:
**69 tests, 0 failures** (was 52 before #32, 55 after the first commit on this
branch). No new compiler warnings from the changed files.

### Actions
- [x] (must-fix) Lockstep claim — `PR#33 body` (deferred: the false claim lives only in the issue and PR bodies, which the host is editing; `grep` over every tracked file finds no such wording in the repo, so there is nothing to fix here)
- [x] (must-fix) NaN quaternion defeats the degenerate guard — `include/mru_transform/water_line_offset.hpp:33` — the four components and `length2()` are now checked with `std::isfinite`; NaN/inf attitudes fall back to the unrotated lever arm. Odometry with a non-finite `position.z` is dropped at ingest, and the average is re-checked before publishing, so no NaN can reach `tide_estimate` or `map_tide`. New cases: a NaN in each component, infinite components, and finite components that overflow when squared
- [x] (must-fix) Publishes and latches a tide it just logged as wrong — `nodes/sea_surface_estimator.cpp:117-134` — a configured-but-unresolvable `water_line_frame` now publishes **nothing** (no `tide_estimate`, no `map_tide`) and logs an error per throttle interval. That also removes the mid-line step: the window is only ever published with the correction applied
- [x] (must-fix) Lever-arm cache not reset in `on_cleanup`, not keyed on `water_line_frame_` — `nodes/sea_surface_estimator.cpp:156-172` — the cache moved into `mru_transform::WaterLineLeverArm`, which keys on both frames and invalidates on either changing; `on_cleanup` calls `reset()` and clears the odometry window. Pinned by two tests
- [x] (must-fix) Node-level logic untested — `test/test_water_line_lever_arm.cpp` (new) — 14 cases over a `tf2_ros::Buffer` + `setTransform` fixture: the lookup argument order in both sign directions (water line above and below the vehicle), the offset as the estimator applies it, disabled / empty-vehicle-frame / missing-transform / non-finite paths, cache reuse against an empty buffer, invalidation on either frame changing, `reset()`, forced refresh with change detection, and a failed refresh keeping the earlier value
- [x] (must-fix) Documentation consequence — `README.md` — new `sea_surface_estimator` section: all seven parameters, the three behaviours of `water_line_frame`, the staticness expectation, the changed meaning of `map_tide` (water line now, vehicle frame before), and that `tide_estimate` (raw) and `map_tide` (accepted) can disagree — consumers needing the tide the system stands behind must read the frame
- [x] (suggestion) One-sided bounds in the attitude-magnitude test — `test/test_water_line_offset.cpp:96-106` — replaced with the closed form `z*(1 - cos(phi))`, which the scalar-offset implementation the test claims to rule out cannot pass
- [x] (suggestion) `have_offset` decided from the newest message, applied to every sample — `nodes/sea_surface_estimator.cpp:114-125` — a change of `child_frame_id` now restarts the averaging window (logged), so every buffered sample belongs to the frame whose lever arm corrects it
- [x] (suggestion) Empty `child_frame_id` blamed on TF — `nodes/sea_surface_estimator.cpp:166` — now a distinct `NoVehicleFrame` status whose message names the odometry topic and says to fix the publisher
- [x] (suggestion) An exception per odometry message — `nodes/sea_surface_estimator.cpp:168` — failed lookups are retried at most once per second (`kLeverArmRetryPeriod`) instead of at 10-50 Hz
- [x] (suggestion) `TimePointZero` + permanent cache freezes a non-static frame — `nodes/sea_surface_estimator.cpp:152-156` — the cache is re-read every 10 s (`kLeverArmRefreshPeriod`); a lever arm that moved more than a millimetre is logged as a WARN naming the staticness expectation, rather than silently changing the tide. A failed refresh keeps the earlier value
- [x] (suggestion) Error log direction — `nodes/sea_surface_estimator.cpp:196` — now source-first (`'<water_line>' -> '<vehicle>'`), matching the repo convention; the argument-order rationale is documented at the `lookupTransform` call and pinned by test
- [x] (suggestion) Correction can newly trip `is_out_of_range`; topic and frame can disagree — `nodes/sea_surface_estimator.cpp:203` — both documented, in the README and at the publish site
- [x] (suggestion) `on_cleanup` resets `tf_buffer_` before `tf_listener_` — `nodes/sea_surface_estimator.cpp:85-86` — reordered (listener first), as recommended
- [x] (suggestion) `on_configure` re-declares parameters — `nodes/sea_surface_estimator.cpp:25-44` (deferred: filed as rolker/mru_transform#34 rather than expanded into this PR, per the dispatch instruction. Worth knowing: it means the cache invalidation added here cannot be exercised in the field until #34 lands — the cleanup -> configure transition throws first)
- [x] (suggestion) Zero linters registered, no workflow, no pre-commit — `mru_transform/package.xml` (deferred: filed as rolker/mru_transform#35)
- [x] (suggestion) No `.agents/README.md`, no root `AGENTS.md` (deferred: filed as rolker/mru_transform#36)

### Follow-ups filed
- rolker/mru_transform#34 — `sea_surface_estimator`: cleanup -> configure throws `ParameterAlreadyDeclaredException`
- rolker/mru_transform#35 — no verification in this repo (lint_auto registers zero linters, no CI workflow, no pre-commit)
- rolker/mru_transform#36 — add `.agents/README.md` and root `AGENTS.md` (ADR-0017)

### Note for the re-review
`WaterLineLeverArm` is a new header (`include/mru_transform/water_line_lever_arm.hpp`)
carrying logic extracted from the node, not merely relocated: the cache key, the
refresh, the finite check and the status taxonomy are new. It is header-only and
installed with the rest of `include/`, so it becomes part of the package's public
surface; that was the cost of making the lookup testable without a node harness.
