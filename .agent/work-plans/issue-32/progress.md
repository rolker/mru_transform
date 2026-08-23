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
- [ ] (must-fix) Lockstep claim is false: an undeclared parameter override is a SILENT no-op, not a startup failure — verified empirically against the installed pre-fix `sea_surface_estimator`, which configured cleanly with `water_line_frame` set. Correct the wording in issue #32, this PR body, and `unh_echoboats_project11` PR #453 — `PR#33 body`
- [ ] (must-fix) NaN quaternion defeats the degenerate guard (`q.length2() <= 0.0` is false for NaN) and propagates NaN through the average into latched `tide_estimate` and the broadcast `map_tide`; `is_out_of_range` cannot reject NaN — `include/mru_transform/water_line_offset.hpp:33`
- [ ] (must-fix) When `water_line_frame` is configured but the lookup fails, the node still publishes and latches (transient_local) a tide it has just logged as wrong, and a late-resolving TF steps the whole rolling window by the full lever arm mid-line — `nodes/sea_surface_estimator.cpp:117-134`
- [ ] (must-fix) Lever-arm cache is not reset in `on_cleanup` and is not keyed on `water_line_frame_`, so a cleanup/re-configure keeps the old frame's lever arm silently (cross-confirmed: Copilot, Lens A, Lens B) — `nodes/sea_surface_estimator.cpp:156-172`
- [ ] (must-fix) The node-level logic is untested — lookup argument order (the sign convention), the `z +=` application, `have_offset` gating, cache reuse/invalidation, empty `child_frame_id`. A `tf2_ros::Buffer` + `setTransform` fixture pins it in ~a dozen lines — `test/`
- [ ] (must-fix) Documentation consequence missing: `README.md` gains no `water_line_frame` entry and no `sea_surface_estimator` parameter section, and the changed meaning of `map_tide`/`tide_estimate` is undocumented — `README.md`
- [ ] (suggestion) `AttitudeTermIsNegligibleForBizzyButNotForALargeLeverArm` asserts one-sided bounds that also pass for the scalar-offset implementation it claims to distinguish; use `EXPECT_NEAR` against the closed form — `test/test_water_line_offset.cpp:96-106`
- [ ] (suggestion) `have_offset` is decided from the newest message's `child_frame_id` but applied to every buffered sample — `nodes/sea_surface_estimator.cpp:114-125`
- [ ] (suggestion) An empty `child_frame_id` produces `Cannot look up '' -> ...` every 10 s, pointing at TF instead of the odometry publisher — `nodes/sea_surface_estimator.cpp:166`
- [ ] (suggestion) On persistent lookup failure a `tf2::TransformException` is constructed and thrown per odometry message (10-50 Hz) while only the log is throttled; rate-limit the retry — `nodes/sea_surface_estimator.cpp:168`
- [ ] (suggestion) `TimePointZero` plus a permanent cache silently freezes the first value if `water_line_frame` is ever pointed at a non-static frame; the comment asserts staticness but nothing enforces it — `nodes/sea_surface_estimator.cpp:152-156`
- [ ] (suggestion) Error log reports `vehicle -> water_line` while `lookupTransform(vehicle, water_line)` is source-into-target; repo convention elsewhere is source-first (Copilot) — `nodes/sea_surface_estimator.cpp:196`
- [ ] (suggestion) On a metre-scale platform the correction can newly trip `is_out_of_range`; note also that `tide_estimate` publishes unconditionally while `map_tide` is suppressed, so topic and frame can disagree — `nodes/sea_surface_estimator.cpp:203`
- [ ] (suggestion) Pre-existing, adjacent, two-line fix: `on_cleanup` resets `tf_buffer_` before `tf_listener_`, a use-after-free against the listener's spin thread; this diff adds a second buffer consumer — `nodes/sea_surface_estimator.cpp:85-86`
- [ ] (suggestion) Pre-existing: `on_configure` re-declares parameters with no undeclare, so any cleanup -> configure cycle throws `ParameterAlreadyDeclaredException`; this diff adds a fifth declaration. File as its own issue — `nodes/sea_surface_estimator.cpp:25-44`
- [ ] (suggestion) Pre-existing repo gap: `ament_lint_auto_find_test_dependencies()` registers zero linters (no `ament_lint_common` test_depend), and the repo has no `.github/workflows` and no `.pre-commit-config.yaml`; merge verification must be a full-scope `ci_local.sh` attestation per ADR-0018. File as its own issue — `mru_transform/package.xml`
- [ ] (suggestion) Governance gaps, not this PR's job: no `.agents/README.md` and no root `AGENTS.md` (ADR-0017) in this repo
