---
issue: 25
---

# Issue #25 — Datum support off-VDatum: polygon-keyed datum config (always-on) + lake_datum override

## Issue Review
**Status**: complete
**When**: 2026-06-13 (local)
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #25
**Comment**: https://github.com/rolker/mru_transform/issues/25#issuecomment-4700414622
**Scope verdict**: well-scoped

### Actions
- [ ] Keep Massabesic polygon data out of the generic `mru_transform` package — ship loader + synthetic example schema here; put the real Massabesic polygon in the platform/site overlay (acceptance item 6).
- [ ] Verify and state in the PR that both downstream consumers (S57 `chart_layer` depth→clearance, `sea_surface_estimator` tide plausibility bound) behave correctly given the new always-on frame, and can distinguish the ellipsoid fallback from a surveyed tidal datum.
- [ ] Tag each published datum with its provenance (VDatum / polygon / param / ellipsoid-default) in logs/diagnostics so the safe default is never mistaken for a real datum (safety / silent-wrong-data).
- [ ] Coordinate the new always-published datum frame's naming/parenting with #8 (Marine TF vertical-datum hierarchy).
- [ ] Prefer a single polygon representation; avoid adding a new geometry dependency for point-in-polygon.
- [ ] Capture the "ellipsoid-default is safe" rationale + override-first-vs-fallback semantics durably (PR body / node header doc; ADR if adopted).
- [ ] Document the new config-path param, override-ordering flag, `lake_datum`, and ellipsoid-default behavior in the node header, `chart_datum_launch.py`, and README.

## Plan Authored
**Status**: complete
**When**: 2026-06-13 (local)
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-25/plan.md` at `cc56d18`
**PR**: https://github.com/rolker/mru_transform/pull/26 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [ ] Config format: confirm adding yaml-cpp dep for the file-based polygon config (vs ROS nested params).
- [ ] Frame naming: should the ellipsoid-default frame keep `chart_datum` or get a distinct name consumers can refuse? Align with #8.
- [ ] `chart_layer` behavior on an ellipsoid-referenced datum (tolerate vs skip clearance) — confirms whether a cross-repo follow-up is needed.

## Plan Review
**Status**: complete
**When**: 2026-06-13 (local)
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-25/plan.md` at `1584e96`
**PR**: https://github.com/rolker/mru_transform/pull/26
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Precedence logic lives in untested node code — widen pure `resolve()` to take the optional VDatum result + config + param + override flag and return final (source, datum), so acceptance item 5's matrix is unit-testable — `plan.md` steps 1,4,7
- [ ] (suggestion) Add `install(DIRECTORY config ...)` so `datum_polygons.example.yaml` ships (config/ not installed today) — `plan.md` step 6
- [ ] (suggestion) Document/decide: a chart_datum-only entry (no mhhw) silently disables sea_surface_estimator's tide-plausibility bound (mhhw lookup throws → don't-filter) — `plan.md` step 4
- [ ] (suggestion) Specify overlapping-polygon first-match semantics + antimeridian limitation of raw lat/lon ray-cast in schema docs — `plan.md` steps 1,8
- [ ] (nit) `lake_datum` is a deployment-flavored name for a generic fixed-datum override — conscious choice per issue

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-14 (local)
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-25 at `248e098`
**Mode**: pre-push
**Depth**: Deep (reason: ~820 lines, lifecycle node, safety-relevant datum, cross-layer consumers)
**Must-fix**: 0 remaining (2 fixed) | **Suggestions**: 4 fixed, 1 open design decision, 2 cross-repo follow-ups

### Findings
- [x] (must-fix) publish_rate/recalc_interval ≤ 0 unvalidated → silent never-publish / pegged core — fixed in `chart_datum_node.cpp` on_configure
- [x] (must-fix) datum_source_pub_ post-cleanup null-publish window (asymmetry vs mllw/mhhw pubs) — fixed by dropping the on_cleanup reset
- [x] (suggestion) lake_datum ±Inf bypassed the NaN sentinel → could reach TF — fixed via std::isfinite + isinf warn
- [x] (suggestion) load_datum_config threw raw YAML::Exception, breaking its std::runtime_error contract — fixed (wrap+rethrow)
- [x] (suggestion) test gaps: collinear-outside / vertex-latitude / concave point_in_ring; malformed-vertex + non-numeric parse — added (21 gtests)
- [x] (suggestion) plan README path nit (repo root, not mru_transform/) — fixed
- [x] (design decision) config-load failure returns CallbackReturn::FAILURE — RESOLVED: keep FAILURE (Roland, 2026-06-14); operator set the path deliberately, so a broken file should stop the node loudly
- [x] (follow-up, cross-repo) s57_layer stale tide_offset_ when datum→none → filed s57_tools#26
- [x] (follow-up, same repo, separate node) sea_surface_estimator warn when chart_datum present but MHHW missing → filed mru_transform#27

### Governance / plan adherence
All principles Pass (Capture-decisions: Watch — VDatum-now-optional lives in code/commit, repo has no ADR system). ADR-0008 compliant. Plan adherence: faithful; no scope creep; all 9 planned files changed.
