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
