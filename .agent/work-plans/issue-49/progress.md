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
