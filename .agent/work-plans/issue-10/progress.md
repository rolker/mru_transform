---
issue: 10
---

# Issue #10 — Don't download VDatum grids during build — check and warn instead

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-02 22:16 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))
**Verdict**: changes-requested (addressed in this round)

**Branch**: feature/issue-10 at `fb929e0`
**Mode**: pre-push
**Depth**: Standard (reason: build-config change with cross-repo + field-host consequences)
**Must-fix**: 3 | **Suggestions**: 5
**Round**: 1 | **Ship**: recommended — all code must-fixes fixed in-round; what holds this PR is the merge gate, not the diff

### Findings
- [x] (must-fix) README/launch/commit claimed absent grids fall back to the polygon/param chain — false wherever no polygon covers the boat; cross-confirmed by governance + both adversarial lenses — `README.md`, `mru_transform/launch/chart_datum_launch.py`
- [x] (must-fix) No test covered the missing-grid configure path this change makes normal — `mru_transform/test/test_lifecycle_reconfigure.cpp`
- [ ] (must-fix) Merge gate unsatisfied: gabby has provisioning evidence (2026-08-20 log), salmon has none — uma#288 item 6 requires both; PR opens as draft
- [x] (suggestion) Commit claimed a missing dir and an empty vdatum_grid_dir are "the same non-fatal path" — different branches, ERROR vs INFO — commit message
- [x] (suggestion) README parameter table showed `""` defaults without noting the launch file overrides them — `README.md`
- [x] (suggestion) expanduser binds defaults to the launching user's home (systemd/container/sudo divergence) — now documented — `mru_transform/launch/chart_datum_launch.py`
- [x] (suggestion) Coverage narrows from national to configured bundles; no pre-deployment verification recipe — now documented — `README.md`
- [x] (suggestion) Orphaned ~/.cache/mru_transform + install share/data left with no cleanup note — now documented — `README.md`
- [ ] (suggestion) `datum_source` has no subscriber anywhere in the workspace, so "no chart datum" reaches no operator surface — out of scope, follow-up
- [ ] (suggestion) `sea_surface_estimator::is_out_of_range()` silently returns false when datum frames are absent, so the tide plausibility bound goes inert with no log — pre-existing fail-open whose probability this change raises — out of scope, follow-up
- [ ] (suggestion) `geoid_grid` existence is never checked; a missing geoid with present grids surfaces as "No VDatum MLLW coverage", the wrong diagnosis — out of scope, follow-up
- [ ] (suggestion) ben and seafloor_echoboat configure no polygon/lake fallback, so VDatum is their only datum source; not in the gate's host list — noted in PR body
