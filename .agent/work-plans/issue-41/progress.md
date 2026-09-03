---
issue: 41
---

# Issue #41 — chart_datum_node: consume marine_vertical_datum instead of the inline copy (ADR-0010 D6 follow-on)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-02 22:58 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))
**Verdict**: changes-requested (addressed in this round)

**Branch**: feature/issue-41 at `a6da79e`
**Mode**: pre-push
**Depth**: Deep (reason: cross-layer dependency + lifecycle resource management + numeric path on a safety frame)
**Must-fix**: 2 | **Suggestions**: 7
**Round**: 1 | **Ship**: recommended — both must-fixes fixed in-round; equivalence of the numeric path independently verified

### Findings
- [x] (must-fix) Dropped per-point MHHW warning removes the only evidence that #43's fail-open has triggered; the "cannot distinguish" argument was defeatable with a `seen_mhhw_` latch — `chart_datum_node.hpp`
- [x] (must-fix) `upstream.repos` absent, so a clean-container `ci_local.sh` (ADR-0018 gate) cannot resolve `marine_vertical_datum` — `upstream.repos`
- [x] (suggestion) Three ERROR-level setup diagnostics silently demoted to WARN by routing all of DiagFn to one level — `chart_datum_node.hpp`
- [x] (suggestion) Plan claimed one behaviour change; the library also `std::sort`s grid paths, which can change which grid answers in an overlap region — plan + PR body corrected; inert on the deployed single-bundle set
- [x] (suggestion) Positional aggregate init of a struct owned by another repo — a field reorder upstream would silently swap geoid path and grid dir — `chart_datum_node.hpp`
- [x] (suggestion) Plan promised this branch would assert preserved behaviour itself; no tests had been added — 3 added, 71 → 74
- [x] (suggestion) Destructor comment implied teardown ordering closes the multi-threaded window; it only narrows it — `chart_datum_node.hpp`
- [x] (suggestion) README did not say the implementation moved upstream — `README.md`
- [ ] (suggestion) `datum_polygons.example.yaml` stays here while the parser moved upstream; the two READMEs now reference each other circularly — noted in PR, not fixed
- [ ] (suggestion) No `.agents/README.md` or root `AGENTS.md` in this repo — pre-existing, tracked as #36
- [ ] (note) Verified independently: units, sign convention, pipeline string, grid-matching rule and every resource-release path are equivalent to the deleted inline code; deleted files were a byte-identical move, tests included
