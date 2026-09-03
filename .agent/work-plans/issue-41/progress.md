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

## Integrated Review
**Status**: complete
**When**: 2026-09-02 23:22 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #46 at `67f6d7c`
**Sources**: 4 (Copilot R2 @ `67f6d7c`, Copilot R1 @ `827795e`, Local Review (Pre-Push) @ `a6da79e`, CI rollup)
**Cross-source confirmations**: 0 (Copilot raised no specific findings in either pass)
**CI**: only the Copilot reviewer check (this repo has no hosted build/test CI — #35). **No `ci_local` attestation exists on this head** — the ADR-0018 merge gate is unmet, and the head changed after the jazzy merge.

### Findings
- [ ] (blocker-for-merge, CI rollup) No full-scope `ci_local` attestation on `67f6d7c`. The #42 attestation covered a different branch and head. Run `.agent/scripts/ci_local.sh` and push `refs/notes/ci-local` before merging — the merge-resolution commit is untested in a clean room
- [ ] (advisory, Copilot R1+R2) Both passes closed with "needs a closer look — warrants final human review", citing the safety-relevant TF/datum numeric path and the new hard dependency on a core package. No defect claimed; it is a recommendation against merging on bot approval alone, and it converges with the local Deep review's own reasons for running at Deep
- [ ] (suggestion, Local Review) `datum_polygons.example.yaml` stays in this repo while its parser moved upstream, so mru_transform's README and marine_vertical_datum's now reference each other circularly — carried, not fixed
- [ ] (suggestion, Local Review) No `.agents/README.md` or root `AGENTS.md` in this repo — pre-existing, tracked as #36

### False positives
- (none) Neither Copilot pass produced an inline comment: R1 reviewed the pre-merge head, R2 reviewed 11/11 files at the current head and generated 0 new comments. Nothing to dismiss.
