---
issue: 23
---

# Issue #23 — subscribeCheck timer re-subscribes every second (ROS 1 one-shot semantics lost in ROS 2 port)

## Issue Review
**Status**: complete
**When**: 2026-05-27 19:47 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #23
**Comment**: https://github.com/rolker/mru_transform/issues/23#issuecomment-4559555297
**Scope verdict**: well-scoped

### Actions
- [ ] Ship the regression test (subscribe() runs once, timer cancels) in the same PR — do not defer.
- [ ] Confirm no package docs describe subscribe/retry behavior; update any that do.
- [ ] Maintainer decision: approach A (minimal cancel) vs B (restructure to single repeating timer + cancel) before implementing.
- [ ] Make subscribe-once semantics a conscious choice; if dynamic type-switching is ever wanted, open a separate issue (don't bury).
- [ ] Add null-safety on the `subscribe_check_timer_->cancel()` call.
- [ ] Decide whether to include the optional "already-subscribed" guard in `subscribe()` (defense-in-depth; keep change minimal if not).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 22:17 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved

**Branch**: feature/issue-23 at `a8baec6`
**Mode**: pre-push
**Depth**: Standard (reason: shared sensor base header + subscription lifecycle, ~180 lines across 6 files)
**Must-fix**: 0 | **Suggestions**: 1

Specialists: Static Analysis (cppcheck; cpplint unavailable), Governance (lead), Claude Adversarial, Copilot Adversarial. Plan Drift skipped (no plan.md). Two Copilot "must-fix" claims verified as false positives (the test's explicit `ASSERT_EQ` after the wait loop catches "never subscribed"; `spin_for` gates on `steady_clock < deadline` so it runs the full duration). Cppcheck style hits were all on pre-existing context lines, not touched here.

### Findings
- [ ] (suggestion) Surface intentional loss of runtime type-switching in PR body — `mru_transform/include/mru_transform/sensor.hpp:111` (cross-confirmed by Claude Adversarial and the earlier Issue Review; subscribe-once semantics match the original ROS 1 intent).
- [x] (suggestion-applied) Test Phase 1 discovery headroom raised 6 s → 15 s for CI flake insurance — `mru_transform/test/test_subscribe_once.cpp:102` (folded into a8baec6).
