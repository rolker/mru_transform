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
