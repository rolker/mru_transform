---
issue: 34
---

# Issue #34 — `SeaSurfaceEstimator::on_configure` re-declares parameters on cleanup->configure, throwing `ParameterAlreadyDeclaredException`

## Issue Review
**Status**: complete
**When**: 2026-08-23 22:57 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #34
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Verification

Read `mru_transform/nodes/sea_surface_estimator.cpp` directly (not taking the
issue body's claim on faith, per the injected-context caveat):

- `on_configure` (lines 26-75) calls `declare_parameter()` seven times:
  `sea_surface_frame`, `minimum_buffer_duration`, `maximum_buffer_duration`,
  `water_line_frame`, `chart_datum_frame`, `mhhw_frame`, `tide_range_margin`.
- `on_cleanup` (lines 87-97) resets `tf_listener_`/`tf_buffer_`, the lever-arm
  cache, and buffered odometry state, but never calls `undeclare_parameter()`
  or guards the declarations with `has_parameter()`.
- `rclcpp`'s `NodeParameters::declare_parameter()` throws
  `rclcpp::exceptions::ParameterAlreadyDeclaredException` when a parameter
  name is already declared on the node — this is standard, well-documented
  `rclcpp` behavior, not something specific to this package. So a
  `cleanup` -> `configure` cycle does throw out of `on_configure` exactly as
  described. **Claim confirmed against source, not just asserted.**
- The launch file (`launch/sea_surface_estimator_launch.py`) drives this node
  as a managed `LifecycleNode`; nothing in the launch wiring prevents an
  operator or a supervising process from issuing `cleanup` then `configure`
  in the field, so the failure mode is reachable, not theoretical.

**Sibling audit (fix sketch names `chart_datum_node` and
`nav_sat_fix_to_velocity`) — checked both, plus one the sketch omits:**

- `chart_datum_node.cpp`: 11 `declare_parameter()` calls in `on_configure`,
  `on_cleanup` present but does not undeclare — same bug shape, correctly
  named in the fix sketch.
- `nav_sat_fix_to_velocity.cpp`: 2 `declare_parameter()` calls
  (`map_frame`, `maximum_interval_seconds`) in `on_configure`; **no
  `on_cleanup` override at all** — same bug shape (the params are declared
  once and never released across a cleanup cycle), correctly named in the
  fix sketch.
- `tide_copier.cpp`: 4 `declare_parameter()` calls (`input_map_frame`,
  `input_map_tide_frame`, `output_map_frame`, `output_map_tide_frame`) in
  `on_configure`; has an `on_cleanup` override that also does not
  undeclare. **Same bug shape, but this file is not named in the issue's
  fix-sketch audit list.** Flagged as a Recommendation below rather than
  blocking — the issue's primary scope (sea_surface_estimator) stands on
  its own regardless.
- `mru_transform_node.cpp` is a plain `rclcpp::Node`, not a
  `LifecycleNode` — not affected by this class of bug, correctly excluded.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | Action needed | Issue body flags the reconfigure path as untested and notes "there is no such fixture in the package yet," but stops short of committing to build one. AGENTS.md Quality Standard ("fix it completely: add the test... check the lifecycle transition") requires the harness be added as part of this fix, not deferred — a parameter-lifecycle regression like this is exactly the kind of bug a "looks fixed" patch can silently un-fix later without a configure->cleanup->configure test. |
| Improve incrementally | OK | Scoping the fix to `sea_surface_estimator` alone (mirroring how #33 kept the tide fix reviewable on its own) is the right call — a repo-wide sweep across all lifecycle nodes in one PR would be harder to review. |
| A change includes its consequences | Watch | The fix sketch explicitly proposes auditing sibling nodes but only lists two of the three affected ones (misses `tide_copier`). Worth completing the audit scope, whether as part of this issue's follow-up note or a fresh issue, so the gap doesn't get lost. |
| Capture decisions, not just implementations | OK | The issue already documents why this was deliberately deferred from #33 and why it blocks #32's lever-arm cache invalidation from being exercised in the field — good context for future readers. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0008 — Follow ROS 2 Official Conventions | Yes | Correct handling of the lifecycle `configure`/`cleanup` transition pair is a core ROS 2 managed-node convention; the fix brings the node into conformance rather than deviating from it. No new ADR needed. |

### Dependencies

None blocking. Builds on the already-merged #32/#33 (lever-arm cache
invalidation + tide fix); this issue is what makes that cache-invalidation
logic reachable in the field, per the issue body.

### Actions
- [ ] Implementation should add a configure->cleanup->configure lifecycle-harness test (there being no such fixture yet is a reason to build one, not to skip the test) rather than shipping the parameter-declaration fix untested.
- [ ] Recommend extending the "audit the package's other lifecycle nodes" follow-up to include `tide_copier.cpp`, not just `chart_datum_node` and `nav_sat_fix_to_velocity` — it has the identical declare-without-undeclare shape.
