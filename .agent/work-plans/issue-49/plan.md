# Plan: The map origin cannot be pre-defined (map anchored at first position fix)

## Issue

https://github.com/rolker/mru_transform/issues/49

## Context

`MRUTransform::updatePosition` (`mru_transform/src/mru_transform.cpp`) builds the
`MapFrame` from the first fix (altitude forced to 0) when `mapFrame_` is null;
`resetMapFrameService` just does `mapFrame_.reset()`, so the next fix re-anchors.
The only map parameters are frame names. `MRUTransform` is a plain `rclcpp::Node`
class constructed by `nodes/mru_transform_node.cpp`.

Owner decisions (2026-10-12), implemented exactly:

1. Lands on its own now; #48 (flat-plane design) stays open and the map frame
   itself is not redesigned.
2. Origin = two optional parameters, `map_origin_latitude` / `map_origin_longitude`
   (degrees). Altitude is always 0 (ellipsoid), mru_transform's own convention
   (not attributed to a REP). `map_tide`, `chart_datum_node`, `sea_surface_estimator`
   inputs are unchanged. Both unset = today's behaviour.
3. `reset_map_frame` re-anchors at the CURRENT fix even when an origin is configured;
   afterwards the configured origin no longer applies until restart. Response
   message and README say so.
4. Throttled WARN when flat-plane height error d²/2R exceeds a threshold parameter;
   nothing refused or stopped. Covers #48's stopgap direction 5 (reference #48, do not close).

## Approach

Revised 2026-10-12 after the Plan Review (all must-fix items and the suggestions
taken; owner decisions 1-4 unchanged, decision 3 and 4 refined below).

1. **Pure helper header `include/mru_transform/map_origin.hpp`** (header-only, no ROS
   node needed, so it is unit-testable without DDS):
   - `std::optional<geographic_msgs::msg::GeoPoint> validateMapOrigin(std::optional<double> lat, std::optional<double> lon)`:
     both unset -> `nullopt`; exactly one set -> `throw std::invalid_argument` naming the
     missing one; non-finite, lat outside [-90, 90] or lon outside [-180, 180] -> throw
     with the offending value. Returned point has altitude exactly 0.
   - `double flatPlaneHeightError(double horizontal_distance_m)` = d^2/(2R), R = 6371008.8 m
     (mean Earth radius). It is an **estimate**, not a bound: the local radius of curvature
     differs from the mean by up to ~1% by latitude/azimuth. d is `hypot(x, y)` of the fix
     in `map` (horizontal tangent-plane distance, before the sensor offset; z ignored).
     Check: 3570 m -> 1.0 m, consistent with #48 (2 m at 5 km, 8 m at 10 km).
   - `bool exceedsFlatPlaneBound(double x, double y, double max_err)`.
   - `validateMaxHeightError(double)`: finite and > 0, else throw.
2. **Parameters, read once at construction**, validated at the top of the constructor body
   **before** the broadcaster, odometry publisher and services are created:
   - `map_origin_latitude`, `map_origin_longitude`: declared `PARAMETER_DOUBLE` with no
     default, so "unset" is distinguishable from any value. Implementer verifies on jazzy
     that `get_parameter` returns false for an uninitialised declared parameter. 
     [Revised after pre-push review: declared dynamically typed; INTEGER or DOUBLE is
     accepted and converted (also for `map_maximum_height_error`), any other type is a
     startup failure. The README no longer says to write `43.0`.]
   - `map_maximum_height_error` (m, default `0.5`, ~2.5 km from the anchor): finite and
     > 0, else startup failure. Named for the repo's spelled-out `maximum_` convention; it
     applies to any anchor, first-fix included, hence no `map_origin_` prefix.
   - **Runtime sets are rejected**: an `add_on_set_parameters_callback` returns
     unsuccessful ("restart required") for any set of these three names, so a
     `ros2 param set` can never silently do nothing. Tested.
3. **Startup failure is a clean exit**: `nodes/mru_transform_node.cpp` `main` wraps node
   and `MRUTransform` construction in try/catch; on exception it logs
   `RCLCPP_FATAL` with the reason, calls `rclcpp::shutdown()` and returns 1 (not
   `std::terminate`). The bizzyboat `core_launch.py` starts the node without `respawn`, so
   launch reports the exit; README says a bad origin config stops the node at startup.
4. **Anchoring** in `updatePosition`: member `std::optional<GeoPoint> configured_origin_`
   (consumed state). When creating `MapFrame`: use `*configured_origin_` if set, else the
   first fix (altitude 0, as today). Log INFO once saying which, with lat/lon.
   `mapOrigin() const` returns the active anchor as `std::optional<GeoPoint>`; it is the
   **primary test seam**; the anchor state is guarded by `state_mu_` so it is safe under
   a multi-threaded executor.
5. **Reset**, two cases:
   - Map already anchored: `mapFrame_.reset()` and `configured_origin_.reset()`; next fix
     re-anchors at the current fix. Message says the configured map origin (if there was
     one) no longer applies until the node restarts.
   - Not yet anchored (no fix seen): nothing to re-anchor. A configured origin is
     **kept**; the message says so and that the first fix will use it. With no origin
     configured the message says the map is not anchored yet and will anchor at the first
     fix.
6. **Far-from-origin WARN** in `updatePosition` after `toLocal`: if
   `exceedsFlatPlaneBound(x, y, max_err)` -> `RCLCPP_WARN_THROTTLE(..., 10000, ...)` with
   d, estimated error, threshold and the remedy (configure `map_origin_latitude/longitude`
   near the work area, or restart near it; see #48). Applies to first-fix anchors too.
   Nothing dropped or refused. The 0.5 m default is an owner decision (2026-10-12). Evidence
   for it: the 2026 field failures in #48 / unh_echoboats_project11#473 were at 10-12 km
   (8-11 m); the estimator's rejection limit is tidal range x `tide_range_margin`
   (default 2.0), so real harm is site dependent. A survey more than 2.5 km from its anchor
   will see one WARN per 10 s for the whole run.
7. **Tests** (new `test/test_map_origin.cpp`, registered in `CMakeLists.txt` with the
   LOCALHOST / dedicated-domain environment of `test_lifecycle_reconfigure`):
   - Pure: unset -> nullopt; valid pair -> altitude exactly 0; one only -> throws; NaN /
     inf / lat 91 / lon -181 -> throws; boundaries 90, -180 accepted; height-error value
     and threshold boundary; max-error validation.
   - Node-level, constructing `MRUTransform` on a node with parameter overrides, calling
     public `updatePosition` and asserting through `mapOrigin()`:
     (a) configured origin used; the first fix is tens of metres+ away from it (> tolerance)
     and the origin altitude is exactly 0; (b) unset -> anchor equals the first fix
     (altitude 0); (c) partial / invalid / a non-numeric typed param -> constructor throws; integers accepted;
     bad threshold -> throws; (d) reset after anchoring with an origin configured ->
     next fix re-anchors at that fix, message says the origin no longer applies; (e) reset
     BEFORE any fix with an origin configured -> origin kept, first fix uses it, message
     says so; (f) a runtime set of each read-once parameter is rejected, a set of an
     unrelated parameter is not; (g) far-WARN wiring: capture the log with
     `rcutils_logging_set_output_handler`, use a **non-default** `map_maximum_height_error`
     so a fix between the default and the custom distance warns (or not) only if the right
     parameter is read; deleting the WARN fails the test; also first-fix anchor + far fix.
   - Each key test is checked to fail with the targeted code reverted.
   - Implementation note: `RCLCPP_WARN_THROTTLE` keeps a static per-call-site timer, so only
     one far-WARN can be observed per test process per 10 s. Tests not about the WARN get a
     large `map_maximum_height_error` from the fixture; the WARN test orders its negative
     case (default would warn, custom threshold must not) before the positive one (default
     would not warn, custom 0.05 m must). The configured-origin path is covered by the
     negative case; the positive case uses a first-fix anchor.
8. **Docs** in the same PR: README `## Parameters` gets the three parameters (default,
   units, validation, read-once and runtime-set rejection, the altitude-0 convention stated
   as mru_transform's own, not a REP), a "Map origin" paragraph under Frames replacing
   "centered on the first position fix", `reset_map_frame` behaviour (both cases), startup
   failure, the far WARN and its relation to #48. The repo has **no `.agents/README.md`**
   (only `.agents/ci_local_upstream_extra.sh`), so there is no verified-parameter table to
   edit; creating the guide is a separate task, noted as a gap.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Fail loudly / robustness (Quality Standard) | Partial, non-finite, out-of-range config and a bad threshold throw at startup; no silent fallback to first-fix when a config was attempted |
| Scope discipline | No map-frame redesign; #48 untouched and not closed; altitude not configurable |
| Test what you change | Every owner decision has a named test |
| Docs from source | README written after reading final code; no REP attribution for altitude 0 |
| Surface consequential defaults | Default threshold 0.5 m is an owner decision (2026-10-12), evidence recorded in Approach 6 |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| None triggered | No | Altitude stays 0, so `chart_datum_node` and `sea_surface_estimator` inputs are unchanged (both consume only the TF `map` frame, not `updatePosition` state). ADR-0010 lives in unh_marine_autonomy and is not affected |
| Workspace ADRs (parameters/lifecycle) | No | Plain-node params read once (runtime sets rejected); lifecycle nodes untouched |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Parameters of mru_transform | README table; `.agents/README.md` table (absent in this repo) | README Yes; `.agents` table: gap noted, No — separate task |
| `reset_map_frame` message text | Any caller parsing it | Closed: no caller of `reset_map_frame` exists in `layers/main` outside this repo (grep, Plan Review) |
| Map origin altitude | `map_tide`, `chart_datum_node`, `sea_surface_estimator` | Unchanged (always 0), Yes |
| Demo/sim/platform configs | Set origin per site | No — follow-ups below |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): README "A `map` frame is created centered on the
  first position fix", Parameters section, and the reset_map_frame description.
- **Agent-instruction candidates** (proposals only): this repo lacks `.agents/README.md`;
  proposing one (with the verified-parameter table) as its own issue.

## Follow-ups in other repos (out of scope here; no other repo is changed)

Nodes started via `mru_transform_node` in `layers/main`, to set an origin per site
(launch-inline dict values must be Python floats, e.g. `43.0`):

- `bizzyboat_project11`: `launch/core_launch.py`, `launch/bizzyboat_sim_core_launch.py`,
  `config/bizzyboat.yaml` (`/**/mru_transform:` block).
- `ben_project11`: `launch/ben_core_launch.py`, `launch/sim_local_cw4.launch`.
- `echoboat_project11`: `launch/echo_launch.py`; `echo_helm`: `launch/echo_helm_launch.py`.
- `molab_hardware`: `launch/mobile_lab_launch.py`; `lr30_project11`: `launch/backseat_launch.py`.
- `vrx_project11`: `launch/wamv_launch.py`.
- `unh_marine_simulation`: `marine_simulation/launch/demo_boats_launch.py:451` (Ocean Discovery
  Day demo, #88): ONE shared origin for all four boats so their namespaced
  `boat.frame('map')` frames coincide.
- cube_bathymetry#164 coverage scorer could drop its per-boat `earth` workaround once
  origins are shared.
- No caller of `reset_map_frame` exists in `layers/main` outside this repo.

## Open Questions

- Resolved by owner (2026-10-12): default `map_maximum_height_error` 0.5 m (~2.5 km).
- Should the INFO log at anchoring also be a topic/latched message so downstream knows the origin? (Proposed: no, out of scope; log only.)

## Estimated Scope

Single PR, one repo (rolker/mru_transform), roughly 150 lines of code plus ~200 lines of tests and docs.

## Revision after pre-push review (#49)

- Integer YAML values are accepted for the three numeric parameters (dynamic typing,
  converted to double); other types are refused with `std::invalid_argument`.
- The far WARN no longer uses `RCLCPP_WARN_THROTTLE` (a per-call-site static). The
  decision lives in `FarFromOriginWarning` (map_origin.hpp, instance state, injected
  steady-clock time) behind `MRUTransform::shouldWarnFarFromOrigin`, which the tests
  drive directly; so the earlier "negative case first" ordering constraint on the tests
  no longer applies. The repeating text is short, has no URL, and names the
  map-frame-shift consequence of a restart or `reset_map_frame`.
- The 10 s throttle and the default threshold (0.5 m) are asserted in tests.
