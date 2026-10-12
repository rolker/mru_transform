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

1. **Pure helper header `include/mru_transform/map_origin.hpp`** (header-only, no ROS
   node needed, so it is unit-testable without DDS):
   - `std::optional<geographic_msgs::msg::GeoPoint> validateMapOrigin(std::optional<double> lat, std::optional<double> lon)`:
     both unset -> `nullopt`; exactly one set -> `throw std::invalid_argument` naming the
     missing one; non-finite, lat outside [-90, 90] or lon outside [-180, 180] -> throw
     with the offending value in the message. Returned point has altitude 0.
   - `double flatPlaneHeightError(double horizontal_distance_m)` = d²/(2R), R = 6371008.8 m
     (WGS84 mean radius; documented as the bound the #48 analysis uses, verified against #48
     text during implementation).
2. **Parameters, read once at construction** in `MRUTransform::MRUTransform`
   (no set-parameter callback; runtime changes have no effect, stated in README):
   - `map_origin_latitude`, `map_origin_longitude`: declared as `PARAMETER_DOUBLE` with
     no default (`declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE)`), so
     "unset" is distinguishable from any value (a NaN sentinel would silently accept an
     explicit `.nan`). Implementer verifies `get_parameter` returns false for an
     uninitialised declared parameter on the target distro (jazzy) and adds a test if not.
     An integer-typed value (`43`) is a type-mismatch exception -> startup failure; README
     says write `43.0`.
   - `map_origin_max_height_error` (m, default `0.5`, ~2.5 km from the origin): must be
     finite and > 0, else throw at startup (no "0 disables" silent-off value).
   - A validation failure throws from the constructor; `mru_transform_node` is left
     unhandled-exception-terminating (non-zero exit, message on stderr and via
     `RCLCPP_FATAL` before the throw) so a launch/supervisor sees the failure.
3. **Anchoring** in `updatePosition`: new member `std::optional<GeoPoint> configured_origin_`
   (consumed state). When creating `MapFrame`: use `*configured_origin_` if set, else the
   first fix (altitude 0, as today). Log INFO once saying which and the lat/lon used.
4. **Reset**: `resetMapFrameService` does `mapFrame_.reset()` and
   `configured_origin_.reset()`. Response message (when an origin had been configured and
   still active): "Map frame reset; it will be re-anchored at the next position fix. The
   configured map origin (map_origin_latitude/longitude) no longer applies until the node
   restarts." Without a configured origin: existing text plus the same re-anchor note.
5. **Far-origin WARN** in `updatePosition` after `toLocal`: `d = hypot(x, y)`;
   if `flatPlaneHeightError(d) > threshold` -> `RCLCPP_WARN_THROTTLE(..., 10000, ...)`
   with d, estimated error, threshold, and a pointer to #48. Applies whether the origin
   was configured or first-fix. Nothing is dropped or refused. The check is factored into a
   small `bool exceedsFlatPlaneBound(double x, double y, double max_err)` in `map_origin.hpp`
   so the threshold is testable without a clock/log capture.
6. **Tests** (new `test/test_map_origin.cpp`, registered in `CMakeLists.txt` like
   `test_nav_source_selection`; the node-level cases need rclcpp and the library):
   - Pure: both unset -> nullopt; valid pair -> point with altitude 0; only lat / only lon
     -> throws; NaN / inf / lat 91 / lon -181 -> throws; boundary values 90, -180 accepted.
   - Height-error: `flatPlaneHeightError(3570 m)` ~ 1.0 m within tolerance;
     `exceedsFlatPlaneBound` false just inside and true just outside a 0.5 m threshold.
   - Node-level (construct `MRUTransform` in a namespace with the localhost-only,
     dedicated-domain isolation pattern from `test_lifecycle_reconfigure`; call the public
     `updatePosition` directly and read the published `earth -> map` TF):
     (a) configured origin used instead of the first fix (TF translation equals ECEF of the
     configured point at altitude 0, not of the fix); (b) unset -> TF equals the first fix;
     (c) partial/invalid params -> constructor throws; non-positive/NaN threshold -> throws;
     (d) after `reset_map_frame` with a configured origin, the next fix re-anchors at that
     fix and the response message mentions the origin no longer applies; a second reset
     stays on first-fix behaviour; (e) fix far from the configured origin still publishes
     TFs (nothing refused).
     If reading TF from a unit test proves flaky, fall back to a narrow
     `MRUTransform::mapOrigin() const` accessor (mutex-free, returns the active anchor) used
     only by the test.
7. **Docs** in the same PR: README `## Parameters` gets the three parameters (table with
   default, units, validation, read-once, altitude-0 convention stated as mru_transform's
   own), a "Map origin" paragraph under Frames updating "centered on the first position
   fix", and the `reset_map_frame` behaviour; the far-origin WARN and its relation to #48.
   `.agents/README.md`: the repo has **no `.agents/README.md`** (only
   `.agents/ci_local_upstream_extra.sh`), so there is no verified-parameter table to edit;
   the README table is the sole parameter doc. Creating one is a separate task (workspace
   rule: own issue), noted as a gap.

## Files to Change

| File | Change |
|------|--------|
| `mru_transform/include/mru_transform/map_origin.hpp` | New: `validateMapOrigin`, `flatPlaneHeightError`, `exceedsFlatPlaneBound` |
| `mru_transform/include/mru_transform/mru_transform.hpp` | Members: `configured_origin_`, `max_height_error_` |
| `mru_transform/src/mru_transform.cpp` | Declare/validate params, anchor choice, reset clears origin + new message, throttled WARN |
| `mru_transform/test/test_map_origin.cpp` | New tests (above) |
| `mru_transform/CMakeLists.txt` | Register `test_map_origin` (isolation env as in lifecycle test) |
| `README.md` | Parameters, Frames text, reset semantics, WARN |
| `.agent/work-plans/issue-49/plan.md` | Keep in sync during implementation |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Fail loudly / robustness (Quality Standard) | Partial, non-finite, out-of-range config and a bad threshold throw at startup; no silent fallback to first-fix when a config was attempted |
| Scope discipline | No map-frame redesign; #48 untouched and not closed; altitude not configurable |
| Test what you change | Every owner decision has a named test |
| Docs from source | README written after reading final code; no REP attribution for altitude 0 |
| Surface consequential defaults | Default threshold 0.5 m is a decision, flagged below |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0010 (vertical datum split, D6) in this repo | No | Altitude stays 0 so `map` height semantics for `chart_datum_node` unchanged |
| Workspace ADRs (parameters/lifecycle) | No | Plain-node params read once; lifecycle nodes untouched |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Parameters of mru_transform | README table; `.agents/README.md` table (absent in this repo) | README Yes; `.agents` table: gap noted, No — separate task |
| `reset_map_frame` message text | Any caller parsing it (none found in this repo; others not checked) | Yes — follow-up check in other repos |
| Map origin altitude | `map_tide`, `chart_datum_node`, `sea_surface_estimator` | Unchanged (always 0), Yes |
| Demo/sim/platform configs | Set origin per site | No — follow-ups below |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): README "A `map` frame is created centered on the
  first position fix", Parameters section, and the reset_map_frame description.
- **Agent-instruction candidates** (proposals only): this repo lacks `.agents/README.md`;
  proposing one (with the verified-parameter table) as its own issue.

## Follow-ups in other repos (out of scope here)

- `unh_marine_simulation` Ocean Discovery Day launch (#88): set one shared
  `map_origin_latitude/longitude` for the four boats so their `<boat>/map` frames coincide.
- Platform configs that set mru_transform params: `platforms_ws/src/ben_project11/config/ben.yaml`,
  `ben_sim.yaml`; `seafloor_echoboat_project11/echoboat_project11/config/echo.yaml`;
  `mobile_lab/molab_hardware/config/molab.yaml`; `site_ws/src/ccomjhc_project11` launches.
- Check callers of `reset_map_frame` for dependence on the response text.
- cube_bathymetry#164 coverage scorer could drop its per-boat `earth` workaround once origins are shared.

## Open Questions

- Default for `map_origin_max_height_error`: proposed 0.5 m (~2.5 km); owner to confirm or choose another.
- Should the INFO log at anchoring also be a topic/latched message so downstream knows the origin? (Proposed: no, out of scope; log only.)

## Estimated Scope

Single PR, one repo (rolker/mru_transform), roughly 150 lines of code plus ~200 lines of tests and docs.
