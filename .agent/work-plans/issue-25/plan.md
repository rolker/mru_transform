# Plan: Datum support off-VDatum: polygon-keyed datum config (always-on) + lake_datum override

## Issue

https://github.com/rolker/mru_transform/issues/25

## Context

`chart_datum_node` (a `LifecycleNode`) publishes `map → chart_datum`/`mhhw` TF
from NOAA VDatum `.gtx` grids via PROJ. Two gaps block "works anywhere":

1. `on_configure` **hard-fails** (`CallbackReturn::FAILURE`) when `vdatum_grid_dir`
   is empty — so the node won't even configure with no grids.
2. `recalc_callback` only sets `has_valid_mllw_` on a VDatum hit; `publish_callback`
   gates publishing on that flag. Off-VDatum (e.g. Massabesic, inside the grid dir
   but outside coverage) **no `chart_datum` frame is published at all**.

We add a datum **resolution chain** that populates `chart_datum` from whichever real
datum is available, tracking and logging the source. When **nothing** matches we
leave `chart_datum` **absent** and navigation continues on `map_tide`.

### Resolution order (resolved 2026-06-13/14)

1. **`lake_datum` param** — if set (NaN sentinel = unset), wins outright everywhere.
   Deliberate operator action for one-offs/testing; least-surprising if it overrides
   VDatum too. Source = `PARAM`.
2. **config override-entries** — polygon entries flagged `override: true` whose ring
   contains the point. Source = `POLYGON_CONFIG`.
3. **VDatum** — PROJ pipeline result where grids cover the point. Source = `VDATUM`.
4. **config fallback-entries** — polygon entries with `override: false` (the default)
   whose ring contains the point. Source = `POLYGON_CONFIG`.
5. **NONE** — `chart_datum` absent + loud WARN.

Within a pass, the **first** matching entry in file order wins (documented).

### Resolved design decisions (issue #25 discussion)

- **Q1 — config format: yaml-cpp file** (`datum_config_path` param); natural for
  variable-length polygon rings. One new, well-established ROS 2 dependency.
- **Q2 — "nothing matches": `chart_datum` absent + loud WARN** (NOT z=0). This is
  #8's "optional/additive" design for `chart_datum`. Honors acceptance item 4's
  *intent* ("never hard-fail") while reinterpreting its *mechanism*: the safe default
  is "no chart_datum, nav on the ellipsoidal `map`/`map_tide`," not a z=0 frame that
  would assert a false `chart_datum == ellipsoid` equality.
- **Q3 — no consumer change in scope.** Verified in code: `sea_surface_estimator`
  (this repo) and `s57_layer` (`s57_tools`) both try/catch the lookup and degrade
  safely. Absence off-VDatum is the pre-existing status quo; this PR only *adds* real
  datums, never a wrong one.
- **Q4 — config positioning: per-entry `override` flag (default false), two-pass
  around VDatum** — NOT a single global flag. Lets one polygon override VDatum (a
  trusted local survey) while another only fills gaps, without flipping the whole
  file into override mode. Safe default (forgotten flag = fallback).

## Approach

1. **Pure, ROS-free resolution core** — `include/mru_transform/datum_config.hpp`
   (+ `src/datum_config.cpp`). Types:
   - `DatumEntry { std::string name; std::vector<std::pair<double,double>> ring;
     double chart_datum_z; std::optional<double> mhhw_z; bool override_vdatum = false; }`
   - `VDatumResult { double mllw_z; std::optional<double> mhhw_z; }`
   - `DatumResult { DatumSource source; std::string name; double chart_datum_z;
     std::optional<double> mhhw_z; }`, `enum class DatumSource {VDATUM, POLYGON_CONFIG, PARAM}`
   Functions: `point_in_ring()` (ray-cast), `load_datum_config(path)` (yaml-cpp parse),
   and the **whole precedence chain** as one pure function:
   `std::optional<DatumResult> resolve_datum(lat, lon, lake_param, lake_mhhw_param,
   std::optional<VDatumResult> vdatum, const std::vector<DatumEntry>& entries)`
   returning `nullopt` for NONE. Encapsulating the chain here (not in the node) is
   what makes the precedence matrix unit-testable — addresses plan-review finding 1.
2. **Make VDatum optional in `on_configure`** — if `vdatum_grid_dir` is empty or no
   MLLW grids found, skip PROJ setup and log once (INFO), do **not** FAILURE. Keep
   `geoid_grid` required only when VDatum is used. PROJ pointers stay null → guarded.
3. **Add parameters** — `datum_config_path` (string, ""), `lake_datum` (double, NaN =
   unset; fixed datum-below-ellipsoid z), `lake_datum_mhhw` (double, NaN = unset).
   (No global override flag — per-entry now.) Declare/get in `on_configure`; load the
   config file there if the path is set (yaml-cpp); each entry parses its optional
   `override` field (default false).
4. **`recalc_callback` becomes thin** — query PROJ for an `optional<VDatumResult>`
   (nullopt when no coverage / VDatum disabled), read the params, call
   `resolve_datum(...)`. On a result: set `chart_datum_z_`, optional `mhhw_z_`, source,
   name; `has_valid_mllw_ = true`; `has_valid_mhhw_` only if the result carries MHHW.
   On `nullopt`: `has_valid_mllw_ = false` so `publish_callback` leaves `chart_datum`
   absent, plus a throttled `RCLCPP_WARN`. Log the source+name with `RCLCPP_INFO` on
   change (source can change as the boat moves).
5. **Publish provenance** — latched `std_msgs::msg::String` publisher `datum_source`
   (`"vdatum"`, `"polygon:<name>"`, `"param"`, `"none"`) so consumers/operators can
   distinguish "no datum here" from a surveyed one. Publish in `publish_callback`
   (including `"none"` when absent).
6. **Ship a generic example config, not Massabesic data** —
   `config/datum_polygons.example.yaml` with a documented schema (including the
   `override` field) and one *synthetic* entry. Real Lake Massabesic polygon goes in
   the platform/site overlay (echoboats/bizzy) as a follow-up, via `datum_config_path`.
   Satisfies acceptance item 6 in the right repo.
7. **Tests** — `test/test_datum_config.cpp` (gtest):
   - `point_in_ring`: inside, outside, on-boundary, overlapping-rings first-match,
     antimeridian limitation (documented, not handled).
   - `load_datum_config`: valid, missing-file, malformed, `override` default false.
   - `resolve_datum` precedence matrix: param-first-wins; override-entry beats VDatum;
     fallback-entry loses to VDatum; fallback-entry fills a VDatum gap; NONE when
     nothing matches; MHHW present vs absent. This covers acceptance item 5's matrix
     in pure code (the PROJ query itself is the only untested seam).
   Register in `CMakeLists.txt`.
8. **Docs** — `chart_datum_node.cpp` header, `launch/chart_datum_launch.py` (new
   params), `README.md`: config schema + `override` semantics, resolution order,
   `lake_datum`, absent-when-nothing-matches, and the note that a `chart_datum`-only
   datum (no MHHW) leaves `sea_surface_estimator`'s tide-plausibility bound disabled.

## Files to Change

| File | Change |
|------|--------|
| `include/mru_transform/datum_config.hpp` | New — `DatumEntry`/`VDatumResult`/`DatumResult`/`DatumSource` + `point_in_ring`/`load_datum_config`/`resolve_datum` decls |
| `src/datum_config.cpp` | New — implementations (yaml-cpp parse, ray-cast, full precedence chain) |
| `nodes/chart_datum_node.cpp` | VDatum optional; new params; thin `recalc` delegating to `resolve_datum`; `datum_source` publisher; absent on NONE |
| `config/datum_polygons.example.yaml` | New — documented schema (incl. `override`) + synthetic entry |
| `launch/chart_datum_launch.py` | Surface new params |
| `test/test_datum_config.cpp` | New — gtest for ring/parse/resolve precedence matrix |
| `mru_transform/CMakeLists.txt` | Build `datum_config` lib, link yaml-cpp, register test, `install(DIRECTORY config ...)` |
| `mru_transform/package.xml` | Add `yaml-cpp` (or `yaml_cpp_vendor`) depend |
| `mru_transform/README.md` | Document config/params/resolution order/absent-default |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Both consumers verified (try/catch tolerance); MHHW-absent interaction documented; absent-default is pre-existing so no consumer change forced. |
| Human control & transparency | `datum_source` topic + per-change logging make provenance visible (incl. `"none"`); per-entry `override` is explicit and defaults safe. |
| Test what breaks | Full precedence chain is pure → the whole matrix (param/override/fallback/VDatum/none/MHHW) is unit-tested. |
| Workspace vs. project separation | Generic mechanism + synthetic example here; Massabesic polygon deferred to the platform overlay. |
| Only what's needed | Single polygon representation; per-entry `override` is ~1 bool + a two-pass loop over the global-flag alternative, and it removes a footgun rather than adding speculative surface. yaml-cpp is the one new dep. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | Params declared with defaults; lifecycle pattern preserved; YAML config conventional; target Rolling-compatible APIs. |
| 0001 — ADRs | No (repo has no ADR log) | Capture absent-default + resolution-order rationale in node header + README + PR body. |

## Consequences

| If we change... | Also update... | Status |
|---|---|---|
| Always-resolve datum chain | `sea_surface_estimator` tide-plausibility bound (same repo) | Verified — try/catch tolerates absence; no change |
| Datum absent off-VDatum | `s57_layer` depth→clearance (`s57_tools`, cross-repo) | Verified — try/catch keeps `tide_offset_=0`; no change |
| chart_datum-only entry (no MHHW) | tide-plausibility bound silently disabled (mhhw lookup throws → no-filter) | Documented in step 8; acceptable degradation |
| `chart_datum` semantics | Frame model in #8 (vertical-datum hierarchy) | Aligned — #8 defines `chart_datum` as optional/additive |
| New params + config schema | `chart_datum_launch.py`, README | In plan (steps 3, 8) |
| Real Massabesic polygon | Platform/site overlay (echoboats) | Follow-up issue in platform repo |

## Open Questions

- All resolved — Q1 yaml-cpp; Q2 absent-default; Q3 no consumer change; Q4 per-entry
  `override` (default false) + `lake_datum` param first. Acceptance item 4's mechanism
  was reinterpreted with the issue author's sign-off; worth a one-line note on the
  issue so the record reflects it.

## Estimated Scope

Single PR for the mechanism + tests + example config in `mru_transform`. The real
Massabesic polygon is a separate follow-up in the platform repo; no `s57_layer`
change is needed.
