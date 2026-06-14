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

We add a resolution chain — VDatum → polygon config → `lake_datum` param — that
populates `chart_datum` from whichever real datum is available, tracking and
logging the source. When **nothing** matches we leave `chart_datum` **absent**
(per the resolved design below) and navigation continues on `map_tide`.

### Resolved design decisions (issue #25 discussion, 2026-06-13)

- **Q1 — config format: yaml-cpp file.** A standalone YAML file (`datum_config_path`
  param) parsed with yaml-cpp; natural for variable-length polygon rings. One new,
  well-established ROS 2 dependency.
- **Q2 — "nothing matches": `chart_datum` absent + loud WARN** (NOT published at z=0).
  This is the #8-designed "optional/additive" state for `chart_datum`. It honors
  acceptance item 4's *intent* ("never hard-fail on datum") while reinterpreting its
  *mechanism*: the safe default is "no chart_datum, nav continues on the ellipsoidal
  `map`/`map_tide`," not "publish a chart_datum at the ellipsoid." Publishing z=0 would
  assert a false `chart_datum == ellipsoid` equality and risk confidently-wrong
  clearance in any consumer that ignored provenance.
- **Q3 — no consumer change in scope.** Verified in code that both `chart_datum`
  consumers already tolerate absence: `sea_surface_estimator` (this repo,
  `is_out_of_range` try/catch → "don't filter") and `s57_layer` (`s57_tools`,
  try/catch → keeps `tide_offset_=0`, raw chart depth). Absence off-VDatum is the
  pre-existing status quo; this PR only *adds* real datums, never a wrong one.

## Approach

1. **Extract a pure, ROS-free resolution core** — new header
   `include/mru_transform/datum_config.hpp` (+ `src/datum_config.cpp`):
   `DatumEntry {name, ring<lat,lon>, chart_datum_z, optional mhhw_z}`,
   `point_in_ring()` (ray-cast), `load_datum_config(path)` (yaml-cpp parse),
   `resolve(lat, lon, entries) -> optional<DatumEntry>`. Pure logic so it unit-tests
   without rclcpp. Define a `DatumSource` enum {VDATUM, POLYGON_CONFIG, PARAM, NONE}.
2. **Make VDatum optional in `on_configure`** — if `vdatum_grid_dir` is empty or no
   MLLW grids found, skip PROJ setup and log once (INFO), do **not** FAILURE. Keep
   `geoid_grid` required only when VDatum is used. PROJ pointers stay null → guarded.
3. **Add parameters** — `datum_config_path` (string, ""), `datum_config_override`
   (bool, false → config is a VDatum *fallback*; true → consult config *first*),
   `lake_datum` (double, NaN sentinel = unset; fixed datum-below-ellipsoid z),
   `lake_datum_mhhw` (double, NaN = unset). Declare/get in `on_configure`; load the
   config file there if the path is set (yaml-cpp).
4. **Rewrite `recalc_callback` as the resolution chain** producing
   `(chart_datum_z_, optional mhhw_z_, DatumSource, name)`:
   override-first config (if flag) → VDatum → fallback config → `lake_datum` param →
   **NONE**. Set `has_valid_mllw_ = true` only when a real datum is found
   (VDATUM/POLYGON_CONFIG/PARAM); on **NONE** set `has_valid_mllw_ = false` so
   `publish_callback` naturally leaves `chart_datum` absent. Log the resolved source +
   name with `RCLCPP_INFO` on change (source can change as the boat moves), and a
   prominent `RCLCPP_WARN` (throttled) when the result is NONE.
5. **Publish provenance** — add a latched `std_msgs::msg::String` publisher
   `datum_source` (`"vdatum"`, `"polygon:<name>"`, `"param"`, `"none"`) so
   consumers/operators can see which datum is active and distinguish "no datum here"
   from a surveyed one. Publish in `publish_callback` (including `"none"` when absent).
6. **Ship a generic example config, not Massabesic data** —
   `config/datum_polygons.example.yaml` with a documented schema and one *synthetic*
   entry. The real Lake Massabesic polygon goes in the platform/site overlay
   (echoboats/bizzy) as a follow-up, passed via `datum_config_path`. Satisfies
   acceptance item 6 in the right repo.
7. **Tests** — `test/test_datum_config.cpp` (gtest): point-in-ring (inside, outside,
   on-boundary, ring that wraps longitude), config parse (valid/missing-file/malformed),
   and `resolve()` precedence (override-first vs fallback; param override; NONE when
   nothing matches). Register in `CMakeLists.txt`.
8. **Docs** — update `chart_datum_node.cpp` header comment, `launch/chart_datum_launch.py`
   (new params), and `README.md` for the config schema, override flag, `lake_datum`,
   and the absent-when-nothing-matches behavior.

## Files to Change

| File | Change |
|------|--------|
| `include/mru_transform/datum_config.hpp` | New — pure types + `point_in_ring`/`load_datum_config`/`resolve` decls + `DatumSource` enum |
| `src/datum_config.cpp` | New — implementations (yaml-cpp parse, ray-cast) |
| `nodes/chart_datum_node.cpp` | VDatum optional; new params; resolution-chain `recalc`; `datum_source` publisher; provenance logging; absent on NONE |
| `config/datum_polygons.example.yaml` | New — documented schema + synthetic entry |
| `launch/chart_datum_launch.py` | Surface new params |
| `test/test_datum_config.cpp` | New — gtest for ring/parse/resolve |
| `mru_transform/CMakeLists.txt` | Build `datum_config` lib, link yaml-cpp, register test |
| `mru_transform/package.xml` | Add `yaml-cpp` (or `yaml_cpp_vendor`) depend |
| `mru_transform/README.md` | Document config/params/absent-default behavior |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Both consumers verified in code (try/catch tolerance); absent-default is the pre-existing state, so no consumer change is forced. |
| Human control & transparency | `datum_source` topic + per-change logging make provenance visible (incl. `"none"`); override-vs-fallback is an explicit configurable flag. |
| Test what breaks | Pure core is unit-tested across all resolution paths + polygon edge cases. |
| Workspace vs. project separation | Generic mechanism + synthetic example here; deployment-specific Massabesic polygon deferred to the platform overlay. |
| Only what's needed | Single polygon representation (lat/lon ring; bbox = degenerate ring); ray-cast hand-rolled, no geometry dep. yaml-cpp is the one new dep, justified by the file-based schema the issue specifies. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | Params declared with defaults; lifecycle pattern preserved; YAML config is conventional; target Rolling-compatible APIs. |
| 0001 — ADRs | No (repo has no ADR log) | Capture the absent-default + override-ordering rationale in node header + README + PR body. |

## Consequences

| If we change... | Also update... | Status |
|---|---|---|
| Always-resolve datum chain | `sea_surface_estimator` tide-plausibility bound (same repo) | Verified — try/catch tolerates absence; no change |
| Datum absent off-VDatum | `s57_layer` depth→clearance (`s57_tools`, cross-repo) | Verified — try/catch keeps `tide_offset_=0`; no change |
| `chart_datum` semantics | Frame model in #8 (vertical-datum hierarchy) | Aligned — #8 defines `chart_datum` as optional/additive |
| New params + config schema | `chart_datum_launch.py`, README | In plan (steps 3, 8) |
| Real Massabesic polygon | Platform/site overlay (echoboats) | Follow-up issue in platform repo |

## Open Questions

- All resolved — see "Resolved design decisions" above (Q1 yaml-cpp, Q2 absent-default,
  Q3 no consumer change). Acceptance item 4's mechanism was reinterpreted with the
  issue author's sign-off; worth a one-line note on the issue so the record reflects it.

## Estimated Scope

Single PR for the mechanism + tests + example config in `mru_transform`. The real
Massabesic polygon is a separate follow-up in the platform repo; no `s57_layer`
change is needed.
