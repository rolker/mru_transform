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

We add a resolution chain — VDatum → polygon config → `lake_datum` param →
ellipsoid default — so a datum is always resolvable, with the source tracked and
logged. The review comment (issue #25) flagged: keep deployment-specific polygon
data out of this generic package, tag every datum with its provenance, and verify
downstream consumers tolerate an always-on frame.

## Approach

1. **Extract a pure, ROS-free resolution core** — new header
   `include/mru_transform/datum_config.hpp` (+ `src/datum_config.cpp`):
   `DatumEntry {name, ring<lat,lon>, chart_datum_z, optional mhhw_z}`,
   `point_in_ring()` (ray-cast), `load_datum_config(path)` (yaml-cpp parse),
   `resolve(lat, lon, entries) -> optional<DatumEntry>`. Pure logic so it unit-tests
   without rclcpp. Define a `DatumSource` enum {VDATUM, POLYGON_CONFIG, PARAM,
   ELLIPSOID_DEFAULT}.
2. **Make VDatum optional in `on_configure`** — if `vdatum_grid_dir` is empty or no
   MLLW grids found, skip PROJ setup and log once (INFO), do **not** FAILURE. Keep
   `geoid_grid` required only when VDatum is used. PROJ pointers stay null → guarded.
3. **Add parameters** — `datum_config_path` (string, ""), `datum_config_override`
   (bool, false → config is a VDatum *fallback*; true → consult config *first*),
   `lake_datum` (double, NaN sentinel = unset; fixed datum-below-ellipsoid z),
   `lake_datum_mhhw` (double, NaN = unset). Declare/get in `on_configure`; load the
   config file there if the path is set.
4. **Rewrite `recalc_callback` as the resolution chain** producing
   `(chart_datum_z_, optional mhhw_z_, DatumSource, name)`:
   override-first config (if flag) → VDatum → fallback config → `lake_datum` param →
   ellipsoid default (z=0). Always ends with a valid datum. Set `has_valid_mllw_` =
   true in all branches; keep `has_valid_mhhw_` only when a source supplies MHHW.
   Log the resolved source + name with `RCLCPP_INFO` on change (not just _ONCE, since
   the source can change as the boat moves), and `RCLCPP_WARN` once when falling to
   the ellipsoid default.
5. **Publish provenance** — add a latched `std_msgs::msg::String` publisher
   `datum_source` (values like `"vdatum"`, `"polygon:Massabesic"`, `"param"`,
   `"ellipsoid_default"`) so consumers/operators can distinguish a surveyed datum
   from the safe fallback. Publish in `publish_callback` alongside the TF.
6. **Ship a generic example config, not Massabesic data** —
   `config/datum_polygons.example.yaml` with a documented schema and one *synthetic*
   entry. The real Lake Massabesic polygon goes in the platform/site overlay
   (echoboats/bizzy) as a follow-up, passed via `datum_config_path`. Satisfies
   acceptance item 6 in the right repo.
7. **Tests** — `test/test_datum_config.cpp` (gtest): point-in-ring (inside, outside,
   on-boundary, ring that wraps longitude), config parse (valid/missing-file/malformed),
   and `resolve()` precedence (override-first vs fallback; param override; ellipsoid
   default when nothing matches). Register in `CMakeLists.txt`.
8. **Docs** — update `chart_datum_node.cpp` header comment, `launch/chart_datum_launch.py`
   (new params), and `README.md` for the config schema, override flag, `lake_datum`,
   and ellipsoid-default behavior.

## Files to Change

| File | Change |
|------|--------|
| `include/mru_transform/datum_config.hpp` | New — pure types + `point_in_ring`/`load_datum_config`/`resolve` decls + `DatumSource` enum |
| `src/datum_config.cpp` | New — implementations (yaml-cpp parse, ray-cast) |
| `nodes/chart_datum_node.cpp` | VDatum optional; new params; resolution-chain `recalc`; `datum_source` publisher; provenance logging |
| `config/datum_polygons.example.yaml` | New — documented schema + synthetic entry |
| `launch/chart_datum_launch.py` | Surface new params |
| `test/test_datum_config.cpp` | New — gtest for ring/parse/resolve |
| `mru_transform/CMakeLists.txt` | Build `datum_config` lib, link yaml-cpp, register test |
| `mru_transform/package.xml` | Add `yaml-cpp` (or `yaml_cpp_vendor`) depend |
| `mru_transform/README.md` | Document config/params/fallback behavior |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Plan checks `sea_surface_estimator` (same repo) tide-plausibility use of the datum; flags S57 `chart_layer` (other repo) as a verify-before-merge item (step 9 / Consequences). |
| Human control & transparency | `datum_source` topic + per-change logging make provenance visible; override-vs-fallback is an explicit configurable flag. |
| Test what breaks | Pure core is unit-tested across all four resolution paths + polygon edge cases. |
| Workspace vs. project separation | Generic mechanism + synthetic example here; deployment-specific Massabesic polygon deferred to the platform overlay. |
| Only what's needed | Single polygon representation (lat/lon ring; bbox expressible as a 4-point ring — no separate bbox type); ray-cast hand-rolled, no geometry dep. yaml-cpp is the one new dep, justified by the file-based schema the issue specifies. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | Params declared with defaults; lifecycle pattern preserved; YAML config is conventional; target Rolling-compatible APIs. |
| 0001 — ADRs | No (repo has no ADR log) | Capture "ellipsoid-default is safe" + override-ordering rationale in node header + README + PR body. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Always-on `chart_datum` frame | `sea_surface_estimator` tide-plausibility bound (same repo) | Yes — verify in step 4/9 |
| Datum semantics off-VDatum | S57 `chart_layer` depth→clearance (other repo) | No — verify-before-merge item; may need follow-up issue |
| New TF frame behavior | Frame naming/parenting in #8 (vertical-datum hierarchy) | Open question — coordinate |
| New params + config schema | `chart_datum_launch.py`, README | Yes |
| Real Massabesic polygon | Platform/site overlay (echoboats) | No — follow-up issue in platform repo |

## Open Questions

- **Config format**: yaml-cpp file (assumed, per issue's "config file (path param)")
  vs. ROS nested params. Plan assumes yaml-cpp — confirm acceptable to add the dep.
- **Frame naming**: should the ellipsoid-default frame keep the name `chart_datum`,
  or get a distinct name so consumers can refuse it? Needs alignment with #8.
- **`chart_layer` behavior** on an ellipsoid-referenced datum: tolerate, or skip
  clearance conversion? Confirms whether a cross-repo follow-up is needed.

## Estimated Scope

Single PR for the mechanism + tests + example config in `mru_transform`. The real
Massabesic polygon and any `chart_layer` change are separate follow-ups in their
owning repos.
