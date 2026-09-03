# Plan: chart_datum_node consumes marine_vertical_datum (ADR-0010 D6 follow-on)

**Issue**: [#41](https://github.com/rolker/mru_transform/issues/41)
**Branch**: `feature/issue-41`

## Goal

Complete the mru_transform half of [ADR-0010 D6](https://github.com/rolker/unh_marine_autonomy/blob/jazzy/docs/decisions/0010-geospatial-world-model.md).
The datum machinery was extracted into `marine_vertical_datum` (core_ws) by
unh_marine_autonomy#274, which listed migrating this package as an explicit
non-goal. This is that follow-on: delete the duplicate and consume the library.

## Findings that shape the approach

- **The two `datum_config` copies are still byte-identical** apart from a
  license header and one line wrap. This is a delete, not a merge — and it
  stops being one the moment either side is patched.
- **`datum_config` is an exported CMake target** (#28 exported it for a
  hypothetical cube_bathymetry consumer). Grepping every `*/src/` tree in the
  workspace, **nothing links it**: `marine_bathymetry_store` already uses the
  library, and cube_bathymetry never took it up. Removing the export breaks no
  in-tree consumer. It is still an API removal and belongs in the PR body.
- **`proj` and `yaml-cpp` become entirely unused** in this package afterwards:
  `proj.h` appears only in `chart_datum_node.hpp`, and `YAML::` only in
  `datum_config.cpp` and its test. Both deps can be dropped.
- The library's `VDatumQueryFn` is **RAII** — the PROJ context and pipelines
  live in the returned closure — which removes `cleanup_proj()` entirely and
  simplifies the #34 failed-configure release path.

## Approach

1. **package.xml** — add `marine_vertical_datum`; drop `proj` and `yaml-cpp`.
2. **CMakeLists.txt** — drop `pkg_check_modules(PROJ)`, `find_package(yaml-cpp)`
   and the `YAML_CPP_TARGET` shim; delete the `datum_config` target, its
   install and its export; `find_package(marine_vertical_datum REQUIRED)` and
   link it into the `chart_datum_node` executable and the lifecycle test;
   drop `yaml-cpp` from `ament_export_dependencies`.
3. **Delete** `include/mru_transform/datum_config.hpp` and `src/datum_config.cpp`.
4. **chart_datum_node.hpp** — replace `#include "proj.h"` +
   `mru_transform/datum_config.hpp` with the two library headers; delete
   `collect_grids`, `create_pipeline`, `setup_proj`, `cleanup_proj`,
   `query_datum` and the three `PJ*` members; hold a `VDatumQueryFn` instead.
5. **Delete** `test/test_datum_config.cpp` and its registration — the
   precedence chain is now the library's to test, and it already does
   (`test_datum_config` + `test_vdatum_query` there).

## Behaviour that must not change

Verified against the current node before editing; each gets an assertion or an
existing test:

- VDatum setup failure stays **non-fatal** — log, disable, still reach
  `inactive`. NOTE: the regression tests for this
  (`ChartDatumNodeConfiguresWhenGridPathsAreMissing` / `...GridDirIsEmpty`)
  were added under #10 and live in the **unmerged** PR#42, not on this branch.
  This branch is cut from `jazzy` and does not carry them, so it must assert
  the behaviour itself rather than lean on them. The two branches touch
  disjoint regions of `CMakeLists.txt` (#42 the deleted VDatum-download block,
  #41 the `datum_config` target and the PROJ/yaml deps) and disjoint files
  otherwise, so they are independent — whichever lands second may need a
  trivial merge, not a rework.
- A malformed `datum_config_path` stays a **loud** `CallbackReturn::FAILURE`
  that releases what the failed configure allocated (#34). With the closure
  being RAII this gets simpler, not more subtle — assert it still holds.
- The **per-point no-coverage warning** is preserved. The library deliberately
  returns `nullopt` with no diagnostic (a coverage gap is normal and the
  polygon chain handles it); the node's throttled
  `"No VDatum MLLW coverage at (%.4f, %.4f)"` must therefore be re-raised in
  the node wrapper, not lost to the library's silence.
- Setup diagnostics keep reaching the ROS log — route the library's `DiagFn`
  into `RCLCPP_WARN`.
- `datum_source` values (`vdatum` / `polygon:<name>` / `param` / `none`) are
  unchanged; they are part of the node's published contract.

## Out of scope

- The build-time grid download and launch defaults — #10 / PR#42.
- #43 (fail-open tide bound), #44 (`datum_source` has no consumer), #45
  (unchecked geoid path). Each is its own issue; folding them in here would
  mix a mechanical migration with behaviour changes.
- Removing the `chart_datum` TF frame (ADR-0010 D5/D10).

## Verification

- `./platforms_ws/build.sh mru_transform` clean; no `proj.h` include and no
  `datum_config` target remain in the package.
- `./platforms_ws/test.sh mru_transform` — the full suite, with no drop in
  count beyond the deliberately retired `test_datum_config` cases.
- `/review-code` before push.

## Implementation notes (kept in sync with the branch)

Landed as planned, with three things the plan did not anticipate:

1. **`chart_datum_node` needed an explicit `target_include_directories`.** It
   had been inheriting `include/` from `datum_config`'s PUBLIC interface, so
   deleting that target broke the build with a missing-header error rather
   than a link error. Every other executable in the file already states it.
2. **The per-point MHHW warning had to be dropped, not ported.** The library
   returns `mhhw_z = nullopt` both for "no MHHW grids loaded" and for "no MHHW
   coverage here", so the node can no longer distinguish them; warning on
   every nullopt would fire forever on any deployment without MHHW grids. The
   MLLW warning IS preserved. The library still reports the MHHW setup
   situation once through `DiagFn`.

   **Round-2 correction**: review rejected this, correctly. Dropping the
   warning removes the only evidence that #43's fail-open has triggered, while
   #43 is still open — `chart_datum_mhhw` stops publishing, `datum_source`
   stays `vdatum` so no source-change INFO fires, and
   `sea_surface_estimator`'s tide bound silently stops filtering. The
   "can't distinguish" argument was also defeatable without touching the
   library: a latched `seen_mhhw_` separates "never had MHHW grids" (never
   warn) from "had it, lost it here" (warn). Implemented; the warning is
   preserved after all.

4. **A second behaviour change, which the plan originally denied.** The
   library `std::sort`s the grid paths before joining them into `+grids=`;
   the deleted `collect_grids` used raw `recursive_directory_iterator` order.
   PROJ takes the first grid with coverage, so in an **overlap** region the
   answering grid — and the published `chart_datum` Z — can change. The
   library's behaviour is the better one (deterministic across machines), and
   it is inert on current deployments: `~/data/world/datum/vdatum` holds a
   single bundle (`MENHMAgome23_8301`), so nothing overlaps. It is still a
   silent numeric change and belongs in the PR body.

5. **Diagnostic severity had to be reconstructed.** `DiagFn` carries no
   severity, and routing it all to `RCLCPP_WARN` demoted three conditions the
   old `setup_proj()` logged at `ERROR`. `setup_vdatum` now buffers the
   messages and picks the level from the outcome — empty query means the fatal
   set (ERROR), live query means MHHW-only (WARN).

6. **`upstream.repos` added.** `find_package(marine_vertical_datum REQUIRED)`
   is a build stopper and the library is not a rosdep key, so a clean-container
   `ci_local.sh` run (the ADR-0018 merge gate) could not resolve it. Mirrors
   `cube_bathymetry/upstream.repos`, which already declares the same repo.
3. **Coverage was verified by name diff, not assumed.** All 21 retired
   `test_datum_config` cases exist in the library's suite under identical
   names (`comm -23` of the two sorted name lists is empty), plus 6 the old
   suite did not have (`MakeVDatumQuery` ×4, `VDatumQuerySeam` ×2). Package
   goes 93 → 71 tests; the library carries 27 datum cases and passes.
