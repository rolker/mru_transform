# Plan: VDatum service for chart datum to ellipsoidal height conversion

## Issue

https://github.com/rolker/mru_transform/issues/7

## Context

The `map → chart_datum` TF transform is needed so that S57 chart depths
(referenced to MLLW) can be correctly compared against the current water
level (`map_tide`) in the costmap. Without it, the trackline planner
can't account for tidal water levels.

The conversion from WGS84 ellipsoid to MLLW requires two steps:
1. Ellipsoid → NAVD88: PROJ geoid grid (`us_noaa_g2018u0.tif`, 16MB)
2. NAVD88 → MLLW: VDatum regional `.gtx` grids (~1.7GB for all US MLLW)

Both are static, position-dependent offsets that can be looked up offline.

## Approach

### 1. Grid download/cache script

`scripts/download_vdatum_grids.sh`:
- Downloads PROJ geoid grid via `projsync` if not cached
- Downloads VDatum regional zip from NOAA if not cached
- Extracts `*_mllw.gtx` + `*.bnd` + `*.met` files
- Cache location: `~/.cache/mru_transform/vdatum/` (outside ROS build)
- Idempotent — skips if cache is populated

### 2. C++ lifecycle node: `chart_datum_node.cpp`

Follows the `sea_surface_estimator` pattern (lifecycle node + TF broadcaster).

**Parameters:**
- `chart_datum_frame` (string, default: `"chart_datum"`)
- `map_frame` (string, default: `"map"`)
- `geoid_grid` (string, default: `""` — path to PROJ geoid .tif)
- `vdatum_grid_dir` (string, default: `""` — directory containing .gtx files)
- `update_distance` (double, default: 1000.0 — meters moved before re-querying)

**On configure:**
- Declare parameters
- Set up TF broadcaster and TF listener (to read robot position)
- Initialize PROJ context with geoid grid

**On activate:**
- Load VDatum `.gtx` grids from directory
- Start position subscription (odom topic) or TF polling timer

**Update logic (on position change > update_distance):**
1. Get current lat/lon from TF (`map → base_link`)
2. PROJ: ellipsoid height 0 → NAVD88 height (gives geoid undulation)
3. VDatum grid: look up NAVD88-to-MLLW offset at lat/lon
4. Compute total: `chart_datum_z = -(geoid_undulation) + navd88_to_mllw`
5. Publish `map → chart_datum` transform with Z = `chart_datum_z`

The offset is static for a given position, so we only re-query when the
robot moves significantly (default 1km).

### 3. VDatum grid reader

Utility class to load `.gtx` files from a directory and look up the
NAVD88-to-MLLW offset at a given lat/lon. Handles:
- Multiple regional grids (selects correct one based on position)
- 0-360 longitude convention in `.gtx` files
- NoData detection (-88.8888)

Can use GDAL (already a dependency via `python3-gdal`) or read the
binary `.gtx` format directly. GDAL is simpler since the grids are
standard rasters.

### 4. Launch file

`launch/chart_datum_launch.py` — lifecycle node with configure+activate
transitions, parameters for grid paths.

## Files to Change

| File | Change |
|------|--------|
| `nodes/chart_datum_node.cpp` | New: lifecycle node |
| `include/mru_transform/vdatum_grid.hpp` | New: VDatum grid reader |
| `src/vdatum_grid.cpp` | New: VDatum grid implementation |
| `launch/chart_datum_launch.py` | New: launch file |
| `scripts/download_vdatum_grids.sh` | New: grid download/cache script |
| `CMakeLists.txt` | Add PROJ dependency, new node, install script |
| `package.xml` | Add libproj-dev dependency |

## Open Questions

1. **GDAL vs raw `.gtx` reader**: GDAL is easier but adds a build
   dependency. The `.gtx` format is simple (header + float array), so
   a raw reader is also feasible. GDAL is already available at runtime
   via python3-gdal but may not be linked in C++. PROJ's C API can
   read `.gtx` files directly via `proj_create` with a vgridshift pipeline.

2. **Position source**: Subscribe to odom (like sea_surface_estimator)
   or use TF listener to look up base_link position in map frame?
   TF listener is more general but adds latency. Odom subscription
   matches the existing pattern.

## Estimated Scope

Single PR. ~300-400 lines of new C++ code + script + launch file.
