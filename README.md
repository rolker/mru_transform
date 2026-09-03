# MRU Transform

This ROS node listens to Motion Reference Unit topics and publishes transforms to `/tf`. Odometry is also published.

Multiple sensors can be configured in a priority list and the node can fallback to secondary sensors if the primary one stops providing data.

This node is designed for integrated sensors that combine a GPS with an IMU to provide complete position and attitude solutions.

## Sensor topics

Each sensor is expected to publish on three topics:

- position: `sensor_msgs/msg/NavSatFix`
- orientation: `sensor_msgs/msg/Imu`
- velocity: `geometry_msgs/msg/PoseWithCovarianceStamped`

Only the linear component of the velocity message is used. It describes velocity of the sensor in an ENU frame. The frame_id should reflect the sensor's frame.

## Frames

A `map` frame is created centered on the first position fix. An `earth` frame is also created and a transformation from ECEF to ENU used by the `map` frame is published. A null transformation from `map` to `odom` is also published. Finally, a transformation from `odom` to `base_link` is published with each sensor update.

Additional frames are published which may be useful in special cases, such as a sensor that publishes data that is already corrected for pitch and roll. The `base_link_north_up` represents a frame at the `base_link` location without any orientation applied. A level frame with only heading applied is also published as `base_link_level`.

## Parameters

### ~sensors

A list of sensors which will be used in order. Each sensor entry has a name and a map of topics.

```yaml
/**:
  ros__parameters:
    # you must "forward declare" the sensors so they can declare 
    # their sub params
    sensor_names:
    - posmv
    - gps
    # then configure the actual sensors
    sensors:
      posmv:
        topics:  
          position: 'sensors/posmv/position'
          orientation: 'sensors/posmv/orientation'
          velocity: 'sensors/posmv/velocity'
      gps:
        topics: 
          position: 'sensors/gps/position'
          orientation: 'sensors/heading' 
          velocity: 'sensors/gps/velocity'
```
    
If no sensors are found, a default sensor will be created with topics `position`, `orientation` and `velocity`.
    
### ~base_frame, ~map_frame and ~odom_frame

Override the frame_id's which default to `base_link`, `map` and `odom`. 

### ~odom_topic

The topic used to publish odometry messages. Defaults to `odom`.

## chart_datum_node

`chart_datum_node` publishes the `map → chart_datum` (MLLW) and
`map → chart_datum_mhhw` TF transforms — the static vertical offset between the
WGS84 ellipsoid (`map`) and the chart datum at the boat's current position. It
resolves the datum from several sources so the boat works on and off NOAA VDatum
coverage (e.g. inland lakes).

### Resolution order

At each recompute the datum is resolved by this precedence (first match wins;
within a config pass, the first matching entry in file order wins):

1. **`lake_datum` param** (if set) — wins outright everywhere.
2. **config entries with `override: true`** whose polygon contains the boat.
3. **VDatum** (PROJ + NOAA grids) where it has coverage.
4. **config entries with `override: false`** (the default) whose polygon
   contains the boat — fills VDatum gaps.
5. **Nothing matches** — `chart_datum` is **not published**; navigation
   continues on the ellipsoidal `map` / `map_tide`, and a warning is logged.
   (`chart_datum` is intentionally optional, per the marine TF frame design.)

The active source is logged on change and published on the latched
`datum_source` topic (`std_msgs/String`: `vdatum`, `polygon:<name>`, `param`, or
`none`) so consumers and operators can tell a surveyed datum from `none`.

**Where the implementation lives.** Neither the precedence chain nor the
VDatum/PROJ query is in this package. Both are
[`marine_vertical_datum`](https://github.com/rolker/unh_marine_autonomy/tree/jazzy/marine_vertical_datum)
(core_ws, ROS-free), shared with the chart importers and CAMP so every consumer
resolves a datum identically — [ADR-0010](https://github.com/rolker/unh_marine_autonomy/blob/jazzy/docs/decisions/0010-geospatial-world-model.md)
D6. `chart_datum_node` is the ROS wrapper over it: parameters, TF, lifecycle.
The polygon config format is documented by the example below, but it is parsed
upstream — `load_datum_config` lives in the library.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `geoid_grid` | `""` | PROJ geoid grid (`.tif`) for ellipsoid → NAVD88. Needed for VDatum. `chart_datum_launch.py` overrides this to `~/data/world/datum/geoid/us_noaa_g2018u0.tif` (see [VDatum grids](#vdatum-grids)). |
| `vdatum_grid_dir` | `""` | Directory of VDatum `*_mllw.gtx` / `*_mhhw.gtx` grids. Empty disables VDatum (non-fatal). `chart_datum_launch.py` overrides this to `~/data/world/datum/vdatum` (see [VDatum grids](#vdatum-grids)). |
| `datum_config_path` | `""` | Path to a polygon→datum YAML (see `config/datum_polygons.example.yaml`). Empty = none. A malformed file fails `on_configure`. |
| `lake_datum` | NaN (unset) | Fixed `chart_datum` height (m, rel. ellipsoid) that overrides VDatum/config everywhere. For quick one-offs/testing. |
| `lake_datum_mhhw` | NaN (unset) | Optional fixed MHHW height to accompany `lake_datum`. |
| `recalc_interval` | `60.0` | Seconds between position-based datum recomputes. Must be finite and > 0; `on_configure` fails otherwise. |
| `publish_rate` | `1.0` | Hz at which cached transforms are republished. Must be finite and > 0; `on_configure` fails otherwise. An infinite rate would arm a zero-period timer. |

### VDatum grids

**This package does not ship, download, or install datum grids.** Under
[ADR-0010](https://github.com/rolker/unh_marine_autonomy/blob/jazzy/docs/decisions/0010-geospatial-world-model.md)
D5/D6 the grids live wherever imports run — dev machines and the boat as
offline tooling — and never in the navigation runtime. The canonical on-host
location is the world tree
([unh_marine_autonomy#288](https://github.com/rolker/unh_marine_autonomy/issues/288)):

```
~/data/world/datum/geoid/us_noaa_g2018u0.tif   ellipsoid → NAVD88
~/data/world/datum/vdatum/                     NAVD88 → MLLW/MHHW (*.gtx)
```

Provisioning is `enc_updater`'s datum provisioner
([s57_tools#37](https://github.com/rolker/s57_tools/issues/37)), which fetches
the SHA-256-pinned geoid and the configured VDatum bundles. Configure
`vdatum_bundles` in the platform's `enc_updater` config (`MENHMAgome23_8301`
for the Gulf of Maine) and run the updater on the host.

`chart_datum_launch.py` defaults `geoid_grid` and `vdatum_grid_dir` to the
world-tree paths above. Those defaults are `os.path.expanduser`-ed, so they
resolve against the **launching process's** `HOME` — a unit or container that
runs the stack as a different user than the one the provisioner populated
resolves somewhere else. Pass absolute paths from the platform launch when the
runtime user is not the provisioning user.

Only the bundles listed in the platform's `enc_updater` config are fetched.
The retired build-time download extracted every US region, so this narrows the
operating envelope: deploying outside a configured bundle's coverage needs a
config edit and a provisioner run **before** travel (the Lewes DE work needed
`DEdelbay33_8301` + `DEVAemb23_8301`, for instance). Verify before deploying:

```
ls ~/data/world/datum/geoid/us_noaa_g2018u0.tif ~/data/world/datum/vdatum/*_mllw.gtx
```

### What absent grids actually cost

VDatum setup failure is **non-fatal to the node** — it logs, disables VDatum,
and still reaches `inactive`. It is **not** harmless to the output, and the
severity depends entirely on what else the deployment configures:

| Deployment | Effect of absent grids |
|---|---|
| A polygon covers the boat (`datum_config_path`), or `lake_datum` is set | Datum still resolves from that entry. Bizzy inside the Massabesic ring is this case. |
| Configured, but the boat is outside every polygon | **No `chart_datum` / `chart_datum_mhhw` TF is published** (resolution order step 5). Bizzy at the Isles of Shoals is this case — there VDatum *is* the datum source. |
| No `datum_config_path` and no `lake_datum` — the launch file's own defaults | **No datum is ever published.** VDatum was the only source. |

So "non-fatal" means the node stays up, not that the datum survives. The
observable signal is the latched `datum_source` topic reading `none`, plus one
`ERROR` (the failed grid scan) and one `WARN` at `on_configure`. Note that a
*missing* `vdatum_grid_dir` logs at `ERROR`, whereas deliberately setting
`vdatum_grid_dir: ""` to disable VDatum logs at `INFO` — prefer the empty
string on hosts that genuinely do not need grids, so a real provisioning gap
stays visible as an error.

Hosts upgrading past the build-time download can reclaim the orphaned copies,
which nothing reads any more and `colcon` never prunes:

```
rm -rf ~/.cache/mru_transform <colcon-install>/mru_transform/share/mru_transform/data
```

### Config file

See [`config/datum_polygons.example.yaml`](mru_transform/config/datum_polygons.example.yaml)
for the schema. Heights are signed metres relative to the WGS84 ellipsoid
(negative when the datum is below it), matching the published `map → chart_datum`
translation. **Deployment-specific polygons** (e.g. a real Lake Massabesic datum)
belong in the platform/site config repo, not this generic package — this package
ships only a synthetic example.

> An entry that sets `chart_datum_z` but omits `mhhw_z` publishes `chart_datum`
> without `chart_datum_mhhw`. Downstream, `sea_surface_estimator`'s tide
> plausibility bound needs both frames, so it stays disabled (no-op) for such an
> entry — an acceptable degradation, but worth knowing.

## sea_surface_estimator

`sea_surface_estimator` estimates the water level by averaging the vehicle's
height over a rolling window of odometry — over minutes, the boat's own motion
averages out and what is left is the tide. It publishes the result two ways:

- the **`map → map_tide`** TF transform (`sea_surface_frame`), the accepted
  estimate that other nodes reference soundings and chart layers against;
- the **`tide_estimate`** topic (`std_msgs/Float64`, latched/`transient_local`),
  the *raw* estimate, published for debugging even when the plausibility bound
  below rejects it.

> Those two can disagree. When an estimate falls outside the plausible tidal
> range, `tide_estimate` still carries it but `map_tide` is not broadcast — the
> topic is the raw number, the frame is the accepted one. **Consumers that need
> the tide the system stands behind must read the frame, not the topic.**

### The water line, not the vehicle frame

The averaged Z comes from `odom.pose.pose.position.z`, which is the height of
the **vehicle** frame (the odometry's `child_frame_id`, typically `base_link`) —
not of the water line. Published as-is it is low, or high, by whatever the
vertical offset between the two happens to be: 0.030 m on BizzyBoat, and
metre-scale on a platform whose origin sits at deck level.

Set **`water_line_frame`** to the URDF frame at the water line and the node
looks the vehicle→water-line lever arm up from TF and applies it. The lever arm
is *rotated by each sample's attitude* before its vertical component is taken,
so a large lever arm stays correct in a seaway.

Behaviour depends on the parameter:

| `water_line_frame` | Behaviour |
|---|---|
| unset (default) | No correction. `map_tide` is the **vehicle frame's** height, the node's pre-2026-08 behaviour. Logged as a warning at configure time so the omission is visible rather than assumed. |
| set and resolvable | The lever arm is applied to every sample. |
| set but **not** resolvable in TF | **Nothing is published** — no `map_tide`, no `tide_estimate` — and an error is logged on every throttle interval. A configured correction that cannot be applied is a misconfiguration, not a degraded mode: publishing (and latching) a tide known to be wrong by the whole lever arm would push that error into every sounding downstream. |

The frame is expected to be **static** (it comes from the URDF). One lookup is
cached and re-read every 10 s; if the lever arm moves, that is logged as a
warning rather than silently changing the tide.

Note that applying a metre-scale correction can move an estimate that used to
sit inside the plausible tidal range outside it — check `tide_range_margin` and
the `chart_datum` / `chart_datum_mhhw` heights if `map_tide` stops being
published after configuring this parameter.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `sea_surface_frame` | `map_tide` | Child frame of the broadcast sea-surface transform. |
| `water_line_frame` | `""` | URDF frame at the water line. Empty disables the correction (see above). |
| `minimum_buffer_duration` | `5.0` | Seconds of odometry required before anything is published. Must be finite and **strictly less than** `maximum_buffer_duration` or `on_configure` fails (see below). A negative value is clamped to 0 — note that `ros2 param get` still reports the number you set, not the clamped 0 the node is using, and that a clamped 0 means the first estimate after every configure is a single unsmoothed sample. |
| `maximum_buffer_duration` | `30.0` | Seconds of odometry kept in the averaging window. Must be finite; a negative or non-finite value fails `on_configure` (see below). |
| `chart_datum_frame` | `chart_datum` | MLLW frame used for the plausibility bound. Empty disables the bound. |
| `mhhw_frame` | `chart_datum_mhhw` | MHHW frame used for the plausibility bound. Empty disables the bound. |
| `tide_range_margin` | `2.0` | Multiplier on the MLLW→MHHW range allowed beyond each end (storm surge, extreme tides). Negative values are clamped to 0. |

The plausibility bound needs **both** datum frames; if either is missing (or
either parameter is empty) it is disabled and every estimate is accepted.

Odometry samples with a non-finite `position.z`, and non-finite attitudes, are
rejected rather than averaged — a NaN would pass straight through the bound
(every comparison against NaN is false) and latch on `tide_estimate`.

### Buffer durations that fail `on_configure`

`on_configure` refuses three kinds of averaging window, for three different
reasons:

- **A non-finite bound** (`.inf`, `-.inf`, `.nan`, on either parameter) is
  refused because neither is survivable and neither is visible. An infinite
  maximum overflows `rclcpp::Duration::from_seconds()`, and the node then dies
  with an uncaught `std::overflow_error` on its **first odometry message** — a
  boat that configures, activates, and then disappears. A NaN on either bound is
  worse-behaved still: every comparison against NaN is false, so a NaN minimum
  passes both checks below and silently disables the smoothing requirement
  entirely.
- **A negative `maximum_buffer_duration`** is a correctness bug. The retention
  prune drops samples older than `now - maximum_buffer_duration`, so a negative
  maximum puts that cutoff *after* `now` and erases each sample as it arrives.
- **`minimum_buffer_duration >= maximum_buffer_duration`** is refused as
  **policy**. The `(0, 0)` boundary is the case worth being clear about: it used
  to *work* — the prune keeps the just-arrived sample, so the node published the
  instantaneous height with no smoothing at all — and it is refused anyway,
  because this node exists to average and the tide feeds every sounding. Any
  other pair with the minimum at or above the maximum describes a window that
  can never be long enough, so the node would sit there silently never
  publishing.

  Note what this does **not** refuse: `minimum_buffer_duration = 0` with a
  positive maximum is accepted, and its first estimate after every configure is
  a single unsmoothed sample as well — latched on `tide_estimate`. The
  difference is that the window then fills and the node starts averaging, so
  that is a start-up transient; at `(0, 0)` the node is unsmoothed permanently.
  Set a minimum you actually want to average over rather than relying on the
  negative-value clamp above to land on 0.

Correct the parameter and `configure` again: the transition is retryable and the
parameters survive the failure, so the corrected value is what the retry reads.

**A failed configure is a quiet failure, not an interlock.** It leaves the
process **up** in `unconfigured`, publishing no `map_tide` and no
`tide_estimate`, having logged one `ERROR` line in the startup chatter.
`launch_ros`'s `LifecycleTransition` matches neither `inactive` nor
`errorprocessing` on a `FAILURE`, so nothing chains off it, nothing announces
that transitions have stopped, and `respawn` never fires. Nobody should read
"the node refuses to configure" as the boat being stopped — check
`ros2 lifecycle get <node>` when a node comes up silent. The same is true of
`chart_datum_node`'s `publish_rate` / `recalc_interval` validation and of a
malformed `datum_config_path`.

## Credits

Originally Developed by: Roland Arsenault,  University of New Hampshire [Center for Coastal and Ocean Mapping](https://github.com/CCOMJHC)

ROS2 migration: Dr. Kristopher Kransosky, [Seaward Science](https://github.com/SeawardScience)
