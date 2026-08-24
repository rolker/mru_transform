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

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `geoid_grid` | `""` | PROJ geoid grid (`.tif`) for ellipsoid → NAVD88. Needed for VDatum. |
| `vdatum_grid_dir` | `""` | Directory of VDatum `*_mllw.gtx` / `*_mhhw.gtx` grids. Empty disables VDatum (non-fatal). |
| `datum_config_path` | `""` | Path to a polygon→datum YAML (see `config/datum_polygons.example.yaml`). Empty = none. A malformed file fails `on_configure`. |
| `lake_datum` | NaN (unset) | Fixed `chart_datum` height (m, rel. ellipsoid) that overrides VDatum/config everywhere. For quick one-offs/testing. |
| `lake_datum_mhhw` | NaN (unset) | Optional fixed MHHW height to accompany `lake_datum`. |
| `recalc_interval` | `60.0` | Seconds between position-based datum recomputes. Must be > 0; `on_configure` fails otherwise. |
| `publish_rate` | `1.0` | Hz at which cached transforms are republished. Must be > 0; `on_configure` fails otherwise. |

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
| `minimum_buffer_duration` | `5.0` | Seconds of odometry required before anything is published. Must be **strictly less than** `maximum_buffer_duration` or `on_configure` fails (see below). A negative value is clamped to 0 — note that `ros2 param get` still reports the number you set, not the clamped 0 the node is using. |
| `maximum_buffer_duration` | `30.0` | Seconds of odometry kept in the averaging window. A negative value fails `on_configure` (see below). |
| `chart_datum_frame` | `chart_datum` | MLLW frame used for the plausibility bound. Empty disables the bound. |
| `mhhw_frame` | `chart_datum_mhhw` | MHHW frame used for the plausibility bound. Empty disables the bound. |
| `tide_range_margin` | `2.0` | Multiplier on the MLLW→MHHW range allowed beyond each end (storm surge, extreme tides). Negative values are clamped to 0. |

The plausibility bound needs **both** datum frames; if either is missing (or
either parameter is empty) it is disabled and every estimate is accepted.

Odometry samples with a non-finite `position.z`, and non-finite attitudes, are
rejected rather than averaged — a NaN would pass straight through the bound
(every comparison against NaN is false) and latch on `tide_estimate`.

### Buffer durations that fail `on_configure`

`on_configure` refuses two kinds of averaging window, for two different reasons:

- **A negative `maximum_buffer_duration`** is a correctness bug. The retention
  prune drops samples older than `now - maximum_buffer_duration`, so a negative
  maximum puts that cutoff *after* `now` and erases each sample as it arrives.
- **`minimum_buffer_duration >= maximum_buffer_duration`** is refused as
  **policy**. The `(0, 0)` boundary is the case worth being clear about: it used
  to *work* — the prune keeps the just-arrived sample, so the node published the
  instantaneous height with no smoothing at all — and it is refused anyway,
  because this node exists to average and an unsmoothed single-sample tide feeds
  every sounding. Any other pair with the minimum at or above the maximum
  describes a window that can never be long enough, so the node would sit there
  silently never publishing.

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
