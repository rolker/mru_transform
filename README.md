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
| `recalc_interval` | `60.0` | Seconds between position-based datum recomputes. |
| `publish_rate` | `1.0` | Hz at which cached transforms are republished. |

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

## Credits

Originally Developed by: Roland Arsenault,  University of New Hampshire [Center for Coastal and Ocean Mapping](https://github.com/CCOMJHC)

ROS2 migration: Dr. Kristopher Kransosky, [Seaward Science](https://github.com/SeawardScience)
