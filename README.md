# MRU Transform

This ROS node listens to Motion Reference Unit topics and publishes transforms to `/tf`. Odometry is also published.

Multiple sensors can be configured in a priority list and the node can fallback to secondary sensors if the primary one stops providing data.

This node is designed for integrated sensors that combine a GPS with an IMU to provide complete position and attitude solutions. **It expects input topics to be synchronized.** In other words, they have the exact same timestamp.

# Sensor topics

Each sensor is expected to publish on three topics:

- position: `sensor_msgs/NavSatFix`
- orientation: `sensor_msgs/Imu`
- velocity: `geometry_msgs/PoseWithCovarianceStamped`

Only the linear component of the velocity message is used. It describes velocity of the sensor in an ENU frame. The frame_id should reflect the sensor's frame.

# Frames

A `map` frame is created centered on the first position fix. An `earth` frame is also created and a transformation from ECEF to ENU used by the `map` frame is published. A null transformation from `map` to `odom` is also published. Finally, a transformation from `odom` to `base_link` is published with each sensor update.

Additional frames are published which may be useful in special cases, such as a sensor that publishes data that is already corrected for pitch and roll. The `base_link_north_up` represents a frame at the `base_link` location without any orientation applied. A level frame with only heading applied is also published as `base_link_level`.

# Parameters

## ~sensors

A list of sensors which will be used in order. Each sensor entry has a name and a map of topics.

    mru_transform/sensors:
    - name: posmv
      topics: { position: 'sensors/posmv/position', orientation: 'sensors/posmv/orientation', velocity: 'sensors/posmv/velocity'}
    - name: gps
      topics: { position: 'sensors/gps/position', orientation: 'sensors/heading', velocity:
    'sensors/gps/velocity'}
    
If no sensors are found, a default sensor will be created with topics `position`, `orientation` and `velocity`.
    
## ~base_frame, ~map_frame and ~odom_frame

Override the frame_id's which default to `base_link`, `map` and `odom`. 

## ~odom_topic

The topic used to publish odometry messages. Defaults to `odom`.

