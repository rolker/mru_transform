#ifndef MRU_TRANSFORM_MRU_TRANSFORM_H
#define MRU_TRANSFORM_MRU_TRANSFORM_H

#include <mru_transform/sensor.h>
#include <mru_transform/map_frame.h>
#include <tf2_ros/transform_broadcaster.h>

namespace mru_transform
{

class MRUTransform
{
public:
  MRUTransform(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  void update();
private:
  // list of sensors, in order of priority
  std::vector<Sensor::Ptr> sensors_;

  sensor_msgs::NavSatFix::ConstPtr last_sent_position_;
  sensor_msgs::Imu::ConstPtr last_sent_orientation_;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr last_sent_velocity_;

  // replublish selected nav messages
  ros::Publisher position_pub_;
  ros::Publisher orientation_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher active_sensor_pub_;

  ros::Duration sensor_timeout_ = ros::Duration(1.0);

  ros::Duration publish_period_ = ros::Duration(0.1);
  ros::Time last_publish_time_;

  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string odom_topic_ = "odom";

  std::shared_ptr<MapFrame> mapFrame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  ros::Publisher odom_pub_;
};

} // namespace mru_transform

#endif
