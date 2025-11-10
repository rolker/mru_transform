#ifndef MRU_TRANSFORM_MRU_TRANSFORM_H
#define MRU_TRANSFORM_MRU_TRANSFORM_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "mru_transform/navigation_sensors.hpp"
#include "mru_transform/map_frame.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "std_srvs/srv/trigger.hpp" // Include the Trigger service header

namespace mru_transform
{

class MRUTransform
{
public:
  MRUTransform(rclcpp::Node::SharedPtr node_ptr);
  void updatePosition(PositionSensor::ValueType position);
  void updateOrientation(const OrientationSensor::ValueType &orientation);
  void updateVelocity(const VelocitySensor::ValueType &velocity);

private:
  void resetMapFrameService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                            std_srvs::srv::Trigger::Response::SharedPtr response);

  NavigationSensors sensors_;

  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string odom_topic_ = "odom";

  std::shared_ptr<MapFrame> mapFrame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_frame_service_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

} // namespace mru_transform

#endif
