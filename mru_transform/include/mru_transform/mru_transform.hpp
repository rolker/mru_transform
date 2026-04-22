#ifndef MRU_TRANSFORM_MRU_TRANSFORM_H
#define MRU_TRANSFORM_MRU_TRANSFORM_H

#include <array>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
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

  // State contributed by each sensor callback, guarded by state_mu_.
  // updateVelocity composes a fresh nav_msgs/Odometry from this snapshot
  // and publishes.  Design goal: callbacks can safely run concurrently
  // under a MultiThreadedExecutor (no shared nav_msgs::msg::Odometry
  // member being mutated from three threads).
  std::mutex state_mu_;
  bool have_position_{false};
  bool have_orientation_{false};
  geometry_msgs::msg::Point latest_position_map_;     // set by updatePosition
  geometry_msgs::msg::Quaternion latest_orientation_; // set by updateOrientation
  geometry_msgs::msg::Vector3 latest_angular_body_;   // set by updateOrientation (rotated)
  std::array<double, 9> latest_angular_cov_body_{};   // row-major 3x3; set by updateOrientation

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_frame_service_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace mru_transform

#endif
