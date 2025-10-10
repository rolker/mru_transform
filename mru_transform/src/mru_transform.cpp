#include "mru_transform/mru_transform.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp> // Include the Trigger service header

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace p11 = project11;

namespace mru_transform{

MRUTransform::MRUTransform(rclcpp::Node::SharedPtr node)
: sensors_(node, true),
  node_(node)
{
  node_->declare_parameter("map_frame", map_frame_);
  node_->get_parameter("map_frame", map_frame_);

  node_->declare_parameter("base_frame", base_frame_);
  node_->get_parameter("base_frame", base_frame_);
  node_->declare_parameter("odom_frame", odom_frame_);
  node_->get_parameter("odom_frame", odom_frame_);

  node_->declare_parameter("odom_topic", odom_topic_);
  node_->get_parameter("odom_topic", odom_topic_);


  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);


  // Initialize the reset map frame service
  reset_map_frame_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "reset_map_frame",
      std::bind(&MRUTransform::resetMapFrameService, this, std::placeholders::_1, std::placeholders::_2));

  sensors_.registerPositionCallback(std::bind(&MRUTransform::updatePosition, this, std::placeholders::_1));
  sensors_.registerOrientationCallback(std::bind(&MRUTransform::updateOrientation, this, std::placeholders::_1));
  sensors_.registerVelocityCallback(std::bind(&MRUTransform::updateVelocity, this, std::placeholders::_1));
}

void MRUTransform::updatePosition(const PositionSensor::ValueType &position)
{
  p11::LatLongDegrees p;
  p11::fromMsg(position.position, p);
  if (std::isnan(p[2]))
    p[2] = 0.0;

  if(!mapFrame_)
  {
    auto map_origin = p;
    map_origin.altitude() = 0.0;
    mapFrame_ = std::shared_ptr<MapFrame>(new MapFrame(node_, map_origin, map_frame_, odom_frame_));
  }
  auto transforms = mapFrame_->getTransforms(position.header.stamp);
  p11::Point position_map = mapFrame_->toLocal(p);

  geometry_msgs::msg::TransformStamped map_to_north_up_base_link;
  map_to_north_up_base_link.header.stamp = position.header.stamp;
  map_to_north_up_base_link.header.frame_id = map_frame_;
  map_to_north_up_base_link.child_frame_id = base_frame_+"_north_up";
  p11::toMsg(position_map, map_to_north_up_base_link.transform.translation);
  map_to_north_up_base_link.transform.rotation.w = 1.0;
  transforms.push_back(map_to_north_up_base_link);
  broadcaster_->sendTransform(transforms);
  p11::toMsg(position_map, odom_.pose.pose.position);
}

void MRUTransform::updateOrientation(const OrientationSensor::ValueType &orientation)
{
  tf2::Quaternion orientation_quat;

  tf2::fromMsg(orientation.orientation, orientation_quat);

  double roll,pitch,yaw;
  tf2::getEulerYPR(orientation_quat, yaw, pitch, roll);

  geometry_msgs::msg::TransformStamped north_up_base_link_to_level_base_link;
  north_up_base_link_to_level_base_link.header.stamp = orientation.header.stamp;
  north_up_base_link_to_level_base_link.header.frame_id = base_frame_+"_north_up";
  north_up_base_link_to_level_base_link.child_frame_id = base_frame_+"_level";
  tf2::Quaternion heading_quat;
  heading_quat.setRPY(0.0,0.0,yaw);
  north_up_base_link_to_level_base_link.transform.rotation = tf2::toMsg(heading_quat);

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(north_up_base_link_to_level_base_link);
  
  geometry_msgs::msg::TransformStamped north_up_base_link_to_base_link;
  north_up_base_link_to_base_link.header.stamp = orientation.header.stamp;
  north_up_base_link_to_base_link.header.frame_id = base_frame_+"_north_up";
  north_up_base_link_to_base_link.child_frame_id = base_frame_;
  north_up_base_link_to_base_link.transform.rotation = orientation.orientation;
  // if we have an uninitialized quat, lets set it to identity
  if(orientation.orientation.x == 0.0 && orientation.orientation.y == 0.0 && orientation.orientation.z == 0 && orientation.orientation.w == 0.0)
    north_up_base_link_to_base_link.transform.rotation.w = 1.0;
  transforms.push_back(north_up_base_link_to_base_link);
  broadcaster_->sendTransform(transforms);

  odom_.pose.pose.orientation = orientation.orientation;
  odom_.twist.twist.angular = orientation.angular_velocity;
}

void MRUTransform::updateVelocity(const VelocitySensor::ValueType &velocity)
{
  odom_.header.frame_id = odom_frame_;
  odom_.header.stamp = velocity.header.stamp;
  odom_.child_frame_id = base_frame_;

  odom_.twist.twist.linear = velocity.twist.linear;
  odom_pub_->publish(odom_);
}

void MRUTransform::resetMapFrameService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                        std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // Reset the map frame
  mapFrame_.reset();
  response->success = true;
  response->message = "Map frame has been reset";
}

} // namespace mru_transform
