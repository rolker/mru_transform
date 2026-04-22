#include "mru_transform/mru_transform.hpp"
#include "mru_transform/twist_rotation_utils.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp> // Include the Trigger service header

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mru_transform{

MRUTransform::MRUTransform(rclcpp::Node::SharedPtr node)
: sensors_(*node, true),
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

  // listener used to determine sensor to baselink transforms
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);


  // Initialize the reset map frame service
  reset_map_frame_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "reset_map_frame",
      std::bind(&MRUTransform::resetMapFrameService, this, std::placeholders::_1, std::placeholders::_2));

  sensors_.registerPositionCallback(std::bind(&MRUTransform::updatePosition, this, std::placeholders::_1));
  sensors_.registerOrientationCallback(std::bind(&MRUTransform::updateOrientation, this, std::placeholders::_1));
  sensors_.registerVelocityCallback(std::bind(&MRUTransform::updateVelocity, this, std::placeholders::_1));
}

void MRUTransform::updatePosition(PositionSensor::ValueType position)
{
  if(std::isnan(position.position.altitude))
    position.position.altitude = 0.0;

  if(!mapFrame_)
  {
    auto map_origin = position.position;
    map_origin.altitude = 0.0;
    mapFrame_ = std::shared_ptr<MapFrame>(new MapFrame(node_, map_origin, map_frame_, odom_frame_));
  }
  auto transforms = mapFrame_->getTransforms(position.header.stamp);
  auto position_map = mapFrame_->toLocal(position.position);

  // account for sensor offset from base_link
  tf2::Vector3 sensor_offset(0.0, 0.0, 0.0);

  if(position.header.frame_id != base_frame_ && position.header.frame_id != "")
  {
    // assuming static sensor to base_link transform, so any time is ok
    if(tf_buffer_->canTransform(base_frame_, position.header.frame_id, tf2::TimePointZero))
    {
      auto transform_stamped = tf_buffer_->lookupTransform(base_frame_, position.header.frame_id, tf2::TimePointZero);
      tf2::fromMsg(transform_stamped.transform.translation, sensor_offset);

      // to properly orient the sensor offset, we need to rotate it according to the base_link orientation relative to the map frame
      if(tf_buffer_->canTransform(map_frame_, base_frame_, tf2::TimePointZero))
      {
        auto base_link_in_map = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        tf2::Quaternion base_link_orientation;
        tf2::fromMsg(base_link_in_map.transform.rotation, base_link_orientation);

        sensor_offset = tf2::quatRotate(base_link_orientation, sensor_offset);
      }
    }
  }

  geometry_msgs::msg::TransformStamped map_to_north_up_base_link;
  map_to_north_up_base_link.header.stamp = position.header.stamp;
  map_to_north_up_base_link.header.frame_id = map_frame_;
  map_to_north_up_base_link.child_frame_id = base_frame_+"_north_up";
  map_to_north_up_base_link.transform.translation.x = position_map.x-sensor_offset.x();
  map_to_north_up_base_link.transform.translation.y = position_map.y-sensor_offset.y();
  map_to_north_up_base_link.transform.translation.z = position_map.z-sensor_offset.z();
  transforms.push_back(map_to_north_up_base_link);
  broadcaster_->sendTransform(transforms);
  odom_.pose.pose.position = position_map;
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

  // REP-105: angular velocity must be expressed in child_frame_id (body).
  // IMU-sourced gyro is in orientation.header.frame_id (typically the IMU's
  // physical frame).  Rotate into body frame using the static sensor-to-base
  // transform.  No-op when frame_id already matches base_frame_.
  //
  // Failure-handling note: unlike updateVelocity (which drops the sample on
  // TF lookup failure), this function still needs to run to broadcast the
  // pose TFs above.  So on TF failure we publish zero angular velocity +
  // zero angular covariance.  That is the honest representation when we
  // cannot express the gyro in body frame: "no angular info available"
  // rather than "angular info in an unknown frame".  The WARN_THROTTLE
  // tells the operator to set up the missing static TF.
  const bool same_frame =
    orientation.header.frame_id.empty() ||
    orientation.header.frame_id == base_frame_;

  if (!same_frame) {
    try {
      auto tf = tf_buffer_->lookupTransform(
        base_frame_, orientation.header.frame_id, tf2::TimePointZero);
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      tf2::Matrix3x3 R(q);

      tf2::Vector3 w_in(orientation.angular_velocity.x,
                        orientation.angular_velocity.y,
                        orientation.angular_velocity.z);
      tf2::Vector3 w_out = R * w_in;
      odom_.twist.twist.angular.x = w_out.x();
      odom_.twist.twist.angular.y = w_out.y();
      odom_.twist.twist.angular.z = w_out.z();

      // Rotate angular-covariance sub-block.  The source (sensor_msgs/Imu)
      // stores angular_velocity_covariance as a flat 9-array; promote into a
      // 36-array with the angular block populated, rotate, then copy back.
      // (Only the 3x3 angular-diagonal sub-block is rotated — see
      //  twist_rotation_utils.hpp for the helper's scope.)
      std::array<double, 36> cov_in_6x6{};
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          cov_in_6x6[(i + 3) * 6 + (j + 3)] =
            orientation.angular_velocity_covariance[i * 3 + j];
        }
      }
      rotate_covariance_block_3x3(cov_in_6x6, R, 3, odom_.twist.covariance);
      return;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "orientation: TF '%s' -> '%s' lookup failed: %s (publishing zero angular — set up a static TF to eliminate this warning)",
        orientation.header.frame_id.c_str(), base_frame_.c_str(), e.what());
      odom_.twist.twist.angular = geometry_msgs::msg::Vector3();
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          odom_.twist.covariance[(i + 3) * 6 + (j + 3)] = 0.0;
        }
      }
      return;
    }
  }

  // Same-frame path (frame_id is base_frame_ or empty) — copy angular directly.
  odom_.twist.twist.angular = orientation.angular_velocity;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom_.twist.covariance[(i + 3) * 6 + (j + 3)] =
        orientation.angular_velocity_covariance[i * 3 + j];
    }
  }
}

void MRUTransform::updateVelocity(const VelocitySensor::ValueType &velocity)
{
  odom_.header.frame_id = odom_frame_;
  odom_.header.stamp = velocity.header.stamp;
  odom_.child_frame_id = base_frame_;

  // REP-105: odom.twist must be expressed in child_frame_id (body).  Incoming
  // velocity is in velocity.header.frame_id — typically a world frame (map,
  // posmv_frame, mru_frame).  Rotate linear velocity + covariance into body
  // before publishing.  No-op when frame_id already matches base_frame_.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      base_frame_, velocity.header.frame_id,
      rclcpp::Time(velocity.header.stamp),
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "velocity: TF '%s' -> '%s' lookup failed: %s (dropping sample)",
      velocity.header.frame_id.c_str(), base_frame_.c_str(), e.what());
    return;
  }

  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation, q);
  tf2::Matrix3x3 R(q);

  // Rotate linear velocity
  tf2::Vector3 v_in(velocity.twist.twist.linear.x,
                    velocity.twist.twist.linear.y,
                    velocity.twist.twist.linear.z);
  tf2::Vector3 v_out = R * v_in;
  odom_.twist.twist.linear.x = v_out.x();
  odom_.twist.twist.linear.y = v_out.y();
  odom_.twist.twist.linear.z = v_out.z();

  // Rotate linear-covariance sub-block (upper-left 3x3).  Cross-covariance
  // blocks between linear and angular are not touched — see the scope note
  // in twist_rotation_utils.hpp.
  rotate_covariance_block_3x3(velocity.twist.covariance, R, 0,
                              odom_.twist.covariance);

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
