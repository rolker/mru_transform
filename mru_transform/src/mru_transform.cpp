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
  // Map origin and flat-plane bound, validated before any publisher or service
  // exists so a bad configuration fails startup with nothing half-built. The
  // origin parameters have a declared type and no default, so "unset" cannot
  // be confused with any value (an integer-typed value is a type mismatch and
  // also fails startup).
  {
    auto read_optional_double = [this](const std::string & name) -> std::optional<double> {
        node_->declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        rclcpp::Parameter parameter;
        if (node_->get_parameter(name, parameter) &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          return parameter.as_double();
        }
        return std::nullopt;
      };
    configured_origin_ = validateMapOrigin(
      read_optional_double("map_origin_latitude"), read_optional_double("map_origin_longitude"));

    node_->declare_parameter("map_maximum_height_error", max_height_error_);
    max_height_error_ = validateMaxHeightError(
      node_->get_parameter("map_maximum_height_error").as_double());

    // These are read once. Refuse a runtime set rather than let it succeed
    // and do nothing.
    set_parameters_handle_ = node_->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          const auto & name = parameter.get_name();
          if (name == "map_origin_latitude" || name == "map_origin_longitude" ||
            name == "map_maximum_height_error")
          {
            result.successful = false;
            result.reason = name + " is read once at startup; restart the node to change it";
            break;
          }
        }
        return result;
      });
  }

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

  std::shared_ptr<MapFrame> map_frame;
  {
    std::lock_guard<std::mutex> lock(state_mu_);
    if(!mapFrame_)
    {
      geographic_msgs::msg::GeoPoint map_origin;
      if(configured_origin_)
      {
        map_origin = *configured_origin_;
        RCLCPP_INFO(node_->get_logger(),
          "Anchoring map frame '%s' at the configured origin lat %.8f lon %.8f (altitude 0)",
          map_frame_.c_str(), map_origin.latitude, map_origin.longitude);
      }
      else
      {
        map_origin = position.position;
        map_origin.altitude = 0.0;
        RCLCPP_INFO(node_->get_logger(),
          "Anchoring map frame '%s' at the first position fix lat %.8f lon %.8f (altitude 0)",
          map_frame_.c_str(), map_origin.latitude, map_origin.longitude);
      }
      mapFrame_ = std::shared_ptr<MapFrame>(new MapFrame(node_, map_origin, map_frame_, odom_frame_));
      map_origin_ = map_origin;
    }
    map_frame = mapFrame_;
  }
  auto transforms = map_frame->getTransforms(position.header.stamp);
  auto position_map = map_frame->toLocal(position.position);

  // The map is a flat tangent plane (#48): its height error grows as d^2/2R.
  // Warn, never refuse -- the fix is still the best position we have.
  if(exceedsFlatPlaneBound(position_map.x, position_map.y, max_height_error_))
  {
    const double distance = std::hypot(position_map.x, position_map.y);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
      "Position is %.0f m from the '%s' frame origin; the flat-plane height error there is "
      "about %.2f m (limit map_maximum_height_error %.2f m). Configure map_origin_latitude/"
      "map_origin_longitude near the work area, or restart the node near it (see "
      "https://github.com/rolker/mru_transform/issues/48)",
      distance, map_frame_.c_str(), flatPlaneHeightError(distance), max_height_error_);
  }

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

  // Record latest position under state_mu_ for updateVelocity to compose
  // into the published Odometry.  Keeps the three callbacks free of a
  // shared nav_msgs::Odometry member and safe under a MultiThreadedExecutor.
  {
    std::lock_guard<std::mutex> lock(state_mu_);
    latest_position_map_ = position_map;
    have_position_ = true;
  }
}

void MRUTransform::updateOrientation(const OrientationSensor::ValueType &orientation)
{
  // Sanitize an all-zero (uninitialized) orientation quaternion to identity
  // once, up front, so the same value flows to both the TF broadcast and the
  // stored latest_orientation_ used in the published odom.  An all-zero quat
  // is invalid (not unit length) and would confuse downstream consumers of
  // odom.pose.pose.orientation.
  geometry_msgs::msg::Quaternion orientation_q = orientation.orientation;
  if (orientation_q.x == 0.0 && orientation_q.y == 0.0 &&
      orientation_q.z == 0.0 && orientation_q.w == 0.0) {
    orientation_q.w = 1.0;
  }

  tf2::Quaternion orientation_quat;
  tf2::fromMsg(orientation_q, orientation_quat);

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
  north_up_base_link_to_base_link.transform.rotation = orientation_q;
  transforms.push_back(north_up_base_link_to_base_link);
  broadcaster_->sendTransform(transforms);

  // Compute angular velocity + covariance in base_frame_ locally, then
  // commit to state under state_mu_.  Keeps this callback free of the
  // shared Odometry mutation the pre-refactor code had.
  //
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
  geometry_msgs::msg::Vector3 angular_body;
  std::array<double, 9> angular_cov_body{};

  // Empty frame_id is a publisher bug (not a signal that the data is already
  // in base_frame_).  Treat it like a TF failure — align with updateVelocity
  // which drops samples with empty/unresolvable frame_id.  Publishes zero
  // angular rather than copying sensor-frame values through as body-frame.
  const bool same_frame = orientation.header.frame_id == base_frame_;
  const bool empty_frame = orientation.header.frame_id.empty();

  bool have_angular = false;
  if (empty_frame) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "orientation: empty header.frame_id; cannot express angular velocity "
      "in '%s' (publishing zero angular — fix the IMU publisher's frame_id)",
      base_frame_.c_str());
    // angular_body + angular_cov_body stay zero-initialized.
    have_angular = true;  // state still "received" — zeros are the honest answer.
  } else if (!same_frame) {
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
      angular_body.x = w_out.x();
      angular_body.y = w_out.y();
      angular_body.z = w_out.z();

      // Rotate angular-covariance sub-block.  (Only the 3x3 angular-diagonal
      // sub-block is rotated — see twist_rotation_utils.hpp for the helper's
      // scope.)  We use a 36-array scratch because the helper operates on
      // 6x6 twist covariance layouts; copy the rotated 3x3 back out.
      std::array<double, 36> cov_in_6x6{};
      std::array<double, 36> cov_out_6x6{};
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          cov_in_6x6[(i + 3) * 6 + (j + 3)] =
            orientation.angular_velocity_covariance[i * 3 + j];
        }
      }
      rotate_covariance_block_3x3(cov_in_6x6, R, 3, cov_out_6x6);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          angular_cov_body[i * 3 + j] =
            cov_out_6x6[(i + 3) * 6 + (j + 3)];
        }
      }
      have_angular = true;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "orientation: TF '%s' -> '%s' lookup failed: %s (publishing zero angular — set up a static TF to eliminate this warning)",
        orientation.header.frame_id.c_str(), base_frame_.c_str(), e.what());
      // angular_body + angular_cov_body stay zero-initialized.
      have_angular = true;  // state still "received" — zeros are the honest answer.
    }
  } else {
    // Same-frame path (frame_id explicitly == base_frame_) — copy directly.
    angular_body = orientation.angular_velocity;
    for (int i = 0; i < 9; ++i) {
      angular_cov_body[i] = orientation.angular_velocity_covariance[i];
    }
    have_angular = true;
  }

  {
    std::lock_guard<std::mutex> lock(state_mu_);
    latest_orientation_ = orientation_q;
    if (have_angular) {
      latest_angular_body_ = angular_body;
      latest_angular_cov_body_ = angular_cov_body;
    }
    have_orientation_ = true;
  }
}

void MRUTransform::updateVelocity(const VelocitySensor::ValueType &velocity)
{
  // REP-105: odom.twist must be expressed in child_frame_id (body).  Incoming
  // velocity is in velocity.header.frame_id — typically a world frame (map,
  // posmv_frame, mru_frame).  Rotate linear velocity + covariance into body
  // before publishing.
  //
  // Prefer the transform sampled at velocity.header.stamp so the rotated
  // twist stays temporally consistent with the published odom stamp,
  // particularly when frame_id is a dynamic frame relative to base_frame_
  // (e.g., map — see nodes/nav_sat_fix_to_velocity.cpp, which publishes
  // velocity in map_frame_).  Both lookups are non-blocking: if the buffer
  // does not yet hold a transform at the message stamp we fall back to the
  // latest available (TimePointZero) rather than wait — a previous
  // timeout-based lookup deadlocked the velocity callback in the field when
  // the buffer fell behind.  If even TimePointZero fails (frame_id empty /
  // not in TF tree), we WARN_THROTTLE and drop the sample.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      base_frame_, velocity.header.frame_id,
      rclcpp::Time(velocity.header.stamp));
  } catch (const tf2::TransformException &stamped_lookup_error) {
    try {
      tf = tf_buffer_->lookupTransform(
        base_frame_, velocity.header.frame_id, tf2::TimePointZero);
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "velocity: TF '%s' -> '%s' lookup at stamp failed (%s); using latest available transform",
        velocity.header.frame_id.c_str(), base_frame_.c_str(),
        stamped_lookup_error.what());
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "velocity: TF '%s' -> '%s' lookup failed: %s (dropping sample)",
        velocity.header.frame_id.c_str(), base_frame_.c_str(), e.what());
      return;
    }
  }

  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation, q);
  tf2::Matrix3x3 R(q);

  // Rotate linear velocity into base_frame_.
  tf2::Vector3 v_in(velocity.twist.twist.linear.x,
                    velocity.twist.twist.linear.y,
                    velocity.twist.twist.linear.z);
  tf2::Vector3 v_out = R * v_in;
  geometry_msgs::msg::Vector3 linear_body;
  linear_body.x = v_out.x();
  linear_body.y = v_out.y();
  linear_body.z = v_out.z();

  // Compose the full Odometry message locally from (a) this velocity's
  // rotated linear + covariance and (b) a snapshot of the latest position /
  // orientation / angular state contributed by the other two callbacks under
  // state_mu_.  The snapshot is taken under the lock and then released — the
  // publish runs outside the critical section.
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = velocity.header.stamp;
  odom.child_frame_id = base_frame_;
  // Identity quaternion as the fallback pose orientation in case no
  // orientation sample has been received yet.  The default-constructed
  // quaternion is all-zero and invalid (non-unit); consumers that normalize
  // or rotate by this field can misbehave.  Overwritten below when
  // have_orientation_ is true.
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear = linear_body;

  // Rotate linear-covariance sub-block (upper-left 3x3).  Cross-covariance
  // blocks between linear and angular are not touched — see the scope note
  // in twist_rotation_utils.hpp.
  rotate_covariance_block_3x3(velocity.twist.covariance, R, 0,
                              odom.twist.covariance);

  {
    std::lock_guard<std::mutex> lock(state_mu_);
    if (have_position_) {
      odom.pose.pose.position = latest_position_map_;
    }
    if (have_orientation_) {
      odom.pose.pose.orientation = latest_orientation_;
      odom.twist.twist.angular = latest_angular_body_;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          odom.twist.covariance[(i + 3) * 6 + (j + 3)] =
            latest_angular_cov_body_[i * 3 + j];
        }
      }
    }
  }

  odom_pub_->publish(odom);
}

void MRUTransform::resetMapFrameService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                        std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(state_mu_);
  response->success = true;
  if(!mapFrame_)
  {
    // Nothing has anchored the map yet, so there is nothing to re-anchor and a
    // configured origin has not been consumed: keep it.
    if(configured_origin_)
    {
      response->message = "Map frame is not anchored yet; nothing was reset. The configured map "
        "origin (map_origin_latitude/map_origin_longitude) is kept and the first position fix "
        "will use it.";
    }
    else
    {
      response->message = "Map frame is not anchored yet; nothing was reset. It will be anchored "
        "at the first position fix.";
    }
    return;
  }
  const bool had_configured_origin = configured_origin_.has_value();
  mapFrame_.reset();
  map_origin_.reset();
  configured_origin_.reset();
  response->message = "Map frame has been reset; it will be re-anchored at the next position fix.";
  if(had_configured_origin)
  {
    response->message += " The configured map origin (map_origin_latitude/map_origin_longitude) "
      "no longer applies until the node restarts.";
  }
}

std::optional<geographic_msgs::msg::GeoPoint> MRUTransform::mapOrigin() const
{
  std::lock_guard<std::mutex> lock(state_mu_);
  return map_origin_;
}

} // namespace mru_transform
