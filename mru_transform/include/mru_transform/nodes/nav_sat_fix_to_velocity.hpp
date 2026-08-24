#ifndef MRU_TRANSFORM_NODES_NAV_SAT_FIX_TO_VELOCITY_HPP
#define MRU_TRANSFORM_NODES_NAV_SAT_FIX_TO_VELOCITY_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geodesy/geodesics.h"
#include "geodesy/wgs84.h"

class NavSatFixToVelocity : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit NavSatFixToVelocity(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :rclcpp_lifecycle::LifecycleNode("nav_sat_fix_to_velocity", options)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
    // Guarded so a second configure does not throw
    // ParameterAlreadyDeclaredException, and deliberately not undeclared in
    // on_cleanup so a value an operator set with `ros2 param set` survives a
    // cleanup -> configure cycle. (#34)
    if (!has_parameter("map_frame")) {
      declare_parameter("map_frame", map_frame_);
    }
    get_parameter("map_frame", map_frame_);

    if (!has_parameter("maximum_interval_seconds")) {
      declare_parameter("maximum_interval_seconds", maximum_interval_.seconds());
    }
    double maximum_interval_seconds = get_parameter("maximum_interval_seconds").as_double();
    maximum_interval_ = rclcpp::Duration::from_seconds(maximum_interval_seconds);

    velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
    navsat_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 10, std::bind(&NavSatFixToVelocity::navsatfix_callback, this, std::placeholders::_1));

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
  {
    navsat_subscription_.reset();
    velocity_publisher_.reset();

    // last_navsatfix_ is the fix the next one is differenced against. Kept
    // across a cleanup, the first fix after a re-configure would be
    // differenced against a pre-cleanup fix and reported as a velocity
    // averaged over the cleanup gap -- whenever that gap is shorter than
    // maximum_interval_ (2 s by default, so routinely). Clearing it makes the
    // first fix after a re-configure seed the state instead. (#34)
    last_navsatfix_ = sensor_msgs::msg::NavSatFix();

    // Parameters stay declared on purpose: see on_configure.
    return LifecycleNode::on_cleanup(state);
  }

private:
  void navsatfix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // rclcpp::Publisher::publish() is a non-virtual template that
    // LifecyclePublisher only hides, so nothing but this check keeps a
    // deactivated node from publishing. Returning before last_navsatfix_ is
    // updated is deliberate: the fix from before the deactivation is then too
    // old for the maximum_interval_ check, so the first sample after
    // re-activation seeds rather than producing a velocity across the gap.
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    rclcpp::Time last_time = rclcpp::Time(last_navsatfix_.header.stamp);
    if(last_time.nanoseconds() != 0)
    {
      rclcpp::Time current_time = rclcpp::Time(msg->header.stamp);
      rclcpp::Duration time_diff = current_time - last_time;

      if(time_diff.seconds() > 0.0 && time_diff <= maximum_interval_)
      {
        auto last_position = geodesy::toMsg(last_navsatfix_);
        auto current_position = geodesy::toMsg(*msg);

        // gedesic functions require altitude to be 0
        last_position.altitude = 0.0;
        current_position.altitude = 0.0;

        auto motion_vector = geodesy::wgs84::inverse_vector(last_position, current_position);

        geometry_msgs::msg::TwistStamped velocity_msg;
        velocity_msg.header.stamp = msg->header.stamp;
        velocity_msg.header.frame_id = map_frame_;
        velocity_msg.twist.linear.x = motion_vector.x / time_diff.seconds();
        velocity_msg.twist.linear.y = motion_vector.y / time_diff.seconds();
        velocity_msg.twist.linear.z = (last_navsatfix_.altitude - msg->altitude) / time_diff.seconds();
        velocity_publisher_->publish(velocity_msg);
      }
    }
    last_navsatfix_ = *msg;
  }

  sensor_msgs::msg::NavSatFix last_navsatfix_;

  // Maximum allowed interval between NavSatFix messages for use in calculating velocity
  rclcpp::Duration maximum_interval_ = rclcpp::Duration::from_seconds(2.0);

  std::string map_frame_ = "map";

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    velocity_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_subscription_;
};

#endif  // MRU_TRANSFORM_NODES_NAV_SAT_FIX_TO_VELOCITY_HPP
