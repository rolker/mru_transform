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

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
  {
    // Deactivation mutes the node, and a muted interval must not become a
    // velocity. The ACTIVE guard in the callback drops fixes received while
    // inactive but cannot make the gap itself safe: a deactivate -> activate
    // cycle shorter than maximum_interval_ (2 s by default -- a quick operator
    // cycle, and routine under bag/sim time) leaves the pre-deactivation fix
    // inside the window, so the first fix after re-activation would be
    // reported as a velocity averaged across the muted gap. Clearing here is
    // what actually gives that guarantee. (#34)
    last_navsatfix_ = sensor_msgs::msg::NavSatFix();
    return LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
  {
    release_everything_on_configure_created();
    // Parameters stay declared on purpose: see on_configure.
    return LifecycleNode::on_cleanup(state);
  }

  // `shutdown` is legal from `unconfigured`, `inactive` AND `active`, and it
  // runs on_shutdown ONLY -- on_deactivate and on_cleanup are both skipped.
  // Without this override nothing released what on_configure created, so a
  // FINALIZED node kept its endpoints up for the rest of the process's life.
  // The release helper is null-safe and idempotent, so one override is correct
  // from all three source states. (#34)
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
  {
    release_everything_on_configure_created();
    return LifecycleNode::on_shutdown(state);
  }

  // An exception out of on_configure or on_activate routes the FSM through
  // `errorprocessing` to `unconfigured` WITHOUT running on_cleanup, and
  // `cleanup` is not a legal transition from `unconfigured` -- so without this
  // override nothing ever releases what the failed transition had already
  // allocated. The supported recovery is another configure, which overwrites
  // both pointers and strands the old endpoints: a `fix` subscription still
  // dispatching into a half-configured node. Same shape as the sibling nodes,
  // same fix. (#34)
  CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
  {
    RCLCPP_ERROR(
      get_logger(),
      "Transition failed with an exception; releasing everything the failed "
      "configure/activate had allocated. Correct the fault and configure again.");
    release_everything_on_configure_created();
    return LifecycleNode::on_error(state);
  }

private:
  // Shared by on_cleanup, on_shutdown and on_error: everything on_configure
  // created,
  // subscription first so no callback can be running against the publisher
  // released below it (sufficient only under the single-threaded executor this
  // node's main() uses). Every reset is null-safe and re-nulls, so the helper
  // is idempotent. Clearing last_navsatfix_ here is the same reasoning as
  // on_deactivate -- cleanup and shutdown are both reachable directly from
  // inactive, and a fix from before the gap must never become the reference
  // for one after it. (#34)
  void release_everything_on_configure_created()
  {
    navsat_subscription_.reset();
    velocity_publisher_.reset();
    last_navsatfix_ = sensor_msgs::msg::NavSatFix();
  }

  void navsatfix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // rclcpp_lifecycle::LifecyclePublisher::publish() IS virtual (jazzy
    // lifecycle_publisher.hpp) and returns early unless is_activated(), so the
    // publish itself is gated by the publisher now that velocity_publisher_ is
    // a LifecyclePublisher. This comment used to say publish() was a
    // non-virtual hide the call site bypassed; that was true of the plain
    // rclcpp::Publisher held on jazzy and went stale when the member type
    // changed. It is not why the check is kept.
    //
    // The check is load-bearing for something the publisher's gate cannot do:
    // it returns BEFORE last_navsatfix_ is updated, so a fix arriving while the
    // node is muted never becomes the reference for a later difference --
    // pinned by NavSatFixToVelocityIgnoresFixesWhileInactive, which fails if
    // this check alone is deleted. The gap itself is made safe by
    // on_deactivate/on_cleanup/on_shutdown clearing last_navsatfix_, not by the
    // maximum_interval_ check, which a sub-2 s cycle would pass.
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
