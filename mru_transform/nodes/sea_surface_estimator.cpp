
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class SeaSurfaceEstimator : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SeaSurfaceEstimator()
  :rclcpp_lifecycle::LifecycleNode("sea_surface_estimator")
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
  {
    declare_parameter("sea_surface_frame", sea_surface_frame_);
    get_parameter("sea_surface_frame", sea_surface_frame_);

    declare_parameter("minimum_buffer_duration", minimum_buffer_duration_);
    get_parameter("minimum_buffer_duration", minimum_buffer_duration_);

    declare_parameter("maximum_buffer_duration", maximum_buffer_duration_);
    get_parameter("maximum_buffer_duration", maximum_buffer_duration_);

    declare_parameter("chart_datum_frame", chart_datum_frame_);
    get_parameter("chart_datum_frame", chart_datum_frame_);

    declare_parameter("mhhw_frame", mhhw_frame_);
    get_parameter("mhhw_frame", mhhw_frame_);

    declare_parameter("tide_range_margin", tide_range_margin_);
    get_parameter("tide_range_margin", tide_range_margin_);
    if (tide_range_margin_ < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'tide_range_margin' is negative (%f); clamping to 0.0",
        tide_range_margin_);
      tide_range_margin_ = 0.0;
    }

    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto latched_qos = rclcpp::QoS(1).transient_local();
    tide_estimate_pub_ = create_publisher<std_msgs::msg::Float64>(
      "tide_estimate", latched_qos);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SeaSurfaceEstimator::odometry_callback, this, std::placeholders::_1));

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn  on_activate(const rclcpp_lifecycle::State & state)
  {
    return LifecycleNode::on_activate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
  {
    tf_buffer_.reset();
    tf_listener_.reset();
    return LifecycleNode::on_cleanup(state);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    return;

    rclcpp::Time now = msg->header.stamp;
    odometry_buffer_[now] = msg;

    // Drop expired messages from the buffer.
    auto oldest_time_to_keep = now - rclcpp::Duration::from_seconds(maximum_buffer_duration_);

    while(!odometry_buffer_.empty() &&
          odometry_buffer_.begin()->first < oldest_time_to_keep)
    {
      odometry_buffer_.erase(odometry_buffer_.begin());
    }

    // Make sure we have a long enough history before publishing the transform.
    auto buffer_duration = now - odometry_buffer_.begin()->first;
    if (buffer_duration.seconds() < minimum_buffer_duration_)
    {
      return;
    }

    double sum = 0.0;
    for (const auto & odometry : odometry_buffer_)
    {
      sum += odometry.second->pose.pose.position.z;
    }
    double average = sum / odometry_buffer_.size();

    // Always publish raw estimate for debugging, even if rejected below.
    std_msgs::msg::Float64 tide_msg;
    tide_msg.data = average;
    tide_estimate_pub_->publish(tide_msg);

    // Check if the estimated tide is within a plausible range using
    // the chart datum (MLLW) and MHHW frames published by chart_datum_node.
    if (!chart_datum_frame_.empty() && !mhhw_frame_.empty()) {
      if (is_out_of_range(average, msg->header.frame_id)) {
        return;
      }
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = sea_surface_frame_;
    transform.transform.translation.z = average;
    transform.transform.rotation.w = 1.0;

    transform_broadcaster_->sendTransform(transform);
  }

private:
  // Check if the estimated sea surface Z is outside the plausible
  // tidal range. Returns true if the estimate should be rejected.
  bool is_out_of_range(double estimated_z, const std::string & frame_id)
  {
    geometry_msgs::msg::TransformStamped mllw_tf, mhhw_tf;
    try {
      mllw_tf = tf_buffer_->lookupTransform(
        frame_id, chart_datum_frame_, tf2::TimePointZero);
      mhhw_tf = tf_buffer_->lookupTransform(
        frame_id, mhhw_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      // If datum frames aren't available, don't filter
      return false;
    }

    double mllw_z = mllw_tf.transform.translation.z;
    double mhhw_z = mhhw_tf.transform.translation.z;

    // Tidal range with configurable margin (e.g., 2.0x for storm surge)
    double tidal_range = std::abs(mhhw_z - mllw_z);
    double margin = tidal_range * tide_range_margin_;
    double lower_bound = mllw_z - margin;
    double upper_bound = mhhw_z + margin;

    if (estimated_z < lower_bound || estimated_z > upper_bound) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Tide estimate %.2f m is outside plausible range "
        "[%.2f, %.2f] (MLLW=%.2f, MHHW=%.2f, margin=%.1fx) — suppressing",
        estimated_z, lower_bound, upper_bound,
        mllw_z, mhhw_z, tide_range_margin_);
      return true;
    }

    return false;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  std::map<rclcpp::Time, nav_msgs::msg::Odometry::SharedPtr> odometry_buffer_;

  // Duration in seconds of messages the buffer should have before
  // the transform is published.
  double minimum_buffer_duration_ = 5.0;

  // Maximum duration in seconds to keep in the buffer.
  double maximum_buffer_duration_ = 30.0;

  std::string sea_surface_frame_ = "map_tide";

  // Datum frame names for tide range validation
  std::string chart_datum_frame_ = "chart_datum";
  std::string mhhw_frame_ = "chart_datum_mhhw";

  // Multiplier on tidal range for rejection margin.
  // 2.0 means accept up to 2x the tidal range beyond MHHW/below MLLW
  // (accounts for storm surge, extreme tides).
  double tide_range_margin_ = 2.0;

  // TODO: figure out the transform between the frame id in the odom
  // message nad the water line. Easy hack is to use a parameter for a
  // vertical offset. It might be better to have the water line defined
  // as a frame in the tf tree (via urdf presumably) and use a
  // transform listener.

  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr tide_estimate_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SeaSurfaceEstimator>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
