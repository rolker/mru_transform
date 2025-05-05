
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

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

    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

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

    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = sea_surface_frame_;
    transform.transform.translation.z = average;
    transform.transform.rotation.w = 1.0;

    transform_broadcaster_->sendTransform(transform);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  std::map<rclcpp::Time, nav_msgs::msg::Odometry::SharedPtr> odometry_buffer_;

  // Duration in seconds of messages the buffer should have before
  // the transform is published.
  double minimum_buffer_duration_ = 5.0;

  // Maximum duration in seconds to keep in the buffer.
  double maximum_buffer_duration_ = 30.0;


  std::string sea_surface_frame_ = "map_tide";

  // TODO: figure out the transform between the frame id in the odom
  // message nad the water line. Easy hack is to use a parameter for a
  // vertical offset. It might be better to have the water line defined
  // as a frame in the tf tree (via urdf presumably) and use a
  // transform listener.

  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SeaSurfaceEstimator>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
