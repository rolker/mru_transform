#include "mru_transform/velocity_sensor.hpp"
#include <functional>
using std::placeholders::_1;

namespace mru_transform
{

template <>
const std::string SensorBase<VelocitySensor>::sensor_type("velocity");

VelocitySensor::VelocitySensor(std::function<void(const rclcpp::Time&)> update_callback):BaseType(update_callback)
{
}

VelocitySensor::VelocitySensor(rclcpp::Node::SharedPtr node, std::string name, std::function<void(const rclcpp::Time&)> update_callback)
    :BaseType(node, name, update_callback)
{
}

bool VelocitySensor::subscribe(const std::string &topic, const std::string &topic_type)
{
  if (topic_type == "geometry_msgs/msg/TwistWithCovarianceStamped")
  {
    subs_.twist_with_covariance_stamped = node_ptr_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        topic_, 5, std::bind(&VelocitySensor::twistWithCovarianceCallback, this, _1));
    return true;
  }
  if(topic_type == "geometry_msgs/msg/TwistStamped")
  {
    subs_.twist_stamped = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic_, 5, std::bind(&VelocitySensor::twistCallback, this, _1));
    return true;
  }
  RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(),
      *node_ptr_->get_clock(),
      30 * 1000,  // Throttle interval in milliseconds
      "Supported velocity types: geometry_msgs/TwistStamped, geometry_msgs/TwistWithCovarianceStamped"
      );
  return false;
}

void VelocitySensor::twistWithCovarianceCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstPtr& msg)
{
  latest_value_.header = msg->header;
  latest_value_.twist = msg->twist.twist;
  update_callback_(msg->header.stamp);
}

void VelocitySensor::twistCallback(const geometry_msgs::msg::TwistStamped::ConstPtr& msg)
{
  latest_value_ = *msg;
  update_callback_(msg->header.stamp);
}


} // namespace mru_transform
