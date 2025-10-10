#include "mru_transform/velocity_sensor.hpp"
#include <functional>
using std::placeholders::_1;

namespace mru_transform
{

template <>
const std::string SensorBase<geometry_msgs::msg::TwistStamped>::sensor_type("velocity");

VelocitySensor::VelocitySensor(CallbackType callback):BaseType(callback)
{
}

VelocitySensor::VelocitySensor(rclcpp::Node::SharedPtr node, std::string name, CallbackType callback)
    :BaseType(node, name, callback)
{
}

bool VelocitySensor::subscribe(const std::string &topic, const std::string &topic_type)
{
  if (topic_type == "geometry_msgs/msg/TwistWithCovarianceStamped")
  {
    subs_.twist_with_covariance_stamped = node_ptr_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        topic_, rclcpp::SensorDataQoS(), std::bind(&VelocitySensor::twistWithCovarianceCallback, this, _1));
    return true;
  }
  if(topic_type == "geometry_msgs/msg/TwistStamped")
  {
    subs_.twist_stamped = node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic_, rclcpp::SensorDataQoS(), std::bind(&VelocitySensor::twistCallback, this, _1));
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

void VelocitySensor::twistWithCovarianceCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  latest_value_.header = msg->header;
  latest_value_.twist = msg->twist.twist;
  call_callbacks_(latest_value_);
}

void VelocitySensor::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_value_ = *msg;
  call_callbacks_(latest_value_);
}

} // namespace mru_transform
