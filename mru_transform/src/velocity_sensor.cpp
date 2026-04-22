#include "mru_transform/velocity_sensor.hpp"
#include <functional>
using std::placeholders::_1;

namespace mru_transform
{

template <>
const std::string SensorBase<geometry_msgs::msg::TwistWithCovarianceStamped>::sensor_type("velocity");

VelocitySensor::VelocitySensor(NodeInterfaces node, std::string name, CallbackType callback)
    :BaseType(node, name, callback)
{
}

bool VelocitySensor::subscribe(const std::vector<std::string> &topic_types)
{
  for(const auto &topic_type: topic_types)
  {
    if (topic_type == "geometry_msgs/msg/TwistWithCovarianceStamped")
    {
      subs_.twist_with_covariance_stamped = rclcpp::create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&VelocitySensor::twistWithCovarianceCallback, this, _1));
      return true;
    }
    if(topic_type == "geometry_msgs/msg/TwistStamped")
    {
      subs_.twist_stamped = rclcpp::create_subscription<geometry_msgs::msg::TwistStamped>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&VelocitySensor::twistCallback, this, _1));
      return true;
    }
  }
  RCLCPP_WARN_THROTTLE(logger_, *clock_,
      30 * 1000,  // Throttle interval in milliseconds
      "Supported velocity types: geometry_msgs/TwistStamped, geometry_msgs/TwistWithCovarianceStamped"
      );
  return false;
}

void VelocitySensor::twistWithCovarianceCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  latest_value_ = *msg;
  call_callbacks_(latest_value_);
}

void VelocitySensor::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_value_.header = msg->header;
  latest_value_.twist.twist = msg->twist;
  // Covariance is unknown for plain TwistStamped — leave at default-constructed zeros.
  for (auto &c : latest_value_.twist.covariance) {
    c = 0.0;
  }
  call_callbacks_(latest_value_);
}

} // namespace mru_transform
