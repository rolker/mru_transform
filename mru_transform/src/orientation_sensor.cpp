#include "mru_transform/orientation_sensor.hpp"
using std::placeholders::_1;
namespace mru_transform
{

template <>
const std::string SensorBase<sensor_msgs::msg::Imu>::sensor_type("orientation");


OrientationSensor::OrientationSensor(NodeInterfaces node, std::string name, CallbackType callback)
  :BaseType(node, name, callback)
{
}

bool OrientationSensor::subscribe(const std::vector<std::string>& topic_types)
{
  for(const auto &topic_type: topic_types)
  {
    if (topic_type == "sensor_msgs/msg/Imu")
    {
      subs_.imu = rclcpp::create_subscription<sensor_msgs::msg::Imu>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&OrientationSensor::imuCallback, this, _1));
      return true;
    }
    if (topic_type == "geometry_msgs/msg/QuaternionStamped")
    {
      subs_.quaternion_stamped = rclcpp::create_subscription<geometry_msgs::msg::QuaternionStamped>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&OrientationSensor::quaternionCallback, this, _1));
      return true;
    }
    if (topic_type == "geographic_msgs/msg/GeoPoseStamped")
    {
      subs_.geopose_stamped = rclcpp::create_subscription<geographic_msgs::msg::GeoPoseStamped>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&OrientationSensor::geoPoseCallback, this, _1));
      return true;
    }
  }
  RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      30 * 1000,  // Throttle interval in milliseconds
      "Supported position types: sensor_msgs/Imu, geometry_msgs/QuaternionStamped, geographic_msgs/GeoPoseStamped"
      );
  return false;
}

void OrientationSensor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->orientation;
  latest_value_.angular_velocity = msg->angular_velocity;
  latest_value_.linear_acceleration = msg->linear_acceleration;
  call_callbacks_(latest_value_);
}

void OrientationSensor::quaternionCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->quaternion;
  call_callbacks_(latest_value_);
}

void OrientationSensor::geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
{
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->pose.orientation;
  call_callbacks_(latest_value_);
}

} // namespace mru_transform
