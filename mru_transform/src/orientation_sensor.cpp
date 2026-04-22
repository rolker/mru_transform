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
  // Copy the full Imu payload, covariances included.  Without the covariance
  // copies below, MRUTransform::updateOrientation would always rotate a
  // default-zero angular_velocity_covariance and the published odom twist
  // covariance would be meaningless regardless of what the IMU reports.
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->orientation;
  latest_value_.orientation_covariance = msg->orientation_covariance;
  latest_value_.angular_velocity = msg->angular_velocity;
  latest_value_.angular_velocity_covariance = msg->angular_velocity_covariance;
  latest_value_.linear_acceleration = msg->linear_acceleration;
  latest_value_.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  call_callbacks_(latest_value_);
}

void OrientationSensor::quaternionCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  RCLCPP_WARN_ONCE(logger_,
    "Orientation source '%s' is a QuaternionStamped — angular_velocity is "
    "not available from this source type and will be published as zero. "
    "Use a sensor_msgs/Imu source if angular velocity is needed downstream.",
    topic_.c_str());
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->quaternion;
  // Explicitly clear angular_velocity to avoid publishing stale data from a
  // prior IMU stream.
  latest_value_.angular_velocity = geometry_msgs::msg::Vector3();
  for (auto &c : latest_value_.angular_velocity_covariance) c = 0.0;
  call_callbacks_(latest_value_);
}

void OrientationSensor::geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
{
  RCLCPP_WARN_ONCE(logger_,
    "Orientation source '%s' is a GeoPoseStamped — angular_velocity is "
    "not available from this source type and will be published as zero. "
    "Use a sensor_msgs/Imu source if angular velocity is needed downstream.",
    topic_.c_str());
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->pose.orientation;
  latest_value_.angular_velocity = geometry_msgs::msg::Vector3();
  for (auto &c : latest_value_.angular_velocity_covariance) c = 0.0;
  call_callbacks_(latest_value_);
}

} // namespace mru_transform
