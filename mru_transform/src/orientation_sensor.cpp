#include "mru_transform/orientation_sensor.hpp"
using std::placeholders::_1;
namespace mru_transform
{

template <>
const std::string SensorBase<OrientationSensor>::sensor_type("orientation");

OrientationSensor::OrientationSensor(std::function<void(const rclcpp::Time&)> update_callback)
  :BaseType(update_callback)
{
}

OrientationSensor::OrientationSensor(rclcpp::Node::SharedPtr node, std::string name, std::function<void(const rclcpp::Time&)> update_callback)
  :BaseType(node, name,  update_callback)
{
}

bool OrientationSensor::subscribe(const std::string &topic, const std::string& topic_type)
{
  if (topic_type == "sensor_msgs/msg/Imu")
  {
    subs_.imu = node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
        topic_, 5, std::bind(&OrientationSensor::imuCallback, this, _1));
    return true;
  }
  if (topic_type == "geometry_msgs/msg/QuaternionStamped")
  {
    subs_.quaternion_stamped = node_ptr_->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        topic_, 5, std::bind(&OrientationSensor::quaternionCallback, this, _1));
    return true;
  }
  if (topic_type == "geographic_msgs/msg/GeoPoseStamped")
  {
    subs_.geopose_stamped = node_ptr_->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
        topic_, 5, std::bind(&OrientationSensor::geoPoseCallback, this, _1));
    return true;
  }
  RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(),
      *node_ptr_->get_clock(),
      30 * 1000,  // Throttle interval in milliseconds
      "Supported position types: sensor_msgs/Imu, geometry_msgs/QuaternionStamped, geographic_msgs/GeoPoseStamped"
      );
  return false;
}

void OrientationSensor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_value_ = *msg;
  update_callback_(msg->header.stamp);
}

void OrientationSensor::quaternionCallback(const geometry_msgs::msg::QuaternionStamped::ConstPtr& msg)
{
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->quaternion;
  update_callback_(msg->header.stamp);
}

void OrientationSensor::geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::ConstPtr& msg)
{
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->pose.orientation;
  update_callback_(msg->header.stamp);
}


} // namespace mru_transform
