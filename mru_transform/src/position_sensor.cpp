#include "mru_transform/position_sensor.hpp"

using std::placeholders::_1;

namespace mru_transform
{

template <>
const std::string SensorBase<geographic_msgs::msg::GeoPointStamped>::sensor_type("position");

PositionSensor::PositionSensor(NodeInterfaces node, std::string name, CallbackType callback)
    :BaseType(node, name, callback)
{
}

bool PositionSensor::subscribe(const std::vector<std::string> &topic_types)
{
  // Already subscribed — don't tear down and recreate a live subscription.
  // Defense-in-depth for issue #23; SensorBase::subscribeCheck() also stops its
  // timer once subscribed, so this is normally never re-entered.
  if(subs_.navsat_fix || subs_.geo_point_stamped || subs_.geo_pose_stamped)
    return true;
  for(const auto &topic_type: topic_types)
  {
    if(topic_type == "sensor_msgs/msg/NavSatFix")
    {
      subs_.navsat_fix = rclcpp::create_subscription<sensor_msgs::msg::NavSatFix>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&PositionSensor::navSatFixCallback, this, _1));
      return true;
    }
    if(topic_type == "geographic_msgs/msg/GeoPointStamped")
    {
      subs_.geo_point_stamped = rclcpp::create_subscription<geographic_msgs::msg::GeoPointStamped>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&PositionSensor::geoPointCallback, this, _1));
      return true;
    }
    if(topic_type == "geographic_msgs/msg/GeoPoseStamped")
    {
      subs_.geo_pose_stamped = rclcpp::create_subscription<geographic_msgs::msg::GeoPoseStamped>(node_, topic_, rclcpp::SensorDataQoS(), std::bind(&PositionSensor::geoPoseCallback, this, _1));
      return true;
    }
  }
  RCLCPP_WARN_THROTTLE(
    logger_,
    *clock_,
    30 * 1000,  // Throttle interval in milliseconds
    "Supported position types: sensor_msgs/NavSatFix, geographic_msgs/GeoPoseStamped, geographic_msgs/GeoPointStamped"
    );
  return false;
}

void PositionSensor::navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if(msg->status.status >= 0)
  {
    latest_value_.header = msg->header;
    latest_value_.position.latitude = msg->latitude;
    latest_value_.position.longitude = msg->longitude;
    latest_value_.position.altitude = msg->altitude;
    call_callbacks_(latest_value_);
  }
}

void PositionSensor::geoPointCallback(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg)
{
  latest_value_ = *msg;
  call_callbacks_(latest_value_);
}

void PositionSensor::geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
{
  latest_value_.header = msg->header;
  latest_value_.position = msg->pose.position;
  call_callbacks_(latest_value_);
}

} // namespace mru_transform
