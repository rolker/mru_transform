#include "mru_transform/position_sensor.h"
using std::placeholders::_1;
namespace mru_transform
{

template <>
const std::string SensorBase<PositionSensor>::sensor_type("position");

PositionSensor::PositionSensor(std::function<void(const rclcpp::Time&)> update_callback)
:BaseType(update_callback)
{
}

PositionSensor::PositionSensor(rclcpp::Node::SharedPtr node, std::string name, std::function<void(const rclcpp::Time&)> update_callback)
    :BaseType(node, name, update_callback)
{
}

bool PositionSensor::subscribe(const std::string &topic, const std::string &topic_type)
{
  //ros::NodeHandle nh;
  if(topic_type == "sensor_msgs/msg/NavSatFix")
  {
    //subscriber_ = nh.subscribe(topic, 5, &PositionSensor::navSatFixCallback, this);
    RCLCPP_INFO(node_ptr_->get_logger(), "subscribing");
    subs_.navsat_fix = node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_, 5, std::bind(&PositionSensor::navSatFixCallback, this, _1));

    return true;
  }
  if(topic_type == "geographic_msgs/msg/GeoPoseStamped")
  {
    //subscriber_ = nh.subscribe(topic, 5, &PositionSensor::geoPoseCallback, this);
    subs_.geo_pose_stamped = node_ptr_->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
        topic_, 5, std::bind(&PositionSensor::geoPoseCallback, this, _1));
    return true;
  }
  RCLCPP_WARN_THROTTLE(
    node_ptr_->get_logger(),
    *node_ptr_->get_clock(),
    30 * 1000,  // Throttle interval in milliseconds
    "Supported position types: sensor_msgs/NavSatFix, geographic_msgs/GeoPoseStamped"
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
    update_callback_(msg->header.stamp);
  }
}

void PositionSensor::geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::ConstPtr& msg)
{
  latest_value_.header = msg->header;
  latest_value_.position = msg->pose.position;
  update_callback_(msg->header.stamp);
}


} // namespace mru_transform
