#include "mru_transform/position_sensor.h"

namespace mru_transform
{

template <>
const std::string SensorBase<PositionSensor>::sensor_type("position");

PositionSensor::PositionSensor(std::function<void(const ros::Time&)> update_callback)
:BaseType(update_callback)
{
}

PositionSensor::PositionSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback):BaseType(sensor_param, update_callback)
{
}

bool PositionSensor::subscribe(const std::string &topic, const std::string &topic_type)
{
  ros::NodeHandle nh;
  if(topic_type == "sensor_msgs/NavSatFix")
  {
    subscriber_ = nh.subscribe(topic, 5, &PositionSensor::navSatFixCallback, this);
    return true;
  }
  if(topic_type == "geographic_msgs/GeoPoseStamped")
  {
    subscriber_ = nh.subscribe(topic, 5, &PositionSensor::geoPoseCallback, this);
    return true;
  }
  ROS_WARN_STREAM_THROTTLE(30.0, "Supported position types: sensor_msgs/NavSatFix, geographic_msgs/GeoPoseStamped");
  return false;
}

void PositionSensor::navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
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

void PositionSensor::geoPoseCallback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg)
{
  latest_value_.header = msg->header;
  latest_value_.position = msg->pose.position;
  update_callback_(msg->header.stamp);
}


} // namespace mru_transform