#include "mru_transform/position_sensor.h"

namespace mru_transform
{

PositionSensor::PositionSensor(std::function<void(const ros::Time&)> update_callback)
{
  initialize("position", "default", update_callback);
}

PositionSensor::PositionSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback):BaseType(sensor_param, "position", update_callback)
{
  initialize(topic_, name_, update_callback);
}

void PositionSensor::initialize(const std::string &topic, std::string name, std::function<void(const ros::Time&)> update_callback)
{
  BaseType::initialize(topic, name, "position", update_callback);
  ros::NodeHandle nh;
  subscriber_ = nh.subscribe(topic, 5, &PositionSensor::navSatFixCallback, this);
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

} // namespace mru_transform