#include "mru_transform/orientation_sensor.h"

namespace mru_transform
{

OrientationSensor::OrientationSensor(std::function<void(const ros::Time&)> update_callback)
{
  initialize("orientation", "default", update_callback);
}

OrientationSensor::OrientationSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback):BaseType(sensor_param, "orientation", update_callback)
{
  initialize(topic_, name_, update_callback);
}

void OrientationSensor::initialize(const std::string &topic, std::string name, std::function<void(const ros::Time&)> update_callback)
{
  BaseType::initialize(topic, name, "orientation", update_callback);
  ros::NodeHandle nh;
  subscriber_ = nh.subscribe(topic, 5, &OrientationSensor::imuCallback, this);
}

void OrientationSensor::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  latest_value_ = *msg;
  update_callback_(msg->header.stamp);
}

} // namespace mru_transform