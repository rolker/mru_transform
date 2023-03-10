#include "mru_transform/orientation_sensor.h"

namespace mru_transform
{

template <>
const std::string SensorBase<OrientationSensor>::sensor_type("orientation");

OrientationSensor::OrientationSensor(std::function<void(const ros::Time&)> update_callback)
  :BaseType(update_callback)
{
}

OrientationSensor::OrientationSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback)
  :BaseType(sensor_param,  update_callback)
{
}

bool OrientationSensor::subscribe(const std::string &topic, const std::string& topic_type)
{
  ros::NodeHandle nh;
  if (topic_type == "sensor_msgs/Imu")
  {
    subscriber_ = nh.subscribe(topic, 5, &OrientationSensor::imuCallback, this);
    return true;
  }
  if (topic_type == "geometry_msgs/QuaternionStamped")
  {
    subscriber_ = nh.subscribe(topic, 5, &OrientationSensor::quaternionCallback, this);
    return true;
  }
  return false;
}

void OrientationSensor::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  latest_value_ = *msg;
  update_callback_(msg->header.stamp);
}

void OrientationSensor::quaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  latest_value_.header = msg->header;
  latest_value_.orientation = msg->quaternion;
  update_callback_(msg->header.stamp);
}

} // namespace mru_transform