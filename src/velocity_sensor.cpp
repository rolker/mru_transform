#include "mru_transform/velocity_sensor.h"

namespace mru_transform
{

template <>
const std::string SensorBase<VelocitySensor>::sensor_type("velocity");

VelocitySensor::VelocitySensor(std::function<void(const ros::Time&)> update_callback):BaseType(update_callback)
{
}

VelocitySensor::VelocitySensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback):BaseType(sensor_param, update_callback)
{
}

bool VelocitySensor::subscribe(const std::string &topic, const std::string &topic_type)
{
  ros::NodeHandle nh;

  if (topic_type == "geometry_msgs/TwistWithCovarianceStamped")
  {
    subscriber_ = nh.subscribe(topic_, 5, &VelocitySensor::twistWithCovarianceCallback, this);
    return true;
  }
  if(topic_type == "geometry_msgs/TwistStamped")
  {
    subscriber_ = nh.subscribe(topic_, 5, &VelocitySensor::twistCallback, this);
    return true;
  }
  return false;
}

void VelocitySensor::twistWithCovarianceCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
  latest_value_.header = msg->header;
  latest_value_.twist = msg->twist.twist;
  update_callback_(msg->header.stamp);
}

void VelocitySensor::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  latest_value_ = *msg;
  update_callback_(msg->header.stamp);
}


} // namespace mru_transform
