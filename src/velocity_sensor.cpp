#include "mru_transform/velocity_sensor.h"

namespace mru_transform
{

VelocitySensor::VelocitySensor(std::function<void(const ros::Time&)> update_callback)
{
  initialize("velocity", "default", update_callback);
}

VelocitySensor::VelocitySensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback):BaseType(sensor_param, "velocity", update_callback)
{
  initialize(topic_, name_, update_callback);
}

void VelocitySensor::initialize(const std::string &topic, std::string name, std::function<void(const ros::Time&)> update_callback)
{
  BaseType::initialize(topic, name, "velocity", update_callback);
  ros::NodeHandle nh;
  auto topic_type = getROSType(nh.resolveName(topic));

  if (topic_type == "geometry_msgs/TwistWithCovarianceStamped")
    subscriber_ = nh.subscribe(topic, 5, &VelocitySensor::twistWithCovarianceCallback, this);
  else if(topic_type == "geometry_msgs/TwistStamped")
    subscriber_ = nh.subscribe(topic, 5, &VelocitySensor::twistCallback, this);
  else
    ROS_INFO_STREAM("Unsupported velocity topic type for: " << topic << ", type: " << topic_type);

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
