#ifndef MRU_TRANSFORM_VELOCITY_SENSOR_H
#define MRU_TRANSFORM_VELOCITY_SENSOR_H

#include "sensor.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace mru_transform
{

class VelocitySensor: public SensorBase<geometry_msgs::TwistStamped>
{
public:
  VelocitySensor(std::function<void(const ros::Time&)> update_callback);
  VelocitySensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback);
  void initialize(const std::string &topic, std::string name, std::function<void(const ros::Time&)> update_callback);

private:
  void twistWithCovarianceCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

};

} // namespace mru_transform

#endif
