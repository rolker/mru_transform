#ifndef MRU_TRANSFORM_VELOCITY_SENSOR_H
#define MRU_TRANSFORM_VELOCITY_SENSOR_H

#include "sensor.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace mru_transform
{

class VelocitySensor: public SensorBase<VelocitySensor>
{
public:
  using ValueType = geometry_msgs::TwistStamped;

  VelocitySensor(std::function<void(const ros::Time&)> update_callback);
  VelocitySensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback);

private:
  ValueType latest_value_;

  bool subscribe(const std::string &topic, const std::string &topic_type);
  friend class SensorBase<VelocitySensor>;

  void twistWithCovarianceCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
};

} // namespace mru_transform

#endif
