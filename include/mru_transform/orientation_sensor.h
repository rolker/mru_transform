#ifndef MRU_TRANSFORM_ORIENTATION_SENSOR_H
#define MRU_TRANSFORM_ORIENTATION_SENSOR_H

#include "sensor.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace mru_transform
{

class OrientationSensor: public SensorBase<OrientationSensor>
{
public:
  using ValueType = sensor_msgs::Imu;

  OrientationSensor(std::function<void(const ros::Time&)> update_callback);
  OrientationSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback);

private:
  ValueType latest_value_;

  friend class SensorBase<OrientationSensor>;
  bool subscribe(const std::string &topic, const std::string &topic_type);

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void quaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

};


} // namespace mru_transform

#endif
