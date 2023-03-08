#ifndef MRU_TRANSFORM_ORIENTATION_SENSOR_H
#define MRU_TRANSFORM_ORIENTATION_SENSOR_H

#include "sensor.h"
#include <sensor_msgs/Imu.h>

namespace mru_transform
{

class OrientationSensor: public SensorBase<sensor_msgs::Imu>
{
public:
  OrientationSensor(std::function<void(const ros::Time&)> update_callback);
  OrientationSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback);
  void initialize(const std::string &topic, std::string name, std::function<void(const ros::Time&)> update_callback);

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

};

} // namespace mru_transform

#endif
