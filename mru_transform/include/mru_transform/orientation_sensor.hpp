#ifndef MRU_TRANSFORM_ORIENTATION_SENSOR_H
#define MRU_TRANSFORM_ORIENTATION_SENSOR_H

#include "sensor.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

namespace mru_transform
{

class OrientationSensor: public SensorBase<sensor_msgs::msg::Imu>
{
public:
  OrientationSensor(CallbackType callback);
  OrientationSensor(NodeInterfaces node, std::string name, CallbackType callback);

private:
  bool subscribe(const std::vector<std::string> &topic_types) override;

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void quaternionCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
  void geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);
};


} // namespace mru_transform

#endif
