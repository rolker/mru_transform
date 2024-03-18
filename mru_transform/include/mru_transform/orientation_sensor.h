#ifndef MRU_TRANSFORM_ORIENTATION_SENSOR_H
#define MRU_TRANSFORM_ORIENTATION_SENSOR_H

#include "sensor.h"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
//#include <geographic_msgs/GeoPoseStamped.h>

namespace mru_transform
{

class OrientationSensor: public SensorBase<OrientationSensor>
{
public:
  using ValueType = sensor_msgs::msg::Imu;

  OrientationSensor(std::function<void(const rclcpp::Time&)> update_callback);
  OrientationSensor(rclcpp::Node::SharedPtr node, std::string name, std::function<void(const rclcpp::Time&)> update_callback);

private:
  ValueType latest_value_;
  struct{
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_stamped;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_stamped;
  }subs_;

  friend class SensorBase<OrientationSensor>;
  bool subscribe(const std::string &topic, const std::string &topic_type);

  void imuCallback(const sensor_msgs::msg::Imu::ConstPtr& msg);
  void quaternionCallback(const geometry_msgs::msg::QuaternionStamped::ConstPtr& msg);
  void geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::ConstPtr& msg);

};


} // namespace mru_transform

#endif
