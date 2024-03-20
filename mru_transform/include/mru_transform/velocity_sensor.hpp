#ifndef MRU_TRANSFORM_VELOCITY_SENSOR_H
#define MRU_TRANSFORM_VELOCITY_SENSOR_H

#include "sensor.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace mru_transform
{

class VelocitySensor: public SensorBase<VelocitySensor>
{
public:
  using ValueType = geometry_msgs::msg::TwistStamped;

  VelocitySensor(std::function<void(const rclcpp::Time&)> update_callback);
  VelocitySensor(rclcpp::Node::SharedPtr node, std::string name, std::function<void(const rclcpp::Time&)> update_callback);

private:
  ValueType latest_value_;
  bool subscribe(const std::string &topic, const std::string &topic_type);
  friend class SensorBase<VelocitySensor>;

  void twistWithCovarianceCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstPtr& msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::ConstPtr& msg);
};

} // namespace mru_transform

#endif
