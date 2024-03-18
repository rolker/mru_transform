#ifndef MRU_TRANSFORM_POSITION_SENSOR_H
#define MRU_TRANSFORM_POSITION_SENSOR_H

#include "sensor.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
//#include <geographic_msgs/GeoPointStamped.h>
//#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

namespace mru_transform
{

class PositionSensor: public SensorBase<PositionSensor>
{
public:
  using ValueType = geographic_msgs::msg::GeoPointStamped;

  PositionSensor(std::function<void(const rclcpp::Time&)> update_callback);
  PositionSensor(rclcpp::Node::SharedPtr node, std::string name, std::function<void(const rclcpp::Time&)> update_callback);

private:
  friend class SensorBase<PositionSensor>;
  ValueType latest_value_;

  struct{
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geo_pose_stamped;
  }subs_;

  bool subscribe(const std::string &topic, const std::string &topic_type);

  void navSatFixCallback(const sensor_msgs::msg::NavSatFix::ConstPtr& msg);

  void geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::ConstPtr& msg);
};

} // namespace mru_transform

#endif
