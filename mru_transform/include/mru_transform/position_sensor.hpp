#ifndef MRU_TRANSFORM_POSITION_SENSOR_H
#define MRU_TRANSFORM_POSITION_SENSOR_H

#include "sensor.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

namespace mru_transform
{

class PositionSensor: public SensorBase<geographic_msgs::msg::GeoPointStamped>
{
public:
  PositionSensor(NodeInterfaces node, std::string name, CallbackType callback);

private:
  bool subscribe(const std::vector<std::string> &topic_types) override;

  void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void geoPoseCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);
};

} // namespace mru_transform

#endif
