#ifndef MRU_TRANSFORM_POSITION_SENSOR_H
#define MRU_TRANSFORM_POSITION_SENSOR_H

#include "sensor.h"
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPointStamped.h>

namespace mru_transform
{

class PositionSensor: public SensorBase<PositionSensor>
{
public:
  using ValueType = geographic_msgs::GeoPointStamped;

  PositionSensor(std::function<void(const ros::Time&)> update_callback);
  PositionSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback);

private:
  friend class SensorBase<PositionSensor>;
  ValueType latest_value_;

  bool subscribe(const std::string &topic, const std::string &topic_type);

  void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};

} // namespace mru_transform

#endif
