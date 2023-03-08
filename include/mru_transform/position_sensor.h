#ifndef MRU_TRANSFORM_POSITION_SENSOR_H
#define MRU_TRANSFORM_POSITION_SENSOR_H

#include "sensor.h"
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPointStamped.h>

namespace mru_transform
{

class PositionSensor: public SensorBase<geographic_msgs::GeoPointStamped>
{
public:
  PositionSensor(std::function<void(const ros::Time&)> update_callback);
  PositionSensor(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback);
  void initialize(const std::string &topic, std::string name, std::function<void(const ros::Time&)> update_callback);

private:
  void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

};

} // namespace mru_transform

#endif
