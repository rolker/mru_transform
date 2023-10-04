#ifndef MRU_TRANSFORM_MAP_FRAME_H
#define MRU_TRANSFORM_MAP_FRAME_H

#include <ros/ros.h>
#include "mru_transform/LatLongToEarth.h"
#include "mru_transform/LatLongToMap.h"
#include "mru_transform/EarthToLatLong.h"
#include "mru_transform/MapToLatLong.h"
#include <tf2_ros/transform_broadcaster.h>
#include "project11/utils.h"

namespace mru_transform
{

class MapFrame
{
public:
  MapFrame(project11::LatLongDegrees const &datum, std::string const &map_frame, std::string const &odom_frame);

  project11::Point toLocal(project11::LatLongDegrees const &p) const;

  std::vector< geometry_msgs::TransformStamped > getTransforms(ros::Time time);
  
private:
  project11::ENUFrame mapFrame_;
  
  geometry_msgs::TransformStamped earth_to_map_transform_;
  geometry_msgs::TransformStamped map_to_odom_transform_;
  
  ros::ServiceServer wgs84_to_map_service_;
  ros::ServiceServer map_to_wgs84_service_;

  bool ll2map(mru_transform::LatLongToMap::Request &req, mru_transform::LatLongToMap::Response &res);

  bool map2ll(mru_transform::MapToLatLong::Request &req, mru_transform::MapToLatLong::Response &res);
};

} // namespace mru_transform

#endif
