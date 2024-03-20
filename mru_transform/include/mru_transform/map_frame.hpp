#ifndef MRU_TRANSFORM_MAP_FRAME_H
#define MRU_TRANSFORM_MAP_FRAME_H

#include <rclcpp/rclcpp.hpp>

#include <mru_transform_interfaces/srv/lat_long_to_earth.hpp>
#include <mru_transform_interfaces/srv/lat_long_to_map.hpp>
#include <mru_transform_interfaces/srv/earth_to_lat_long.hpp>
#include <mru_transform_interfaces/srv/map_to_lat_long.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include "project11/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



namespace mru_transform
{

class MapFrame
{
public:
  MapFrame(rclcpp::Node::SharedPtr node,
           project11::LatLongDegrees const &datum,
           std::string const &map_frame,
           std::string const &odom_frame);

  project11::Point toLocal(project11::LatLongDegrees const &p) const;

  std::vector<geometry_msgs::msg::TransformStamped> getTransforms(rclcpp::Time time);

private:
  project11::ENUFrame mapFrame_;

  geometry_msgs::msg::TransformStamped earth_to_map_transform_;
  geometry_msgs::msg::TransformStamped map_to_odom_transform_;

  rclcpp::Service<mru_transform_interfaces::srv::LatLongToMap>::SharedPtr wgs84_to_map_service_;
  rclcpp::Service<mru_transform_interfaces::srv::MapToLatLong>::SharedPtr map_to_wgs84_service_;

  bool ll2map(const std::shared_ptr<mru_transform_interfaces::srv::LatLongToMap::Request> req,
              std::shared_ptr<mru_transform_interfaces::srv::LatLongToMap::Response> res);

  bool map2ll(const std::shared_ptr<mru_transform_interfaces::srv::MapToLatLong::Request>req,
              std::shared_ptr<mru_transform_interfaces::srv::MapToLatLong::Response> res);
};

} // namespace mru_transform

#endif
