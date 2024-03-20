#include <mru_transform/map_frame.h>



namespace p11 = project11;

namespace mru_transform
{

MapFrame::MapFrame(rclcpp::Node::SharedPtr node, project11::LatLongDegrees const &datum, std::string const &map_frame, std::string const &odom_frame)
    :mapFrame_(project11::ENUFrame(datum))
{
  earth_to_map_transform_.header.frame_id = "earth";
  earth_to_map_transform_.child_frame_id = map_frame;
  
  gz4d::GeoPointECEF originECEF(datum);
  earth_to_map_transform_.transform.translation.x = originECEF[0];
  earth_to_map_transform_.transform.translation.y = originECEF[1];
  earth_to_map_transform_.transform.translation.z = originECEF[2];
  
  tf2::Quaternion longQuat;
  longQuat.setRPY(0.0,0.0,(datum.longitude()+90.0)*M_PI/180.0);
  tf2::Quaternion latQuat;
  latQuat.setRPY((90-datum.latitude())*M_PI/180.0,0.0,0.0);
  tf2::Quaternion earth_to_map_rotation = longQuat*latQuat;\

  earth_to_map_transform_.transform.rotation = tf2::toMsg(earth_to_map_rotation);

  map_to_odom_transform_.header.frame_id = map_frame;
  map_to_odom_transform_.child_frame_id = odom_frame;
  map_to_odom_transform_.transform.rotation.w = 1.0; // make null quaternion unit length
  
  wgs84_to_map_service_ =
      node->create_service<mru_transform_interfaces::srv::LatLongToMap>(
          "wgs84_to_map",
          std::bind(
              &MapFrame::ll2map,
              this,
              std::placeholders::_1,
              std::placeholders::_2
              ));


  map_to_wgs84_service_ =
      node->create_service<mru_transform_interfaces::srv::MapToLatLong>(
          "map_to_wgs84",
          std::bind(
              &MapFrame::map2ll,
              this,
              std::placeholders::_1,
              std::placeholders::_2
              ));
}

project11::Point MapFrame::toLocal(project11::LatLongDegrees const &p) const
{
  return mapFrame_.toLocal(p);
}

std::vector< geometry_msgs::msg::TransformStamped > MapFrame::getTransforms(rclcpp::Time time)
{
  std::vector< geometry_msgs::msg::TransformStamped > ret;
  earth_to_map_transform_.header.stamp = time;
  map_to_odom_transform_.header.stamp = time;
  ret.push_back(earth_to_map_transform_);
  ret.push_back(map_to_odom_transform_);
  return ret;
}

bool MapFrame::ll2map(const std::shared_ptr<mru_transform_interfaces::srv::LatLongToMap::Request> req,
                      std::shared_ptr<mru_transform_interfaces::srv::LatLongToMap::Response> res)
{
  p11::LatLongDegrees p_ll;
  p11::fromMsg(req->wgs84.position, p_ll);
  p11::ECEF p_ecef(p_ll);

  p11::Point position = mapFrame_.toLocal(p_ecef);

  res->map.header.frame_id = earth_to_map_transform_.child_frame_id;
  res->map.header.stamp = req->wgs84.header.stamp;
  p11::toMsg(position, res->map.point);
  return true;
}

bool MapFrame::map2ll(const std::shared_ptr<mru_transform_interfaces::srv::MapToLatLong::Request>req,
                      std::shared_ptr<mru_transform_interfaces::srv::MapToLatLong::Response> res)
{
  p11::Point position;
  p11::fromMsg(req->map.point, position);
  p11::LatLongDegrees latlon = mapFrame_.toLatLong(position);
  p11::toMsg(latlon, res->wgs84.position);
    
  res->wgs84.header.frame_id = "wgs84";
  res->wgs84.header.stamp = req->map.header.stamp;
  return true;
}


} // namespace mru_transform
