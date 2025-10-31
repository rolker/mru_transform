#include <mru_transform/map_frame.hpp>


namespace mru_transform
{

MapFrame::MapFrame(rclcpp::Node::SharedPtr node, geographic_msgs::msg::GeoPoint const &datum, std::string const &map_frame, std::string const &odom_frame)
{

  earth_to_map_transform_.header.frame_id = "earth";
  earth_to_map_transform_.child_frame_id = map_frame;

  geodesy::ECEFPoint datum_ecef(datum);
 
  earth_to_map_transform_.transform.translation.x = datum_ecef.x;
  earth_to_map_transform_.transform.translation.y = datum_ecef.y;
  earth_to_map_transform_.transform.translation.z = datum_ecef.z;

  tf2::Quaternion longQuat;
  longQuat.setRPY(0.0,0.0,(datum.longitude+90.0)*M_PI/180.0);
  tf2::Quaternion latQuat;
  latQuat.setRPY((90-datum.latitude)*M_PI/180.0,0.0,0.0);
  tf2::Quaternion earth_to_map_rotation = longQuat*latQuat;

  earth_to_map_transform_.transform.rotation = tf2::toMsg(earth_to_map_rotation);

  tf2::Transform transform;
  tf2::fromMsg(earth_to_map_transform_.transform, transform);
  tf2::toMsg(transform.inverse(), map_to_earth_transform_.transform);
  map_to_earth_transform_.header.frame_id = map_frame;
  map_to_earth_transform_.child_frame_id = "earth";

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

geometry_msgs::msg::Point MapFrame::toLocal(geographic_msgs::msg::GeoPoint const &p) const
{
  auto p_ecef = toGeometry(geodesy::ECEFPoint(p));
  geometry_msgs::msg::Point local_point;
  tf2::doTransform(p_ecef, local_point, map_to_earth_transform_);
  return local_point;
}

geographic_msgs::msg::GeoPoint MapFrame::toEarth(const geometry_msgs::msg::Point &p) const
{
  geometry_msgs::msg::Point ecef_point;
  tf2::doTransform(p, ecef_point, earth_to_map_transform_);
  return toMsg(geodesy::ECEFPoint(ecef_point));
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
  res->map.header.frame_id = earth_to_map_transform_.child_frame_id;
  res->map.header.stamp = req->wgs84.header.stamp;
  res->map.point = toLocal(req->wgs84.position);
  return true;
}

bool MapFrame::map2ll(const std::shared_ptr<mru_transform_interfaces::srv::MapToLatLong::Request>req,
                      std::shared_ptr<mru_transform_interfaces::srv::MapToLatLong::Response> res)
{
  res->wgs84.header.frame_id = "wgs84";
  res->wgs84.header.stamp = req->map.header.stamp;
  res->wgs84.position = toEarth(req->map.point);
  return true;
}


} // namespace mru_transform
