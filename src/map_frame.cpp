#include <mru_transform/map_frame.h>



namespace p11 = project11;

namespace mru_transform
{

MapFrame::MapFrame(project11::LatLongDegrees const &datum, std::string const &map_frame, std::string const &odom_frame, std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster, ros::Duration broadcast_interval):mapFrame_(project11::ENUFrame(datum)), broadcaster_(broadcaster), broadcast_interval_(broadcast_interval)
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
  tf2::Quaternion earth_to_map_rotation = longQuat*latQuat;
  
  earth_to_map_transform_.transform.rotation = tf2::toMsg(earth_to_map_rotation);
  
  broadcaster->sendTransform(earth_to_map_transform_);
  
  map_to_odom_transform_.header.frame_id = map_frame;
  map_to_odom_transform_.child_frame_id = odom_frame;
  map_to_odom_transform_.transform.rotation.w = 1.0; // make null quaternion unit length
  
  broadcaster->sendTransform(map_to_odom_transform_);
  
  ros::NodeHandle nh;
  wgs84_to_map_service_ = nh.advertiseService("wgs84_to_map", &MapFrame::ll2map, this);
  map_to_wgs84_service_ = nh.advertiseService("map_to_wgs84", &MapFrame::map2ll, this);
  
  //m_timer = nh.createTimer(broadcast_interval, &MapFrame::timerCallback, this);
}

project11::Point MapFrame::toLocal(project11::LatLongDegrees const &p) const
{
  return mapFrame_.toLocal(p);
}

void MapFrame::sendTransforms()
{
  broadcaster_->sendTransform(earth_to_map_transform_);
  broadcaster_->sendTransform(map_to_odom_transform_);
}

std::vector< geometry_msgs::TransformStamped > MapFrame::getTransforms(ros::Time time)
{
  std::vector< geometry_msgs::TransformStamped > ret;
  earth_to_map_transform_.header.stamp = time+broadcast_interval_;
  map_to_odom_transform_.header.stamp = time+broadcast_interval_;
  ret.push_back(earth_to_map_transform_);
  ret.push_back(map_to_odom_transform_);
  return ret;
}

void MapFrame::timerCallback(const ros::TimerEvent &timerEvent)
{
  // set a bit to the future to prevent blocking transforms
  // remember to take this into account if updating datum
  ros::Time a_bit_later = timerEvent.current_real+broadcast_interval_;
  
  // make sure we don't go back in time when sending transform
  if(a_bit_later > earth_to_map_transform_.header.stamp)
  {
    earth_to_map_transform_.header.stamp = a_bit_later;
    map_to_odom_transform_.header.stamp = a_bit_later;
    sendTransforms();
  }
}

bool MapFrame::ll2map(mru_transform::LatLongToMap::Request &req, mru_transform::LatLongToMap::Response &res)
{
    p11::LatLongDegrees p_ll;
    p11::fromMsg(req.wgs84.position, p_ll); 
    p11::ECEF p_ecef(p_ll);
    
    p11::Point position = mapFrame_.toLocal(p_ecef);
    
    res.map.header.frame_id = earth_to_map_transform_.child_frame_id;
    res.map.header.stamp = req.wgs84.header.stamp;
    p11::toMsg(position, res.map.point);
    return true;
}

bool MapFrame::map2ll(mru_transform::MapToLatLong::Request &req, mru_transform::MapToLatLong::Response &res)
{
    p11::Point position;
    p11::fromMsg(req.map.point, position);
    p11::LatLongDegrees latlon = mapFrame_.toLatLong(position);
    p11::toMsg(latlon, res.wgs84.position);
    
    res.wgs84.header.frame_id = "wgs84";
    res.wgs84.header.stamp = req.map.header.stamp;
    return true;
}


} // namespace mru_transform
