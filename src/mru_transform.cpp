#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include "project11/utils.h"

#include "mru_transform/LatLongToEarth.h"
#include "mru_transform/LatLongToMap.h"
#include "mru_transform/EarthToLatLong.h"
#include "mru_transform/MapToLatLong.h"

namespace p11 = project11;

class MapFrame
{
public:
  MapFrame(p11::LatLongDegrees const &datum, std::string const &map_frame, std::string const &odom_frame, std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster, ros::Duration broadcast_interval):m_mapFrame(p11::ENUFrame(datum)), m_broadcaster(broadcaster), m_broadcast_interval(broadcast_interval)
  {
    m_earth_to_map_transform.header.frame_id = "earth";
    m_earth_to_map_transform.child_frame_id = map_frame;
    
    gz4d::GeoPointECEF originECEF(datum);
    m_earth_to_map_transform.transform.translation.x = originECEF[0];
    m_earth_to_map_transform.transform.translation.y = originECEF[1];
    m_earth_to_map_transform.transform.translation.z = originECEF[2];
    
    tf2::Quaternion longQuat;
    longQuat.setRPY(0.0,0.0,(datum.longitude()+90.0)*M_PI/180.0);
    tf2::Quaternion latQuat;
    latQuat.setRPY((90-datum.latitude())*M_PI/180.0,0.0,0.0);
    tf2::Quaternion earth_to_map_rotation = longQuat*latQuat;
    
    m_earth_to_map_transform.transform.rotation = tf2::toMsg(earth_to_map_rotation);
    
    broadcaster->sendTransform(m_earth_to_map_transform);
    
    m_map_to_odom_transform.header.frame_id = map_frame;
    m_map_to_odom_transform.child_frame_id = odom_frame;
    m_map_to_odom_transform.transform.rotation.w = 1.0; // make null quaternion unit length
    
    broadcaster->sendTransform(m_map_to_odom_transform);
    
    ros::NodeHandle nh;
    m_wgs84_to_map_service = nh.advertiseService("wgs84_to_map", &MapFrame::ll2map, this);
    m_map_to_wgs84_service = nh.advertiseService("map_to_wgs84", &MapFrame::map2ll, this);
    
    m_timer = nh.createTimer(broadcast_interval, &MapFrame::timerCallback, this);
  }

  p11::Point toLocal(p11::LatLongDegrees const &p) const
  {
    return m_mapFrame.toLocal(p);
  }
  
private:
  p11::ENUFrame m_mapFrame;
  
  geometry_msgs::TransformStamped m_earth_to_map_transform;
  geometry_msgs::TransformStamped m_map_to_odom_transform;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;
  ros::Duration m_broadcast_interval;
  ros::Timer m_timer;

  ros::ServiceServer m_wgs84_to_map_service;
  ros::ServiceServer m_map_to_wgs84_service;

  void timerCallback(const ros::TimerEvent &timerEvent)
  {
    // set a bit to the future to prevent blocking transforms
    // remember to take this into account if updating datum
    ros::Time a_bit_later = timerEvent.current_real+m_broadcast_interval;
    
    // make sure we don't go back in time when sending transform
    if(a_bit_later > m_earth_to_map_transform.header.stamp)
    {
      m_earth_to_map_transform.header.stamp = a_bit_later;
      m_broadcaster->sendTransform(m_earth_to_map_transform);
      m_map_to_odom_transform.header.stamp = a_bit_later;
      m_broadcaster->sendTransform(m_map_to_odom_transform);
    }
  }

  bool ll2map(mru_transform::LatLongToMap::Request &req, mru_transform::LatLongToMap::Response &res)
  {
      p11::LatLongDegrees p_ll;
      p11::fromMsg(req.wgs84.position, p_ll); 
      p11::ECEF p_ecef(p_ll);
      
      p11::Point position = m_mapFrame.toLocal(p_ecef);
      
      res.map.header.frame_id = m_earth_to_map_transform.child_frame_id;
      res.map.header.stamp = req.wgs84.header.stamp;
      p11::toMsg(position, res.map.point);
      return true;
  }

  bool map2ll(mru_transform::MapToLatLong::Request &req, mru_transform::MapToLatLong::Response &res)
  {
      p11::Point position;
      p11::fromMsg(req.map.point, position);
      p11::LatLongDegrees latlon = m_mapFrame.toLatLong(position);
      p11::toMsg(latlon, res.wgs84.position);
      
      res.wgs84.header.frame_id = "wgs84";
      res.wgs84.header.stamp = req.map.header.stamp;
      return true;
  }

};

// forward declare so Sensor can call it.
void update();

class Sensor
{
public:
  Sensor(XmlRpc::XmlRpcValue const &sensor_param)
  {
    std::string position_topic, orientation_topic, velocity_topic, name;
    position_topic = std::string(sensor_param["topics"]["position"]);
    orientation_topic = std::string(sensor_param["topics"]["orientation"]);
    velocity_topic = std::string(sensor_param["topics"]["velocity"]);
    name = std::string(sensor_param["name"]);
    Initialize(position_topic, orientation_topic, velocity_topic, name);
  }
  
  Sensor()
  {
    Initialize("position", "orientation", "velocity");
  }
  
  void Initialize(const std::string &position_topic, const std::string &orientation_topic , const std::string &velocity_topic, std::string name="default")
  {
    ros::NodeHandle nh;

    m_position_sub = std::shared_ptr<PositionSub>(new PositionSub(nh, position_topic, 1));
    m_orientation_sub =  std::shared_ptr<OrientationSub>(new OrientationSub(nh, orientation_topic, 1));
    m_velocity_sub = std::shared_ptr<VelocitySub>(new VelocitySub(nh, velocity_topic, 1));

    m_name = name;

    m_sync = std::shared_ptr<SyncType>(new SyncType(*m_position_sub, *m_orientation_sub, *m_velocity_sub, 10));
    m_sync->registerCallback(boost::bind(&Sensor::callback, this, _1, _2, _3));
  }

  const std::string &getName() const {return m_name;}

  sensor_msgs::NavSatFix::ConstPtr lastPositionMessage() const {return m_last_position;}
  sensor_msgs::Imu::ConstPtr lastOrientationMessage() const {return m_last_orientation;}
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr lastVelocityMessage() const {return m_last_velocity;}

  typedef std::shared_ptr<Sensor> Ptr;
private:
  void callback(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr & velocity)
  {
    m_last_position = position;
    m_last_orientation = orientation;
    m_last_velocity = velocity;
    update();
  }
  
  typedef message_filters::Subscriber<sensor_msgs::NavSatFix> PositionSub;
  std::shared_ptr<PositionSub> m_position_sub;

  typedef message_filters::Subscriber<sensor_msgs::Imu> OrientationSub;
  std::shared_ptr<OrientationSub> m_orientation_sub;

  typedef message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> VelocitySub;
  std::shared_ptr<VelocitySub> m_velocity_sub;
  
  typedef message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, sensor_msgs::Imu, geometry_msgs::TwistWithCovarianceStamped> SyncType;
  std::shared_ptr<SyncType> m_sync;
  
  sensor_msgs::NavSatFix::ConstPtr m_last_position;
  sensor_msgs::Imu::ConstPtr m_last_orientation;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr m_last_velocity;
  std::string m_name;
};

// list of sensors, in order of priority
std::vector<Sensor::Ptr> sensors;

sensor_msgs::NavSatFix::ConstPtr last_sent_position;
sensor_msgs::Imu::ConstPtr last_sent_orientation;
geometry_msgs::TwistWithCovarianceStamped::ConstPtr last_sent_velocity;

// replublish selected nav messages
ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;
ros::Publisher active_sensor_pub;

ros::Duration sensor_timeout(1.0);

std::string base_frame = "base_link";
std::string map_frame = "map";
std::string odom_frame = "odom";
std::string odom_topic = "odom";

std::shared_ptr<MapFrame> mapFrame;
std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
ros::Publisher odom_pub;

void update()
{
  ros::Time now = ros::Time::now();
  
  sensor_msgs::NavSatFix::ConstPtr position;
  sensor_msgs::Imu::ConstPtr orientation;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr velocity;

  // loop through the sensors until we get an unexpired message of each type
  for(auto s: sensors)
  {
    if(!position && s->lastPositionMessage() && now - s->lastPositionMessage()->header.stamp < sensor_timeout)
      position = s->lastPositionMessage();
    if(!orientation && s->lastOrientationMessage() && now - s->lastOrientationMessage()->header.stamp < sensor_timeout)
      orientation = s->lastOrientationMessage();
    if(!velocity && s->lastVelocityMessage() && now - s->lastVelocityMessage()->header.stamp < sensor_timeout)
      velocity = s->lastVelocityMessage();
    if(position && orientation && velocity)
    {
      std_msgs::String active;
      active.data = s->getName();
      active_sensor_pub.publish(active);
      break;
    }
  }

  nav_msgs::Odometry odom;
  odom.header.frame_id = odom_frame;
  
  if(position && (!last_sent_position || position->header.stamp > last_sent_position->header.stamp))
  {
    if(!mapFrame && position->status.status >= 0)
    {
      p11::LatLongDegrees datum;
      p11::fromMsg(*position,datum);
      mapFrame = std::shared_ptr<MapFrame>(new MapFrame(datum, map_frame, odom_frame, broadcaster, ros::Duration(2.0) ));
    }
    
    position_pub.publish(position);
    
    p11::LatLongDegrees p;
    p11::fromMsg(*position, p);
    if (std::isnan(p[2]))
      p[2] = 0.0;
    p11::Point position_map = mapFrame->toLocal(p);
    
    geometry_msgs::TransformStamped map_to_north_up_base_link;
    map_to_north_up_base_link.header.stamp = position->header.stamp;
    map_to_north_up_base_link.header.frame_id = map_frame;
    map_to_north_up_base_link.child_frame_id = base_frame+"_north_up";
    p11::toMsg(position_map, map_to_north_up_base_link.transform.translation);
    map_to_north_up_base_link.transform.rotation.w = 1.0;
    broadcaster->sendTransform(map_to_north_up_base_link);
    
    p11::toMsg(position_map, odom.pose.pose.position);
    odom.header.stamp = position->header.stamp;
    
    last_sent_position = position;
  }

  tf2::Quaternion orientation_quat;
  
  if(orientation && (!last_sent_orientation || orientation->header.stamp > last_sent_orientation->header.stamp))
  {
    orientation_pub.publish(orientation);

    tf2::fromMsg(orientation->orientation, orientation_quat);
    
    double roll,pitch,yaw;
    tf2::getEulerYPR(orientation_quat, yaw, pitch, roll);
       
    geometry_msgs::TransformStamped north_up_base_link_to_level_base_link;
    north_up_base_link_to_level_base_link.header.stamp = orientation->header.stamp;
    north_up_base_link_to_level_base_link.header.frame_id = base_frame+"_north_up";
    north_up_base_link_to_level_base_link.child_frame_id = base_frame+"_level";
    tf2::Quaternion heading_quat;
    heading_quat.setRPY(0.0,0.0,yaw);
    north_up_base_link_to_level_base_link.transform.rotation = tf2::toMsg(heading_quat);
    broadcaster->sendTransform(north_up_base_link_to_level_base_link);
    
    geometry_msgs::TransformStamped north_up_base_link_to_base_link;
    north_up_base_link_to_base_link.header.stamp = orientation->header.stamp;
    north_up_base_link_to_base_link.header.frame_id = base_frame+"_north_up";
    north_up_base_link_to_base_link.child_frame_id = base_frame;
    north_up_base_link_to_base_link.transform.rotation = orientation->orientation;
    broadcaster->sendTransform(north_up_base_link_to_base_link);

    odom.pose.pose.orientation = orientation->orientation;
    odom.twist.twist.angular = orientation->angular_velocity;
    
    last_sent_orientation = orientation;
  }
  
  if(velocity && (!last_sent_velocity || velocity->header.stamp > last_sent_velocity->header.stamp))
  {
    velocity_pub.publish(velocity);
    odom.child_frame_id = velocity->header.frame_id;
    geometry_msgs::TransformStamped odom_base_rotation;
    odom_base_rotation.transform.rotation = tf2::toMsg(orientation_quat.inverse());
    tf2::doTransform(velocity->twist.twist.linear, odom.twist.twist.linear, odom_base_rotation);
    odom_pub.publish(odom);
    last_sent_velocity = velocity;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mru_transform");
    
  ros::NodeHandle nh_private("~");
  
  nh_private.getParam("map_frame", map_frame);
  nh_private.getParam("base_frame", base_frame);
  nh_private.getParam("odom_frame", odom_frame);
  nh_private.getParam("odom_topic", odom_topic);

  broadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);

  ros::NodeHandle nh;
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 50);

  XmlRpc::XmlRpcValue sensors_param;
  
  if(nh_private.getParam("sensors", sensors_param))
  {
    if(sensors_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for(int i = 0; i < sensors_param.size(); i++)
        sensors.push_back(std::shared_ptr<Sensor>(new Sensor(sensors_param[i])));
    }
  }
  
  // add a default sensor in none have been found
  if(sensors.empty())
    sensors.push_back(std::shared_ptr<Sensor>(new Sensor()));

  // publishers for the selected messages. This should allow subscribers to get the best available from a single set of topics
  position_pub = nh.advertise<sensor_msgs::NavSatFix>("nav/position",10);
  orientation_pub = nh.advertise<sensor_msgs::Imu>("nav/orientation",10);
  velocity_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("nav/velocity",10);
  active_sensor_pub = nh.advertise<std_msgs::String>("nav/active_sensor",10);
  
  ros::spin();
  return 0;
}
