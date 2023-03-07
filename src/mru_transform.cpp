#include <mru_transform/mru_transform.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace p11 = project11;


namespace mru_transform
{

MRUTransform::MRUTransform(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  nh_private.getParam("map_frame", map_frame_);
  nh_private.getParam("base_frame", base_frame_);
  nh_private.getParam("odom_frame", odom_frame_);
  nh_private.getParam("odom_topic", odom_topic_);
  double st = sensor_timeout_.toSec();
  if(nh_private.getParam("sensor_timeout", st))
    sensor_timeout_ = ros::Duration(st);

  broadcaster_ = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);

  odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic_, 50);

  XmlRpc::XmlRpcValue sensors_param;
  
  if(nh_private.getParam("sensors", sensors_param))
  {
    if(sensors_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for(int i = 0; i < sensors_param.size(); i++)
      {
        sensors_.push_back(std::shared_ptr<Sensor>(new Sensor(sensors_param[i])));
        sensors_.back()->setCallback([this]()->void{this->update();});
      }
    }
  }
  
  // add a default sensor in none have been found
  if(sensors_.empty())
  {
    sensors_.push_back(std::shared_ptr<Sensor>(new Sensor()));
    sensors_.back()->setCallback([this]()->void{this->update();});
  }

  // publishers for the selected messages. This should allow subscribers to get the best available from a single set of topics
  position_pub_ = nh.advertise<sensor_msgs::NavSatFix>("nav/position",10);
  orientation_pub_ = nh.advertise<sensor_msgs::Imu>("nav/orientation",10);
  velocity_pub_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("nav/velocity",10);
  active_sensor_pub_ = nh.advertise<std_msgs::String>("nav/active_sensor",10);
}

void MRUTransform::update()
{
  ros::Time now = ros::Time::now();
  
  sensor_msgs::NavSatFix::ConstPtr position;
  sensor_msgs::Imu::ConstPtr orientation;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr velocity;

  // loop through the sensors until we get an unexpired message of each type
  for(auto s: sensors_)
  {
    ROS_DEBUG_STREAM(now << " sensor: " << s->getName() << " p: " << *s->lastPositionMessage() << " o: " << *s->lastOrientationMessage() << " v: " << *s->lastVelocityMessage());
    if(!position && s->lastPositionMessage() && now - s->lastPositionMessage()->header.stamp < sensor_timeout_ && s->lastPositionMessage()->status.status >= 0)
      position = s->lastPositionMessage();
    if(!orientation && s->lastOrientationMessage() && now - s->lastOrientationMessage()->header.stamp < sensor_timeout_)
      orientation = s->lastOrientationMessage();
    if(!velocity && s->lastVelocityMessage() && now - s->lastVelocityMessage()->header.stamp < sensor_timeout_)
      velocity = s->lastVelocityMessage();
    if(position && orientation && velocity)
    {
      std_msgs::String active;
      active.data = s->getName();
      active_sensor_pub_.publish(active);
      break;
    }
  }

  nav_msgs::Odometry odom;
  odom.header.frame_id = odom_frame_;

  std::vector< geometry_msgs::TransformStamped > transforms;

  if(position && (!last_sent_position_ || position->header.stamp > last_sent_position_->header.stamp))
  {
    p11::LatLongDegrees p;
    p11::fromMsg(*position, p);
    if (std::isnan(p[2]))
      p[2] = 0.0;

    if(!mapFrame_ && position->status.status >= 0)
    {
      mapFrame_ = std::shared_ptr<MapFrame>(new MapFrame(p, map_frame_, odom_frame_));
    }
    
    if(mapFrame_)
      transforms = mapFrame_->getTransforms(position->header.stamp);
      //mapFrame->sendTransforms();

    position_pub_.publish(position);
    
    p11::Point position_map = mapFrame_->toLocal(p);
    
    geometry_msgs::TransformStamped map_to_north_up_base_link;
    map_to_north_up_base_link.header.stamp = position->header.stamp;
    map_to_north_up_base_link.header.frame_id = map_frame_;
    map_to_north_up_base_link.child_frame_id = base_frame_+"_north_up";
    p11::toMsg(position_map, map_to_north_up_base_link.transform.translation);
    map_to_north_up_base_link.transform.rotation.w = 1.0;
    transforms.push_back(map_to_north_up_base_link);
    
    p11::toMsg(position_map, odom.pose.pose.position);
    odom.header.stamp = position->header.stamp;
    
    last_sent_position_ = position;
  }

  tf2::Quaternion orientation_quat;
  
  if(orientation && (!last_sent_orientation_ || orientation->header.stamp > last_sent_orientation_->header.stamp))
  {
    orientation_pub_.publish(orientation);

    tf2::fromMsg(orientation->orientation, orientation_quat);
    
    double roll,pitch,yaw;
    tf2::getEulerYPR(orientation_quat, yaw, pitch, roll);
       
    geometry_msgs::TransformStamped north_up_base_link_to_level_base_link;
    north_up_base_link_to_level_base_link.header.stamp = orientation->header.stamp;
    north_up_base_link_to_level_base_link.header.frame_id = base_frame_+"_north_up";
    north_up_base_link_to_level_base_link.child_frame_id = base_frame_+"_level";
    tf2::Quaternion heading_quat;
    heading_quat.setRPY(0.0,0.0,yaw);
    north_up_base_link_to_level_base_link.transform.rotation = tf2::toMsg(heading_quat);
    transforms.push_back(north_up_base_link_to_level_base_link);
    
    geometry_msgs::TransformStamped north_up_base_link_to_base_link;
    north_up_base_link_to_base_link.header.stamp = orientation->header.stamp;
    north_up_base_link_to_base_link.header.frame_id = base_frame_+"_north_up";
    north_up_base_link_to_base_link.child_frame_id = base_frame_;
    north_up_base_link_to_base_link.transform.rotation = orientation->orientation;
    transforms.push_back(north_up_base_link_to_base_link);

    odom.pose.pose.orientation = orientation->orientation;
    odom.twist.twist.angular = orientation->angular_velocity;
    
    last_sent_orientation_ = orientation;
  }

  if(last_publish_time_+publish_period_ < now)
  {
    broadcaster_->sendTransform(transforms);
    last_publish_time_ = now;
  }

  if(velocity && (!last_sent_velocity_ || velocity->header.stamp > last_sent_velocity_->header.stamp))
  {
    velocity_pub_.publish(velocity);
    geometry_msgs::TransformStamped odom_base_rotation;
    odom_base_rotation.transform.rotation = tf2::toMsg(orientation_quat.inverse());
    tf2::doTransform(velocity->twist.twist.linear, odom.twist.twist.linear, odom_base_rotation);
    odom.child_frame_id = base_frame_;
    odom_pub_.publish(odom);
    last_sent_velocity_ = velocity;
  }
}


} // namespace mru_transform


