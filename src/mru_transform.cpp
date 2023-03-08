#include <mru_transform/mru_transform.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
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
        position_sensors_.push_back(std::make_shared<PositionSensor>(sensors_param[i], [this](ros::Time stamp)->void{this->updatePosition(stamp);}));
        orientation_sensors_.push_back(std::make_shared<OrientationSensor>(sensors_param[i], [this](ros::Time stamp)->void{this->updateOrientation(stamp);}));
        velocity_sensors_.push_back(std::make_shared<VelocitySensor>(sensors_param[i], [this](ros::Time stamp)->void{this->updateVelocity(stamp);}));
      }
    }
  }
  
  // add a default sensor if none have been found
  if(position_sensors_.empty())
  {
    position_sensors_.push_back(std::make_shared<PositionSensor>([this](ros::Time stamp)->void{this->updatePosition(stamp);}));
  }
  if(orientation_sensors_.empty())
  {
    orientation_sensors_.push_back(std::make_shared<OrientationSensor>([this](ros::Time stamp)->void{this->updateOrientation(stamp);}));
  }
  if(velocity_sensors_.empty())
  {
    velocity_sensors_.push_back(std::make_shared<VelocitySensor>([this](ros::Time stamp)->void{this->updateVelocity(stamp);}));
  }

  for(auto s: std::vector<std::string>({"position", "orientation", "velocity"}))
    active_sensor_pubs_[s] = nh.advertise<std_msgs::String>("nav/active_sensor/"+s,10);
}

void MRUTransform::updatePosition(const ros::Time &now)
{
  if(updateLatest(latest_position_, position_sensors_, now))
  {
    p11::LatLongDegrees p;
    p11::fromMsg(latest_position_.position, p);
    if (std::isnan(p[2]))
      p[2] = 0.0;

    if(!mapFrame_)
      mapFrame_ = std::shared_ptr<MapFrame>(new MapFrame(p, map_frame_, odom_frame_));
    auto transforms = mapFrame_->getTransforms(latest_position_.header.stamp);
    p11::Point position_map = mapFrame_->toLocal(p);

    geometry_msgs::TransformStamped map_to_north_up_base_link;
    map_to_north_up_base_link.header.stamp = latest_position_.header.stamp;
    map_to_north_up_base_link.header.frame_id = map_frame_;
    map_to_north_up_base_link.child_frame_id = base_frame_+"_north_up";
    p11::toMsg(position_map, map_to_north_up_base_link.transform.translation);
    map_to_north_up_base_link.transform.rotation.w = 1.0;
    transforms.push_back(map_to_north_up_base_link);
    broadcaster_->sendTransform(transforms);
    p11::toMsg(position_map, odom_.pose.pose.position);
  }
}

void MRUTransform::updateOrientation(const ros::Time &now)
{
  if(updateLatest(latest_orientation_, orientation_sensors_, now))
  {
    tf2::Quaternion orientation_quat;

    tf2::fromMsg(latest_orientation_.orientation, orientation_quat);
    
    double roll,pitch,yaw;
    tf2::getEulerYPR(orientation_quat, yaw, pitch, roll);
       
    geometry_msgs::TransformStamped north_up_base_link_to_level_base_link;
    north_up_base_link_to_level_base_link.header.stamp = latest_orientation_.header.stamp;
    north_up_base_link_to_level_base_link.header.frame_id = base_frame_+"_north_up";
    north_up_base_link_to_level_base_link.child_frame_id = base_frame_+"_level";
    tf2::Quaternion heading_quat;
    heading_quat.setRPY(0.0,0.0,yaw);
    north_up_base_link_to_level_base_link.transform.rotation = tf2::toMsg(heading_quat);

    std::vector<geometry_msgs::TransformStamped> transforms;
    transforms.push_back(north_up_base_link_to_level_base_link);
    
    geometry_msgs::TransformStamped north_up_base_link_to_base_link;
    north_up_base_link_to_base_link.header.stamp = latest_orientation_.header.stamp;
    north_up_base_link_to_base_link.header.frame_id = base_frame_+"_north_up";
    north_up_base_link_to_base_link.child_frame_id = base_frame_;
    north_up_base_link_to_base_link.transform.rotation = latest_orientation_.orientation;
    transforms.push_back(north_up_base_link_to_base_link);
    broadcaster_->sendTransform(transforms);

    odom_.pose.pose.orientation = latest_orientation_.orientation;
    odom_.twist.twist.angular = latest_orientation_.angular_velocity;
  }
}

void MRUTransform::updateVelocity(const ros::Time &now)
{
  if(updateLatest(latest_velocity_, velocity_sensors_, now))
  {
    odom_.header.frame_id = odom_frame_;
    odom_.header.stamp = latest_velocity_.header.stamp;
    odom_.child_frame_id = base_frame_;


    tf2::Quaternion orientation_quat;
    tf2::fromMsg(latest_orientation_.orientation, orientation_quat);

    geometry_msgs::TransformStamped odom_base_rotation;
    odom_base_rotation.transform.rotation = tf2::toMsg(orientation_quat.inverse());
    tf2::doTransform(latest_velocity_.twist.linear, odom_.twist.twist.linear, odom_base_rotation);
    odom_pub_.publish(odom_);
  }
}

} // namespace mru_transform


