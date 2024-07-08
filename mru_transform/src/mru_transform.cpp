#include "mru_transform.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace p11 = project11;

namespace mru_transform{

MRUTransform::MRUTransform(rclcpp::Node::SharedPtr node_ptr)
{
  node_ptr_ = node_ptr;

  node_ptr->declare_parameter("map_frame", map_frame_);
  node_ptr->get_parameter("map_frame", map_frame_);

  node_ptr->declare_parameter("base_frame", base_frame_);
  node_ptr->get_parameter("base_frame", base_frame_);

  node_ptr->declare_parameter("odom_frame", odom_frame_);
  node_ptr->get_parameter("odom_frame", odom_frame_);

  node_ptr->declare_parameter("odom_topic", odom_topic_);
  node_ptr->get_parameter("odom_topic", odom_topic_);

  double st = sensor_timeout_.seconds();
  node_ptr->declare_parameter("sensor_timeout", st);
  node_ptr->get_parameter("sensor_timeout", st);
  sensor_timeout_ = rclcpp::Duration::from_seconds(st);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_ptr);

  odom_pub_ = node_ptr->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);

  node_ptr->declare_parameter("sensors_names",sensor_names_);
  node_ptr->get_parameter("sensors_names",sensor_names_);

  for(auto sensor_name : sensor_names_){
    position_sensors_.push_back(std::make_shared<PositionSensor>(       node_ptr, sensor_name, [this](rclcpp::Time stamp)->void{this->updatePosition(stamp);}));
    orientation_sensors_.push_back(std::make_shared<OrientationSensor>( node_ptr, sensor_name, [this](rclcpp::Time stamp)->void{this->updateOrientation(stamp);}));
    velocity_sensors_.push_back(std::make_shared<VelocitySensor>(       node_ptr, sensor_name, [this](rclcpp::Time stamp)->void{this->updateVelocity(stamp);}));
  }
  
  // add a default sensor if none have been found
  if(position_sensors_.empty())
  {
    position_sensors_.push_back(std::make_shared<PositionSensor>([this](rclcpp::Time stamp)->void{this->updatePosition(stamp);}));
  }
  if(orientation_sensors_.empty())
  {
    orientation_sensors_.push_back(std::make_shared<OrientationSensor>([this](rclcpp::Time stamp)->void{this->updateOrientation(stamp);}));
  }
  if(velocity_sensors_.empty())
  {
    velocity_sensors_.push_back(std::make_shared<VelocitySensor>([this](rclcpp::Time stamp)->void{this->updateVelocity(stamp);}));
  }

  for(auto s: std::vector<std::string>({"position", "orientation", "velocity"})){
    active_sensor_pubs_[s] = node_ptr->create_publisher<std_msgs::msg::String>(
        "nav/active_sensor/"+s,10);
  }
}

void MRUTransform::updatePosition(const rclcpp::Time &now)
{
  if(updateLatest(latest_position_, position_sensors_, now))
  {
    p11::LatLongDegrees p;
    p11::fromMsg(latest_position_.position, p);
    if (std::isnan(p[2]))
      p[2] = 0.0;

    if(!mapFrame_)
    {
      auto map_origin = p;
      map_origin.altitude() = 0.0;
      mapFrame_ = std::shared_ptr<MapFrame>(new MapFrame(node_ptr_, map_origin, map_frame_, odom_frame_));
    }
    auto transforms = mapFrame_->getTransforms(latest_position_.header.stamp);
    p11::Point position_map = mapFrame_->toLocal(p);

    geometry_msgs::msg::TransformStamped map_to_north_up_base_link;
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

void MRUTransform::updateOrientation(const rclcpp::Time &now)
{
  if(updateLatest(latest_orientation_, orientation_sensors_, now))
  {
    tf2::Quaternion orientation_quat;

    tf2::fromMsg(latest_orientation_.orientation, orientation_quat);
    
    double roll,pitch,yaw;
    tf2::getEulerYPR(orientation_quat, yaw, pitch, roll);

    geometry_msgs::msg::TransformStamped north_up_base_link_to_level_base_link;
    north_up_base_link_to_level_base_link.header.stamp = latest_orientation_.header.stamp;
    north_up_base_link_to_level_base_link.header.frame_id = base_frame_+"_north_up";
    north_up_base_link_to_level_base_link.child_frame_id = base_frame_+"_level";
    tf2::Quaternion heading_quat;
    heading_quat.setRPY(0.0,0.0,yaw);
    north_up_base_link_to_level_base_link.transform.rotation = tf2::toMsg(heading_quat);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(north_up_base_link_to_level_base_link);
    
    geometry_msgs::msg::TransformStamped north_up_base_link_to_base_link;
    north_up_base_link_to_base_link.header.stamp = latest_orientation_.header.stamp;
    north_up_base_link_to_base_link.header.frame_id = base_frame_+"_north_up";
    north_up_base_link_to_base_link.child_frame_id = base_frame_;
    north_up_base_link_to_base_link.transform.rotation = latest_orientation_.orientation;
    // if we have an uninitialized quat, lets set it to identity
    if(latest_orientation_.orientation.x == 0.0 && latest_orientation_.orientation.y == 0.0 && latest_orientation_.orientation.z == 0 && latest_orientation_.orientation.w == 0.0)
      north_up_base_link_to_base_link.transform.rotation.w = 1.0;
    transforms.push_back(north_up_base_link_to_base_link);
    broadcaster_->sendTransform(transforms);

    odom_.pose.pose.orientation = latest_orientation_.orientation;
    odom_.twist.twist.angular = latest_orientation_.angular_velocity;
  }
}

void MRUTransform::updateVelocity(const rclcpp::Time &now)
{
  if(updateLatest(latest_velocity_, velocity_sensors_, now))
  {
    odom_.header.frame_id = odom_frame_;
    odom_.header.stamp = latest_velocity_.header.stamp;
    odom_.child_frame_id = base_frame_;


    tf2::Quaternion orientation_quat;
    tf2::fromMsg(latest_orientation_.orientation, orientation_quat);

    //geometry_msgs::msg::TransformStamped odom_base_rotation;
    // odom_base_rotation.transform.rotation = tf2::toMsg(orientation_quat.inverse());
    // tf2::doTransform(latest_velocity_.twist.linear, odom_.twist.twist.linear, odom_base_rotation);
    odom_.twist.twist.linear = latest_velocity_.twist.linear;
    odom_pub_->publish(odom_);
  }
}

} // namespace mru_transform


