#ifndef MRU_TRANSFORM_MRU_TRANSFORM_H
#define MRU_TRANSFORM_MRU_TRANSFORM_H

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <mru_transform/orientation_sensor.hpp>
#include <mru_transform/position_sensor.hpp>
#include <mru_transform/velocity_sensor.hpp>
#include <mru_transform/map_frame.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp> // Include the Trigger service header


namespace mru_transform
{

class MRUTransform
{
public:
  MRUTransform(rclcpp::Node::SharedPtr node_ptr);
  void update();
  void updatePosition(const rclcpp::Time &timestamp);
  void updateOrientation(const rclcpp::Time &timestamp);
  void updateVelocity(const rclcpp::Time &timestamp);

private:
  template<typename T, typename VST> bool updateLatest(T &value,  const VST& sensors, const rclcpp::Time& now)
  {
    for(auto s: sensors){
      rclcpp::Time sensor_time = s->lastValue().header.stamp;
      rclcpp::Time value_time = value.header.stamp;
      auto msg_age = now - sensor_time;
      if(now - s->lastValue().header.stamp < sensor_timeout_){
        if(sensor_time > value_time){
          value = s->lastValue();
          std_msgs::msg::String active;
          active.data = s->name();
          active_sensor_pubs_[s->sensor_type]->publish(active);
          return true;
        }
        else{
          double age = (msg_age.nanoseconds() / 1.0e9) - (msg_age.nanoseconds() / 1.0e9);
          RCLCPP_WARN(node_ptr_->get_logger(),
                      "time from sensor %s received out of order",
                      s->name().c_str() );
        }
      }else{
        double age = msg_age.nanoseconds() / 1.0e9;
        RCLCPP_WARN(node_ptr_->get_logger(), "time from sensor %s timeout, age: %fs", s->name().c_str(),age);
      }
    }
    return false;
  }

  void resetMapFrameService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                            std_srvs::srv::Trigger::Response::SharedPtr response);


  // list of sensors, in order of priority
  std::vector<std::shared_ptr<PositionSensor> > position_sensors_;
  std::vector<std::shared_ptr<OrientationSensor> > orientation_sensors_;
  std::vector<std::shared_ptr<VelocitySensor> > velocity_sensors_;

  PositionSensor::ValueType latest_position_;
  OrientationSensor::ValueType latest_orientation_;
  VelocitySensor::ValueType latest_velocity_;

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr > active_sensor_pubs_;

  rclcpp::Duration sensor_timeout_ = rclcpp::Duration(1.0s);

  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string odom_topic_ = "odom";
  std::vector<std::string> sensor_names_ = {"example_sensor"};

  std::shared_ptr<MapFrame> mapFrame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_frame_service_;

};

} // namespace mru_transform

#endif
