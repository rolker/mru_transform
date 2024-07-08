#ifndef MRU_TRANSFORM_SENSOR_H
#define MRU_TRANSFORM_SENSOR_H

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

namespace mru_transform
{

std::string getROSType(std::string topic);

template<class T> 
class SensorBase
{
public:
  static const std::string sensor_type;

  const std::string &name() const
  {
    return name_;
  }

  const auto &lastValue() const
  {
    return static_cast<const T*>(this)->latest_value_;
  }
  using BaseType = SensorBase<T>;
protected:

  SensorBase(std::function<void(const rclcpp::Time&)> update_callback):
    update_callback_(update_callback)
  {
    subscribeCheck();
  }

  SensorBase(rclcpp::Node::SharedPtr node, std::string name, std::function<void(const rclcpp::Time&)> update_callback):
    update_callback_(update_callback),
    node_ptr_(node),
    name_(name)
  {
    //node_ptr_ = node;
    //topic_ = std::string(sensor_param["topics"][sensor_type]);
    //name_ = std::string(sensor_param["name"]);
    node_ptr_->declare_parameter<std::string>("sensors."+name+".topics." + sensor_type, "");
    node_ptr_->get_parameter("sensors."+name+".topics." + sensor_type, topic_);
    // name_ = name;
    if(topic_!=""){
      subscribeCheck();
    }
  }

  void subscribeCheckCallback()
  {
      subscribeCheck();
  }

  std::string resolve_topic_name(const std::string& topic_name) {
    std::string namespace_ = node_ptr_->get_namespace();
    if (!namespace_.empty() && namespace_.front() != '/') {
      namespace_ = "/" + namespace_;
    }
    return namespace_ + "/" + topic_name;
  }
  void subscribeCheck()
  {
    //auto topic_type = getROSType(nh.resolveName(topic_));
    //auto topic_types = node_ptr_->get_topic_names_and_types()[topic_];

    auto topic = resolve_topic_name(topic_);

    auto topic_type = node_ptr_->get_topic_names_and_types()[topic];

    if(topic_type.empty()){
      std::stringstream msg;
      msg << "Unknown " << T::sensor_type << " topic type for: " << topic;

      RCLCPP_WARN_THROTTLE(
          node_ptr_->get_logger(),
          *node_ptr_->get_clock(),
          30 * 1000,  // Throttle interval in milliseconds
          msg.str().c_str()
          );
    }
    else if (!static_cast<T*>(this)->subscribe(topic_, topic_type[0])){
      std::stringstream msg;
      msg <<"Unsupported " << T::sensor_type << " topic type for: " << topic_ << ", type: " << topic_type[0];
      RCLCPP_WARN_THROTTLE(
          node_ptr_->get_logger(),
          *node_ptr_->get_clock(),
          30 * 1000,  // Throttle interval in milliseconds
          msg.str().c_str()
          );
    }
      //ROS_WARN_STREAM_THROTTLE(30.0,"Unsupported " << T::sensor_type << " topic type for: " << topic_ << ", type: " << topic_type);
    else
      return; // subscribed, so bail out before setting a new timer

    //subscribe_check_timer_ = nh.createTimer(ros::Duration(1.0), std::bind(&SensorBase<T>::subscribeCheckCallback, this, std::placeholders::_1) , true);
    subscribe_check_timer_ = node_ptr_->create_wall_timer(
        1000ms, std::bind(&SensorBase<T>::subscribeCheckCallback, this));
  }
  rclcpp::Node::SharedPtr node_ptr_;
  //rclcpp::GenericSubscription::SharedPtr subscriber_;
  struct{
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geo_pose_stamped;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_stamped;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_stamped;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_stamped;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped;
  }subs_;


  std::string name_ = "default";
  std::string topic_ = T::sensor_type;
  std::function<void(const rclcpp::Time&)> update_callback_;
  rclcpp::TimerBase::SharedPtr subscribe_check_timer_;
};

} // namespace mru_transform

#endif
