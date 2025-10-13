#ifndef MRU_TRANSFORM_SENSOR_H
#define MRU_TRANSFORM_SENSOR_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/node_interfaces/node_interfaces.hpp"

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

using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
  rclcpp::node_interfaces::NodeBaseInterface,
  rclcpp::node_interfaces::NodeClockInterface,
  rclcpp::node_interfaces::NodeGraphInterface,
  rclcpp::node_interfaces::NodeLoggingInterface,
  rclcpp::node_interfaces::NodeParametersInterface,
  rclcpp::node_interfaces::NodeTimersInterface,
  rclcpp::node_interfaces::NodeTopicsInterface
>;


template<class T> 
class SensorBase
{
public:
  static const std::string sensor_type;



  const std::string &name() const
  {
    return name_;
  }

  using ValueType = T;
  using CallbackType = std::function<void(const ValueType&)>;

  const ValueType &latest_value() const
  {
    return latest_value_;
  }

protected:
  using BaseType = SensorBase<T>;

  SensorBase(NodeInterfaces node, std::string name, CallbackType callback):
    node_(node),
    clock_(node.get_node_clock_interface()->get_clock()),
    logger_(node.get_node_logging_interface()->get_logger()),
    name_(name),
    topic_(sensor_type)
  {
    if(callback)
      callbacks_.push_back(callback);
    node_.get_node_parameters_interface()->declare_parameter("sensors."+name+".topics." + sensor_type, rclcpp::ParameterValue(""));
    topic_ = node_.get_node_parameters_interface()->get_parameter("sensors."+name+".topics." + sensor_type).as_string();
    if(topic_!=""){
      subscribeCheck();
    }
  }

  virtual bool subscribe(const std::vector<std::string> &topic_types) = 0;


  void subscribeCheck()
  {
    auto topic = node_.get_node_base_interface()->resolve_topic_or_service_name(topic_, false);

    auto topic_types = node_.get_node_graph_interface()->get_topic_names_and_types()[topic];

    if(topic_types.empty()){
      std::stringstream msg;
      msg << "Unknown " << sensor_type << " topic type for: " << topic;

      RCLCPP_WARN_THROTTLE(logger_, *clock_,
          30 * 1000, msg.str().c_str()); // Throttle interval in milliseconds
    }
    else if (!subscribe(topic_types)){
      std::stringstream msg;
      msg <<"Unsupported " << sensor_type << " topic type for: " << topic_ << ", types: ";
      for(const auto &topic_type: topic_types)
        msg << topic_type << ", ";
      RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_,
        30 * 1000,  // Throttle interval in milliseconds
        msg.str()
      );
    }
    else
      return; // subscribed, so bail out before setting a new timer

    subscribe_check_timer_ = rclcpp::create_wall_timer(1000ms, [this]{this->subscribeCheck();}, nullptr, node_.get_node_base_interface().get(), node_.get_node_timers_interface().get());
  }

  void call_callbacks_(const ValueType& value)
  {
    for(auto &cb: callbacks_)
      cb(value);
  }

  NodeInterfaces node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  struct{
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geo_pose_stamped;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_stamped;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_stamped;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_stamped;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped;
  }subs_;

  ValueType latest_value_;
  std::string name_ = "default";
  std::string topic_;
  std::vector<CallbackType> callbacks_;
  rclcpp::TimerBase::SharedPtr subscribe_check_timer_;
};

} // namespace mru_transform

#endif
