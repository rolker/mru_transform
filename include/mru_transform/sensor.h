#ifndef MRU_TRANSFORM_SENSOR_H
#define MRU_TRANSFORM_SENSOR_H

#include <ros/ros.h>

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

protected:
  using BaseType = SensorBase<T>;
  SensorBase(std::function<void(const ros::Time&)> update_callback):
    update_callback_(update_callback)
  {
    subscribeCheck();
  }

  SensorBase(XmlRpc::XmlRpcValue const &sensor_param, std::function<void(const ros::Time&)> update_callback):
    update_callback_(update_callback)
  {
    topic_ = std::string(sensor_param["topics"][sensor_type]);
    name_ = std::string(sensor_param["name"]);
    subscribeCheck();
  }

  void subscribeCheckCallback(const ros::TimerEvent& event)
  {
      subscribeCheck();
  }

  void subscribeCheck()
  {
    ros::NodeHandle nh;
    auto topic_type = getROSType(nh.resolveName(topic_));

    if(topic_type.empty())
      ROS_WARN_STREAM_THROTTLE(30.0,"Unknown " << T::sensor_type << " topic type for: " << topic_);
    else if (!static_cast<T*>(this)->subscribe(topic_, topic_type))
      ROS_WARN_STREAM_THROTTLE(30.0,"Unsupported " << T::sensor_type << " topic type for: " << topic_ << ", type: " << topic_type);
    else
      return; // subscribed, so bail out before setting a new timer

    subscribe_check_timer_ = nh.createTimer(ros::Duration(1.0), std::bind(&SensorBase<T>::subscribeCheckCallback, this, std::placeholders::_1) , true);
  }

  ros::Subscriber subscriber_;
  std::string name_ = "default";
  std::string topic_ = T::sensor_type;
  std::function<void(const ros::Time&)> update_callback_;
  ros::Timer subscribe_check_timer_;
};

} // namespace mru_transform

#endif