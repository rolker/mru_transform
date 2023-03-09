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
  using BaseType = SensorBase<T>;
  using ValueType = T;

  const std::string &name() const
  {
    return name_;
  }

  const std::string &category() const
  {
    return category_;
  }

  const T &lastValue() const
  {
    return latest_value_;
  }

protected:
  SensorBase()
  {
  }

  SensorBase(XmlRpc::XmlRpcValue const &sensor_param, std::string topic_category, std::function<void(const ros::Time&)> update_callback)
  {
    std::string topic, name;
    topic = std::string(sensor_param["topics"][topic_category]);
    name = std::string(sensor_param["name"]);
    initialize(topic, name, topic_category, update_callback);
  }

  virtual void initialize(const std::string &topic, std::string name, std::string topic_category, std::function<void(const ros::Time&)> update_callback)
  {
    name_ = name;
    topic_ = topic;
    category_ = topic_category;
    update_callback_ = update_callback;
  }

  ros::Subscriber subscriber_;
  T latest_value_;
  std::string name_;
  std::string topic_;
  std::string category_;
  std::function<void(const ros::Time&)> update_callback_;
};

} // namespace mru_transform

#endif