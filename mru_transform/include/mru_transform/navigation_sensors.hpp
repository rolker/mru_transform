#ifndef MRU_TRANSFORM_NAVIGATION_SENSORS_HPP
#define MRU_TRANSFORM_NAVIGATION_SENSORS_HPP

#include "mru_transform/position_sensor.hpp"
#include "mru_transform/velocity_sensor.hpp"
#include "mru_transform/orientation_sensor.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"

namespace mru_transform
{

/// NavigationSensors class manages multiple navigation-related sensors and provides the latest valid readings.
class NavigationSensors
{
public:
  NavigationSensors(NodeInterfaces node, bool publish_active_sensors = false);

  /// @brief Register a callback to be called when a new valid position is available
  /// @param callback Function to be called with the latest position
  void registerPositionCallback(PositionSensor::CallbackType  callback);
  /// @brief Register a callback to be called when a new valid orientation is available
  /// @param callback Function to be called with the latest orientation
  void registerOrientationCallback(OrientationSensor::CallbackType  callback);
  /// @brief Register a callback to be called when a new valid velocity is available
  /// @param callback Function to be called with the latest velocity
  void registerVelocityCallback(VelocitySensor::CallbackType  callback);

  const PositionSensor::ValueType & latest_position() const;
  const OrientationSensor::ValueType & latest_orientation() const;
  const VelocitySensor::ValueType & latest_velocity() const;

private:
  /// @brief Updates the value from the first non-timed-out sensor.
  /// @tparam T Sensor value type
  /// @tparam SensorVectorT Vector of sensors type
  /// @param value Contains the last value for this sensor type ans is used to return the updated sensor value
  /// @param sensors Vector of sensors 
  /// @param now Current time used to detect expired messages
  /// @return True if a valid value was found
  template<typename T, typename SensorVectorT> bool updateLatest(T &value,  const SensorVectorT& sensors, const rclcpp::Time& now)
  {
    for(auto s: sensors){
      rclcpp::Time sensor_time = s->latest_value().header.stamp;
      rclcpp::Time value_time = value.header.stamp;
      auto msg_age = now - sensor_time;
      if(msg_age < sensor_timeout_){
        if(sensor_time > value_time){
          value = s->latest_value();
          if(active_sensor_pubs_.find(s->sensor_type) != active_sensor_pubs_.end())
          {
            std_msgs::msg::String active;
            active.data = s->name();
            active_sensor_pubs_[s->sensor_type]->publish(active);
          }
          return true;
        }
        else{
          RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "skipping message with time " <<  (value_time - sensor_time).seconds() << " seconds behind last value from sensor " << s->name());
        }
      }else{
        RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 5000, "sensor " << s->name() << "'s value is stale, age: " << msg_age.seconds() << " seconds");
      }
    }
    return false;
  }

  void positionCallback(const PositionSensor::ValueType &position);
  void orientationCallback(const OrientationSensor::ValueType &orientation);
  void velocityCallback(const VelocitySensor::ValueType &velocity);

  NodeInterfaces node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  // list of sensors, in order of priority
  std::vector<std::shared_ptr<PositionSensor> > position_sensors_;
  std::vector<std::shared_ptr<OrientationSensor> > orientation_sensors_;
  std::vector<std::shared_ptr<VelocitySensor> > velocity_sensors_;

  PositionSensor::ValueType latest_position_;
  OrientationSensor::ValueType latest_orientation_;
  VelocitySensor::ValueType latest_velocity_;

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr > active_sensor_pubs_;

  rclcpp::Duration sensor_timeout_ = rclcpp::Duration(1.0s);

  std::vector<std::string> sensor_names_ = {"default"};

  std::vector<PositionSensor::CallbackType> position_callbacks_;
  std::vector<OrientationSensor::CallbackType> orientation_callbacks_;
  std::vector<VelocitySensor::CallbackType> velocity_callbacks_;

};

} // namespace mru_transform

#endif // MRU_TRANSFORM_NAVIGATION_SENSORS_HPP