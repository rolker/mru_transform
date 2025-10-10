#include "mru_transform/navigation_sensors.hpp"

namespace mru_transform
{

NavigationSensors::NavigationSensors(rclcpp::Node::SharedPtr node, bool publish_active_sensors)
  : node_(node)
{
  double st = sensor_timeout_.seconds();
  node->declare_parameter("sensor_timeout", st);
  node->get_parameter("sensor_timeout", st);
  sensor_timeout_ = rclcpp::Duration::from_seconds(st);

  node->declare_parameter("sensor_names",sensor_names_);
  node->get_parameter("sensor_names",sensor_names_);

  for(auto sensor_name : sensor_names_){
    position_sensors_.push_back(std::make_shared<PositionSensor>(       node, sensor_name, std::bind(
      &NavigationSensors::positionCallback, this, std::placeholders::_1
    )));
    orientation_sensors_.push_back(std::make_shared<OrientationSensor>( node, sensor_name, std::bind(&NavigationSensors::orientationCallback, this, std::placeholders::_1)));
    velocity_sensors_.push_back(std::make_shared<VelocitySensor>(       node, sensor_name, std::bind(&NavigationSensors::velocityCallback, this, std::placeholders::_1)));
  }
  
  // add a default sensor if none have been found
  if(position_sensors_.empty())
  {
    position_sensors_.push_back(std::make_shared<PositionSensor>(std::bind(&NavigationSensors::positionCallback, this, std::placeholders::_1)));
  }
  if(orientation_sensors_.empty())
  {
    orientation_sensors_.push_back(std::make_shared<OrientationSensor>(std::bind(&NavigationSensors::orientationCallback, this, std::placeholders::_1)));
  }
  if(velocity_sensors_.empty())
  {
    velocity_sensors_.push_back(std::make_shared<VelocitySensor>(std::bind(&NavigationSensors::velocityCallback, this, std::placeholders::_1)));
  }

  if(publish_active_sensors)
  {
    for(auto s: std::vector<std::string>({"position", "orientation", "velocity"})){
      active_sensor_pubs_[s] = node_->create_publisher<std_msgs::msg::String>(
          "nav/active_sensor/"+s,10);
    }
  }

}

void NavigationSensors::positionCallback(const PositionSensor::ValueType &position)
{
  if(updateLatest<PositionSensor::ValueType>(latest_position_, position_sensors_, position.header.stamp))
  {
    // we have a new valid position
    for(auto &cb: position_callbacks_)
      cb(latest_position_);
  }
}

void NavigationSensors::orientationCallback(const OrientationSensor::ValueType &orientation)
{
  if(updateLatest<OrientationSensor::ValueType>(latest_orientation_, orientation_sensors_, orientation.header.stamp))
  {
    // we have a new valid orientation
    for(auto &cb: orientation_callbacks_)
      cb(latest_orientation_);
  }
}

void NavigationSensors::velocityCallback(const VelocitySensor::ValueType &velocity)
{
  if(updateLatest<VelocitySensor::ValueType>(latest_velocity_, velocity_sensors_, velocity.header.stamp))
  {
    // we have a new valid velocity
    for(auto &cb: velocity_callbacks_)
      cb(latest_velocity_);
  }
}

void NavigationSensors::registerPositionCallback(PositionSensor::CallbackType callback)
{
  position_callbacks_.push_back(callback);
}

void NavigationSensors::registerOrientationCallback(OrientationSensor::CallbackType callback)
{
  orientation_callbacks_.push_back(callback);
}

void NavigationSensors::registerVelocityCallback(VelocitySensor::CallbackType callback)
{
  velocity_callbacks_.push_back(callback);
}

} // namespace mru_transform

