#ifndef MRU_TRANSFORM_MRU_TRANSFORM_H
#define MRU_TRANSFORM_MRU_TRANSFORM_H

#include <mru_transform/orientation_sensor.h>
#include <mru_transform/position_sensor.h>
#include <mru_transform/velocity_sensor.h>
#include <mru_transform/map_frame.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

namespace mru_transform
{

class MRUTransform
{
public:
  MRUTransform(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  void update();
  void updatePosition(const ros::Time &timestamp);
  void updateOrientation(const ros::Time &timestamp);
  void updateVelocity(const ros::Time &timestamp);

private:
  template<typename T, typename VST> bool updateLatest(T &value,  const VST& sensors, const ros::Time& now)
  {
    for(auto s: sensors)
      if(now - s->lastValue().header.stamp < sensor_timeout_ &&
s->lastValue().header.stamp > value.header.stamp)
        {
          value = s->lastValue();
          std_msgs::String active;
          active.data = s->name();
          active_sensor_pubs_[s->category()].publish(active);
          return true;
        }
    return false;
  }

  // list of sensors, in order of priority
  std::vector<std::shared_ptr<PositionSensor> > position_sensors_;
  std::vector<std::shared_ptr<OrientationSensor> > orientation_sensors_;
  std::vector<std::shared_ptr<VelocitySensor> > velocity_sensors_;

  PositionSensor::ValueType latest_position_;
  OrientationSensor::ValueType latest_orientation_;
  VelocitySensor::ValueType latest_velocity_;

  std::map<std::string, ros::Publisher> active_sensor_pubs_;

  ros::Duration sensor_timeout_ = ros::Duration(1.0);

  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string odom_topic_ = "odom";

  std::shared_ptr<MapFrame> mapFrame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  ros::Publisher odom_pub_;
  nav_msgs::Odometry odom_;
};

} // namespace mru_transform

#endif
