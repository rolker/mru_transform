#ifndef MRU_TRANSFORM_SENSOR_H
#define MRU_TRANSFORM_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace mru_transform
{

class Sensor
{
public:
  Sensor();
  Sensor(XmlRpc::XmlRpcValue const &sensor_param);
  
  void Initialize(const std::string &position_topic, const std::string &orientation_topic , const std::string &velocity_topic, std::string name="default");

  void setCallback(std::function<void()> update_callback);

  const std::string &getName() const;

  sensor_msgs::NavSatFix::ConstPtr lastPositionMessage() const;
  sensor_msgs::Imu::ConstPtr lastOrientationMessage() const;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr lastVelocityMessage() const;

  typedef std::shared_ptr<Sensor> Ptr;
private:
  void callback(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr & velocity);
  
  typedef message_filters::Subscriber<sensor_msgs::NavSatFix> PositionSub;
  std::shared_ptr<PositionSub> position_sub_;

  typedef message_filters::Subscriber<sensor_msgs::Imu> OrientationSub;
  std::shared_ptr<OrientationSub> orientation_sub_;

  typedef message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> VelocitySub;
  std::shared_ptr<VelocitySub> velocity_sub_;
  
  typedef message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, sensor_msgs::Imu, geometry_msgs::TwistWithCovarianceStamped> SyncType;
  std::shared_ptr<SyncType> sync_;
  
  sensor_msgs::NavSatFix::ConstPtr last_position_;
  sensor_msgs::Imu::ConstPtr last_orientation_;
  geometry_msgs::TwistWithCovarianceStamped::ConstPtr last_velocity_;
  std::string name_;

  std::function<void()> update_callback_;
};

} // namespace mru_transform

#endif