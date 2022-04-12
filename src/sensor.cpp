#include "mru_transform/sensor.h"

namespace mru_transform
{

Sensor::Sensor()
{
  Initialize("position", "orientation", "velocity");
}

Sensor::Sensor(XmlRpc::XmlRpcValue const &sensor_param)
{
  std::string position_topic, orientation_topic, velocity_topic, name;
  position_topic = std::string(sensor_param["topics"]["position"]);
  orientation_topic = std::string(sensor_param["topics"]["orientation"]);
  velocity_topic = std::string(sensor_param["topics"]["velocity"]);
  name = std::string(sensor_param["name"]);
  Initialize(position_topic, orientation_topic, velocity_topic, name);
}

void Sensor::Initialize(const std::string &position_topic, const std::string &orientation_topic , const std::string &velocity_topic, std::string name)
{
  ros::NodeHandle nh;

  position_sub_ = std::shared_ptr<PositionSub>(new PositionSub(nh, position_topic, 1));
  orientation_sub_ =  std::shared_ptr<OrientationSub>(new OrientationSub(nh, orientation_topic, 1));
  velocity_sub_ = std::shared_ptr<VelocitySub>(new VelocitySub(nh, velocity_topic, 1));

  name_ = name;

  sync_ = std::shared_ptr<SyncType>(new SyncType(*position_sub_, *orientation_sub_, *velocity_sub_, 10));
  sync_->registerCallback(boost::bind(&Sensor::callback, this, _1, _2, _3));
}

void Sensor::setCallback(std::function<void()> update_callback)
{
  update_callback_ = update_callback;
}

const std::string &Sensor::getName() const
{
  return name_;
}

sensor_msgs::NavSatFix::ConstPtr Sensor::lastPositionMessage() const
{
  return last_position_;
}

sensor_msgs::Imu::ConstPtr Sensor::lastOrientationMessage() const
{
  return last_orientation_;
}

geometry_msgs::TwistWithCovarianceStamped::ConstPtr Sensor::lastVelocityMessage() const
{
  return last_velocity_;
}

void Sensor::callback(const sensor_msgs::NavSatFix::ConstPtr & position, const sensor_msgs::Imu::ConstPtr & orientation, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr & velocity)
{
  last_position_ = position;
  last_orientation_ = orientation;
  last_velocity_ = velocity;
  update_callback_();
}


} // namespace mru_transform
