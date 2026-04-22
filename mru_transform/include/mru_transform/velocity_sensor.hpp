#ifndef MRU_TRANSFORM_VELOCITY_SENSOR_H
#define MRU_TRANSFORM_VELOCITY_SENSOR_H

#include "sensor.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace mru_transform
{

// Uses TwistWithCovarianceStamped as its value type so covariance is
// preserved end-to-end for REP-105-compliant downstream consumers.  The
// plain TwistStamped subscription path fills a zero covariance block.
class VelocitySensor: public SensorBase<geometry_msgs::msg::TwistWithCovarianceStamped>
{
public:
  VelocitySensor(NodeInterfaces node, std::string name, CallbackType callback);

private:
  bool subscribe(const std::vector<std::string> &topic_types) override;

  void twistWithCovarianceCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};

} // namespace mru_transform

#endif
