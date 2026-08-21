#ifndef MRU_TRANSFORM_WATER_LINE_OFFSET_HPP
#define MRU_TRANSFORM_WATER_LINE_OFFSET_HPP

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace mru_transform
{

/// Vertical component, in the parent frame, of a lever arm expressed in the
/// vehicle frame.
///
/// `sea_surface_estimator` averages `odom.pose.pose.position.z`, which is the
/// height of the *vehicle* frame, and publishes it as the sea surface. The
/// water line is not the vehicle frame: on BizzyBoat `base_link` sits 0.030 m
/// below it, and on a platform whose origin is at deck level the gap is far
/// larger. This returns the height the water line sits above the vehicle frame
/// so the estimator can add it.
///
/// The lever arm is rotated by the vehicle's attitude rather than added as a
/// scalar. For BizzyBoat's 0.030 m that is worth about 0.1 mm at 5 degrees of
/// roll and could fairly be ignored; for a metre-scale offset on a boat working
/// in a seaway it is not, and getting it right costs one quaternion rotation.
///
/// A zero-length quaternion — the default-constructed value in an Odometry
/// message that nobody filled in — cannot be normalised, so the lever arm is
/// returned unrotated rather than producing a NaN.
inline double waterLineOffset(
  const tf2::Vector3 & lever_arm,
  const geometry_msgs::msg::Quaternion & orientation)
{
  const tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  if (q.length2() <= 0.0) {
    return lever_arm.z();
  }
  return tf2::quatRotate(q.normalized(), lever_arm).z();
}

}  // namespace mru_transform

#endif  // MRU_TRANSFORM_WATER_LINE_OFFSET_HPP
