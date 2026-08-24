#ifndef MRU_TRANSFORM_WATER_LINE_OFFSET_HPP
#define MRU_TRANSFORM_WATER_LINE_OFFSET_HPP

#include <cmath>

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
/// An unusable orientation returns the lever arm unrotated rather than a NaN.
/// Two cases matter, and a comparison-based guard catches only the first:
///
///  - A zero-length quaternion — the default-constructed value in an Odometry
///    message that nobody filled in — cannot be normalised.
///  - A quaternion carrying NaN or infinity, which a sensor dropout or a
///    divide-by-zero upstream can produce. Every comparison against NaN is
///    false, so `length2() <= 0.0` does *not* reject it; the NaN would survive
///    the rotation, poison the estimator's rolling average, pass straight
///    through the plausibility bound (whose comparisons are false for NaN too)
///    and latch on `tide_estimate`, where it feeds soundings. It is checked
///    explicitly.
///
/// Returning the unrotated lever arm keeps this function total. Callers are
/// still responsible for the sample's own position.z — a NaN there is rejected
/// by `sea_surface_estimator` before the sample is buffered.
inline double waterLineOffset(
  const tf2::Vector3 & lever_arm,
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    return lever_arm.z();
  }
  const tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  const double length2 = q.length2();
  // Finite components can still square to infinity, which normalises to zero.
  if (!std::isfinite(length2) || length2 <= 0.0) {
    return lever_arm.z();
  }
  return tf2::quatRotate(q.normalized(), lever_arm).z();
}

}  // namespace mru_transform

#endif  // MRU_TRANSFORM_WATER_LINE_OFFSET_HPP
