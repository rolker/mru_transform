#ifndef MRU_TRANSFORM_WATER_LINE_LEVER_ARM_HPP
#define MRU_TRANSFORM_WATER_LINE_LEVER_ARM_HPP

#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>

namespace mru_transform
{

/// The vehicle-frame → water-line lever arm, looked up from TF and cached.
///
/// Split out of `sea_surface_estimator` so the lookup — in particular its
/// argument order, which fixes the *sign* of a correction applied to every
/// sounding-bearing tide estimate — can be pinned by a test against a
/// `tf2_ros::Buffer` populated with `setTransform`, without standing up a node.
///
/// The cache is keyed on **both** the vehicle frame the odometry reports and
/// the configured water-line frame: changing either invalidates it, so a
/// re-configure with a different frame cannot keep serving the old platform's
/// lever arm.
class WaterLineLeverArm
{
public:
  enum class Status
  {
    Disabled,        ///< No water-line frame configured; correction is off.
    Cached,          ///< Served from the cache; no lookup was attempted.
    Updated,         ///< A fresh lookup succeeded.
    NoVehicleFrame,  ///< The odometry carried an empty `child_frame_id`.
    LookupFailed,    ///< TF threw, or returned a non-finite translation.
  };

  struct Result
  {
    Status status = Status::Disabled;

    /// The lever arm, valid only when `valid()`.
    tf2::Vector3 lever_arm{0.0, 0.0, 0.0};

    /// TF's explanation, populated only for `LookupFailed`.
    std::string error;

    /// A refreshed lever arm that moved more than a millimetre from the
    /// cached one. The frame is documented as static (it comes from the
    /// URDF); this is how a violation of that becomes visible instead of
    /// silently changing the tide.
    bool changed = false;

    bool valid() const {return status == Status::Cached || status == Status::Updated;}
  };

  /// Distance, in metres, beyond which a refreshed lever arm counts as moved.
  static constexpr double kMovedThreshold = 1e-3;

  /// Configure the water-line frame. An empty name disables the correction.
  /// Changing the name invalidates any cached lever arm.
  void setWaterLineFrame(const std::string & water_line_frame)
  {
    if (water_line_frame != water_line_frame_) {
      water_line_frame_ = water_line_frame;
      reset();
    }
  }

  const std::string & waterLineFrame() const {return water_line_frame_;}

  /// Whether the correction is configured at all.
  bool enabled() const {return !water_line_frame_.empty();}

  /// Drop the cached lever arm (but keep the configured frame). Called on
  /// lifecycle cleanup so a re-configure re-reads TF rather than reusing a
  /// lever arm looked up against the previous configuration.
  void reset()
  {
    have_lever_arm_ = false;
    lever_arm_.setValue(0.0, 0.0, 0.0);
    vehicle_frame_.clear();
  }

  bool haveLeverArm() const {return have_lever_arm_;}
  const tf2::Vector3 & leverArm() const {return lever_arm_;}
  const std::string & vehicleFrame() const {return vehicle_frame_;}

  /// Whether `vehicle_frame` can be served from the cache without a lookup.
  bool isCachedFor(const std::string & vehicle_frame) const
  {
    return have_lever_arm_ && enabled() && !vehicle_frame.empty() &&
           vehicle_frame == vehicle_frame_;
  }

  /// Look the lever arm up, or serve it from the cache.
  ///
  /// `force_refresh` re-reads TF even when the cache would answer, so a caller
  /// can periodically confirm the frame really is static. A failed refresh
  /// leaves the cached value in place — the caller decides whether to keep
  /// using it.
  Result update(
    const tf2_ros::Buffer & buffer,
    const std::string & vehicle_frame,
    bool force_refresh = false)
  {
    Result result;
    if (!enabled()) {
      result.status = Status::Disabled;
      return result;
    }
    if (vehicle_frame.empty()) {
      result.status = Status::NoVehicleFrame;
      return result;
    }
    if (!force_refresh && isCachedFor(vehicle_frame)) {
      result.status = Status::Cached;
      result.lever_arm = lever_arm_;
      return result;
    }

    try {
      // lookupTransform(target, source) returns the pose of `source` expressed
      // in `target`. Target is the vehicle frame and source is the water line,
      // so the translation IS the vehicle→water-line lever arm in vehicle-frame
      // axes: positive z means the water line sits ABOVE the vehicle origin,
      // which is the sign `sea_surface_estimator` adds to odometry z. Reversing
      // these two arguments would negate the correction — double the error
      // instead of removing it — which is why this ordering is pinned by test.
      const geometry_msgs::msg::TransformStamped water_line_tf =
        buffer.lookupTransform(vehicle_frame, water_line_frame_, tf2::TimePointZero);
      const tf2::Vector3 lever_arm(
        water_line_tf.transform.translation.x,
        water_line_tf.transform.translation.y,
        water_line_tf.transform.translation.z);
      if (!std::isfinite(lever_arm.x()) || !std::isfinite(lever_arm.y()) ||
        !std::isfinite(lever_arm.z()))
      {
        result.status = Status::LookupFailed;
        result.error = "transform translation is not finite";
        return result;
      }

      // Only a refresh against the SAME vehicle frame can be "changed"; a new
      // vehicle frame is a different measurement, not a moved one.
      result.changed = have_lever_arm_ && vehicle_frame == vehicle_frame_ &&
        (lever_arm - lever_arm_).length() > kMovedThreshold;
      lever_arm_ = lever_arm;
      vehicle_frame_ = vehicle_frame;
      have_lever_arm_ = true;
      result.status = Status::Updated;
      result.lever_arm = lever_arm;
      return result;
    } catch (const tf2::TransformException & e) {
      result.status = Status::LookupFailed;
      result.error = e.what();
      return result;
    }
  }

private:
  std::string water_line_frame_;
  std::string vehicle_frame_;
  tf2::Vector3 lever_arm_{0.0, 0.0, 0.0};
  bool have_lever_arm_ = false;
};

}  // namespace mru_transform

#endif  // MRU_TRANSFORM_WATER_LINE_LEVER_ARM_HPP
