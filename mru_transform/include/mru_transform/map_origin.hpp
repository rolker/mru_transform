#ifndef MRU_TRANSFORM_MAP_ORIGIN_H
#define MRU_TRANSFORM_MAP_ORIGIN_H

// Helpers for a pre-defined map origin and for the flat-plane validity check
// (mru_transform#49; the flat-plane design is #48). Header-only and free of
// ROS node dependencies so they can be unit-tested without DDS.

#include <chrono>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geographic_msgs/msg/geo_point.hpp>

namespace mru_transform
{

/// Mean Earth radius (m) used for the flat-plane height-error estimate.
constexpr double kMeanEarthRadius = 6371008.8;

/// Validate the optional map origin parameters (degrees).
///
/// Both unset -> nullopt (first-fix anchoring). Exactly one set, a non-finite
/// value, or a value outside latitude [-90, 90] / longitude [-180, 180] throws
/// std::invalid_argument. The returned point's altitude is exactly 0: the
/// origin sits on the WGS84 ellipsoid, mru_transform's own convention.
inline std::optional<geographic_msgs::msg::GeoPoint> validateMapOrigin(
  std::optional<double> latitude, std::optional<double> longitude)
{
  if (!latitude && !longitude) {
    return std::nullopt;
  }
  if (!latitude || !longitude) {
    throw std::invalid_argument(
      std::string("map origin: both map_origin_latitude and map_origin_longitude must be set, "
                  "but ") +
      (latitude ? "map_origin_longitude" : "map_origin_latitude") + " is missing");
  }
  if (!std::isfinite(*latitude) || *latitude < -90.0 || *latitude > 90.0) {
    std::ostringstream os;
    os << "map origin: map_origin_latitude must be finite and within [-90, 90] degrees, got "
       << *latitude;
    throw std::invalid_argument(os.str());
  }
  if (!std::isfinite(*longitude) || *longitude < -180.0 || *longitude > 180.0) {
    std::ostringstream os;
    os << "map origin: map_origin_longitude must be finite and within [-180, 180] degrees, got "
       << *longitude;
    throw std::invalid_argument(os.str());
  }
  geographic_msgs::msg::GeoPoint point;
  point.latitude = *latitude;
  point.longitude = *longitude;
  point.altitude = 0.0;
  return point;
}

/// The flat-plane height error threshold must be finite and > 0.
inline double validateMaxHeightError(double max_height_error)
{
  if (!std::isfinite(max_height_error) || max_height_error <= 0.0) {
    std::ostringstream os;
    os << "map_maximum_height_error must be finite and > 0 metres, got " << max_height_error;
    throw std::invalid_argument(os.str());
  }
  return max_height_error;
}

/// Estimated height error (m) of the flat tangent plane at a horizontal
/// distance from its origin: d^2 / (2R) with R the mean Earth radius. An
/// estimate, not a bound: the local radius of curvature differs from the mean
/// by up to about 1%.
inline double flatPlaneHeightError(double horizontal_distance)
{
  return horizontal_distance * horizontal_distance / (2.0 * kMeanEarthRadius);
}

/// True when the fix at (x, y) in the map frame is far enough from the map
/// origin that the estimated flat-plane height error exceeds max_error.
/// Horizontal only (z ignored), measured before any sensor offset. d is
/// sqrt(x^2 + y^2); d^2 is formed directly rather than via a square root that
/// is then squared again.
inline bool exceedsFlatPlaneBound(double x, double y, double max_error)
{
  return (x * x + y * y) / (2.0 * kMeanEarthRadius) > max_error;
}

/// Decides when the far-from-origin WARN is due: while the bound is exceeded,
/// at most once per interval. The state lives in the instance (not in a
/// per-call-site static, as RCLCPP_WARN_THROTTLE keeps it) and the time is
/// passed in, so tests drive it directly and independently of each other.
class FarFromOriginWarning
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr std::chrono::seconds kInterval{10};

  bool shouldWarn(bool bound_exceeded, Clock::time_point now)
  {
    if (!bound_exceeded) {
      return false;
    }
    if (last_ && now - *last_ < kInterval) {
      return false;
    }
    last_ = now;
    return true;
  }

private:
  std::optional<Clock::time_point> last_;
};

}  // namespace mru_transform

#endif
