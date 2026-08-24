// Tests for the water-line lever arm applied by sea_surface_estimator.
//
// The estimator averages odom.pose.pose.position.z -- the height of the
// *vehicle* frame -- and published it directly as the sea surface for the first
// fifteen months of the node's life. On BizzyBoat that is 0.030 m low; on a
// platform whose base_link sits at deck level it would be far worse.
//
// waterLineOffset rotates the lever arm by the vehicle's attitude before taking
// its vertical component. These tests pin the rotation, the degenerate
// quaternion cases that would otherwise produce NaN, and the magnitude of the
// attitude term that a scalar offset would have thrown away.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "mru_transform/water_line_offset.hpp"

using mru_transform::waterLineOffset;

namespace
{

geometry_msgs::msg::Quaternion rpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

geometry_msgs::msg::Quaternion raw(double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion out;
  out.x = x;
  out.y = y;
  out.z = z;
  out.w = w;
  return out;
}

// BizzyBoat: bizzy/waterline is 0.030 m above bizzy/base_link.
constexpr double kBizzyWaterLine = 0.030;

}  // namespace

TEST(WaterLineOffset, LevelVehicleReturnsTheLeverArm)
{
  const tf2::Vector3 lever(0.0, 0.0, kBizzyWaterLine);
  EXPECT_NEAR(waterLineOffset(lever, rpy(0, 0, 0)), kBizzyWaterLine, 1e-12);
}

TEST(WaterLineOffset, YawDoesNotChangeAVerticalLeverArm)
{
  const tf2::Vector3 lever(0.0, 0.0, kBizzyWaterLine);
  for (double yaw = 0.0; yaw < 2 * M_PI; yaw += M_PI / 4) {
    EXPECT_NEAR(waterLineOffset(lever, rpy(0, 0, yaw)), kBizzyWaterLine, 1e-12);
  }
}

TEST(WaterLineOffset, NinetyDegreePitchTipsTheLeverArmHorizontal)
{
  const tf2::Vector3 lever(0.0, 0.0, 1.0);
  EXPECT_NEAR(waterLineOffset(lever, rpy(0, M_PI / 2, 0)), 0.0, 1e-12);
}

TEST(WaterLineOffset, InvertedVehicleFlipsTheSign)
{
  const tf2::Vector3 lever(0.0, 0.0, 1.0);
  EXPECT_NEAR(waterLineOffset(lever, rpy(M_PI, 0, 0)), -1.0, 1e-12);
}

TEST(WaterLineOffset, ForeAftLeverArmContributesUnderPitch)
{
  // A lever arm ahead of the origin rises as the bow pitches up.
  const tf2::Vector3 lever(1.0, 0.0, 0.0);
  EXPECT_NEAR(waterLineOffset(lever, rpy(0, -M_PI / 6, 0)), std::sin(M_PI / 6), 1e-9);
  EXPECT_NEAR(waterLineOffset(lever, rpy(0, 0, 0)), 0.0, 1e-12);
}

TEST(WaterLineOffset, AttitudeTermIsNegligibleForBizzyButNotForALargeLeverArm)
{
  const double roll = 5.0 * M_PI / 180.0;

  // A purely vertical lever arm rolled by phi contributes z*cos(phi), so the
  // error a scalar offset would make is exactly z*(1 - cos(phi)). Pin that
  // closed form: one-sided bounds would also pass for the scalar
  // implementation this test claims to distinguish.
  const double shortening = 1.0 - std::cos(roll);

  const tf2::Vector3 small(0.0, 0.0, kBizzyWaterLine);
  const double small_error = kBizzyWaterLine - waterLineOffset(small, rpy(roll, 0, 0));
  EXPECT_NEAR(small_error, kBizzyWaterLine * shortening, 1e-12);
  EXPECT_LT(small_error, 0.0002);  // ~0.1 mm: a scalar offset would do here

  const tf2::Vector3 large(0.0, 0.0, 1.5);
  const double large_error = 1.5 - waterLineOffset(large, rpy(roll, 0, 0));
  EXPECT_NEAR(large_error, 1.5 * shortening, 1e-12);
  EXPECT_GT(large_error, 0.005);   // ~6 mm: it would not
}

TEST(WaterLineOffset, ZeroQuaternionFallsBackToTheUnrotatedLeverArm)
{
  // Default-constructed Odometry orientation: cannot be normalised, must not
  // produce NaN.
  const tf2::Vector3 lever(0.0, 0.0, kBizzyWaterLine);
  const double z = waterLineOffset(lever, raw(0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(std::isnan(z));
  EXPECT_NEAR(z, kBizzyWaterLine, 1e-12);
}

TEST(WaterLineOffset, UnnormalisedQuaternionIsNormalisedNotScaled)
{
  // Scaling a quaternion must not scale the lever arm.
  const tf2::Vector3 lever(0.0, 0.0, kBizzyWaterLine);
  auto q = rpy(0.1, 0.2, 0.3);
  auto scaled = raw(q.x * 5.0, q.y * 5.0, q.z * 5.0, q.w * 5.0);
  EXPECT_NEAR(waterLineOffset(lever, scaled), waterLineOffset(lever, q), 1e-12);
}

TEST(WaterLineOffset, ZeroLeverArmIsAlwaysZero)
{
  const tf2::Vector3 lever(0.0, 0.0, 0.0);
  EXPECT_NEAR(waterLineOffset(lever, rpy(0.3, -0.2, 1.1)), 0.0, 1e-12);
}

TEST(WaterLineOffset, MagnitudeIsBoundedByTheLeverArmLength)
{
  const tf2::Vector3 lever(0.4, -0.2, kBizzyWaterLine);
  for (double roll = -M_PI; roll <= M_PI; roll += M_PI / 8) {
    for (double pitch = -M_PI / 2; pitch <= M_PI / 2; pitch += M_PI / 8) {
      EXPECT_LE(std::fabs(waterLineOffset(lever, rpy(roll, pitch, 0.0))),
                lever.length() + 1e-9);
    }
  }
}

TEST(WaterLineOffset, NanQuaternionFallsBackToTheUnrotatedLeverArm)
{
  // A NaN attitude is not caught by a `length2() <= 0.0` guard -- every
  // comparison against NaN is false -- and a NaN that got through would
  // propagate into the tide that feeds soundings.
  const tf2::Vector3 lever(0.0, 0.0, kBizzyWaterLine);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int component = 0; component < 4; ++component) {
    auto q = rpy(0.1, 0.2, 0.3);
    switch (component) {
      case 0: q.x = nan; break;
      case 1: q.y = nan; break;
      case 2: q.z = nan; break;
      default: q.w = nan; break;
    }
    const double z = waterLineOffset(lever, q);
    EXPECT_FALSE(std::isnan(z)) << "component " << component;
    EXPECT_NEAR(z, kBizzyWaterLine, 1e-12) << "component " << component;
  }
}

TEST(WaterLineOffset, InfiniteQuaternionFallsBackToTheUnrotatedLeverArm)
{
  const tf2::Vector3 lever(0.0, 0.0, kBizzyWaterLine);
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_NEAR(waterLineOffset(lever, raw(inf, 0.0, 0.0, 1.0)), kBizzyWaterLine, 1e-12);
  EXPECT_NEAR(waterLineOffset(lever, raw(0.0, 0.0, 0.0, -inf)), kBizzyWaterLine, 1e-12);

  // Finite components whose squares overflow to infinity: length2() is inf, so
  // normalised() would divide by infinity and rotate by a zero quaternion.
  const double huge = 1e300;
  const double z = waterLineOffset(lever, raw(0.0, 0.0, 0.0, huge));
  EXPECT_TRUE(std::isfinite(z));
  EXPECT_NEAR(z, kBizzyWaterLine, 1e-12);
}
