// Tests for the TF lookup sea_surface_estimator uses to find the water line.
//
// The node itself was untested: the lookup's ARGUMENT ORDER decides the sign of
// a correction applied to every tide estimate that later feeds soundings, and
// nothing pinned it. Reversing the two frames does not fail, throw, or log --
// it silently doubles the error it was added to remove. These tests stand up a
// tf2_ros::Buffer with setTransform (no node, no spin) and pin the sign, the
// caching, the invalidation, and the failure paths.

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>

#include "mru_transform/water_line_lever_arm.hpp"
#include "mru_transform/water_line_offset.hpp"

using mru_transform::WaterLineLeverArm;
using Status = mru_transform::WaterLineLeverArm::Status;

namespace
{

constexpr char kVehicle[] = "bizzy/base_link";
constexpr char kWaterLine[] = "bizzy/waterline";

// BizzyBoat: the water line is 0.030 m ABOVE base_link, so odom z (a base_link
// height) is 0.030 m low as a sea surface and the correction is positive.
constexpr double kBizzyWaterLine = 0.030;

class WaterLineLeverArmTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    // A permanently empty buffer, used to prove that a call which must not
    // touch TF really does not: tf2_ros::Buffer::clear() does not drop static
    // transforms, so emptying the populated one is not a usable stand-in.
    empty_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    lever_arm_.setWaterLineFrame(kWaterLine);
  }

  // Publish `parent -> child` as a static transform, the way a URDF does.
  // Returns what tf2 made of it -- it validates before storing.
  bool setStatic(
    const std::string & parent, const std::string & child,
    double x, double y, double z)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.w = 1.0;
    return buffer_->setTransform(tf, "test", true);
  }

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::Buffer> empty_buffer_;
  WaterLineLeverArm lever_arm_;
};

}  // namespace

// The load-bearing test: the sign convention nothing else pins.
TEST_F(WaterLineLeverArmTest, LeverArmPointsFromTheVehicleUpToTheWaterLine)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));

  const auto result = lever_arm_.update(*buffer_, kVehicle);
  ASSERT_EQ(result.status, Status::Updated);
  ASSERT_TRUE(result.valid());

  // Positive: the water line is above base_link, so the estimator must ADD it
  // to odom z. A reversed lookupTransform would return -0.030 here and the
  // node would publish a sea surface 0.060 m low instead of correcting it.
  EXPECT_NEAR(result.lever_arm.z(), kBizzyWaterLine, 1e-12);
  EXPECT_GT(result.lever_arm.z(), 0.0);
  EXPECT_NEAR(result.lever_arm.x(), 0.0, 1e-12);
  EXPECT_NEAR(result.lever_arm.y(), 0.0, 1e-12);
  EXPECT_FALSE(result.changed);
}

// The same fact stated the way the estimator consumes it.
TEST_F(WaterLineLeverArmTest, AppliedOffsetRaisesTheEstimateToTheWaterLine)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_TRUE(lever_arm_.update(*buffer_, kVehicle).valid());

  geometry_msgs::msg::Quaternion level;
  level.w = 1.0;

  const double odom_z = -0.626;  // a plausible ellipsoidal base_link height
  const double corrected =
    odom_z + mru_transform::waterLineOffset(lever_arm_.leverArm(), level);
  EXPECT_NEAR(corrected, odom_z + kBizzyWaterLine, 1e-12);
  EXPECT_GT(corrected, odom_z);
}

// A deck-level origin: the water line is BELOW the vehicle frame, and the
// correction must then be negative.
TEST_F(WaterLineLeverArmTest, WaterLineBelowTheVehicleGivesANegativeLeverArm)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, -1.5));
  const auto result = lever_arm_.update(*buffer_, kVehicle);
  ASSERT_EQ(result.status, Status::Updated);
  EXPECT_NEAR(result.lever_arm.z(), -1.5, 1e-12);
}

TEST_F(WaterLineLeverArmTest, AnUnconfiguredFrameDisablesTheCorrection)
{
  WaterLineLeverArm unconfigured;
  EXPECT_FALSE(unconfigured.enabled());
  const auto result = unconfigured.update(*buffer_, kVehicle);
  EXPECT_EQ(result.status, Status::Disabled);
  EXPECT_FALSE(result.valid());
}

TEST_F(WaterLineLeverArmTest, AnEmptyVehicleFrameIsBlamedOnTheOdometryNotOnTf)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  const auto result = lever_arm_.update(*buffer_, "");
  EXPECT_EQ(result.status, Status::NoVehicleFrame);
  EXPECT_FALSE(result.valid());
  EXPECT_FALSE(lever_arm_.haveLeverArm());
}

TEST_F(WaterLineLeverArmTest, AMissingTransformFailsAndCachesNothing)
{
  const auto result = lever_arm_.update(*buffer_, kVehicle);
  EXPECT_EQ(result.status, Status::LookupFailed);
  EXPECT_FALSE(result.valid());
  EXPECT_FALSE(result.error.empty());
  EXPECT_FALSE(lever_arm_.haveLeverArm());
}

TEST_F(WaterLineLeverArmTest, ANonFiniteTransformNeverBecomesALeverArm)
{
  // tf2 refuses to store a NaN transform at all (TF_NAN_INPUT), so the frame
  // simply does not exist and the lookup fails. Pinned here because it is the
  // first of two layers keeping a NaN out of the tide: the class re-checks the
  // translation after the lookup, so a future TF that admitted one would still
  // be caught rather than silently corrected by NaN metres.
  EXPECT_FALSE(setStatic(kVehicle, kWaterLine, 0.0, 0.0,
    std::numeric_limits<double>::quiet_NaN()));

  const auto result = lever_arm_.update(*buffer_, kVehicle);
  EXPECT_EQ(result.status, Status::LookupFailed);
  EXPECT_FALSE(lever_arm_.haveLeverArm());
}

TEST_F(WaterLineLeverArmTest, ASecondCallIsServedFromTheCache)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_EQ(lever_arm_.update(*buffer_, kVehicle).status, Status::Updated);

  // Asking against an EMPTY buffer proves the second call looked nothing up.
  const auto cached = lever_arm_.update(*empty_buffer_, kVehicle);
  EXPECT_EQ(cached.status, Status::Cached);
  EXPECT_NEAR(cached.lever_arm.z(), kBizzyWaterLine, 1e-12);
}

TEST_F(WaterLineLeverArmTest, ADifferentVehicleFrameIsNotServedFromTheCache)
{
  // One tree, two candidate vehicle frames: a mast frame 1.2 m above base_link
  // sees the water line 1.2 m further down, so the cached value would be wrong
  // by that much if it were reused.
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_TRUE(setStatic(kVehicle, "bizzy/mast", 0.0, 0.0, 1.2));
  ASSERT_EQ(lever_arm_.update(*buffer_, kVehicle).status, Status::Updated);
  ASSERT_FALSE(lever_arm_.isCachedFor("bizzy/mast"));

  const auto result = lever_arm_.update(*buffer_, "bizzy/mast");
  EXPECT_EQ(result.status, Status::Updated);
  EXPECT_NEAR(result.lever_arm.z(), kBizzyWaterLine - 1.2, 1e-12);
  // Not flagged as "moved": a different vehicle frame is a different
  // measurement, not a static frame that shifted.
  EXPECT_FALSE(result.changed);
}

// The cross-confirmed finding: a cleanup -> configure with a new water-line
// frame must not keep serving the old frame's lever arm.
TEST_F(WaterLineLeverArmTest, ChangingTheWaterLineFrameInvalidatesTheCache)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_TRUE(setStatic(kVehicle, "bizzy/deck", 0.0, 0.0, 0.9));
  ASSERT_EQ(lever_arm_.update(*buffer_, kVehicle).status, Status::Updated);

  lever_arm_.setWaterLineFrame("bizzy/deck");
  EXPECT_FALSE(lever_arm_.haveLeverArm());

  const auto result = lever_arm_.update(*buffer_, kVehicle);
  EXPECT_EQ(result.status, Status::Updated);
  EXPECT_NEAR(result.lever_arm.z(), 0.9, 1e-12);
}

TEST_F(WaterLineLeverArmTest, SettingTheSameFrameKeepsTheCache)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_EQ(lever_arm_.update(*buffer_, kVehicle).status, Status::Updated);

  lever_arm_.setWaterLineFrame(kWaterLine);
  EXPECT_TRUE(lever_arm_.haveLeverArm());
}

TEST_F(WaterLineLeverArmTest, ResetForgetsTheCachedLeverArmButKeepsTheFrame)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_EQ(lever_arm_.update(*buffer_, kVehicle).status, Status::Updated);

  lever_arm_.reset();
  EXPECT_FALSE(lever_arm_.haveLeverArm());
  EXPECT_TRUE(lever_arm_.enabled());
  EXPECT_EQ(lever_arm_.waterLineFrame(), kWaterLine);
  EXPECT_FALSE(lever_arm_.isCachedFor(kVehicle));
}

TEST_F(WaterLineLeverArmTest, ARefreshReportsAFrameThatIsNotActuallyStatic)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_EQ(lever_arm_.update(*buffer_, kVehicle).status, Status::Updated);

  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine + 0.5));
  const auto refreshed = lever_arm_.update(*buffer_, kVehicle, /*force_refresh=*/true);
  EXPECT_EQ(refreshed.status, Status::Updated);
  EXPECT_TRUE(refreshed.changed);
  EXPECT_NEAR(refreshed.lever_arm.z(), kBizzyWaterLine + 0.5, 1e-12);

  // Sub-millimetre wobble is not worth shouting about.
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine + 0.5 + 1e-5));
  EXPECT_FALSE(lever_arm_.update(*buffer_, kVehicle, /*force_refresh=*/true).changed);
}

TEST_F(WaterLineLeverArmTest, AFailedRefreshKeepsTheLeverArmAlreadyLookedUp)
{
  ASSERT_TRUE(setStatic(kVehicle, kWaterLine, 0.0, 0.0, kBizzyWaterLine));
  ASSERT_EQ(lever_arm_.update(*buffer_, kVehicle).status, Status::Updated);

  // TF gone (the listener's buffer replaced on a re-configure, say).
  const auto result = lever_arm_.update(*empty_buffer_, kVehicle, /*force_refresh=*/true);
  EXPECT_EQ(result.status, Status::LookupFailed);
  EXPECT_TRUE(lever_arm_.haveLeverArm());
  EXPECT_NEAR(lever_arm_.leverArm().z(), kBizzyWaterLine, 1e-12);
}
