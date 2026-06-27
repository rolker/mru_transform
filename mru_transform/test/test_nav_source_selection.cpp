// Unit tests for selectNavigationSource() — strict priority-preference nav-source
// arbitration (unh_echoboats_project11#339).
//
// The regression this guards: a fresh but slower primary (SBG INS, ~4 Hz) was
// being out-voted by a faster lower-priority source (FCU, ~10 Hz) because the
// old arbitration adopted the newest fresh sample rather than the highest-
// priority fresh source. The fix stops at the first fresh source in priority
// order and never falls through to a lower-priority one while it is fresh.

#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include "mru_transform/navigation_source_selection.hpp"

using mru_transform::selectNavigationSource;

namespace
{
// Build a ROS-time stamp from seconds (all stamps share RCL_ROS_TIME so the
// comparisons/subtractions match what rclcpp::Time(header.stamp) produces).
rclcpp::Time t(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

const rclcpp::Duration kTimeout = rclcpp::Duration::from_seconds(0.5);
// Priority order: index 0 = SBG (primary), index 1 = FCU (fallback).
}  // namespace

// The core fix: when the FCU just published (now == FCU stamp) but the primary
// SBG is fresh, the FCU must NOT be selected even though it is newer.
TEST(NavSourceSelection, FreshPrimaryBlocksFasterFallback)
{
  // SBG sample 0.1 s old (fresh); FCU just arrived; we already published SBG's
  // 99.9 sample, so it carries no new data this tick.
  std::vector<rclcpp::Time> stamps{t(99.9), t(100.0)};
  EXPECT_EQ(selectNavigationSource(stamps, /*last_value=*/t(99.9), /*now=*/t(100.0), kTimeout), -1);
}

// A new SBG sample is adopted at index 0.
TEST(NavSourceSelection, NewPrimarySampleSelected)
{
  std::vector<rclcpp::Time> stamps{t(100.0), t(99.95)};
  EXPECT_EQ(selectNavigationSource(stamps, t(99.9), t(100.0), kTimeout), 0);
}

// Primary stale past the timeout -> fail over to the fresh fallback.
TEST(NavSourceSelection, FailoverToFallbackWhenPrimaryStale)
{
  // SBG 1.0 s old (stale, > 0.5 s); FCU just arrived.
  std::vector<rclcpp::Time> stamps{t(99.0), t(100.0)};
  EXPECT_EQ(selectNavigationSource(stamps, t(99.0), t(100.0), kTimeout), 1);
}

// Primary recovers: its first fresh sample after a failover is adopted again,
// even though the fallback had been publishing newer stamps meanwhile.
TEST(NavSourceSelection, PrimaryRecoversAfterFailover)
{
  std::vector<rclcpp::Time> stamps{t(101.0), t(100.95)};
  EXPECT_EQ(selectNavigationSource(stamps, /*last_value=*/t(100.95), t(101.0), kTimeout), 0);
}

// Fallback fresh but with no new sample (already published) -> no update.
TEST(NavSourceSelection, StalePrimaryFreshFallbackNoNewData)
{
  std::vector<rclcpp::Time> stamps{t(99.0), t(99.95)};
  EXPECT_EQ(selectNavigationSource(stamps, /*last_value=*/t(99.95), t(100.0), kTimeout), -1);
}

// Everything stale -> nothing selected.
TEST(NavSourceSelection, AllStale)
{
  std::vector<rclcpp::Time> stamps{t(99.0), t(99.4)};
  EXPECT_EQ(selectNavigationSource(stamps, t(99.0), t(100.0), kTimeout), -1);
}

// Boundary: a sample exactly at the timeout age counts as stale (age < timeout
// is the freshness test), so the loop falls through.
TEST(NavSourceSelection, TimeoutBoundaryIsStale)
{
  std::vector<rclcpp::Time> stamps{t(99.5)};  // exactly 0.5 s old
  EXPECT_EQ(selectNavigationSource(stamps, t(99.0), t(100.0), kTimeout), -1);
}

// Single fresh source with new data is adopted (default/legacy single-sensor case).
TEST(NavSourceSelection, SingleFreshSource)
{
  std::vector<rclcpp::Time> stamps{t(100.0)};
  EXPECT_EQ(selectNavigationSource(stamps, t(99.0), t(100.0), kTimeout), 0);
}

// No sources configured -> nothing selected.
TEST(NavSourceSelection, EmptyList)
{
  std::vector<rclcpp::Time> stamps{};
  EXPECT_EQ(selectNavigationSource(stamps, t(99.0), t(100.0), kTimeout), -1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
