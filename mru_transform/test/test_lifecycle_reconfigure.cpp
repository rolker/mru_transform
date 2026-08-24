// Regression tests for issue #34: a LifecycleNode whose on_configure declares
// its parameters unconditionally throws ParameterAlreadyDeclaredException on
// the second configure, so `cleanup` -> `configure` -- the transition an
// operator uses to reconfigure a boat between survey lines -- cannot complete.
//
// The fix guards each declare with has_parameter() rather than undeclaring in
// on_cleanup, so these tests pin both halves of that decision:
//   * the cycle completes without throwing, and
//   * a value set with `ros2 param set` while the node is configured SURVIVES
//     the cycle and is what the re-configure acts on.
// A node that undeclared on cleanup would pass the first and fail the second.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "mru_transform/nodes/chart_datum_node.hpp"
#include "mru_transform/nodes/nav_sat_fix_to_velocity.hpp"
#include "mru_transform/nodes/sea_surface_estimator.hpp"

namespace
{

constexpr std::uint8_t kUnconfigured =
  lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
constexpr std::uint8_t kInactive =
  lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

// configure -> cleanup -> configure. The second configure is what used to
// throw. The state assertions matter as much as the NO_THROW: were the
// exception ever swallowed by the FSM, the node would land in a state other
// than `inactive` rather than raising here.
template<typename NodeT>
void expect_reconfigure_cycle(const std::shared_ptr<NodeT> & node)
{
  ASSERT_EQ(node->get_current_state().id(), kUnconfigured);

  ASSERT_NO_THROW(node->configure());
  ASSERT_EQ(node->get_current_state().id(), kInactive);

  ASSERT_NO_THROW(node->cleanup());
  ASSERT_EQ(node->get_current_state().id(), kUnconfigured);

  ASSERT_NO_THROW(node->configure())
    << "second configure threw -- the declares are not guarded (issue #34)";
  EXPECT_EQ(node->get_current_state().id(), kInactive);
}

// Spin until `predicate` holds or the deadline passes. Returns what the
// predicate last reported, so a caller can assert on it.
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

void spin_for(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::milliseconds duration)
{
  spin_until(executor, [] {return false;}, duration);
}

sensor_msgs::msg::NavSatFix make_fix(double seconds, double latitude)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = rclcpp::Time(
    static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
  fix.header.frame_id = "gps";
  fix.latitude = latitude;
  fix.longitude = -70.6;
  fix.altitude = 0.0;
  return fix;
}

}  // namespace

class LifecycleReconfigureTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(LifecycleReconfigureTest, SeaSurfaceEstimatorReconfigures)
{
  expect_reconfigure_cycle(std::make_shared<SeaSurfaceEstimator>());
}

TEST_F(LifecycleReconfigureTest, SeaSurfaceEstimatorKeepsOperatorParameter)
{
  auto node = std::make_shared<SeaSurfaceEstimator>();

  ASSERT_EQ(node->get_current_state().id(), kUnconfigured);
  EXPECT_FALSE(node->has_parameter("sea_surface_frame"))
    << "parameters should not exist before the first configure";

  ASSERT_NO_THROW(node->configure());

  // What an operator does between lines: set the value on the running node.
  node->set_parameter(
    rclcpp::Parameter("sea_surface_frame", std::string("survey/map_tide")));
  node->set_parameter(rclcpp::Parameter("minimum_buffer_duration", 12.5));

  ASSERT_NO_THROW(node->cleanup());

  // Deliberately still declared after cleanup: undeclaring would discard the
  // value above, and it could not be re-staged while unconfigured because
  // setting an undeclared parameter is rejected.
  EXPECT_TRUE(node->has_parameter("sea_surface_frame"));
  EXPECT_EQ(
    node->get_parameter("sea_surface_frame").as_string(), "survey/map_tide");

  ASSERT_NO_THROW(node->configure());

  EXPECT_EQ(
    node->get_parameter("sea_surface_frame").as_string(), "survey/map_tide")
    << "the re-configure re-declared the parameter and lost the operator's value";
  EXPECT_DOUBLE_EQ(
    node->get_parameter("minimum_buffer_duration").as_double(), 12.5);
}

TEST_F(LifecycleReconfigureTest, ChartDatumNodeReconfigures)
{
  expect_reconfigure_cycle(std::make_shared<ChartDatumNode>());
}

TEST_F(LifecycleReconfigureTest, ChartDatumNodeKeepsOperatorParameter)
{
  auto node = std::make_shared<ChartDatumNode>();

  ASSERT_NO_THROW(node->configure());
  node->set_parameter(
    rclcpp::Parameter("chart_datum_frame", std::string("survey/mllw")));
  ASSERT_NO_THROW(node->cleanup());
  ASSERT_NO_THROW(node->configure());

  EXPECT_EQ(
    node->get_parameter("chart_datum_frame").as_string(), "survey/mllw");
}

// The parameter surviving is only half of it -- on_configure must also ACT on
// the surviving value. publish_rate is the cheapest observable proof: the node
// validates it and fails the transition, so setting it invalid on a configured
// node and re-configuring must fail. If the re-configure re-read the launch
// default instead, this would succeed.
TEST_F(LifecycleReconfigureTest, ChartDatumNodeReconfigureUsesTheNewValue)
{
  auto node = std::make_shared<ChartDatumNode>();

  ASSERT_NO_THROW(node->configure());
  ASSERT_EQ(node->get_current_state().id(), kInactive);

  node->set_parameter(rclcpp::Parameter("publish_rate", -1.0));
  ASSERT_NO_THROW(node->cleanup());
  ASSERT_NO_THROW(node->configure());

  EXPECT_EQ(node->get_current_state().id(), kUnconfigured)
    << "the re-configure did not read the value the operator set";
}

// A configure that returns FAILURE partway through its declares goes back to
// `unconfigured` WITHOUT on_cleanup running, and `cleanup` is not legal from
// `unconfigured` -- so nothing that released parameters during cleanup could
// ever run here. Correcting the bad value and re-configuring must work; before
// the fix the retry threw on the already-declared chart_datum_frame.
TEST_F(LifecycleReconfigureTest, ChartDatumNodeRecoversFromFailedConfigure)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("publish_rate", 0.0)});
  auto node = std::make_shared<ChartDatumNode>(options);

  ASSERT_NO_THROW(node->configure());
  ASSERT_EQ(node->get_current_state().id(), kUnconfigured)
    << "publish_rate 0 should have failed the transition";

  // The parameters declared before the validation are still declared, and the
  // node is unconfigured -- exactly the state an undeclare-on-cleanup fix
  // cannot reach.
  ASSERT_TRUE(node->has_parameter("chart_datum_frame"));

  node->set_parameter(rclcpp::Parameter("publish_rate", 1.0));

  ASSERT_NO_THROW(node->configure())
    << "retry after a failed configure threw (issue #34)";
  EXPECT_EQ(node->get_current_state().id(), kInactive);
}

TEST_F(LifecycleReconfigureTest, NavSatFixToVelocityReconfigures)
{
  expect_reconfigure_cycle(std::make_shared<NavSatFixToVelocity>());
}

TEST_F(LifecycleReconfigureTest, NavSatFixToVelocityKeepsOperatorParameter)
{
  auto node = std::make_shared<NavSatFixToVelocity>();

  ASSERT_NO_THROW(node->configure());
  node->set_parameter(rclcpp::Parameter("map_frame", std::string("survey/map")));
  node->set_parameter(rclcpp::Parameter("maximum_interval_seconds", 0.75));
  ASSERT_NO_THROW(node->cleanup());
  ASSERT_NO_THROW(node->configure());

  EXPECT_EQ(node->get_parameter("map_frame").as_string(), "survey/map");
  EXPECT_DOUBLE_EQ(
    node->get_parameter("maximum_interval_seconds").as_double(), 0.75);
}

// Two things at once, because they share a harness:
//   * an inactive node must not publish. velocity_publisher_ used to be an
//     rclcpp::Publisher, whose non-virtual publish() bypasses the lifecycle
//     activation gate entirely, and the callback did not check state.
//   * the first fix after a re-configure must not be differenced against a fix
//     from before the cleanup: with maximum_interval_ defaulting to 2 s, a
//     quick reconfigure would otherwise report a velocity averaged across the
//     gap. on_cleanup clears last_navsatfix_.
TEST_F(LifecycleReconfigureTest, NavSatFixToVelocityRespectsLifecycleState)
{
  auto node = std::make_shared<NavSatFixToVelocity>();
  auto peer = std::make_shared<rclcpp::Node>("nav_sat_fix_to_velocity_peer");

  auto fix_pub = peer->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
  std::vector<geometry_msgs::msg::TwistStamped> velocities;
  auto velocity_sub = peer->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/velocity", 10,
    [&velocities](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      velocities.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  executor.add_node(node->get_node_base_interface());

  ASSERT_NO_THROW(node->configure());
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return fix_pub->get_subscription_count() > 0;},
      std::chrono::seconds(15)))
    << "the node never subscribed to /fix";

  // Configured but NOT active: two fixes a valid interval apart must produce
  // nothing at all.
  fix_pub->publish(make_fix(100.0, 43.10));
  spin_for(executor, std::chrono::milliseconds(300));
  fix_pub->publish(make_fix(100.5, 43.11));
  spin_for(executor, std::chrono::milliseconds(300));
  EXPECT_TRUE(velocities.empty())
    << "an inactive node published " << velocities.size() << " velocity message(s)";

  ASSERT_NO_THROW(node->activate());
  fix_pub->publish(make_fix(101.0, 43.12));
  spin_for(executor, std::chrono::milliseconds(300));
  EXPECT_TRUE(velocities.empty()) << "the first fix cannot yield a velocity";

  fix_pub->publish(make_fix(101.5, 43.13));
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return velocities.size() == 1;}, std::chrono::seconds(5)))
    << "an active node published " << velocities.size()
    << " velocity message(s) for a valid pair of fixes";
  velocities.clear();

  // Cleanup and re-configure, then a fix 0.5 s later -- well inside the 2 s
  // maximum_interval_. Before the fix, last_navsatfix_ survived the cleanup and
  // this produced a velocity computed across the cleanup gap.
  ASSERT_NO_THROW(node->deactivate());
  ASSERT_NO_THROW(node->cleanup());
  ASSERT_NO_THROW(node->configure());
  ASSERT_NO_THROW(node->activate());
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return fix_pub->get_subscription_count() > 0;},
      std::chrono::seconds(15)))
    << "the node never re-subscribed to /fix";

  fix_pub->publish(make_fix(102.0, 43.14));
  spin_for(executor, std::chrono::milliseconds(500));
  EXPECT_TRUE(velocities.empty())
    << "a velocity was computed across the cleanup gap -- last_navsatfix_ "
       "survived on_cleanup";

  // ...and the node still works afterwards: the next pair does publish.
  fix_pub->publish(make_fix(102.5, 43.15));
  EXPECT_TRUE(
    spin_until(
      executor, [&] {return velocities.size() == 1;}, std::chrono::seconds(5)))
    << "the re-configured node stopped producing velocities";

  node->deactivate();
}
