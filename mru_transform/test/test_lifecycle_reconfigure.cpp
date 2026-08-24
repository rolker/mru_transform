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
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mru_transform/nodes/chart_datum_node.hpp"
#include "mru_transform/nodes/nav_sat_fix_to_velocity.hpp"
#include "mru_transform/nodes/sea_surface_estimator.hpp"
#include "mru_transform/nodes/tide_copier.hpp"

namespace
{

constexpr std::uint8_t kUnconfigured =
  lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
constexpr std::uint8_t kInactive =
  lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
constexpr std::uint8_t kFinalized =
  lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;

// Isolation. The live pub/sub cases below prove NEGATIVES -- "an inactive node
// published nothing" -- which any unrelated traffic on the machine can break,
// and `get_subscription_count()` waits, which an external subscriber can
// satisfy vacuously. Two layers keep that from happening:
//   * a dedicated ROS_DOMAIN_ID and localhost-only discovery, set on the test
//     in CMakeLists.txt, so nothing outside this process is even discovered;
//   * every node -- including the peer -- in this namespace, with the absolute
//     /tf that tide_copier hardcodes remapped into it, so a domain collision
//     still cannot cross-talk.
// Topic names in this file are therefore RELATIVE on purpose; an absolute
// "/tf" or "/fix" here would defeat both layers.
constexpr char kTestNamespace[] = "/mru_transform_lifecycle_test";

rclcpp::NodeOptions isolated_options()
{
  rclcpp::NodeOptions options;
  options.arguments(
    {
      "--ros-args",
      "-r", std::string("__ns:=") + kTestNamespace,
      "-r", std::string("/tf:=") + kTestNamespace + "/tf",
    });
  return options;
}

// A plain peer node in the same namespace, so its relative topic names resolve
// to the same place as the node under test's.
std::shared_ptr<rclcpp::Node> make_peer(const std::string & name)
{
  return std::make_shared<rclcpp::Node>(name, kTestNamespace);
}

// configure -> cleanup -> configure. The second configure is the one that
// breaks. NOTE: rclcpp_lifecycle CATCHES an exception thrown by a transition
// callback, logs "Caught exception in callback for transition 10" and reports
// ERROR, so the ParameterAlreadyDeclaredException does NOT escape configure().
// The state assertion, not the NO_THROW, is what detects the bug -- every
// re-configure in this file is followed by one.
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

tf2_msgs::msg::TFMessage make_tide_tf(
  const std::string & parent, const std::string & child, double z)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.z = z;
  transform.transform.rotation.w = 1.0;

  tf2_msgs::msg::TFMessage message;
  message.transforms.push_back(transform);
  return message;
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
  expect_reconfigure_cycle(std::make_shared<SeaSurfaceEstimator>(isolated_options()));
}

TEST_F(LifecycleReconfigureTest, SeaSurfaceEstimatorKeepsOperatorParameter)
{
  auto node = std::make_shared<SeaSurfaceEstimator>(isolated_options());

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
  ASSERT_EQ(node->get_current_state().id(), kInactive)
    << "the second configure did not complete (issue #34)";

  EXPECT_EQ(
    node->get_parameter("sea_surface_frame").as_string(), "survey/map_tide")
    << "the re-configure re-declared the parameter and lost the operator's value";
  EXPECT_DOUBLE_EQ(
    node->get_parameter("minimum_buffer_duration").as_double(), 12.5);
}

// The two buffer durations bound the averaging window and were the only
// numeric parameters in either node with no validation at all. A non-positive
// maximum prunes the buffer empty on every sample -- including the sample just
// inserted -- and the callback then dereferenced odometry_buffer_.begin() on an
// empty map, which is undefined behaviour, not a missing publication. A minimum
// at or above the maximum can never be satisfied, so the node would sit there
// silently never publishing a tide. Both must fail the transition, and the
// corrected value must be what the retry reads.
TEST_F(LifecycleReconfigureTest, SeaSurfaceEstimatorRejectsUnusableBufferDurations)
{
  {
    auto options = isolated_options();
    options.parameter_overrides(
      {rclcpp::Parameter("maximum_buffer_duration", -1.0)});
    auto node = std::make_shared<SeaSurfaceEstimator>(options);

    ASSERT_NO_THROW(node->configure());
    ASSERT_EQ(node->get_current_state().id(), kUnconfigured)
      << "a negative maximum_buffer_duration was accepted; the odometry "
         "callback would dereference an empty buffer";

    node->set_parameter(rclcpp::Parameter("maximum_buffer_duration", 30.0));
    ASSERT_NO_THROW(node->configure())
      << "retry after the failed configure threw (issue #34)";
    EXPECT_EQ(node->get_current_state().id(), kInactive);
  }

  {
    auto options = isolated_options();
    options.parameter_overrides(
      {
        rclcpp::Parameter("minimum_buffer_duration", 30.0),
        rclcpp::Parameter("maximum_buffer_duration", 30.0),
      });
    auto node = std::make_shared<SeaSurfaceEstimator>(options);

    ASSERT_NO_THROW(node->configure());
    EXPECT_EQ(node->get_current_state().id(), kUnconfigured)
      << "a minimum_buffer_duration at the maximum was accepted; the window "
         "can never be long enough and the node would never publish";
  }
}

TEST_F(LifecycleReconfigureTest, ChartDatumNodeReconfigures)
{
  expect_reconfigure_cycle(std::make_shared<ChartDatumNode>(isolated_options()));
}

TEST_F(LifecycleReconfigureTest, ChartDatumNodeKeepsOperatorParameter)
{
  auto node = std::make_shared<ChartDatumNode>(isolated_options());

  ASSERT_NO_THROW(node->configure());
  node->set_parameter(
    rclcpp::Parameter("chart_datum_frame", std::string("survey/mllw")));
  ASSERT_NO_THROW(node->cleanup());
  ASSERT_NO_THROW(node->configure());
  ASSERT_EQ(node->get_current_state().id(), kInactive)
    << "the second configure did not complete (issue #34)";

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
  auto node = std::make_shared<ChartDatumNode>(isolated_options());

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
  rclcpp::NodeOptions options = isolated_options();
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

// on_cleanup must release what on_configure created, and an endpoint is the
// only part of that a peer can observe: a released subscription stops counting
// against a publisher, a released publisher stops counting against a
// subscriber. Without these two cases, deleting the resets in either node's
// on_cleanup leaves the whole file green -- and a teardown nothing tests is how
// this class of bug comes back. (tide_copier's and nav_sat_fix_to_velocity's
// resets are already pinned by their live lifecycle cases, which fail if a
// cleaned-up node keeps receiving.)
TEST_F(LifecycleReconfigureTest, SeaSurfaceEstimatorCleanupReleasesItsEndpoints)
{
  auto node = std::make_shared<SeaSurfaceEstimator>(isolated_options());
  auto peer = make_peer("sea_surface_estimator_peer");

  auto odom_pub = peer->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  auto tide_sub = peer->create_subscription<std_msgs::msg::Float64>(
    "tide_estimate", rclcpp::QoS(1).transient_local(),
    [](std_msgs::msg::Float64::SharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  executor.add_node(node->get_node_base_interface());

  ASSERT_NO_THROW(node->configure());
  ASSERT_TRUE(
    spin_until(
      executor,
      [&] {
        return odom_pub->get_subscription_count() == 1 &&
        tide_sub->get_publisher_count() == 1;
      },
      std::chrono::seconds(15)))
    << "configure did not create both endpoints (odom subscribers: "
    << odom_pub->get_subscription_count() << ", tide_estimate publishers: "
    << tide_sub->get_publisher_count() << ")";

  ASSERT_NO_THROW(node->cleanup());
  EXPECT_TRUE(
    spin_until(
      executor,
      [&] {
        return odom_pub->get_subscription_count() == 0 &&
        tide_sub->get_publisher_count() == 0;
      },
      std::chrono::seconds(15)))
    << "on_cleanup did not release both endpoints (odom subscribers: "
    << odom_pub->get_subscription_count() << ", tide_estimate publishers: "
    << tide_sub->get_publisher_count() << ")";
}

TEST_F(LifecycleReconfigureTest, ChartDatumNodeCleanupReleasesItsPublishers)
{
  auto node = std::make_shared<ChartDatumNode>(isolated_options());
  auto peer = make_peer("chart_datum_peer");

  const auto latched = rclcpp::QoS(1).transient_local();
  auto mllw_sub = peer->create_subscription<std_msgs::msg::Float64>(
    "mllw_offset", latched, [](std_msgs::msg::Float64::SharedPtr) {});
  auto mhhw_sub = peer->create_subscription<std_msgs::msg::Float64>(
    "mhhw_offset", latched, [](std_msgs::msg::Float64::SharedPtr) {});
  auto source_sub = peer->create_subscription<std_msgs::msg::String>(
    "datum_source", latched, [](std_msgs::msg::String::SharedPtr) {});

  auto publisher_counts = [&] {
      return mllw_sub->get_publisher_count() + mhhw_sub->get_publisher_count() +
             source_sub->get_publisher_count();
    };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  executor.add_node(node->get_node_base_interface());

  ASSERT_NO_THROW(node->configure());
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return publisher_counts() == 3;}, std::chrono::seconds(15)))
    << "configure did not create all three latched publishers (saw "
    << publisher_counts() << " of 3)";

  ASSERT_NO_THROW(node->cleanup());
  EXPECT_TRUE(
    spin_until(
      executor, [&] {return publisher_counts() == 0;}, std::chrono::seconds(15)))
    << "on_cleanup did not release the latched publishers (" << publisher_counts()
    << " still up) -- a cleaned-up node still latches a datum for late subscribers";
}

// `shutdown` from `active` is the transition the other cases never take, and
// it is the one with no teardown behind it: it runs on_shutdown ONLY -- not
// on_deactivate, not on_cleanup -- and no node in this package overrides
// on_shutdown. chart_datum_node's timers are therefore still armed in
// `finalized`, and publish_callback used to run unguarded, so a finalized node
// kept putting out datum_source AND the map -> chart_datum transform on /tf --
// the frame every sounding is reduced against.
//
// datum_source is the observable half because it is the one branch of
// publish_callback that does not need a resolved datum (the TF branches need an
// earth -> base_link lookup this in-process fixture cannot supply). Both are
// behind the same single state check at the top of the callback, so pinning
// this pins the transform too. (#34)
TEST_F(LifecycleReconfigureTest, ChartDatumNodeStopsPublishingWhenFinalized)
{
  auto options = isolated_options();
  // 20 Hz so the observation window below spans many periods rather than one.
  options.parameter_overrides({rclcpp::Parameter("publish_rate", 20.0)});
  auto node = std::make_shared<ChartDatumNode>(options);
  auto peer = make_peer("chart_datum_shutdown_peer");

  std::vector<std_msgs::msg::String> sources;
  auto source_sub = peer->create_subscription<std_msgs::msg::String>(
    "datum_source", rclcpp::QoS(20).transient_local(),
    [&sources](std_msgs::msg::String::SharedPtr msg) {sources.push_back(*msg);});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  executor.add_node(node->get_node_base_interface());

  ASSERT_NO_THROW(node->configure());
  ASSERT_NO_THROW(node->activate());
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return sources.size() >= 2;}, std::chrono::seconds(5)))
    << "an active node published " << sources.size()
    << " datum_source message(s); the timer never ran";

  ASSERT_NO_THROW(node->shutdown());
  ASSERT_EQ(node->get_current_state().id(), kFinalized);

  // Drain whatever was already in flight when the transition happened, then
  // watch a fresh window: 700 ms is fourteen publish periods.
  spin_for(executor, std::chrono::milliseconds(300));
  sources.clear();
  spin_for(executor, std::chrono::milliseconds(700));

  EXPECT_TRUE(sources.empty())
    << "a FINALIZED node published " << sources.size()
    << " datum_source message(s) -- the publish timer survives `shutdown` from "
       "`active` (on_deactivate and on_cleanup are both skipped), so "
       "map -> chart_datum is still going out on /tf too";
}

TEST_F(LifecycleReconfigureTest, NavSatFixToVelocityReconfigures)
{
  expect_reconfigure_cycle(std::make_shared<NavSatFixToVelocity>(isolated_options()));
}

TEST_F(LifecycleReconfigureTest, NavSatFixToVelocityKeepsOperatorParameter)
{
  auto node = std::make_shared<NavSatFixToVelocity>(isolated_options());

  ASSERT_NO_THROW(node->configure());
  node->set_parameter(rclcpp::Parameter("map_frame", std::string("survey/map")));
  node->set_parameter(rclcpp::Parameter("maximum_interval_seconds", 0.75));
  ASSERT_NO_THROW(node->cleanup());
  ASSERT_NO_THROW(node->configure());
  ASSERT_EQ(node->get_current_state().id(), kInactive)
    << "the second configure did not complete (issue #34)";

  EXPECT_EQ(node->get_parameter("map_frame").as_string(), "survey/map");
  EXPECT_DOUBLE_EQ(
    node->get_parameter("maximum_interval_seconds").as_double(), 0.75);
}

// Two things at once, because they share a harness:
//   * an inactive node must not publish. velocity_publisher_ used to be an
//     rclcpp::Publisher, whose non-virtual publish() bypasses the lifecycle
//     activation gate entirely, and the callback did not check state.
//   * the first fix after a re-configure -- or after a re-activation -- must
//     not be differenced against a fix from before the gap: with
//     maximum_interval_ defaulting to 2 s, a quick cycle would otherwise report
//     a velocity averaged across an interval the node was muted for.
//     on_cleanup and on_deactivate both clear last_navsatfix_.
TEST_F(LifecycleReconfigureTest, NavSatFixToVelocityRespectsLifecycleState)
{
  auto node = std::make_shared<NavSatFixToVelocity>(isolated_options());
  auto peer = make_peer("nav_sat_fix_to_velocity_peer");

  auto fix_pub = peer->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
  std::vector<geometry_msgs::msg::TwistStamped> velocities;
  auto velocity_sub = peer->create_subscription<geometry_msgs::msg::TwistStamped>(
    "velocity", 10,
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
    << "the node never subscribed to fix";

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
  ASSERT_EQ(node->get_current_state().id(), kInactive)
    << "the second configure did not complete (issue #34)";
  ASSERT_NO_THROW(node->activate());
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return fix_pub->get_subscription_count() > 0;},
      std::chrono::seconds(15)))
    << "the node never re-subscribed to fix";

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
  velocities.clear();

  // The same gap without a cleanup: deactivate, re-activate, and feed a fix
  // 0.5 s after the last one the node accepted -- well inside the 2 s
  // maximum_interval_. The ACTIVE guard alone does not cover this: it drops the
  // fixes arriving while inactive but leaves the pre-deactivation fix in the
  // window, so the first fix after re-activation was reported as a velocity
  // averaged across the muted gap. on_deactivate clears last_navsatfix_.
  ASSERT_NO_THROW(node->deactivate());
  ASSERT_NO_THROW(node->activate());

  fix_pub->publish(make_fix(103.0, 43.16));
  spin_for(executor, std::chrono::milliseconds(500));
  EXPECT_TRUE(velocities.empty())
    << "a velocity was computed across the deactivation gap -- "
       "last_navsatfix_ survived on_deactivate";

  fix_pub->publish(make_fix(103.5, 43.17));
  EXPECT_TRUE(
    spin_until(
      executor, [&] {return velocities.size() == 1;}, std::chrono::seconds(5)))
    << "the re-activated node stopped producing velocities";

  node->deactivate();
}

TEST_F(LifecycleReconfigureTest, TideCopierReconfigures)
{
  expect_reconfigure_cycle(std::make_shared<TideCopier>(isolated_options()));
}

TEST_F(LifecycleReconfigureTest, TideCopierKeepsOperatorParameter)
{
  auto node = std::make_shared<TideCopier>(isolated_options());

  ASSERT_NO_THROW(node->configure());
  node->set_parameter(
    rclcpp::Parameter("output_map_tide_frame", std::string("survey/map_tide")));
  ASSERT_NO_THROW(node->cleanup());
  ASSERT_NO_THROW(node->configure());
  ASSERT_EQ(node->get_current_state().id(), kInactive)
    << "the second configure did not complete (issue #34)";

  EXPECT_EQ(
    node->get_parameter("output_map_tide_frame").as_string(),
    "survey/map_tide");
}

// tide_copier held its /tf publisher as an rclcpp::Publisher, whose
// non-virtual publish() bypasses the activation gate, and never released its
// /tf subscription on cleanup -- so a deactivated or cleaned-up node kept
// copying map_tide into /tf. map_tide is the tide applied to every sounding,
// so it has to follow the node's lifecycle state.
//
// The last phase doubles as the value-survival check with teeth: the frame the
// operator set while the node was configured is the frame that comes out after
// the cleanup -> configure cycle.
TEST_F(LifecycleReconfigureTest, TideCopierRespectsLifecycleState)
{
  auto node = std::make_shared<TideCopier>(isolated_options());
  auto peer = make_peer("tide_copier_peer");

  auto tf_pub = peer->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
  std::vector<geometry_msgs::msg::TransformStamped> copies;
  auto tf_sub = peer->create_subscription<tf2_msgs::msg::TFMessage>(
    "tf", 10,
    [&copies](tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & transform : msg->transforms) {
        // Ignore the input this test publishes; keep only the node's copies.
        if (transform.header.frame_id != "in/map") {
          copies.push_back(transform);
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(peer);
  executor.add_node(node->get_node_base_interface());

  ASSERT_NO_THROW(node->configure());
  // The peer's own subscription matches its publisher, so the node's makes two.
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return tf_pub->get_subscription_count() >= 2;},
      std::chrono::seconds(15)))
    << "the node never subscribed to tf";

  tf_pub->publish(make_tide_tf("in/map", "in/map_tide", 1.5));
  spin_for(executor, std::chrono::milliseconds(400));
  EXPECT_TRUE(copies.empty())
    << "a configured-but-inactive node copied the tide into /tf";

  ASSERT_NO_THROW(node->activate());
  tf_pub->publish(make_tide_tf("in/map", "in/map_tide", 1.5));
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return copies.size() == 1;}, std::chrono::seconds(5)))
    << "an active node did not copy the tide";
  EXPECT_EQ(copies.front().header.frame_id, "out/map");
  EXPECT_EQ(copies.front().child_frame_id, "out/map_tide");
  copies.clear();

  ASSERT_NO_THROW(node->deactivate());
  tf_pub->publish(make_tide_tf("in/map", "in/map_tide", 1.6));
  spin_for(executor, std::chrono::milliseconds(400));
  EXPECT_TRUE(copies.empty())
    << "a deactivated node kept copying the tide into /tf";

  // The reconfigure workflow this issue exists to enable, end to end: set the
  // output frame on the running node, cycle, and the copy comes out renamed.
  node->set_parameter(
    rclcpp::Parameter("output_map_tide_frame", std::string("survey/map_tide")));
  ASSERT_NO_THROW(node->cleanup());
  tf_pub->publish(make_tide_tf("in/map", "in/map_tide", 1.7));
  spin_for(executor, std::chrono::milliseconds(400));
  EXPECT_TRUE(copies.empty())
    << "a cleaned-up node kept copying the tide into /tf";

  ASSERT_NO_THROW(node->configure());
  ASSERT_EQ(node->get_current_state().id(), kInactive)
    << "the second configure did not complete (issue #34)";
  ASSERT_NO_THROW(node->activate());
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return tf_pub->get_subscription_count() >= 2;},
      std::chrono::seconds(15)))
    << "the node never re-subscribed to tf";

  tf_pub->publish(make_tide_tf("in/map", "in/map_tide", 1.8));
  ASSERT_TRUE(
    spin_until(
      executor, [&] {return copies.size() == 1;}, std::chrono::seconds(5)))
    << "the re-configured node did not copy the tide";
  EXPECT_EQ(copies.front().child_frame_id, "survey/map_tide")
    << "the re-configure did not use the frame the operator set";

  node->deactivate();
}
