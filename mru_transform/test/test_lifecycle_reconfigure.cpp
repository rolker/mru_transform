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

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

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
