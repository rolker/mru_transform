// Regression test for issue #23: SensorBase::subscribeCheck() must stop polling
// once a sensor has subscribed. rclcpp wall timers always repeat (there is no
// one-shot option like the ROS 1 timer this logic was ported from), so before
// the fix the check timer kept firing every period and re-ran subscribe(),
// tearing down and recreating the live subscription ~1 Hz. Under rmw_zenoh that
// raced in-flight samples and produced "SubscriberCallback triggered over ..."
// ERROR spam.

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "mru_transform/orientation_sensor.hpp"

using namespace std::chrono_literals;

namespace
{

// Counts how many times subscribeCheck() invokes subscribe() and exposes the
// check timer's state. Overriding the (private, virtual) subscribe() is legal
// for a derived class; subscribeCheck() dispatches to it through the base. The
// override creates a real subscription so subscribeCheck() takes its
// "subscribed" branch, exercising the actual base-class control flow.
class CountingOrientationSensor : public mru_transform::OrientationSensor
{
public:
  using mru_transform::OrientationSensor::OrientationSensor;

  int subscribe_calls() const { return subscribe_calls_; }

  bool timer_active() const
  {
    return subscribe_check_timer_ && !subscribe_check_timer_->is_canceled();
  }

private:
  bool subscribe(const std::vector<std::string> & topic_types) override
  {
    ++subscribe_calls_;
    for (const auto & topic_type : topic_types) {
      if (topic_type == "sensor_msgs/msg/Imu") {
        subs_.imu = rclcpp::create_subscription<sensor_msgs::msg::Imu>(
          node_, topic_, rclcpp::SensorDataQoS(),
          [](sensor_msgs::msg::Imu::SharedPtr) {});
        return true;
      }
    }
    return false;
  }

  int subscribe_calls_{0};
};

void spin_for(
  rclcpp::executors::SingleThreadedExecutor & exec,
  std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }
}

}  // namespace

class SubscribeOnceTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(SubscribeOnceTest, SubscribesOnceThenStops)
{
  auto node = std::make_shared<rclcpp::Node>("subscribe_once_test");

  // SensorBase reads this in its constructor and only arms the check timer when
  // the topic is non-empty, so it must be set before the sensor is created.
  node->declare_parameter("sensors.default.topics.orientation", "test_imu");

  // Advertise the topic with a supported type so subscribeCheck() can resolve a
  // type and take its subscribe() path.
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>(
    "test_imu", rclcpp::SensorDataQoS());

  auto sensor = std::make_shared<CountingOrientationSensor>(
    *node, "default", mru_transform::OrientationSensor::CallbackType());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // Phase 1: wait for the first successful subscribe (timer period is 1 s).
  // Generous headroom so a loaded CI box doesn't fail discovery before the
  // first timer tick; the assertion below distinguishes "never subscribed".
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (sensor->subscribe_calls() < 1 &&
    std::chrono::steady_clock::now() < deadline && rclcpp::ok())
  {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_EQ(sensor->subscribe_calls(), 1) << "sensor never subscribed";
  EXPECT_FALSE(sensor->timer_active())
    << "check timer should be cancelled after subscribing";

  // Phase 2: spin past several more timer periods. subscribe() must not run
  // again — the pre-fix bug re-subscribed every period.
  spin_for(exec, 3500ms);

  EXPECT_EQ(sensor->subscribe_calls(), 1)
    << "subscribe() ran more than once — the check timer was not stopped";
  EXPECT_FALSE(sensor->timer_active());
}
