// mru_transform#49: a pre-defined map origin (map_origin_latitude /
// map_origin_longitude), reset_map_frame semantics with and without one, and
// the throttled far-from-origin WARN (map_maximum_height_error, see #48).

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mru_transform/map_origin.hpp"
#include "mru_transform/mru_transform.hpp"

using namespace std::chrono_literals;
using mru_transform::exceedsFlatPlaneBound;
using mru_transform::flatPlaneHeightError;
using mru_transform::validateMapOrigin;
using mru_transform::validateMaxHeightError;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// Anchor used by the tests, and a first fix ~3.3 km away from it, so a bug that
// anchors at the fix cannot pass the tolerance below.
constexpr double kOriginLat = 43.0;
constexpr double kOriginLon = -70.0;
constexpr double kFixLat = 43.03;
constexpr double kFixLon = -70.04;
constexpr double kTolerance = 1e-9;  // degrees; the anchor is stored, not computed
}  // namespace

// ---- pure helpers ----------------------------------------------------------

TEST(MapOriginValidation, UnsetIsNullopt)
{
  EXPECT_FALSE(validateMapOrigin(std::nullopt, std::nullopt).has_value());
}

TEST(MapOriginValidation, ValidPairHasAltitudeExactlyZero)
{
  const auto origin = validateMapOrigin(43.1, -70.2);
  ASSERT_TRUE(origin.has_value());
  EXPECT_DOUBLE_EQ(origin->latitude, 43.1);
  EXPECT_DOUBLE_EQ(origin->longitude, -70.2);
  EXPECT_EQ(origin->altitude, 0.0);
}

TEST(MapOriginValidation, BoundariesAccepted)
{
  EXPECT_NO_THROW(validateMapOrigin(90.0, 180.0));
  EXPECT_NO_THROW(validateMapOrigin(-90.0, -180.0));
}

TEST(MapOriginValidation, OnlyOneSetThrowsNamingTheMissingOne)
{
  try {
    validateMapOrigin(43.0, std::nullopt);
    FAIL() << "expected throw";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("map_origin_longitude"), std::string::npos);
  }
  try {
    validateMapOrigin(std::nullopt, -70.0);
    FAIL() << "expected throw";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("map_origin_latitude"), std::string::npos);
  }
}

TEST(MapOriginValidation, NonFiniteOrOutOfRangeThrows)
{
  EXPECT_THROW(validateMapOrigin(kNaN, -70.0), std::invalid_argument);
  EXPECT_THROW(validateMapOrigin(43.0, kNaN), std::invalid_argument);
  EXPECT_THROW(validateMapOrigin(kInf, -70.0), std::invalid_argument);
  EXPECT_THROW(validateMapOrigin(43.0, -kInf), std::invalid_argument);
  EXPECT_THROW(validateMapOrigin(91.0, -70.0), std::invalid_argument);
  EXPECT_THROW(validateMapOrigin(-90.5, -70.0), std::invalid_argument);
  EXPECT_THROW(validateMapOrigin(43.0, -181.0), std::invalid_argument);
  EXPECT_THROW(validateMapOrigin(43.0, 180.5), std::invalid_argument);
}

TEST(FlatPlaneBound, HeightErrorMatchesIssue48Figures)
{
  EXPECT_NEAR(flatPlaneHeightError(3570.0), 1.0, 0.01);
  EXPECT_NEAR(flatPlaneHeightError(5000.0), 2.0, 0.1);
  EXPECT_NEAR(flatPlaneHeightError(10000.0), 7.85, 0.1);
  EXPECT_EQ(flatPlaneHeightError(0.0), 0.0);
}

TEST(FlatPlaneBound, ThresholdBoundaryIsHorizontalHypot)
{
  // 0.5 m <-> d = sqrt(2 R 0.5) ~ 2524 m.
  EXPECT_FALSE(exceedsFlatPlaneBound(2500.0, 0.0, 0.5));
  EXPECT_TRUE(exceedsFlatPlaneBound(2550.0, 0.0, 0.5));
  // 3-4-5: hypot(1500, 2000) = 2500
  EXPECT_FALSE(exceedsFlatPlaneBound(1500.0, 2000.0, 0.5));
  EXPECT_TRUE(exceedsFlatPlaneBound(1800.0, 1800.0, 0.5));  // ~2546 m
}

TEST(FlatPlaneBound, MaxErrorValidation)
{
  EXPECT_DOUBLE_EQ(validateMaxHeightError(0.5), 0.5);
  EXPECT_THROW(validateMaxHeightError(0.0), std::invalid_argument);
  EXPECT_THROW(validateMaxHeightError(-1.0), std::invalid_argument);
  EXPECT_THROW(validateMaxHeightError(kNaN), std::invalid_argument);
  EXPECT_THROW(validateMaxHeightError(kInf), std::invalid_argument);
}

// ---- node-level ------------------------------------------------------------

namespace
{
std::mutex g_log_mu;
std::vector<std::string> g_warnings;
rcutils_logging_output_handler_t g_previous_handler = nullptr;

void capture_handler(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (severity == RCUTILS_LOG_SEVERITY_WARN) {
    va_list copy;
    va_copy(copy, *args);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    std::lock_guard<std::mutex> lock(g_log_mu);
    g_warnings.push_back(buffer);
  }
  if (g_previous_handler) {
    g_previous_handler(location, severity, name, timestamp, format, args);
  }
}

size_t flat_plane_warnings()
{
  std::lock_guard<std::mutex> lock(g_log_mu);
  size_t count = 0;
  for (const auto & w : g_warnings) {
    if (w.find("estimated height error") != std::string::npos) {
      ++count;
    }
  }
  return count;
}

geographic_msgs::msg::GeoPointStamped fix(double lat, double lon)
{
  geographic_msgs::msg::GeoPointStamped p;
  p.header.stamp = rclcpp::Time(100, 0);
  p.header.frame_id = "base_link";
  p.position.latitude = lat;
  p.position.longitude = lon;
  p.position.altitude = std::numeric_limits<double>::quiet_NaN();
  return p;
}
}  // namespace

class MapOriginNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    g_previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_handler);
    {
      std::lock_guard<std::mutex> lock(g_log_mu);
      g_warnings.clear();
    }
  }
  void TearDown() override
  {
    rcutils_logging_set_output_handler(g_previous_handler);
    node_.reset();
    rclcpp::shutdown();
  }

  // Constructs a node (own namespace per test) with parameter overrides. The
  // far-from-origin WARN throttle is per instance, so tests are independent.
  rclcpp::Node::SharedPtr make_node(std::vector<rclcpp::Parameter> overrides)
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<rclcpp::Node>(
      "mru_transform", std::string("map_origin_test_") + info->name(), options);
    return node_;
  }

  static std::vector<rclcpp::Parameter> origin_params()
  {
    return {rclcpp::Parameter("map_origin_latitude", kOriginLat),
      rclcpp::Parameter("map_origin_longitude", kOriginLon)};
  }

  std::pair<bool, std::string> call_reset(rclcpp::Node::SharedPtr node)
  {
    auto client = node->create_client<std_srvs::srv::Trigger>("reset_map_frame");
    if (!client->wait_for_service(10s)) {
      ADD_FAILURE() << "reset_map_frame service not available";
      return {false, ""};
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (rclcpp::spin_until_future_complete(node, future, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "reset_map_frame call did not complete";
      return {false, ""};
    }
    auto response = future.get();
    return {response->success, response->message};
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(MapOriginNodeTest, ConfiguredOriginIsUsedInsteadOfTheFirstFix)
{
  auto node = make_node(origin_params());
  mru_transform::MRUTransform mru(node);
  EXPECT_FALSE(mru.mapOrigin().has_value());

  mru.updatePosition(fix(kFixLat, kFixLon));

  const auto origin = mru.mapOrigin();
  ASSERT_TRUE(origin.has_value());
  EXPECT_NEAR(origin->latitude, kOriginLat, kTolerance);
  EXPECT_NEAR(origin->longitude, kOriginLon, kTolerance);
  EXPECT_EQ(origin->altitude, 0.0);
  EXPECT_GT(std::abs(origin->latitude - kFixLat), 1e-3);  // the fix really is elsewhere
}

TEST_F(MapOriginNodeTest, UnsetAnchorsAtTheFirstFixWithAltitudeZero)
{
  auto node = make_node({});
  mru_transform::MRUTransform mru(node);
  mru.updatePosition(fix(kFixLat, kFixLon));

  const auto origin = mru.mapOrigin();
  ASSERT_TRUE(origin.has_value());
  EXPECT_NEAR(origin->latitude, kFixLat, kTolerance);
  EXPECT_NEAR(origin->longitude, kFixLon, kTolerance);
  EXPECT_EQ(origin->altitude, 0.0);

  // A later fix does not move the anchor.
  mru.updatePosition(fix(kOriginLat, kOriginLon));
  EXPECT_NEAR(mru.mapOrigin()->latitude, kFixLat, kTolerance);
}

TEST_F(MapOriginNodeTest, InvalidConfigurationThrowsFromTheConstructor)
{
  // Only one of the pair.
  EXPECT_THROW(
    mru_transform::MRUTransform(make_node({rclcpp::Parameter("map_origin_latitude", 43.0)})),
    std::invalid_argument);
  node_.reset();
  EXPECT_THROW(
    mru_transform::MRUTransform(make_node({rclcpp::Parameter("map_origin_longitude", -70.0)})),
    std::invalid_argument);
  node_.reset();
  // Out of range / non-finite.
  EXPECT_THROW(
    mru_transform::MRUTransform(make_node({rclcpp::Parameter("map_origin_latitude", 91.0),
        rclcpp::Parameter("map_origin_longitude", -70.0)})),
    std::invalid_argument);
  node_.reset();
  EXPECT_THROW(
    mru_transform::MRUTransform(make_node({rclcpp::Parameter("map_origin_latitude", 43.0),
        rclcpp::Parameter("map_origin_longitude", kNaN)})),
    std::invalid_argument);
  node_.reset();
  // Threshold.
  EXPECT_THROW(
    mru_transform::MRUTransform(make_node({rclcpp::Parameter("map_maximum_height_error", 0.0)})),
    std::invalid_argument);
  node_.reset();
  EXPECT_THROW(
    mru_transform::MRUTransform(make_node({rclcpp::Parameter("map_maximum_height_error", kNaN)})),
    std::invalid_argument);
}

TEST_F(MapOriginNodeTest, IntegerOriginAndThresholdAreAcceptedAndUsedExactly)
{
  // A YAML `latitude: 43` / `map_maximum_height_error: 1` must not stop the node.
  auto node = make_node({rclcpp::Parameter("map_origin_latitude", 43),
      rclcpp::Parameter("map_origin_longitude", -70),
      rclcpp::Parameter("map_maximum_height_error", 1000)});
  mru_transform::MRUTransform mru(node);
  EXPECT_EQ(mru.maxHeightError(), 1000.0);
  mru.updatePosition(fix(kFixLat, kFixLon));
  ASSERT_TRUE(mru.mapOrigin().has_value());
  EXPECT_EQ(mru.mapOrigin()->latitude, 43.0);
  EXPECT_EQ(mru.mapOrigin()->longitude, -70.0);
}

TEST_F(MapOriginNodeTest, NonNumericTypesAreRefusedWithTheParameterName)
{
  try {
    mru_transform::MRUTransform mru(make_node({rclcpp::Parameter("map_origin_latitude", "43.0"),
        rclcpp::Parameter("map_origin_longitude", -70.0)}));
    FAIL() << "expected throw";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("map_origin_latitude"), std::string::npos) << e.what();
    EXPECT_NE(std::string(e.what()).find("number"), std::string::npos) << e.what();
  }
  node_.reset();
  EXPECT_THROW(
    mru_transform::MRUTransform(make_node({rclcpp::Parameter("map_maximum_height_error", true)})),
    std::invalid_argument);
}

TEST_F(MapOriginNodeTest, ResetAfterAnchoringReanchorsAtTheCurrentFix)
{
  auto node = make_node(origin_params());
  mru_transform::MRUTransform mru(node);
  mru.updatePosition(fix(kFixLat, kFixLon));
  ASSERT_NEAR(mru.mapOrigin()->latitude, kOriginLat, kTolerance);

  const auto first = call_reset(node);
  EXPECT_TRUE(first.first);
  EXPECT_NE(first.second.find("no longer applies"), std::string::npos) << first.second;
  EXPECT_FALSE(mru.mapOrigin().has_value());

  mru.updatePosition(fix(kFixLat, kFixLon));
  EXPECT_NEAR(mru.mapOrigin()->latitude, kFixLat, kTolerance);
  EXPECT_NEAR(mru.mapOrigin()->longitude, kFixLon, kTolerance);

  // A second reset stays on first-fix behaviour, and no longer mentions the origin.
  const auto second = call_reset(node);
  EXPECT_TRUE(second.first);
  EXPECT_EQ(second.second.find("no longer applies"), std::string::npos) << second.second;
  mru.updatePosition(fix(kOriginLat, kOriginLon));
  EXPECT_NEAR(mru.mapOrigin()->latitude, kOriginLat, kTolerance);
  EXPECT_NEAR(mru.mapOrigin()->longitude, kOriginLon, kTolerance);
}

TEST_F(MapOriginNodeTest, ResetBeforeAnyFixKeepsTheConfiguredOrigin)
{
  auto node = make_node(origin_params());
  mru_transform::MRUTransform mru(node);

  const auto result = call_reset(node);
  EXPECT_TRUE(result.first);
  EXPECT_NE(result.second.find("is kept"), std::string::npos) << result.second;
  EXPECT_FALSE(mru.mapOrigin().has_value());

  mru.updatePosition(fix(kFixLat, kFixLon));
  ASSERT_TRUE(mru.mapOrigin().has_value());
  EXPECT_NEAR(mru.mapOrigin()->latitude, kOriginLat, kTolerance);
  EXPECT_NEAR(mru.mapOrigin()->longitude, kOriginLon, kTolerance);
}

TEST_F(MapOriginNodeTest, ResetBeforeAnyFixWithoutOriginAnchorsAtFirstFix)
{
  auto node = make_node({});
  mru_transform::MRUTransform mru(node);
  const auto result = call_reset(node);
  EXPECT_TRUE(result.first);
  EXPECT_NE(result.second.find("not anchored yet"), std::string::npos) << result.second;
  mru.updatePosition(fix(kFixLat, kFixLon));
  EXPECT_NEAR(mru.mapOrigin()->latitude, kFixLat, kTolerance);
}

TEST_F(MapOriginNodeTest, RuntimeSetOfReadOnceParametersIsRejected)
{
  auto node = make_node(origin_params());
  mru_transform::MRUTransform mru(node);

  for (const auto & parameter : {rclcpp::Parameter("map_origin_latitude", 10.0),
      rclcpp::Parameter("map_origin_longitude", 10.0),
      rclcpp::Parameter("map_maximum_height_error", 5.0)})
  {
    const auto result = node->set_parameter(parameter);
    EXPECT_FALSE(result.successful) << parameter.get_name();
    EXPECT_NE(result.reason.find("restart"), std::string::npos) << result.reason;
  }
  // Unrelated parameters can still be set.
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("odom_topic", "odom_other")).successful);
  // The origin in effect is unchanged.
  mru.updatePosition(fix(kFixLat, kFixLon));
  EXPECT_NEAR(mru.mapOrigin()->latitude, kOriginLat, kTolerance);
}

// The WARN decision (bound exceeded + per-instance throttle on an injected
// clock) is driven directly, so these do not depend on wall time or on other
// tests.
TEST_F(MapOriginNodeTest, DefaultThresholdIsHalfAMetre)
{
  auto node = make_node({});
  mru_transform::MRUTransform mru(node);
  EXPECT_EQ(mru.maxHeightError(), 0.5);
  // 0.5 m <-> ~2524 m.
  const auto t0 = mru_transform::FarFromOriginWarning::Clock::now();
  EXPECT_FALSE(mru.shouldWarnFarFromOrigin(2500.0, 0.0, t0));
  EXPECT_TRUE(mru.shouldWarnFarFromOrigin(2550.0, 0.0, t0));
}

TEST_F(MapOriginNodeTest, FarWarnDecisionHonoursTheConfiguredThreshold)
{
  using Clock = mru_transform::FarFromOriginWarning::Clock;
  const auto t0 = Clock::now();
  {
    // 3 km: above the default 0.5 m, below a configured 2.0 m.
    auto node = make_node({rclcpp::Parameter("map_maximum_height_error", 2.0)});
    mru_transform::MRUTransform mru(node);
    EXPECT_FALSE(mru.shouldWarnFarFromOrigin(3000.0, 0.0, t0));
    node_.reset();
  }
  // 1 km: below the default, above a configured 0.05 m.
  auto node = make_node({rclcpp::Parameter("map_maximum_height_error", 0.05)});
  mru_transform::MRUTransform mru(node);
  EXPECT_TRUE(mru.shouldWarnFarFromOrigin(1000.0, 0.0, t0));
}

TEST_F(MapOriginNodeTest, FarWarnRepeatsEveryTenSecondsWhileExceeded)
{
  using Clock = mru_transform::FarFromOriginWarning::Clock;
  auto node = make_node({});
  mru_transform::MRUTransform mru(node);
  const auto t0 = Clock::now();
  EXPECT_TRUE(mru.shouldWarnFarFromOrigin(5000.0, 0.0, t0));
  EXPECT_FALSE(mru.shouldWarnFarFromOrigin(5000.0, 0.0, t0 + 9s));
  EXPECT_FALSE(mru.shouldWarnFarFromOrigin(10.0, 0.0, t0 + 9s));  // within bound: never
  EXPECT_TRUE(mru.shouldWarnFarFromOrigin(5000.0, 0.0, t0 + 10s));
  EXPECT_FALSE(mru.shouldWarnFarFromOrigin(5000.0, 0.0, t0 + 19s));
  EXPECT_TRUE(mru.shouldWarnFarFromOrigin(5000.0, 0.0, t0 + 20s));
}

TEST_F(MapOriginNodeTest, FarFixLogsOneWarnAndStillPublishesTheTransform)
{
  // Wiring: updatePosition logs the WARN through the decision above, and
  // nothing is refused. A first-fix anchor, then a fix ~1 km out against a
  // 0.05 m threshold (the default would not warn there).
  auto node = make_node({rclcpp::Parameter("map_maximum_height_error", 0.05)});
  mru_transform::MRUTransform mru(node);
  mru.updatePosition(fix(kOriginLat, kOriginLon));
  EXPECT_EQ(flat_plane_warnings(), 0u);  // the anchoring fix itself is at distance 0

  double published_north = std::numeric_limits<double>::quiet_NaN();
  auto listener = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100), [&published_north](tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & t : msg->transforms) {
        if (t.child_frame_id == "base_link_north_up") {
          published_north = t.transform.translation.y;
        }
      }
    });
  // Discovery first, with no far fix sent: however long it takes on a loaded
  // runner, it cannot run into the WARN interval.
  const auto discovery_deadline = std::chrono::steady_clock::now() + 30s;
  while (listener->get_publisher_count() == 0 &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GT(listener->get_publisher_count(), 0u) << "/tf broadcaster never discovered";

  // Far fixes are repeated (matching may complete on the publisher side a little
  // after this side), but only for less than one WARN interval from the first,
  // so exactly one WARN is the only correct outcome.
  const auto first_far = std::chrono::steady_clock::now();
  const auto deadline =
    first_far + mru_transform::FarFromOriginWarning::kInterval - 2s;
  while (std::isnan(published_north) && std::chrono::steady_clock::now() < deadline) {
    mru.updatePosition(fix(kOriginLat + 0.009, kOriginLon));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(flat_plane_warnings(), 1u);  // repeated fixes within 10 s: one line
  EXPECT_NEAR(published_north, 1000.0, 10.0);  // 0.009 deg of latitude ~ 1000 m
  const auto text = [] {
      std::lock_guard<std::mutex> lock(g_log_mu);
      for (const auto & w : g_warnings) {
        if (w.find("estimated height error") != std::string::npos) {
          return w;
        }
      }
      return std::string();
    }();
  EXPECT_NE(text.find("reset_map_frame"), std::string::npos) << text;
  EXPECT_EQ(text.find("http"), std::string::npos) << text;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
