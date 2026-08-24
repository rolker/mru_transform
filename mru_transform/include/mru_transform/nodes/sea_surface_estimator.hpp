#ifndef MRU_TRANSFORM_NODES_SEA_SURFACE_ESTIMATOR_HPP
#define MRU_TRANSFORM_NODES_SEA_SURFACE_ESTIMATOR_HPP

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "mru_transform/water_line_lever_arm.hpp"
#include "mru_transform/water_line_offset.hpp"

class SeaSurfaceEstimator : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SeaSurfaceEstimator(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :rclcpp_lifecycle::LifecycleNode("sea_surface_estimator", options)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
    // Every declare is guarded: declare_parameter() throws
    // ParameterAlreadyDeclaredException on a second configure, and the
    // parameters are deliberately NOT undeclared in on_cleanup so that a value
    // an operator set with `ros2 param set` survives a cleanup -> configure
    // cycle. get_parameter() below therefore reads the operator's value on a
    // re-configure and the launch/default value on the first one. (#34)
    if (!has_parameter("sea_surface_frame")) {
      declare_parameter("sea_surface_frame", sea_surface_frame_);
    }
    get_parameter("sea_surface_frame", sea_surface_frame_);

    if (!has_parameter("minimum_buffer_duration")) {
      declare_parameter("minimum_buffer_duration", minimum_buffer_duration_);
    }
    get_parameter("minimum_buffer_duration", minimum_buffer_duration_);

    if (!has_parameter("maximum_buffer_duration")) {
      declare_parameter("maximum_buffer_duration", maximum_buffer_duration_);
    }
    get_parameter("maximum_buffer_duration", maximum_buffer_duration_);

    // Both durations bound the averaging window and neither was validated,
    // unlike tide_range_margin below and publish_rate / recalc_interval in
    // chart_datum_node. The two rejections below are rejections for DIFFERENT
    // reasons, and it is worth being exact about which is which.
    //
    // NON-FINITE is its own rejection and has to be checked explicitly, first.
    // This comment used to claim the non-negative check below covered it. It
    // does not: `!(NaN >= 0.0)` is true so NaN failed that check, but
    // `+inf >= 0.0` is TRUE, so an infinite maximum configured cleanly -- and
    // rclcpp::Duration::from_seconds(+inf) casts inf to int64_t (INT64_MIN in
    // practice), after which `now - Duration(INT64_MIN)` in the prune below
    // throws std::overflow_error out of odometry_callback, out of
    // rclcpp::spin, and kills the node on its first odometry message. A NaN
    // MINIMUM slipped through in mirror image: `NaN < 0.0` is false so it is
    // not clamped, `NaN >= maximum` is false so it is not rejected, and
    // `buffer_duration.seconds() < NaN` is false forever -- so the node
    // publishes exactly the unsmoothed single sample the (0, 0) branch below
    // exists to refuse. std::isfinite() on both closes both.
    //
    // A NEGATIVE maximum is a correctness bug. The prune erases while
    // `begin()->first < now - maximum`, so a negative maximum puts the cutoff
    // *after* `now` and erases the sample just inserted; the callback then
    // dereferenced odometry_buffer_.begin() on an empty map, which is
    // undefined behaviour, not a missing publication.
    //
    // A ZERO maximum is NOT that, and the earlier claim that it emptied the
    // buffer was simply wrong: the comparison is `<`, so at maximum == 0 the
    // cutoff equals `now`, the just-inserted key is exactly `now`, and the
    // sample is KEPT. (minimum 0, maximum 0) was a working configuration
    // before this change -- "publish the instantaneous height, no smoothing".
    // It is rejected below as POLICY, not because of any mechanism: this node
    // exists to average, and an unsmoothed single-sample tide on the path that
    // feeds every sounding is not a configuration anyone should be running.
    // Every other minimum >= maximum pair could additionally never be
    // satisfied, so the node would sit there silently never publishing a tide.
    //
    // Both fail the transition rather than being clamped: configure can be
    // retried, and the parameters survive the failure so the corrected value
    // is what the retry reads. (#34)
    if (!std::isfinite(minimum_buffer_duration_) ||
      !std::isfinite(maximum_buffer_duration_))
    {
      RCLCPP_ERROR(
        get_logger(),
        "minimum_buffer_duration (%f) and maximum_buffer_duration (%f) must "
        "both be finite. An infinite duration overflows "
        "rclcpp::Duration::from_seconds() and then throws std::overflow_error "
        "out of the odometry callback; a NaN passes silently through every "
        "comparison it appears in",
        minimum_buffer_duration_, maximum_buffer_duration_);
      return CallbackReturn::FAILURE;
    }
    if (maximum_buffer_duration_ < 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "maximum_buffer_duration must be a non-negative number (got %.3f); "
        "the retention prune would erase every sample as it arrived",
        maximum_buffer_duration_);
      return CallbackReturn::FAILURE;
    }
    if (minimum_buffer_duration_ < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'minimum_buffer_duration' is negative (%f); clamping to 0.0",
        minimum_buffer_duration_);
      minimum_buffer_duration_ = 0.0;
    }
    if (minimum_buffer_duration_ >= maximum_buffer_duration_) {
      RCLCPP_ERROR(
        get_logger(),
        "minimum_buffer_duration (%.3f) must be < maximum_buffer_duration "
        "(%.3f). (0, 0) does work -- it publishes a single unsmoothed sample -- "
        "but this node exists to average, so it is rejected as policy; any "
        "other minimum >= maximum could never be satisfied at all and the node "
        "would never publish",
        minimum_buffer_duration_, maximum_buffer_duration_);
      return CallbackReturn::FAILURE;
    }

    if (!has_parameter("water_line_frame")) {
      declare_parameter("water_line_frame", water_line_frame_);
    }
    get_parameter("water_line_frame", water_line_frame_);
    // Registering the frame here also invalidates any lever arm cached under a
    // previous configuration, so a cleanup -> configure with a different frame
    // cannot keep applying the old one.
    water_line_lever_arm_.setWaterLineFrame(water_line_frame_);
    if (water_line_frame_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'water_line_frame' is unset: the sea surface will be "
        "published at the vehicle frame, not the water line. Set it to the "
        "URDF water-line frame to correct the offset.");
    }

    if (!has_parameter("chart_datum_frame")) {
      declare_parameter("chart_datum_frame", chart_datum_frame_);
    }
    get_parameter("chart_datum_frame", chart_datum_frame_);

    if (!has_parameter("mhhw_frame")) {
      declare_parameter("mhhw_frame", mhhw_frame_);
    }
    get_parameter("mhhw_frame", mhhw_frame_);

    if (!has_parameter("tide_range_margin")) {
      declare_parameter("tide_range_margin", tide_range_margin_);
    }
    get_parameter("tide_range_margin", tide_range_margin_);
    if (tide_range_margin_ < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'tide_range_margin' is negative (%f); clamping to 0.0",
        tide_range_margin_);
      tide_range_margin_ = 0.0;
    }

    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // One-argument form: the listener creates its OWN internal node with
    // default options and spins it on its own thread, so it subscribes to the
    // global /tf and /tf_static -- this node's namespace and remap rules do not
    // apply to it. Usual TF-listener behaviour; called out because it is the
    // one part of this node that a test cannot namespace.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto latched_qos = rclcpp::QoS(1).transient_local();
    tide_estimate_pub_ = create_publisher<std_msgs::msg::Float64>(
      "tide_estimate", latched_qos);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SeaSurfaceEstimator::odometry_callback, this, std::placeholders::_1));

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn  on_activate(const rclcpp_lifecycle::State & state) override
  {
    return LifecycleNode::on_activate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
  {
    // Release everything on_configure created, in reverse: the subscription
    // goes first so no callback can be running against members torn down
    // below it. That ordering is only sufficient under a single-threaded
    // executor -- shared_ptr::reset() itself synchronizes nothing, so with a
    // multi-threaded executor a callback already dispatched could still be
    // running here. Every main() in this package uses a single-threaded
    // executor; the installed headers now take NodeOptions, so a composed
    // future user has to keep to that or add real synchronization. (The
    // one-argument TransformListener does spin a thread of its own regardless
    // of the executor, so "single-threaded" is a claim about this node's own
    // callbacks; that thread only fills the buffer, and the reset ordering
    // below is what makes it safe.)
    odometry_subscription_.reset();
    tide_estimate_pub_.reset();
    transform_broadcaster_.reset();

    // The listener holds a reference to the buffer and fills it from its own
    // spin thread, so it must be torn down FIRST: releasing the buffer while
    // the listener is still running is a use-after-free.
    tf_listener_.reset();
    tf_buffer_.reset();

    // Nothing cached here survives the configuration that produced it.
    water_line_lever_arm_.reset();
    have_lookup_attempt_ = false;
    logged_lever_arm_ = false;
    odometry_buffer_.clear();
    buffered_child_frame_id_.clear();

    // Parameters are intentionally left declared: see on_configure. An
    // operator's `ros2 param set` value must survive the cycle, and a value
    // cannot be staged on an unconfigured node because setting an undeclared
    // parameter is rejected. (#34)
    return LifecycleNode::on_cleanup(state);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    return;

    rclcpp::Time now = msg->header.stamp;

    // A non-finite height is not an estimate of anything: buffering it would
    // poison the average, and neither the plausibility bound nor any consumer
    // downstream of `tide_estimate` can reject a NaN once it is in there.
    if (!std::isfinite(msg->pose.pose.position.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Odometry on '%s' carries a non-finite position.z; dropping the sample.",
        msg->child_frame_id.c_str());
      return;
    }

    // Every buffered sample is a height of the frame named by child_frame_id,
    // and the water-line lever arm is specific to that frame. Mixing frames
    // would average heights of different points and correct them all by one
    // frame's lever arm, so a change starts a fresh window.
    if (!odometry_buffer_.empty() && msg->child_frame_id != buffered_child_frame_id_) {
      RCLCPP_INFO(
        get_logger(),
        "Odometry child_frame_id changed from '%s' to '%s'; restarting the sea "
        "surface averaging window.",
        buffered_child_frame_id_.c_str(), msg->child_frame_id.c_str());
      odometry_buffer_.clear();
    }
    buffered_child_frame_id_ = msg->child_frame_id;

    odometry_buffer_[now] = msg;

    // Drop expired messages from the buffer.
    auto oldest_time_to_keep = now - rclcpp::Duration::from_seconds(maximum_buffer_duration_);

    while(!odometry_buffer_.empty() &&
          odometry_buffer_.begin()->first < oldest_time_to_keep)
    {
      odometry_buffer_.erase(odometry_buffer_.begin());
    }

    // Defensive: on_configure rejects a maximum_buffer_duration that could
    // prune the just-inserted sample away, so the buffer holds at least that
    // one. begin() on an empty map would be undefined behaviour, not an empty
    // average, so the invariant is checked rather than assumed.
    if (odometry_buffer_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Odometry buffer emptied by the retention prune; publishing nothing.");
      return;
    }

    // Make sure we have a long enough history before publishing the transform.
    auto buffer_duration = now - odometry_buffer_.begin()->first;
    if (buffer_duration.seconds() < minimum_buffer_duration_)
    {
      return;
    }

    // The buffered Z values are heights of the *vehicle* frame. The sea
    // surface is the water line, so each sample is lifted by the water-line
    // lever arm rotated into the parent frame by that sample's attitude.
    const bool have_offset = update_water_line_lever_arm(msg->child_frame_id);

    // Configured but unresolved is not a degraded estimate, it is the wrong
    // number: publishing here would put out — and, on a transient_local topic,
    // LATCH for every late subscriber — a tide this node has just logged as
    // wrong by the whole lever arm, and a TF that resolved mid-line would then
    // step the entire rolling window by that lever arm. Publish nothing until
    // the correction the operator asked for can actually be applied.
    if (water_line_lever_arm_.enabled() && !have_offset) {
      return;
    }

    double sum = 0.0;
    for (const auto & odometry : odometry_buffer_)
    {
      double z = odometry.second->pose.pose.position.z;
      if (have_offset) {
        z += mru_transform::waterLineOffset(
          water_line_lever_arm_.leverArm(), odometry.second->pose.pose.orientation);
      }
      sum += z;
    }
    double average = sum / odometry_buffer_.size();

    // Defensive: samples are finite on ingest and waterLineOffset is total, so
    // this should be unreachable short of an overflow.
    if (!std::isfinite(average)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Sea surface estimate is not finite; publishing nothing.");
      return;
    }

    // Always publish raw estimate for debugging, even if the plausibility
    // bound below rejects it. NOTE: that means `tide_estimate` can carry a
    // value that `map_tide` does not — the topic is the raw estimate, the
    // frame is the accepted one. Consumers that need the accepted tide must
    // read the frame, not the topic.
    std_msgs::msg::Float64 tide_msg;
    tide_msg.data = average;
    tide_estimate_pub_->publish(tide_msg);

    // Check if the estimated tide is within a plausible range using
    // the chart datum (MLLW) and MHHW frames published by chart_datum_node.
    if (!chart_datum_frame_.empty() && !mhhw_frame_.empty()) {
      if (is_out_of_range(average, msg->header.frame_id)) {
        return;
      }
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = sea_surface_frame_;
    transform.transform.translation.z = average;
    transform.transform.rotation.w = 1.0;

    transform_broadcaster_->sendTransform(transform);
  }

private:
  // Resolve the vehicle-frame-to-water-line lever arm, from cache where
  // possible. It comes from the URDF via a static transform, so one successful
  // lookup normally holds for the life of the node; the cache is nevertheless
  // refreshed every kLeverArmRefreshPeriod seconds so that a `water_line_frame`
  // pointed at a frame that is NOT static shows up as a logged change rather
  // than a value silently frozen at whatever TF held first. Failed lookups are
  // retried no more often than kLeverArmRetryPeriod, so a misconfiguration
  // costs one lookup per second and not one per odometry message.
  //
  // Returns false when no water-line frame is configured (in which case the
  // estimator keeps its historical behaviour) or when no lever arm is
  // available; the caller publishes nothing in the latter case.
  bool update_water_line_lever_arm(const std::string & vehicle_frame)
  {
    using Status = mru_transform::WaterLineLeverArm::Status;

    if (!water_line_lever_arm_.enabled()) {
      return false;
    }

    const bool cached = water_line_lever_arm_.isCachedFor(vehicle_frame);
    const rclcpp::Time now = get_clock()->now();
    if (have_lookup_attempt_) {
      const double since = (now - last_lookup_attempt_).seconds();
      const double period = cached ? kLeverArmRefreshPeriod : kLeverArmRetryPeriod;
      // A negative interval means the clock jumped backwards (a bag replay, a
      // sim time reset); treat that as due rather than waiting it out.
      if (since >= 0.0 && since < period) {
        return cached;
      }
    }
    last_lookup_attempt_ = now;
    have_lookup_attempt_ = true;

    const auto result = water_line_lever_arm_.update(*tf_buffer_, vehicle_frame, cached);

    switch (result.status) {
      case Status::Updated:
        if (result.changed) {
          RCLCPP_WARN(
            get_logger(),
            "Water line '%s' MOVED to [%.3f, %.3f, %.3f] from '%s'. This frame "
            "is expected to be static (URDF); a moving one makes the tide "
            "estimate step. Using the new value.",
            water_line_frame_.c_str(), result.lever_arm.x(), result.lever_arm.y(),
            result.lever_arm.z(), vehicle_frame.c_str());
        } else if (!logged_lever_arm_) {
          RCLCPP_INFO(
            get_logger(),
            "Water line '%s' is [%.3f, %.3f, %.3f] from '%s'; correcting the sea "
            "surface estimate by that lever arm.",
            water_line_frame_.c_str(), result.lever_arm.x(), result.lever_arm.y(),
            result.lever_arm.z(), vehicle_frame.c_str());
          logged_lever_arm_ = true;
        }
        return true;

      case Status::Cached:
        return true;

      case Status::NoVehicleFrame:
        // Point at the odometry publisher, not at TF: TF cannot be asked about
        // a frame with no name.
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Odometry on '%s' has an empty child_frame_id, so there is no frame to "
          "resolve the water line '%s' against. Fix the odometry publisher. No "
          "sea surface is being published.",
          odometry_subscription_->get_topic_name(), water_line_frame_.c_str());
        return false;

      case Status::LookupFailed:
        if (water_line_lever_arm_.haveLeverArm()) {
          // A refresh failed. The previously looked-up lever arm is static, so
          // keeping it is right; say so rather than dropping the estimate.
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "Cannot refresh '%s' -> '%s' (%s); continuing with the lever arm "
            "looked up earlier.",
            water_line_frame_.c_str(), vehicle_frame.c_str(), result.error.c_str());
          return true;
        }
        // Configured but unavailable is a real misconfiguration, not a quiet
        // degradation: say so on every throttle interval. The first lookup
        // happens only after minimum_buffer_duration_ of odometry, by which
        // time a static transform from the URDF is long since available, so
        // this does not fire spuriously at start-up.
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Cannot look up '%s' -> '%s' (%s); the water-line correction cannot be "
          "applied, so NO sea surface is being published.",
          water_line_frame_.c_str(), vehicle_frame.c_str(), result.error.c_str());
        return false;

      case Status::Disabled:
      default:
        return false;
    }
  }

  // Check if the estimated sea surface Z is outside the plausible
  // tidal range. Returns true if the estimate should be rejected.
  bool is_out_of_range(double estimated_z, const std::string & frame_id)
  {
    geometry_msgs::msg::TransformStamped mllw_tf, mhhw_tf;
    try {
      mllw_tf = tf_buffer_->lookupTransform(
        frame_id, chart_datum_frame_, tf2::TimePointZero);
      mhhw_tf = tf_buffer_->lookupTransform(
        frame_id, mhhw_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      // If datum frames aren't available, don't filter
      return false;
    }

    double mllw_z = mllw_tf.transform.translation.z;
    double mhhw_z = mhhw_tf.transform.translation.z;

    // Tidal range with configurable margin (e.g., 2.0x for storm surge)
    double tidal_range = std::abs(mhhw_z - mllw_z);
    double margin = tidal_range * tide_range_margin_;
    double lower_bound = mllw_z - margin;
    double upper_bound = mhhw_z + margin;

    if (estimated_z < lower_bound || estimated_z > upper_bound) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Tide estimate %.2f m is outside plausible range "
        "[%.2f, %.2f] (MLLW=%.2f, MHHW=%.2f, margin=%.1fx) — suppressing",
        estimated_z, lower_bound, upper_bound,
        mllw_z, mhhw_z, tide_range_margin_);
      return true;
    }

    return false;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  std::map<rclcpp::Time, nav_msgs::msg::Odometry::SharedPtr> odometry_buffer_;

  // Duration in seconds of messages the buffer should have before
  // the transform is published.
  double minimum_buffer_duration_ = 5.0;

  // Maximum duration in seconds to keep in the buffer.
  double maximum_buffer_duration_ = 30.0;

  std::string sea_surface_frame_ = "map_tide";

  // Datum frame names for tide range validation
  std::string chart_datum_frame_ = "chart_datum";
  std::string mhhw_frame_ = "chart_datum_mhhw";

  // Multiplier on tidal range for rejection margin.
  // 2.0 means accept up to 2x the tidal range beyond MHHW/below MLLW
  // (accounts for storm surge, extreme tides).
  double tide_range_margin_ = 2.0;

  // Water line frame, from the URDF. Empty disables the correction and
  // reproduces the pre-2026-08-21 behaviour of reporting the vehicle frame's
  // height as the sea surface.
  std::string water_line_frame_ = "";

  // Cached vehicle-frame-to-water-line lever arm, keyed on both frames.
  mru_transform::WaterLineLeverArm water_line_lever_arm_;
  bool logged_lever_arm_ = false;

  // Seconds between re-reads of a resolved lever arm (staticness check) and
  // between retries of an unresolved one.
  static constexpr double kLeverArmRefreshPeriod = 10.0;
  static constexpr double kLeverArmRetryPeriod = 1.0;
  rclcpp::Time last_lookup_attempt_;
  bool have_lookup_attempt_ = false;

  // child_frame_id the buffered samples belong to.
  std::string buffered_child_frame_id_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr tide_estimate_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // MRU_TRANSFORM_NODES_SEA_SURFACE_ESTIMATOR_HPP
