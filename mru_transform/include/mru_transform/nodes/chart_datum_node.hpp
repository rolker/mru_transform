// chart_datum_node.cpp — Publishes map → chart_datum TF transforms
//
// Resolves the vertical datum (MLLW, and optionally MHHW) at the robot's
// current position from a precedence chain so the boat works on and off
// VDatum coverage:
//   1. lake_datum param (if set)            — wins outright everywhere
//   2. config entries with override == true — beat VDatum
//   3. VDatum (PROJ) result                 — where grids cover the point
//   4. config entries with override == false (default) — fill VDatum gaps
//   5. nothing matches                      — chart_datum is NOT published
//      (navigation continues on the ellipsoidal map / map_tide); a loud
//      warning is logged. This is the #8 "optional/additive" chart_datum.
//
// Neither the precedence chain nor the VDatum/PROJ query lives here: both are
// the ROS-free marine_vertical_datum library in core_ws (ADR-0010 D6), shared
// with the chart importers and CAMP so all three resolve a datum the same way.
// This node is the ROS wrapper over it -- parameters, TF, lifecycle. The
// active source is logged and published on the latched `datum_source` topic so
// consumers and operators can distinguish a surveyed datum from "none".
//
// The offsets are position-dependent and recalculated periodically. Transforms
// are published at a faster rate using cached values. Position is obtained from
// the TF tree (earth → base_link → ECEF → lat/lon via geodesy).
//
// VDatum is optional. When used it requires:
//   - PROJ geoid grid (e.g., us_noaa_g2018u0.tif) for ellipsoid → NAVD88
//   - VDatum regional .gtx grids for NAVD88 → MLLW (and optionally → MHHW)
#ifndef MRU_TRANSFORM_NODES_CHART_DATUM_NODE_HPP
#define MRU_TRANSFORM_NODES_CHART_DATUM_NODE_HPP

#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geodesy/ecef.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "marine_vertical_datum/datum_config.hpp"
#include "marine_vertical_datum/vdatum_query.hpp"

class ChartDatumNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ChartDatumNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("chart_datum", options)
  {
  }

  // The destructor is the last exit path, and the one a `finalized` node is
  // most likely to be sitting on. Releasing the VDatum query alone is not
  // enough: member subobjects are destroyed only AFTER this body returns, so
  // publish_timer_ and recalc_timer_ would still be armed while the PROJ
  // context they reach through vdatum_query_ was already freed -- precisely
  // the use-after-free release_everything_on_configure_created() names and
  // orders against. That ordering still matters after #41: the PROJ context
  // moved inside the library's callable, but destroying vdatum_query_ still
  // frees it, so the timers must be stopped first either way. Benign
  // under the single-threaded executor this node's main() uses, a real UAF
  // under a composed multi-threaded one. The helper is idempotent, so running
  // it here after an on_cleanup/on_shutdown/on_error has already run costs
  // nothing. (#34)
  ~ChartDatumNode()
  {
    release_everything_on_configure_created();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    // Every declare is guarded so a second configure does not throw
    // ParameterAlreadyDeclaredException, and the parameters are deliberately
    // NOT undeclared in on_cleanup so a value an operator set with
    // `ros2 param set` survives a cleanup -> configure cycle.
    //
    // The guard matters twice over on this node: the publish_rate /
    // recalc_interval validation below returns FAILURE partway through the
    // declares, and a failed configure returns the FSM to `unconfigured`
    // WITHOUT calling on_cleanup -- from which `cleanup` is not a legal
    // transition. Anything that released parameters in on_cleanup could never
    // run on that path, so correcting a bad rate and re-configuring would
    // still throw. (#34)
    if (!has_parameter("chart_datum_frame")) {
      declare_parameter("chart_datum_frame", chart_datum_frame_);
    }
    get_parameter("chart_datum_frame", chart_datum_frame_);

    if (!has_parameter("mhhw_frame")) {
      declare_parameter("mhhw_frame", mhhw_frame_);
    }
    get_parameter("mhhw_frame", mhhw_frame_);

    if (!has_parameter("map_frame")) {
      declare_parameter("map_frame", map_frame_);
    }
    get_parameter("map_frame", map_frame_);

    if (!has_parameter("base_frame")) {
      declare_parameter("base_frame", base_frame_);
    }
    get_parameter("base_frame", base_frame_);

    if (!has_parameter("geoid_grid")) {
      declare_parameter("geoid_grid", std::string(""));
    }
    get_parameter("geoid_grid", geoid_grid_path_);

    if (!has_parameter("vdatum_grid_dir")) {
      declare_parameter("vdatum_grid_dir", std::string(""));
    }
    get_parameter("vdatum_grid_dir", vdatum_grid_dir_);

    if (!has_parameter("publish_rate")) {
      declare_parameter("publish_rate", publish_rate_);
    }
    get_parameter("publish_rate", publish_rate_);

    if (!has_parameter("recalc_interval")) {
      declare_parameter("recalc_interval", recalc_interval_);
    }
    get_parameter("recalc_interval", recalc_interval_);

    // Non-positive timer periods would divide by zero (never publish) or peg a
    // core (zero-period recalc) — both silent failures. Non-finite ones reach
    // the same places by a different route and a bare `<= 0.0` does not stop
    // them: `+inf <= 0.0` is false, so publish_rate = .inf passed and
    // create_wall_timer(1.0 / inf) armed a ZERO-PERIOD timer — the core-pegging
    // failure this check exists to prevent — while a NaN passed into an
    // out-of-range float-to-integral conversion inside the duration cast, which
    // is undefined behaviour. Check finiteness explicitly.
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
      RCLCPP_ERROR(
        get_logger(), "publish_rate must be a finite number > 0 (got %f)",
        publish_rate_);
      return CallbackReturn::FAILURE;
    }
    if (!std::isfinite(recalc_interval_) || recalc_interval_ <= 0.0) {
      RCLCPP_ERROR(
        get_logger(), "recalc_interval must be a finite number > 0 (got %f)",
        recalc_interval_);
      return CallbackReturn::FAILURE;
    }

    if (!has_parameter("datum_config_path")) {
      declare_parameter("datum_config_path", std::string(""));
    }
    get_parameter("datum_config_path", datum_config_path_);

    // NaN sentinel means "unset". lake_datum, when set, overrides everything.
    const double kUnset = std::numeric_limits<double>::quiet_NaN();
    if (!has_parameter("lake_datum")) {
      declare_parameter("lake_datum", kUnset);
    }
    get_parameter("lake_datum", lake_datum_);
    if (!has_parameter("lake_datum_mhhw")) {
      declare_parameter("lake_datum_mhhw", kUnset);
    }
    get_parameter("lake_datum_mhhw", lake_datum_mhhw_);

    // NaN is the "unset" sentinel; an infinite value is an operator error, not
    // a datum — normalize it away so it can't reach the TF tree.
    if (std::isinf(lake_datum_)) {
      RCLCPP_WARN(
        get_logger(), "lake_datum is infinite — ignoring (treated as unset)");
      lake_datum_ = kUnset;
    }
    if (std::isinf(lake_datum_mhhw_)) {
      RCLCPP_WARN(
        get_logger(),
        "lake_datum_mhhw is infinite — ignoring (treated as unset)");
      lake_datum_mhhw_ = kUnset;
    }

    // VDatum is optional: enabled only when both grids are configured and the
    // library's pipeline sets up. Failure here is non-fatal — the
    // config/param/absent paths still let the boat operate anywhere.
    vdatum_query_ = {};
    if (vdatum_grid_dir_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "No vdatum_grid_dir — VDatum disabled; using config/param datums only");
    } else if (geoid_grid_path_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "vdatum_grid_dir is set but geoid_grid is empty — VDatum disabled");
    } else if (!setup_vdatum()) {
      RCLCPP_WARN(
        get_logger(),
        "VDatum setup failed — continuing without VDatum (config/param/absent)");
    }

    // Load the polygon→datum config, if a path was given. A malformed config is
    // an operator error worth failing loudly on (configure can be retried).
    if (!datum_config_path_.empty()) {
      try {
        datum_entries_ = marine_vertical_datum::load_datum_config(datum_config_path_);
        RCLCPP_INFO(
          get_logger(), "Loaded %zu datum polygon(s) from %s",
          datum_entries_.size(), datum_config_path_.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to load datum config: %s", e.what());
        // A failed configure returns the FSM to `unconfigured` WITHOUT calling
        // on_cleanup, and `cleanup` is not a legal transition from there -- so
        // nothing else will ever release what this configure already allocated.
        // The supported recovery is to fix the config and configure again
        // (ChartDatumNodeRecoversFromFailedConfigure), and that retry would
        // overwrite the VDatum query, dropping the previous one's PROJ
        // context and pipelines only if nothing else holds it. Release here,
        // on the way out, so the retry starts clean. (#34)
        //
        // The query is RAII -- the library's callable owns the PROJ context
        // and both pipelines, so assigning an empty function destroys them.
        // That is what retired the hand-written cleanup_proj(). (#41)
        vdatum_query_ = {};
        datum_entries_.clear();
        return CallbackReturn::FAILURE;
      }
    }

    if (!std::isnan(lake_datum_)) {
      RCLCPP_INFO(
        get_logger(),
        "lake_datum override set: chart_datum %.3f m (rel. ellipsoid) — "
        "wins over VDatum and config everywhere",
        lake_datum_);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // One-argument form: the listener creates its OWN internal node with
    // default options and spins it on its own thread, so it subscribes to the
    // global /tf and /tf_static -- this node's namespace and remap rules do not
    // apply to it. Usual TF-listener behaviour; called out because it is the
    // one part of this node that a test cannot namespace.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    auto latched_qos = rclcpp::QoS(1).transient_local();
    mllw_pub_ = create_publisher<std_msgs::msg::Float64>("mllw_offset", latched_qos);
    mhhw_pub_ = create_publisher<std_msgs::msg::Float64>("mhhw_offset", latched_qos);
    datum_source_pub_ =
      create_publisher<std_msgs::msg::String>("datum_source", latched_qos);

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&ChartDatumNode::publish_callback, this));

    recalc_timer_ = create_wall_timer(
      std::chrono::duration<double>(recalc_interval_),
      std::bind(&ChartDatumNode::recalc_callback, this));

    // Trigger immediate first recalculation
    recalc_callback();

    return LifecycleNode::on_activate(state);
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    publish_timer_.reset();
    recalc_timer_.reset();
    return LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    release_everything_on_configure_created();
    // Parameters stay declared on purpose: see on_configure.
    return LifecycleNode::on_cleanup(state);
  }

  // `shutdown` is legal from `unconfigured`, `inactive` AND `active`, and it
  // runs on_shutdown ONLY -- on_deactivate and on_cleanup are both skipped.
  // Without this override nothing released what on_configure and on_activate
  // created, so a FINALIZED node kept both timers armed (recalc_callback has
  // no state gate, so it went on doing earth -> base_link lookups, PROJ
  // queries and INFO logging forever) and kept all three transient_local
  // publishers latched, still handing a datum to every late-joining
  // subscriber. The release helper is null-safe and idempotent, so one
  // override is correct from all three source states. (#34)
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    release_everything_on_configure_created();
    return LifecycleNode::on_shutdown(state);
  }

  // An exception thrown out of on_configure or on_activate routes the FSM
  // through `errorprocessing` to `unconfigured` -- and on_cleanup is NOT one of
  // the callbacks that runs on that path, so without this override nothing
  // would ever release what the failed transition had already allocated. The
  // PROJ context is the one that matters (the same leak the datum-config catch
  // block in on_configure exists to prevent, by the same argument), but
  // on_activate is worse: a throwing create_wall_timer would strand the
  // context, both pipelines, the TF members and all three publishers. The
  // supported recovery from `unconfigured` is another configure, which would
  // overwrite every one of those pointers. Release them here instead. (#34)
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_ERROR(
      get_logger(),
      "Transition failed with an exception; releasing everything the failed "
      "configure/activate had allocated. Correct the fault and configure again.");
    release_everything_on_configure_created();
    return LifecycleNode::on_error(state);
  }

private:
  // Shared by on_cleanup, on_shutdown and on_error: every reset here is
  // null-safe and
  // re-nulls what it releases, so it is idempotent and safe to run twice or on
  // a half-built node.
  void release_everything_on_configure_created()
  {
    // Timers are normally released by on_deactivate, but cleanup is also
    // reachable from inactive after a configure that never activated -- and a
    // timer outliving the PROJ context it calls into would be a use-after-free.
    // Resetting a timer does not wait for a callback already dispatched;
    // ordering the teardown is only sufficient because this node's main() uses
    // a single-threaded executor. (The one-argument TransformListener created
    // in on_configure spins a thread of its own regardless, so that claim is
    // about this node's own callbacks; that thread only fills tf_buffer_ and
    // never enters the PROJ code, and the reset ordering below is what makes
    // it safe.)
    publish_timer_.reset();
    recalc_timer_.reset();

    vdatum_query_ = {};
    datum_entries_.clear();
    datum_source_ = "none";
    has_valid_mllw_ = false;
    has_valid_mhhw_ = false;

    // The listener fills the buffer from its own spin thread, so it is torn
    // down before the buffer it references.
    tf_listener_.reset();
    tf_buffer_.reset();
    tf_broadcaster_.reset();

    // Everything else on_configure created.
    mllw_pub_.reset();
    mhhw_pub_.reset();
    datum_source_pub_.reset();
  }

  // Build the VDatum query from the library (ADR-0010 D6). The returned
  // callable owns the PROJ context and pipelines, so there is nothing to
  // release by hand -- dropping it destroys them. An EMPTY function means
  // setup failed; the reason has already gone to `diag` below.
  bool setup_vdatum()
  {
    auto diag = [this](const std::string & message) {
        RCLCPP_WARN(get_logger(), "VDatum: %s", message.c_str());
      };
    vdatum_query_ = marine_vertical_datum::make_vdatum_query(
      {geoid_grid_path_, vdatum_grid_dir_}, diag);
    if (!vdatum_query_) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "VDatum ready from %s", vdatum_grid_dir_.c_str());
    return true;
  }

  // Query the VDatum PROJ pipelines at a point. Returns nullopt when VDatum is
  // disabled or has no MLLW coverage there.
  std::optional<marine_vertical_datum::VDatumResult> query_vdatum(
    double latitude, double longitude)
  {
    if (!vdatum_query_) {
      return std::nullopt;
    }
    auto result = vdatum_query_(latitude, longitude);
    if (!result) {
      // The library reports a per-point gap as a plain nullopt with no
      // diagnostic, because a coverage gap is normal and the polygon chain
      // handles it. The node still says so once every 30 s, because here a
      // gap means the boat may end up with no chart datum at all.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "No VDatum MLLW coverage at (%.4f, %.4f)", latitude, longitude);
    }
    return result;
  }

  static std::string source_label(
    marine_vertical_datum::DatumSource source, const std::string & name)
  {
    switch (source) {
      case marine_vertical_datum::DatumSource::VDATUM:
        return "vdatum";
      case marine_vertical_datum::DatumSource::POLYGON_CONFIG:
        return "polygon:" + name;
      case marine_vertical_datum::DatumSource::PARAM:
        return "param";
    }
    return "unknown";
  }

  void recalc_callback()
  {
    // Look up earth → base_link to get ECEF position
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        "earth", base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        get_logger(), "Cannot look up earth → %s: %s",
        base_frame_.c_str(), ex.what());
      return;
    }

    // Convert ECEF → geographic (lat/lon)
    geometry_msgs::msg::Point ecef_point;
    ecef_point.x = tf_stamped.transform.translation.x;
    ecef_point.y = tf_stamped.transform.translation.y;
    ecef_point.z = tf_stamped.transform.translation.z;

    auto geo = geodesy::toMsg(geodesy::ECEFPoint(ecef_point));

    // Resolve the datum via the pure precedence chain.
    auto vdatum = query_vdatum(geo.latitude, geo.longitude);
    std::optional<double> lake = std::isfinite(lake_datum_) ?
      std::optional<double>(lake_datum_) : std::nullopt;
    std::optional<double> lake_mhhw = std::isfinite(lake_datum_mhhw_) ?
      std::optional<double>(lake_datum_mhhw_) : std::nullopt;

    auto result = marine_vertical_datum::resolve_datum(
      geo.latitude, geo.longitude, lake, lake_mhhw, vdatum, datum_entries_);

    if (!result.has_value()) {
      has_valid_mllw_ = false;
      has_valid_mhhw_ = false;
      datum_source_ = "none";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "No datum at (%.4f, %.4f) — VDatum/config/param all unavailable; "
        "chart_datum will not be published (navigation uses map_tide)",
        geo.latitude, geo.longitude);
      return;
    }

    chart_datum_z_ = result->chart_datum_z;
    has_valid_mllw_ = true;
    if (result->mhhw_z.has_value()) {
      mhhw_z_ = *result->mhhw_z;
      has_valid_mhhw_ = true;
    } else {
      has_valid_mhhw_ = false;
    }

    const std::string source = source_label(result->source, result->name);
    if (source != datum_source_) {
      datum_source_ = source;
      if (has_valid_mhhw_) {
        RCLCPP_INFO(
          get_logger(),
          "Datum at (%.4f, %.4f): chart_datum %.3f m, MHHW %.3f m "
          "(rel. ellipsoid) [source: %s]",
          geo.latitude, geo.longitude, chart_datum_z_, mhhw_z_,
          datum_source_.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Datum at (%.4f, %.4f): chart_datum %.3f m (rel. ellipsoid) "
          "[source: %s]",
          geo.latitude, geo.longitude, chart_datum_z_, datum_source_.c_str());
      }
    }
  }

  void publish_callback()
  {
    // Two things stop a non-active node from publishing here, and only one of
    // them is the publisher's own gate.
    // rclcpp_lifecycle::LifecyclePublisher::publish() IS virtual (jazzy
    // lifecycle_publisher.hpp) and returns early unless is_activated(), so
    // mllw_pub_, mhhw_pub_ and datum_source_pub_ are gated by the publisher.
    // (This comment used to call that gate "a non-virtual hide that publish()
    // bypasses" -- false, and not the reason the check is here.)
    // tf_broadcaster_ is a plain tf2_ros::TransformBroadcaster with NO
    // activation gate at all, so for the map -> chart_datum transform this
    // check is the only gate there is.
    //
    // Both timers are created in on_activate and released by on_deactivate,
    // on_cleanup, on_shutdown and on_error, so a non-active node should not
    // reach this callback at all. `shutdown` from `active` was the hole: it
    // runs on_shutdown ONLY, skipping on_deactivate and on_cleanup, and nothing
    // overrode on_shutdown -- so both timers stayed armed into `finalized`.
    // That is now closed at the cause, and this check is the second gate behind
    // it. chart_datum is the frame every sounding is reduced against: it must
    // follow this node's lifecycle state. The same check also covers
    // `errorprocessing`, which is not ACTIVE either. (#34)
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    auto stamp = now();

    if (has_valid_mllw_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = chart_datum_frame_;
      transform.transform.translation.z = chart_datum_z_;
      transform.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(transform);

      std_msgs::msg::Float64 msg;
      msg.data = chart_datum_z_;
      mllw_pub_->publish(msg);
    }

    if (has_valid_mhhw_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = mhhw_frame_;
      transform.transform.translation.z = mhhw_z_;
      transform.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(transform);

      std_msgs::msg::Float64 msg;
      msg.data = mhhw_z_;
      mhhw_pub_->publish(msg);
    }

    // Provenance: lets consumers/operators distinguish a surveyed datum from
    // "none" (chart_datum absent) without inspecting the TF tree.
    std_msgs::msg::String src_msg;
    src_msg.data = datum_source_;
    datum_source_pub_->publish(src_msg);
  }

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Debug publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr mllw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr mhhw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr datum_source_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr recalc_timer_;

  // VDatum state. The library's callable owns the PROJ context and pipelines;
  // an empty function means VDatum is disabled or failed to set up. (#41)
  marine_vertical_datum::VDatumQueryFn vdatum_query_;

  // Polygon→datum config + fixed-value override.
  std::vector<marine_vertical_datum::DatumEntry> datum_entries_;
  std::string datum_config_path_;
  double lake_datum_ = std::numeric_limits<double>::quiet_NaN();
  double lake_datum_mhhw_ = std::numeric_limits<double>::quiet_NaN();

  // Parameters
  std::string chart_datum_frame_ = "chart_datum";
  std::string mhhw_frame_ = "chart_datum_mhhw";
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  std::string geoid_grid_path_;
  std::string vdatum_grid_dir_;
  double publish_rate_ = 1.0;       // Hz
  double recalc_interval_ = 60.0;   // seconds

  // Cached state
  double chart_datum_z_ = 0.0;
  double mhhw_z_ = 0.0;
  bool has_valid_mllw_ = false;
  bool has_valid_mhhw_ = false;
  std::string datum_source_ = "none";
};

#endif  // MRU_TRANSFORM_NODES_CHART_DATUM_NODE_HPP
