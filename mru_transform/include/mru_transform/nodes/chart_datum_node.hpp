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
// The resolution itself lives in the pure mru_transform::resolve_datum() core
// (datum_config.hpp) so the precedence is unit-testable. The active source is
// logged and published on the latched `datum_source` topic so consumers and
// operators can distinguish a surveyed datum from "none".
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

#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "proj.h"

#include "geodesy/ecef.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "mru_transform/datum_config.hpp"

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

  ~ChartDatumNode()
  {
    cleanup_proj();
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
    // core (zero-period recalc) — both silent failures.
    if (publish_rate_ <= 0.0) {
      RCLCPP_ERROR(
        get_logger(), "publish_rate must be > 0 (got %.3f)", publish_rate_);
      return CallbackReturn::FAILURE;
    }
    if (recalc_interval_ <= 0.0) {
      RCLCPP_ERROR(
        get_logger(), "recalc_interval must be > 0 (got %.3f)", recalc_interval_);
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
    // PROJ pipeline sets up. Failure here is non-fatal — the config/param/absent
    // paths still let the boat operate anywhere.
    vdatum_enabled_ = false;
    if (vdatum_grid_dir_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "No vdatum_grid_dir — VDatum disabled; using config/param datums only");
    } else if (geoid_grid_path_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "vdatum_grid_dir is set but geoid_grid is empty — VDatum disabled");
    } else if (setup_proj()) {
      vdatum_enabled_ = true;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "VDatum setup failed — continuing without VDatum (config/param/absent)");
    }

    // Load the polygon→datum config, if a path was given. A malformed config is
    // an operator error worth failing loudly on (configure can be retried).
    if (!datum_config_path_.empty()) {
      try {
        datum_entries_ = mru_transform::load_datum_config(datum_config_path_);
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
        // overwrite the PROJ context and both pipelines, leaking them once per
        // retry. Release them here, on the way out. (#34)
        cleanup_proj();
        vdatum_enabled_ = false;
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
    // Timers are normally released by on_deactivate, but cleanup is also
    // reachable from inactive after a configure that never activated -- and a
    // timer outliving the PROJ context it calls into would be a use-after-free.
    // Resetting a timer does not wait for a callback already dispatched;
    // ordering the teardown is only sufficient because this node's main() uses
    // a single-threaded executor.
    publish_timer_.reset();
    recalc_timer_.reset();

    cleanup_proj();
    vdatum_enabled_ = false;
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

    // Parameters stay declared on purpose: see on_configure.
    return LifecycleNode::on_cleanup(state);
  }

private:
  // Collect .gtx grid files matching a suffix (e.g., "_mllw")
  std::string collect_grids(const std::string & suffix)
  {
    std::string grids;
    for (const auto & entry :
      std::filesystem::recursive_directory_iterator(vdatum_grid_dir_))
    {
      if (entry.path().extension() == ".gtx" &&
        entry.path().stem().string().find(suffix) !=
        std::string::npos)
      {
        if (!grids.empty()) {
          grids += ",";
        }
        grids += entry.path().string();
      }
    }
    return grids;
  }

  // Create a PROJ pipeline: ellipsoid → NAVD88 → target datum
  PJ * create_pipeline(const std::string & datum_grids)
  {
    std::string pipeline =
      "+proj=pipeline "
      "+step +proj=vgridshift +grids=" + geoid_grid_path_ + " "
      "+step +proj=vgridshift +grids=" + datum_grids;

    PJ * pj = proj_create(proj_context_, pipeline.c_str());
    if (!pj) {
      RCLCPP_ERROR(
        get_logger(), "Failed to create PROJ pipeline: %s",
        proj_context_errno_string(
          proj_context_, proj_context_errno(proj_context_)));
    }
    return pj;
  }

  bool setup_proj()
  {
    std::string mllw_grids;
    std::string mhhw_grids;
    try {
      mllw_grids = collect_grids("_mllw");
      mhhw_grids = collect_grids("_mhhw");
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_ERROR(
        get_logger(), "Error scanning vdatum_grid_dir '%s': %s",
        vdatum_grid_dir_.c_str(), e.what());
      return false;
    }

    if (mllw_grids.empty()) {
      RCLCPP_ERROR(
        get_logger(), "No *_mllw.gtx files found in %s",
        vdatum_grid_dir_.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Found MLLW grids in %s",
      vdatum_grid_dir_.c_str());

    proj_context_ = proj_context_create();
    proj_context_set_enable_network(proj_context_, false);

    proj_mllw_ = create_pipeline(mllw_grids);
    if (!proj_mllw_) {
      proj_context_destroy(proj_context_);
      proj_context_ = nullptr;
      return false;
    }

    RCLCPP_INFO(get_logger(), "PROJ MLLW pipeline ready");

    if (!mhhw_grids.empty()) {
      proj_mhhw_ = create_pipeline(mhhw_grids);
      if (proj_mhhw_) {
        RCLCPP_INFO(get_logger(), "PROJ MHHW pipeline ready");
      } else {
        RCLCPP_WARN(get_logger(),
          "Failed to create MHHW pipeline — MHHW frame will not be published");
      }
    } else {
      RCLCPP_WARN(get_logger(),
        "No *_mhhw.gtx files found — MHHW frame will not be published");
    }

    return true;
  }

  void cleanup_proj()
  {
    if (proj_mllw_) {
      proj_destroy(proj_mllw_);
      proj_mllw_ = nullptr;
    }
    if (proj_mhhw_) {
      proj_destroy(proj_mhhw_);
      proj_mhhw_ = nullptr;
    }
    if (proj_context_) {
      proj_context_destroy(proj_context_);
      proj_context_ = nullptr;
    }
  }

  // Query a PROJ pipeline and return the negated Z (datum below ellipsoid)
  bool query_datum(PJ * pipeline, double lon_rad, double lat_rad,
    double & result_z)
  {
    PJ_COORD input = proj_coord(lon_rad, lat_rad, 0.0, 0.0);
    PJ_COORD output = proj_trans(pipeline, PJ_FWD, input);

    if (output.xyz.z == HUGE_VAL ||
      std::isinf(output.xyz.z) || std::isnan(output.xyz.z))
    {
      return false;
    }

    result_z = -output.xyz.z;
    return true;
  }

  // Query the VDatum PROJ pipelines at a point. Returns nullopt when VDatum is
  // disabled or has no MLLW coverage there.
  std::optional<mru_transform::VDatumResult> query_vdatum(
    double latitude, double longitude)
  {
    if (!vdatum_enabled_) {
      return std::nullopt;
    }
    const double lon_rad = proj_torad(longitude);
    const double lat_rad = proj_torad(latitude);

    double mllw_z;
    if (!query_datum(proj_mllw_, lon_rad, lat_rad, mllw_z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "No VDatum MLLW coverage at (%.4f, %.4f)", latitude, longitude);
      return std::nullopt;
    }

    mru_transform::VDatumResult vr;
    vr.mllw_z = mllw_z;
    if (proj_mhhw_) {
      double mhhw_z;
      if (query_datum(proj_mhhw_, lon_rad, lat_rad, mhhw_z)) {
        vr.mhhw_z = mhhw_z;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 30000,
          "No VDatum MHHW coverage at (%.4f, %.4f)", latitude, longitude);
      }
    }
    return vr;
  }

  static std::string source_label(
    mru_transform::DatumSource source, const std::string & name)
  {
    switch (source) {
      case mru_transform::DatumSource::VDATUM:
        return "vdatum";
      case mru_transform::DatumSource::POLYGON_CONFIG:
        return "polygon:" + name;
      case mru_transform::DatumSource::PARAM:
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

    auto result = mru_transform::resolve_datum(
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
    // `shutdown` from `active` runs on_shutdown ONLY -- it skips on_deactivate
    // and on_cleanup, and no node here overrides on_shutdown -- so both timers
    // survive into `finalized` and this callback keeps firing. tf_broadcaster_
    // is a plain TransformBroadcaster with no activation gate, and
    // LifecyclePublisher's gate is a non-virtual hide that publish() bypasses,
    // so nothing but this check stops a finalized node from putting
    // map -> chart_datum on /tf. That is the frame every sounding is reduced
    // against: it must follow this node's lifecycle state. The same check also
    // covers `errorprocessing`, which is not ACTIVE either. (#34)
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

  // PROJ state
  PJ_CONTEXT * proj_context_ = nullptr;
  PJ * proj_mllw_ = nullptr;
  PJ * proj_mhhw_ = nullptr;
  bool vdatum_enabled_ = false;

  // Polygon→datum config + fixed-value override.
  std::vector<mru_transform::DatumEntry> datum_entries_;
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
