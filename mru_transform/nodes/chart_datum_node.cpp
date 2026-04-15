// chart_datum_node.cpp — Publishes map → chart_datum TF transforms
//
// Uses PROJ to compute the static vertical offset between the WGS84
// ellipsoid (map frame) and tidal datums (MLLW, MHHW) at the robot's
// current position. The offsets are position-dependent and recalculated
// periodically. Transforms are published at a faster rate using
// cached values.
//
// Position is obtained from the TF tree (earth → base_link → ECEF →
// lat/lon via geodesy).
//
// Requires:
//   - PROJ geoid grid (e.g., us_noaa_g2018u0.tif) for ellipsoid → NAVD88
//   - VDatum regional .gtx grids for NAVD88 → MLLW
//   - VDatum regional .gtx grids for NAVD88 → MHHW (optional)

#include <cmath>
#include <filesystem>
#include <string>

#include "proj.h"

#include "geodesy/ecef.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;

class ChartDatumNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ChartDatumNode()
  : rclcpp_lifecycle::LifecycleNode("chart_datum")
  {
  }

  ~ChartDatumNode()
  {
    cleanup_proj();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state)
  {
    declare_parameter("chart_datum_frame", chart_datum_frame_);
    get_parameter("chart_datum_frame", chart_datum_frame_);

    declare_parameter("mhhw_frame", mhhw_frame_);
    get_parameter("mhhw_frame", mhhw_frame_);

    declare_parameter("map_frame", map_frame_);
    get_parameter("map_frame", map_frame_);

    declare_parameter("base_frame", base_frame_);
    get_parameter("base_frame", base_frame_);

    declare_parameter("geoid_grid", std::string(""));
    get_parameter("geoid_grid", geoid_grid_path_);

    declare_parameter("vdatum_grid_dir", std::string(""));
    get_parameter("vdatum_grid_dir", vdatum_grid_dir_);

    declare_parameter("publish_rate", publish_rate_);
    get_parameter("publish_rate", publish_rate_);

    declare_parameter("recalc_interval", recalc_interval_);
    get_parameter("recalc_interval", recalc_interval_);

    if (geoid_grid_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "geoid_grid parameter is required");
      return CallbackReturn::FAILURE;
    }

    if (vdatum_grid_dir_.empty()) {
      RCLCPP_ERROR(get_logger(), "vdatum_grid_dir parameter is required");
      return CallbackReturn::FAILURE;
    }

    if (!setup_proj()) {
      return CallbackReturn::FAILURE;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    mllw_pub_ = create_publisher<std_msgs::msg::Float64>("mllw_offset", 10);
    mhhw_pub_ = create_publisher<std_msgs::msg::Float64>("mhhw_offset", 10);

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
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

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    publish_timer_.reset();
    recalc_timer_.reset();
    return LifecycleNode::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state)
  {
    cleanup_proj();
    tf_buffer_.reset();
    tf_listener_.reset();
    tf_broadcaster_.reset();
    return LifecycleNode::on_cleanup(state);
  }

private:
  // Collect .gtx grid files matching a suffix (e.g., "_mllw")
  std::string collect_grids(const std::string & suffix)
  {
    std::string grids;
    for (const auto & entry :
      fs::recursive_directory_iterator(vdatum_grid_dir_))
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
    } catch (const fs::filesystem_error & e) {
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

    double lon_rad = proj_torad(geo.longitude);
    double lat_rad = proj_torad(geo.latitude);

    // MLLW (required)
    double mllw_z;
    if (!query_datum(proj_mllw_, lon_rad, lat_rad, mllw_z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 30000,
        "No VDatum MLLW coverage at (%.4f, %.4f)",
        geo.latitude, geo.longitude);
      return;
    }

    chart_datum_z_ = mllw_z;
    has_valid_mllw_ = true;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "Chart datum at (%.4f, %.4f): %.3f m (MLLW below ellipsoid)",
      geo.latitude, geo.longitude, chart_datum_z_);

    // MHHW (optional)
    if (proj_mhhw_) {
      double mhhw_z;
      if (query_datum(proj_mhhw_, lon_rad, lat_rad, mhhw_z)) {
        mhhw_z_ = mhhw_z;
        has_valid_mhhw_ = true;

        RCLCPP_INFO_ONCE(
          get_logger(),
          "MHHW at (%.4f, %.4f): %.3f m (below ellipsoid), "
          "tidal range: %.3f m",
          geo.latitude, geo.longitude, mhhw_z_,
          mhhw_z_ - chart_datum_z_);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 30000,
          "No VDatum MHHW coverage at (%.4f, %.4f)",
          geo.latitude, geo.longitude);
      }
    }
  }

  void publish_callback()
  {
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
  }

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Debug publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr mllw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr mhhw_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr recalc_timer_;

  // PROJ state
  PJ_CONTEXT * proj_context_ = nullptr;
  PJ * proj_mllw_ = nullptr;
  PJ * proj_mhhw_ = nullptr;

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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChartDatumNode>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
