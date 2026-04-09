// chart_datum_node.cpp — Publishes the map → chart_datum TF transform
//
// Uses PROJ to compute the static vertical offset between the WGS84
// ellipsoid (map frame) and chart datum (MLLW) at the robot's current
// position. The offset is position-dependent and only re-queried when
// the robot moves beyond update_distance.
//
// Requires:
//   - PROJ geoid grid (e.g., us_noaa_g2018u0.tif) for ellipsoid → NAVD88
//   - VDatum regional .gtx grids for NAVD88 → MLLW
//
// The PROJ pipeline:
//   +proj=pipeline
//   +step +proj=vgridshift +grids=<geoid_grid>
//   +step +proj=vgridshift +grids=<mllw_grid1>,<mllw_grid2>,...

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "proj.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/transform_broadcaster.h"

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

    declare_parameter("geoid_grid", std::string(""));
    get_parameter("geoid_grid", geoid_grid_path_);

    declare_parameter("vdatum_grid_dir", std::string(""));
    get_parameter("vdatum_grid_dir", vdatum_grid_dir_);

    declare_parameter("update_distance", update_distance_);
    get_parameter("update_distance", update_distance_);

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

    transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to odom for distance-based re-query and TF timestamps
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(
        &ChartDatumNode::odometry_callback, this,
        std::placeholders::_1));

    // Subscribe to position for geographic coordinates (lat/lon)
    position_subscription_ =
      create_subscription<sensor_msgs::msg::NavSatFix>(
      "position", 10,
      std::bind(
        &ChartDatumNode::position_callback, this,
        std::placeholders::_1));

    return LifecycleNode::on_configure(state);
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    return LifecycleNode::on_activate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state)
  {
    cleanup_proj();
    return LifecycleNode::on_cleanup(state);
  }

private:
  bool setup_proj()
  {
    // Find all MLLW .gtx grids in the vdatum directory
    std::string mllw_grids;
    try {
      for (const auto & entry :
        fs::recursive_directory_iterator(vdatum_grid_dir_))
      {
        if (entry.path().extension() == ".gtx" &&
          entry.path().stem().string().find("_mllw") !=
          std::string::npos)
        {
          if (!mllw_grids.empty()) {
            mllw_grids += ",";
          }
          mllw_grids += entry.path().string();
        }
      }
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

    RCLCPP_INFO(
      get_logger(), "Found MLLW grids in %s", vdatum_grid_dir_.c_str());

    // Build the PROJ pipeline string
    std::string pipeline =
      "+proj=pipeline "
      "+step +proj=vgridshift +grids=" + geoid_grid_path_ + " "
      "+step +proj=vgridshift +grids=" + mllw_grids;

    proj_context_ = proj_context_create();
    proj_context_set_enable_network(proj_context_, false);

    proj_ = proj_create(proj_context_, pipeline.c_str());
    if (!proj_) {
      RCLCPP_ERROR(
        get_logger(), "Failed to create PROJ pipeline: %s",
        proj_context_errno_string(
          proj_context_, proj_context_errno(proj_context_)));
      proj_context_destroy(proj_context_);
      proj_context_ = nullptr;
      return false;
    }

    RCLCPP_INFO(get_logger(), "PROJ pipeline ready");
    return true;
  }

  void cleanup_proj()
  {
    if (proj_) {
      proj_destroy(proj_);
      proj_ = nullptr;
    }
    if (proj_context_) {
      proj_context_destroy(proj_context_);
      proj_context_ = nullptr;
    }
  }

  void position_callback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_longitude_ = msg->longitude;
    last_latitude_ = msg->latitude;
    has_geographic_position_ = true;
  }

  void odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      return;
    }

    if (!has_geographic_position_) {
      return;
    }

    // Check if we need to re-query (robot moved beyond update_distance)
    double dx = msg->pose.pose.position.x - last_query_x_;
    double dy = msg->pose.pose.position.y - last_query_y_;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (has_valid_offset_ && dist < update_distance_) {
      publish_transform(msg->header);
      return;
    }

    // Query PROJ: (lon, lat, 0) → (lon, lat, height_above_mllw)
    PJ_COORD input = proj_coord(
      last_longitude_, last_latitude_, 0.0, 0.0);
    PJ_COORD output = proj_trans(proj_, PJ_FWD, input);

    if (output.xyz.z == HUGE_VAL ||
      std::isinf(output.xyz.z) || std::isnan(output.xyz.z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "No VDatum coverage at (%.4f, %.4f)",
        last_longitude_, last_latitude_);
      return;
    }

    // chart_datum (MLLW) is height_above_mllw meters below the
    // ellipsoid (map frame)
    chart_datum_z_ = -output.xyz.z;
    has_valid_offset_ = true;
    last_query_x_ = msg->pose.pose.position.x;
    last_query_y_ = msg->pose.pose.position.y;

    RCLCPP_INFO_ONCE(
      get_logger(),
      "Chart datum at (%.4f, %.4f): %.3f m (MLLW below ellipsoid)",
      last_longitude_, last_latitude_, chart_datum_z_);

    publish_transform(msg->header);
  }

  void publish_transform(const std_msgs::msg::Header & header)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = header;
    transform.child_frame_id = chart_datum_frame_;
    transform.transform.translation.z = chart_datum_z_;
    transform.transform.rotation.w = 1.0;

    transform_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
    position_subscription_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  // PROJ state
  PJ_CONTEXT * proj_context_ = nullptr;
  PJ * proj_ = nullptr;

  // Parameters
  std::string chart_datum_frame_ = "chart_datum";
  std::string geoid_grid_path_;
  std::string vdatum_grid_dir_;
  double update_distance_ = 1000.0;  // meters

  // Cached state
  double chart_datum_z_ = 0.0;
  bool has_valid_offset_ = false;
  double last_query_x_ = 0.0;
  double last_query_y_ = 0.0;

  // Geographic position (from NavSatFix subscription)
  double last_longitude_ = 0.0;
  double last_latitude_ = 0.0;
  bool has_geographic_position_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChartDatumNode>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
