// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// dock_scan_match rclcpp node implementation. See header for the full
// design rationale + invariants. Follows RESEARCH §Code Examples
// Example 1 with the missing helpers fleshed out.

#include "mowgli_lidar_docking/dock_scan_match_node.hpp"

#include "mowgli_lidar_docking/confidence_metrics.hpp"
#include "mowgli_lidar_docking/dock_scan_io.hpp"
#include "mowgli_lidar_docking/dock_scan_meta_loader.hpp"

#include <mowgli_geometry/key_value_parser.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace mowgli_lidar_docking
{

namespace
{

// Convert a geometry_msgs::Transform → Sophus::SE3d.
Sophus::SE3d tf2_to_sophus(const geometry_msgs::msg::Transform& t)
{
  const Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  const Eigen::Vector3d v(t.translation.x, t.translation.y, t.translation.z);
  return Sophus::SE3d(q.normalized(), v);
}

// Yaw of an SE3 about Z. Mirrors KinematicIcpDockMatcher's helper but
// kept local — no cross-include dependency between the matcher and the
// node's helpers.
double yaw_of(const Sophus::SE3d& pose)
{
  const Eigen::Matrix3d R = pose.rotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

// Build a Sophus::SE3d from x/y/yaw_rad. Z = 0, roll = pitch = 0 — matches
// the dock_calibration.yaml convention (single yaw + translation, robot
// is on the ground).
Sophus::SE3d se3_from_xy_yaw(double x, double y, double yaw_rad)
{
  Eigen::AngleAxisd aa(yaw_rad, Eigen::Vector3d::UnitZ());
  return Sophus::SE3d(Eigen::Quaterniond(aa), Eigen::Vector3d(x, y, 0.0));
}

}  // namespace

DockScanMatchNode::DockScanMatchNode(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("dock_scan_match", opts)
{
  declare_all_parameters();
  init_publishers();
  init_tf();

  if (try_load_matcher_files())
  {
    degraded_ = false;
    RCLCPP_INFO(get_logger(),
                "dock_scan_match ready: dock=(%.3f, %.3f, %.1f deg), "
                "thresholds inlier>=%.2f rmse<=%.3f m, gate=%.1f m",
                dock_anchor_in_map_.translation().x(),
                dock_anchor_in_map_.translation().y(),
                yaw_of(dock_anchor_in_map_) * 180.0 / M_PI,
                min_inlier_ratio_, max_rmse_m_, gate_distance_m_);
  }
  else
  {
    degraded_ = true;
    RCLCPP_WARN(get_logger(),
                "dock_scan_match in DEGRADED mode (missing %s or %s); "
                "polling for capture every 30 s",
                dock_scan_path_.c_str(), dock_calibration_path_.c_str());
    warned_no_pcd_ = true;
    // 30-s polling timer for Open Q3 self-activation when files appear.
    degraded_poll_timer_ = create_wall_timer(
        std::chrono::seconds(30),
        std::bind(&DockScanMatchNode::poll_for_pcd_in_degraded_mode, this));
  }

  init_subscriber_and_timer();
}

DockScanMatchNode::DockScanMatchNode(const rclcpp::NodeOptions& opts,
                                     std::unique_ptr<IDockMatcher> injected_matcher,
                                     std::optional<Sophus::SE3d> dock_anchor_in_map)
    : rclcpp::Node("dock_scan_match", opts)
{
  declare_all_parameters();
  init_publishers();
  init_tf();

  if (injected_matcher && dock_anchor_in_map)
  {
    matcher_ = std::move(injected_matcher);
    dock_anchor_in_map_ = *dock_anchor_in_map;
    degraded_ = false;
  }
  else
  {
    degraded_ = true;
  }

  // No init_subscriber_and_timer() in the test ctor — tests drive
  // tick_once_for_test() and inject scans directly. No 30-s polling
  // timer either; tests would block on it.
}

void DockScanMatchNode::declare_all_parameters()
{
  crop_radius_m_         = declare_parameter<double>("crop_radius_m", 3.0);
  publish_rate_hz_       = declare_parameter<double>("publish_rate_hz", 10.0);
  max_correspondence_m_  = declare_parameter<double>("max_correspondence_distance", 0.30);
  max_iterations_        = declare_parameter<int>("max_iterations", 20);
  min_inlier_ratio_      = declare_parameter<double>("min_inlier_ratio", 0.70);
  max_rmse_m_            = declare_parameter<double>("max_rmse_m", 0.05);
  gate_distance_m_       = declare_parameter<double>("gate_distance_m", 5.0);
  dock_calibration_path_ = declare_parameter<std::string>(
      "dock_calibration_path", "/ros2_ws/maps/dock_calibration.yaml");
  dock_scan_path_        = declare_parameter<std::string>(
      "dock_scan_path", "/ros2_ws/maps/dock_scan.pcd");
  dock_scan_meta_path_   = declare_parameter<std::string>(
      "dock_scan_meta_path", "/ros2_ws/maps/dock_scan_meta.yaml");
  scan_topic_            = declare_parameter<std::string>("scan_topic", "/scan_kicp");
  pose_topic_            = declare_parameter<std::string>("pose_topic", "/dock_match/pose");
  confidence_topic_      = declare_parameter<std::string>(
      "confidence_topic", "/dock_match/confidence");
  robot_frame_           = declare_parameter<std::string>(
      "robot_frame", "base_footprint_wheels");
  map_frame_             = declare_parameter<std::string>("map_frame", "map");
}

void DockScanMatchNode::init_publishers()
{
  rclcpp::QoS pose_qos(1);
  pose_qos.reliable();
  pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, pose_qos);
  pub_conf_ = create_publisher<mowgli_interfaces::msg::DockMatchConfidence>(
      confidence_topic_, rclcpp::SensorDataQoS());
}

void DockScanMatchNode::init_tf()
{
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void DockScanMatchNode::init_subscriber_and_timer()
{
  sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DockScanMatchNode::on_scan, this, std::placeholders::_1));

  const auto period_ms =
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period_ms,
                             std::bind(&DockScanMatchNode::tick, this));
}

bool DockScanMatchNode::try_load_matcher_files()
{
  // Load dock_calibration.yaml — required for the dock anchor.
  const auto calib =
      mowgli_geometry::load_dock_calibration_file(dock_calibration_path_);
  if (!calib)
  {
    return false;
  }

  // Load dock_scan.pcd — required as the seeded reference.
  std::vector<Eigen::Vector3d> dock_points;
  if (!load_dock_scan_pcd(dock_scan_path_, dock_points))
  {
    return false;
  }
  if (dock_points.empty())
  {
    RCLCPP_ERROR(get_logger(),
                 "dock_scan.pcd has no points; refusing to seed matcher");
    return false;
  }

  dock_anchor_in_map_ = se3_from_xy_yaw(calib->x, calib->y, calib->yaw_rad);

  // Build a fresh KinematicIcpDockMatcher. Reload-style: we always
  // reconstruct rather than mutate to keep the ownership graph simple.
  kinematic_icp::pipeline::Config kicp_cfg;
  kicp_cfg.max_range = 8.0;
  kicp_cfg.min_range = 0.4;
  kicp_cfg.voxel_size = 0.1;
  kicp_cfg.max_points_per_voxel = 5;
  kicp_cfg.use_adaptive_threshold = false;
  kicp_cfg.fixed_threshold = max_correspondence_m_;
  kicp_cfg.max_num_iterations = max_iterations_;
  kicp_cfg.convergence_criterion = 0.001;
  kicp_cfg.max_num_threads = 1;
  kicp_cfg.use_adaptive_odometry_regularization = true;
  kicp_cfg.deskew = false;

  matcher_ = std::make_unique<KinematicIcpDockMatcher>(
      kicp_cfg, dock_points, dock_anchor_in_map_, max_correspondence_m_);

  // Cache the PCD mtime for the watcher.
  std::error_code ec;
  cached_pcd_mtime_ = std::filesystem::last_write_time(dock_scan_path_, ec);
  if (ec)
  {
    cached_pcd_mtime_.reset();
  }

  return true;
}

void DockScanMatchNode::poll_for_pcd_in_degraded_mode()
{
  if (!degraded_) return;
  if (try_load_matcher_files())
  {
    degraded_ = false;
    RCLCPP_INFO(get_logger(),
                "dock_scan_match self-activated: dock_scan.pcd appeared on "
                "disk; matcher seeded");
    // Cancel the 30-s polling timer; mtime watcher takes over.
    degraded_poll_timer_.reset();
  }
}

void DockScanMatchNode::check_pcd_mtime_and_reload()
{
  if (!cached_pcd_mtime_) return;
  std::error_code ec;
  const auto current_mtime =
      std::filesystem::last_write_time(dock_scan_path_, ec);
  if (ec) return;  // file vanished — keep working with the existing matcher
  if (current_mtime <= *cached_pcd_mtime_) return;

  // File changed on disk. Try to reload — but if any step fails, KEEP
  // the existing matcher (T-04-01 mitigation: never replace a working
  // matcher with a broken one).
  std::vector<Eigen::Vector3d> fresh_points;
  if (!load_dock_scan_pcd(dock_scan_path_, fresh_points) || fresh_points.empty())
  {
    RCLCPP_ERROR(get_logger(),
                 "dock_scan.pcd mtime changed but reload failed; keeping "
                 "previous matcher");
    return;
  }

  // dock_calibration.yaml may have been updated atomically alongside the
  // PCD (the GUI Recapture button writes both). Re-read it; tolerate it
  // being stale (use existing anchor).
  Sophus::SE3d new_anchor = dock_anchor_in_map_;
  const auto fresh_calib =
      mowgli_geometry::load_dock_calibration_file(dock_calibration_path_);
  if (fresh_calib)
  {
    new_anchor = se3_from_xy_yaw(fresh_calib->x, fresh_calib->y,
                                 fresh_calib->yaw_rad);
  }

  // Down-cast the IDockMatcher to the production type to call Reload().
  // In the test ctor we use a MockMatcher and the cast returns nullptr;
  // the watcher quietly skips the Reload (tests drive Reload through a
  // direct accessor instead).
  auto* prod = dynamic_cast<KinematicIcpDockMatcher*>(matcher_.get());
  if (prod)
  {
    prod->Reload(fresh_points, new_anchor);
    dock_anchor_in_map_ = new_anchor;
    cached_pcd_mtime_ = current_mtime;
    RCLCPP_INFO(get_logger(),
                "dock_scan.pcd mtime change: matcher reloaded with %zu points",
                fresh_points.size());
  }
  else
  {
    cached_pcd_mtime_ = current_mtime;  // mark seen so we don't loop
  }
}

void DockScanMatchNode::on_scan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(scan_mtx_);
  last_scan_ = msg;
}

bool DockScanMatchNode::robot_close_to_dock(double& out_distance)
{
  // Test injection short-circuits the TF lookup.
  {
    std::lock_guard<std::mutex> lock(test_inject_mtx_);
    if (injected_robot_in_map_)
    {
      const auto& t = injected_robot_in_map_->translation();
      const double dx = t.x() - dock_anchor_in_map_.translation().x();
      const double dy = t.y() - dock_anchor_in_map_.translation().y();
      out_distance = std::sqrt(dx * dx + dy * dy);
      return out_distance <= gate_distance_m_;
    }
  }

  if (!tf_buffer_) return false;
  try
  {
    auto tf = tf_buffer_->lookupTransform(
        map_frame_, robot_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    const double dx = tf.transform.translation.x -
                      dock_anchor_in_map_.translation().x();
    const double dy = tf.transform.translation.y -
                      dock_anchor_in_map_.translation().y();
    out_distance = std::sqrt(dx * dx + dy * dy);
    return out_distance <= gate_distance_m_;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "TF lookup %s -> %s failed: %s; assuming far from "
                         "dock",
                         map_frame_.c_str(), robot_frame_.c_str(), ex.what());
    out_distance = std::numeric_limits<double>::infinity();
    return false;
  }
}

void DockScanMatchNode::publish_not_trusted()
{
  mowgli_interfaces::msg::DockMatchConfidence conf;
  conf.header.stamp = this->now();
  conf.header.frame_id = map_frame_;
  conf.inlier_ratio = 0.0f;
  conf.rmse_m = std::numeric_limits<float>::infinity();
  conf.trusted = false;
  pub_conf_->publish(conf);
}

void DockScanMatchNode::publish_match(const MatchResult& result)
{
  mowgli_interfaces::msg::DockMatchConfidence conf;
  conf.header.stamp = this->now();
  conf.header.frame_id = map_frame_;
  conf.inlier_ratio = static_cast<float>(result.inlier_ratio);
  conf.rmse_m = std::isfinite(result.rmse_m)
                    ? static_cast<float>(result.rmse_m)
                    : std::numeric_limits<float>::infinity();

  ConfidenceResult cr{result.inlier_ratio, result.rmse_m};
  const bool trusted_metric = is_trusted(cr, min_inlier_ratio_, max_rmse_m_);
  conf.trusted = result.valid && trusted_metric;
  pub_conf_->publish(conf);

  if (conf.trusted)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = conf.header;
    pose.pose.pose.position.x = result.pose_in_map.translation().x();
    pose.pose.pose.position.y = result.pose_in_map.translation().y();
    pose.pose.pose.position.z = 0.0;
    const Eigen::Quaterniond q(result.pose_in_map.unit_quaternion());
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.orientation.w = q.w();
    // Covariance: simple isotropic, derived from RMSE squared. Plan 02-05
    // will refine if needed; the consumer (dock_yaw_to_set_pose cascade)
    // mostly gates on trusted, not on covariance values.
    const double sigma2 = std::max(result.rmse_m * result.rmse_m, 1e-6);
    for (auto& c : pose.pose.covariance) c = 0.0;
    pose.pose.covariance[0]  = sigma2;        // xx
    pose.pose.covariance[7]  = sigma2;        // yy
    pose.pose.covariance[35] = sigma2 * 4.0;  // yaw — slightly looser
    pub_pose_->publish(pose);
  }
}

void DockScanMatchNode::crop_around_dock_in_lidar_frame(
    std::vector<Eigen::Vector3d>& frame,
    const Sophus::SE3d& lidar_to_base,
    const Sophus::SE3d& robot_in_map)
{
  // Express the dock anchor in the lidar frame.
  // points are in lidar frame (lidar_link_wheels). The dock anchor lives
  // in the map frame. lidar_in_map = robot_in_map * lidar_in_base, where
  // lidar_in_base = lidar_to_base.inverse() (note: the ROS extrinsic in
  // the parallel tree is base -> lidar; lidar_to_base is the same SE3
  // re-named per kinematic_icp pipeline convention — we treat it
  // symmetrically and double-check via .inverse()).
  const Sophus::SE3d lidar_in_map = robot_in_map * lidar_to_base;
  const Sophus::SE3d dock_in_lidar = lidar_in_map.inverse() * dock_anchor_in_map_;
  const Eigen::Vector3d dock_xy(dock_in_lidar.translation().x(),
                                dock_in_lidar.translation().y(),
                                0.0);
  const double r2 = crop_radius_m_ * crop_radius_m_;

  frame.erase(
      std::remove_if(frame.begin(), frame.end(),
                     [&](const Eigen::Vector3d& p) {
                       const double dx = p.x() - dock_xy.x();
                       const double dy = p.y() - dock_xy.y();
                       return (dx * dx + dy * dy) > r2;
                     }),
      frame.end());
}

void DockScanMatchNode::tick()
{
  // Pitfall 6: publish trusted=false EVERY tick — even before any scan.
  // Below we'll override with publish_match(...) on the trusted path.
  if (degraded_ || !matcher_)
  {
    publish_not_trusted();
    return;
  }

  // mtime watcher — Open Q4. Done before TF/scan checks so a fresh
  // capture takes effect on the very next scan tick.
  check_pcd_mtime_and_reload();

  // TF distance gate — Pitfall 5.
  double dist = 0.0;
  if (!robot_close_to_dock(dist))
  {
    publish_not_trusted();
    return;
  }

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mtx_);
    scan = last_scan_;
  }
  if (!scan)
  {
    publish_not_trusted();
    return;
  }

  // Project LaserScan -> Eigen points in the lidar frame.
  sensor_msgs::msg::PointCloud2 pc2;
  projector_.projectLaser(*scan, pc2, -1.0,
                          laser_geometry::channel_option::Timestamp);
  std::vector<Eigen::Vector3d> frame;
  frame.reserve(pc2.height * pc2.width);
  sensor_msgs::PointCloud2ConstIterator<float> it(pc2, "x");
  for (size_t i = 0; i < pc2.height * pc2.width; ++i, ++it)
  {
    frame.emplace_back(it[0], it[1], it[2]);
  }

  // Lookup lidar_to_base (static — base_footprint_wheels -> scan frame_id).
  Sophus::SE3d lidar_to_base;
  Sophus::SE3d robot_in_map;
  bool tf_ok = true;
  try
  {
    auto tf_lidar = tf_buffer_->lookupTransform(
        robot_frame_, scan->header.frame_id,
        tf2::TimePointZero, tf2::durationFromSec(0.2));
    lidar_to_base = tf2_to_sophus(tf_lidar.transform);
    auto tf_robot = tf_buffer_->lookupTransform(
        map_frame_, robot_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    robot_in_map = tf2_to_sophus(tf_robot.transform);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "tick TF lookup failed: %s", ex.what());
    tf_ok = false;
  }

  if (!tf_ok)
  {
    publish_not_trusted();
    return;
  }

  crop_around_dock_in_lidar_frame(frame, lidar_to_base, robot_in_map);

  ++register_frame_calls_;
  const auto result = matcher_->Match(frame, lidar_to_base);
  publish_match(result);
}

}  // namespace mowgli_lidar_docking
