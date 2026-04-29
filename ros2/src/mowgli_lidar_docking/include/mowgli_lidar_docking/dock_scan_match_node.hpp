// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
#pragma once

// Plan 02-04 SPEC R-2 + R-3: dock_scan_match rclcpp node.
//
// Subscribes /scan_kicp (SensorDataQoS) and publishes:
//   * /dock_match/pose       — geometry_msgs/PoseWithCovarianceStamped,
//                              reliable depth=1
//   * /dock_match/confidence — mowgli_interfaces/DockMatchConfidence,
//                              SensorDataQoS
//
// 10 Hz wall-timer cadence (D-07). Reads dock_calibration.yaml +
// dock_scan.pcd at startup; if either file is missing, enters degraded
// mode (continuously publishes confidence{trusted=false} + retries the
// load every 30 s so the operator's GUI Recapture button hot-attaches the
// matcher without restarting the node).
//
// SPEC gates implemented:
//   * R-2 publish_rate_hz default 10.0 (>= 5 Hz minimum)
//   * R-3 trust thresholds: min_inlier_ratio=0.70, max_rmse_m=0.05
//          → DockMatchConfidence.trusted is precomputed via is_trusted()
//   * Pitfall 5 TF distance gate: when robot is > gate_distance_m (5 m)
//          from the dock anchor, RegisterFrame is skipped to save Pi5
//          CPU; trusted=false published.
//   * Open Q4 mtime watcher: each tick checks
//          std::filesystem::last_write_time(dock_scan.pcd); if newer,
//          calls matcher_->Reload() with the freshly-loaded points.
//   * Pitfall 6 baseline: the 10 Hz timer publishes trusted=false on its
//          first tick BEFORE any scan has arrived, so consumers see a
//          clear "not yet trusted" baseline (FineDock onRunning bail
//          condition).
//
// Architecture Invariant #1 honoured: this node does NOT publish TF, does
// NOT subscribe to /odometry/filtered_map, and only reads
// `map → base_footprint_wheels` (parallel TF tree from
// kinematic_icp_scan_frame_relay). The published /dock_match/pose is a
// measurement; Plan 02-05's dock_yaw_to_set_pose cascade is the SOLE path
// that turns this into an EKF seed (via /set_pose, never via TF).

#include "mowgli_lidar_docking/idock_matcher.hpp"
#include "mowgli_lidar_docking/kinematic_icp_dock_matcher.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <mowgli_interfaces/msg/dock_match_confidence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sophus/se3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mowgli_lidar_docking
{

class DockScanMatchNode : public rclcpp::Node
{
public:
  /// Test-friendly ctor — accepts a NodeOptions with parameter overrides.
  explicit DockScanMatchNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{});

  /// Test-only constructor: inject a fully-formed matcher and skip the
  /// kinematic_icp dependency. The node still loads dock_calibration.yaml
  /// for the dock anchor + dock_scan_meta.yaml so the test can drive
  /// degraded / TF-gate / mtime-watcher paths without standing up
  /// kiss_icp.
  ///
  /// `injected_matcher` may be nullptr — that simulates the "PCD missing
  /// at startup" path (degraded mode).
  DockScanMatchNode(const rclcpp::NodeOptions& opts,
                    std::unique_ptr<IDockMatcher> injected_matcher,
                    std::optional<Sophus::SE3d> dock_anchor_in_map);

  // Test-introspection accessors. Production code does not call these;
  // they exist so test_dock_scan_match_node.cpp can assert internal state
  // without spinning the node.
  bool degraded_for_test() const { return degraded_; }
  std::size_t register_frame_calls_for_test() const { return register_frame_calls_; }
  void inject_scan_for_test(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(scan_mtx_);
    last_scan_ = scan;
  }
  /// Inject a fake `map -> base_footprint_wheels` translation for the TF
  /// distance gate test. The fake position bypasses tf_buffer_ when set.
  void inject_robot_in_map_for_test(const Sophus::SE3d& robot_in_map)
  {
    std::lock_guard<std::mutex> lock(test_inject_mtx_);
    injected_robot_in_map_ = robot_in_map;
  }
  /// Drive a single tick synchronously — bypasses the wall_timer for
  /// deterministic tests.
  void tick_once_for_test() { tick(); }

private:
  // ---- Initialisation helpers ----
  void declare_all_parameters();
  void init_publishers();
  void init_tf();
  void init_subscriber_and_timer();
  /// Attempts to (re)load dock_scan.pcd + dock_calibration.yaml + meta.
  /// Returns true on success, false if any required file is missing or
  /// malformed. Called from the constructor and from the 30-s degraded-
  /// mode polling timer.
  bool try_load_matcher_files();

  // ---- Per-tick path ----
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  void tick();
  void poll_for_pcd_in_degraded_mode();
  void check_pcd_mtime_and_reload();
  /// TF distance gate: returns true iff robot is within gate_distance_m of
  /// the dock anchor. Side-effect: when the test injection path is set,
  /// uses that translation instead of the live tf_buffer_.
  bool robot_close_to_dock(double& out_distance);
  void publish_not_trusted();
  void publish_match(const MatchResult& result);
  /// Cropping helper: in-place removes points further than crop_radius_m
  /// from the dock anchor, expressed in the lidar frame.
  void crop_around_dock_in_lidar_frame(std::vector<Eigen::Vector3d>& frame,
                                       const Sophus::SE3d& lidar_to_base,
                                       const Sophus::SE3d& robot_in_map);

  // ---- Parameters (all declared in declare_all_parameters) ----
  double crop_radius_m_{3.0};
  double publish_rate_hz_{10.0};
  double max_correspondence_m_{0.30};
  int max_iterations_{20};
  double min_inlier_ratio_{0.70};
  double max_rmse_m_{0.05};
  double gate_distance_m_{5.0};
  std::string dock_calibration_path_;
  std::string dock_scan_path_;
  std::string dock_scan_meta_path_;
  std::string scan_topic_;
  std::string pose_topic_;
  std::string confidence_topic_;
  std::string robot_frame_;     ///< default "base_footprint_wheels" (parallel TF tree)
  std::string map_frame_;       ///< default "map"

  // ---- State ----
  bool degraded_{true};                ///< true until try_load_matcher_files succeeds
  Sophus::SE3d dock_anchor_in_map_;
  std::optional<std::filesystem::file_time_type> cached_pcd_mtime_;
  std::unique_ptr<IDockMatcher> matcher_;
  std::size_t register_frame_calls_{0};
  bool warned_no_pcd_{false};

  // ---- ROS plumbing ----
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<mowgli_interfaces::msg::DockMatchConfidence>::SharedPtr pub_conf_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr degraded_poll_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  laser_geometry::LaserProjection projector_;

  // ---- Concurrency ----
  std::mutex scan_mtx_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_scan_;
  std::mutex test_inject_mtx_;
  std::optional<Sophus::SE3d> injected_robot_in_map_;
};

}  // namespace mowgli_lidar_docking
