// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "mowgli_behavior/docking_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "action_msgs/msg/goal_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "mowgli_lidar_docking/dock_approach_loader.hpp"
#include "mowgli_lidar_docking/dock_scan_io.hpp"
#include "mowgli_lidar_docking/dock_scan_meta_loader.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

BT::NodeStatus DockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string dock_id = "home_dock";
  if (auto res = getInput<std::string>("dock_id"))
  {
    dock_id = res.value();
  }

  std::string dock_type = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<DockAction>(ctx->node, "/dock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: /dock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  DockAction::Goal goal_msg;
  goal_msg.dock_id = dock_id;
  goal_msg.dock_type = dock_type;
  goal_msg.navigate_to_staging_pose = true;

  auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions{};
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "DockRobot: goal sent (dock_id='%s', dock_type='%s')",
              dock_id.c_str(),
              dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "DockRobot: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: docking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void DockRobot::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

BT::NodeStatus UndockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string dock_type = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<UndockAction>(ctx->node, "/undock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: /undock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  UndockAction::Goal goal_msg;
  goal_msg.dock_type = dock_type;

  auto send_goal_options = rclcpp_action::Client<UndockAction>::SendGoalOptions{};
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "UndockRobot: goal sent (dock_type='%s')",
              dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UndockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "UndockRobot: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: undocking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void UndockRobot::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// RecordResumeUndockFailure
// ---------------------------------------------------------------------------

BT::NodeStatus RecordResumeUndockFailure::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->resume_undock_failures++;
  RCLCPP_WARN(ctx->node->get_logger(),
              "RecordResumeUndockFailure: resume undock failures = %d",
              ctx->resume_undock_failures);
  return BT::NodeStatus::SUCCESS;
}

// ===========================================================================
// Phase 2 — LiDAR-based dock pose estimation BT nodes (Plan 02-06)
// ===========================================================================

namespace
{

// ---------------------------------------------------------------------------
// Local helpers shared across the 5 Phase-2 nodes.
// ---------------------------------------------------------------------------

/// Format a UTC ISO-8601 timestamp (YYYY-MM-DDTHH:MM:SSZ) for the current
/// system clock. Used for the captured_at field in dock_approach.yaml /
/// dock_scan_meta.yaml.
std::string iso_utc_now()
{
  const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  std::ostringstream os;
  os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return os.str();
}

/// Read dock_calibration.yaml — flat key=value (matches behavior_tree_node's
/// in-tree parser). Returns nullopt on any field missing / unparseable so
/// the caller can degrade gracefully.
std::optional<std::pair<double, double>> load_dock_xy(const std::string& path)
{
  std::ifstream f(path);
  if (!f.good()) return std::nullopt;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string content = ss.str();

  auto parse_double = [&](const std::string& key) -> std::optional<double> {
    const std::string needle = key + ":";
    auto pos = content.find(needle);
    if (pos == std::string::npos) return std::nullopt;
    pos += needle.size();
    while (pos < content.size() && (content[pos] == ' ' || content[pos] == '\t')) ++pos;
    auto end = pos;
    while (end < content.size() && content[end] != '\n' && content[end] != '\r') ++end;
    try
    {
      return std::stod(content.substr(pos, end - pos));
    }
    catch (...)
    {
      return std::nullopt;
    }
  };

  auto x = parse_double("dock_pose_x");
  auto y = parse_double("dock_pose_y");
  if (!x || !y) return std::nullopt;
  return std::make_pair(*x, *y);
}

/// Look up `map -> base_footprint` translation. Returns nullopt on TF
/// failure; the caller decides degraded behaviour.
std::optional<std::pair<double, double>> lookup_map_to_base(
    const std::shared_ptr<BTContext>& ctx, const char* who)
{
  try
  {
    auto tf = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    return std::make_pair(tf.transform.translation.x,
                          tf.transform.translation.y);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "%s: TF lookup map->base_footprint failed: %s", who, ex.what());
    return std::nullopt;
  }
}

/// Project a /scan_kicp message into a vector of Eigen::Vector3d (z=0)
/// in the scan's own frame. Inline implementation to avoid pulling
/// laser_geometry into mowgli_behavior — the math is trivial and the
/// dependency surface is smaller this way (Plan 02-06 deviation rationale,
/// captured in SUMMARY.md).
std::vector<Eigen::Vector3d> scan_to_points(const sensor_msgs::msg::LaserScan& scan)
{
  std::vector<Eigen::Vector3d> out;
  out.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double angle = scan.angle_min + i * scan.angle_increment;
    out.emplace_back(r * std::cos(angle), r * std::sin(angle), 0.0);
  }
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// RecordDockApproachPose (SPEC R-5 + R-11)
// ---------------------------------------------------------------------------

BT::NodeStatus RecordDockApproachPose::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  double match_max_age_s = 1.0;
  int refresh_age_days = 7;
  double refresh_min_inlier = 0.95;
  getInput("match_max_age_s", match_max_age_s);
  getInput("refresh_age_days", refresh_age_days);
  getInput("refresh_min_inlier", refresh_min_inlier);

  // ---- Resolve here_x, here_y, source -----------------------------------

  double here_x = 0.0, here_y = 0.0;
  std::string source;

  // Snapshot trust + age + pose under the lock; do file I/O without it.
  bool match_trusted = false;
  geometry_msgs::msg::PoseWithCovarianceStamped match_pose;
  std::chrono::steady_clock::time_point match_at;
  bool match_received = false;
  mowgli_interfaces::msg::DockMatchConfidence match_conf;
  sensor_msgs::msg::LaserScan scan;
  bool scan_received = false;
  {
    std::lock_guard<std::mutex> lk(ctx->context_mutex);
    match_trusted = ctx->last_dock_match_trusted;
    match_pose = ctx->latest_dock_match_pose;
    match_at = ctx->last_dock_match_pose_at;
    match_received = ctx->latest_dock_match_pose_received;
    match_conf = ctx->latest_dock_match_conf;
    scan = ctx->latest_scan_kicp;
    scan_received = ctx->latest_scan_kicp_received;
  }

  const auto now = std::chrono::steady_clock::now();
  const double match_age_s =
      std::chrono::duration<double>(now - match_at).count();
  const bool match_fresh_and_trusted =
      match_received && match_trusted && (match_age_s <= match_max_age_s);

  if (match_fresh_and_trusted)
  {
    here_x = match_pose.pose.pose.position.x;
    here_y = match_pose.pose.pose.position.y;
    source = "lidar";
  }
  else
  {
    auto xy = lookup_map_to_base(ctx, "RecordDockApproachPose");
    if (!xy)
    {
      // Neither LiDAR nor TF — log + return SUCCESS so UndockSequence
      // still completes; operator can re-attempt next undock.
      RCLCPP_WARN(ctx->node->get_logger(),
                  "RecordDockApproachPose: no trusted /dock_match/pose AND "
                  "no map->base_footprint TF — skipping write.");
      return BT::NodeStatus::SUCCESS;
    }
    here_x = xy->first;
    here_y = xy->second;
    source = "tf";
  }

  // ---- Compute yaw_to_dock from dock_calibration.yaml --------------------

  auto dock_xy = load_dock_xy(ctx->dock_calibration_path);
  if (!dock_xy)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "RecordDockApproachPose: cannot load dock_pose_x/y from %s — "
                "skipping write.", ctx->dock_calibration_path.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  const double yaw_to_dock = std::atan2(dock_xy->second - here_y,
                                        dock_xy->first - here_x);

  // ---- Write dock_approach.yaml -----------------------------------------

  mowgli_lidar_docking::DockApproach approach{here_x, here_y, yaw_to_dock,
                                              source, iso_utc_now()};
  if (!mowgli_lidar_docking::save_dock_approach_yaml(ctx->dock_approach_path,
                                                     approach))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "RecordDockApproachPose: save_dock_approach_yaml(%s) FAILED",
                 ctx->dock_approach_path.c_str());
    // Persist failure is non-fatal; the rest of UndockSequence has already
    // succeeded by the time we run.
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordDockApproachPose: wrote %s "
              "(source=%s here=(%.3f, %.3f) yaw_to_dock=%.1f°)",
              ctx->dock_approach_path.c_str(), source.c_str(),
              here_x, here_y, yaw_to_dock * 180.0 / M_PI);

  // ---- SPEC R-11 auto-refresh (PCD + meta) ------------------------------

  auto meta_opt =
      mowgli_lidar_docking::load_dock_scan_meta_yaml(ctx->dock_scan_meta_path);
  if (!meta_opt)
  {
    return BT::NodeStatus::SUCCESS;  // No prior meta — no refresh.
  }

  const std::string now_iso = iso_utc_now();
  if (!mowgli_lidar_docking::dock_scan_meta_age_exceeds(*meta_opt,
                                                        refresh_age_days,
                                                        now_iso))
  {
    return BT::NodeStatus::SUCCESS;  // Meta still fresh.
  }

  // Past the age horizon — gate on inlier ratio.
  if (match_conf.inlier_ratio < refresh_min_inlier)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "RecordDockApproachPose: dock_scan age past %d d but "
                "inlier_ratio %.2f < %.2f — skip refresh, keeping older "
                "snapshot.",
                refresh_age_days, match_conf.inlier_ratio, refresh_min_inlier);
    return BT::NodeStatus::SUCCESS;
  }

  if (!scan_received || scan.ranges.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "RecordDockApproachPose: refresh wanted but /scan_kicp not "
                "received — skipping refresh.");
    return BT::NodeStatus::SUCCESS;
  }

  // Atomic refresh: write PCD, then meta. PCD is the source of truth; meta
  // is the index. If PCD write fails, do NOT bump meta (T-04-01 mitigation).
  const auto pts = scan_to_points(scan);
  if (!mowgli_lidar_docking::save_dock_scan_pcd_atomic(ctx->dock_scan_path, pts))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "RecordDockApproachPose: save_dock_scan_pcd_atomic FAILED — "
                 "keeping older snapshot.");
    return BT::NodeStatus::SUCCESS;
  }

  // Build refreshed meta, preserving everything except captured_at +
  // point_count (we use whatever the new scan actually has).
  mowgli_lidar_docking::DockScanMeta meta_new = *meta_opt;
  meta_new.captured_at = now_iso;
  meta_new.point_count = static_cast<std::int64_t>(pts.size());
  if (!mowgli_lidar_docking::save_dock_scan_meta_yaml(ctx->dock_scan_meta_path,
                                                       meta_new))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "RecordDockApproachPose: save_dock_scan_meta_yaml FAILED — "
                 "PCD bumped but meta stale (operator must re-run calibrate).");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordDockApproachPose: dock_scan auto-refreshed "
              "(inlier=%.2f, points=%zu)",
              match_conf.inlier_ratio, pts.size());
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// ApproachDock (SPEC R-6)
// ---------------------------------------------------------------------------

BT::NodeStatus ApproachDock::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string path;
  getInput("dock_approach_path", path);
  if (path.empty()) path = ctx->dock_approach_path;

  timeout_sec_ = 60.0;
  getInput("timeout_sec", timeout_sec_);

  auto approach = mowgli_lidar_docking::load_dock_approach_yaml(path);
  if (!approach)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "ApproachDock: load_dock_approach_yaml(%s) failed — FAILURE",
                 path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_)
  {
    action_client_ =
        rclcpp_action::create_client<NavigateToPose>(ctx->node, "/navigate_to_pose");
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(3)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "ApproachDock: /navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.stamp = ctx->node->now();
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = approach->x;
  goal.pose.pose.position.y = approach->y;
  goal.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, approach->yaw_to_dock_rad);
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
  goal_future_ = action_client_->async_send_goal(goal, opts);
  goal_handle_.reset();
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(ctx->node->get_logger(),
              "ApproachDock: sending NavigateToPose goal "
              "x=%.3f y=%.3f yaw=%.1f° (timeout=%.0fs)",
              approach->x, approach->y,
              approach->yaw_to_dock_rad * 180.0 / M_PI, timeout_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachDock::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wall-clock timeout.
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed > timeout_sec_)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "ApproachDock: timeout after %.1fs — FAILURE", elapsed);
    if (goal_handle_)
    {
      action_client_->async_cancel_goal(goal_handle_);
    }
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_handle_)
  {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "ApproachDock: NavigateToPose goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  switch (goal_handle_->get_status())
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "ApproachDock: NavigateToPose SUCCEEDED");
      return BT::NodeStatus::SUCCESS;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "ApproachDock: NavigateToPose ABORTED");
      return BT::NodeStatus::FAILURE;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "ApproachDock: NavigateToPose CANCELED");
      return BT::NodeStatus::FAILURE;
    default:
      return BT::NodeStatus::RUNNING;
  }
}

void ApproachDock::onHalted()
{
  if (goal_handle_ && action_client_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "ApproachDock: halted, canceling goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// FineDock (SPEC R-7, R-8, R-9)
// ---------------------------------------------------------------------------

void FineDock::publish_zero()
{
  if (!cmd_pub_) return;
  geometry_msgs::msg::Twist twist;  // zero by default
  cmd_pub_->publish(twist);
}

BT::NodeStatus FineDock::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  crawl_speed_ms_ = 0.05;
  k_lateral_ = 1.5;
  k_yaw_ = 2.0;
  confidence_loss_timeout_s_ = 1.0;
  no_pose_bail_s_ = 5.0;
  std::string approach_path;
  getInput("crawl_speed_ms", crawl_speed_ms_);
  getInput("k_lateral", k_lateral_);
  getInput("k_yaw", k_yaw_);
  getInput("confidence_loss_timeout_s", confidence_loss_timeout_s_);
  getInput("no_pose_bail_s", no_pose_bail_s_);
  getInput("dock_approach_path", approach_path);
  if (approach_path.empty()) approach_path = ctx->dock_approach_path;

  // Lazy init twist publisher (twist_mux priority 15 per AI #13). FineDock
  // publishes ONLY here — never /cmd_vel, /cmd_vel_nav, /cmd_vel_teleop.
  if (!cmd_pub_)
  {
    cmd_pub_ = ctx->node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_docking", 10);
  }

  auto approach = mowgli_lidar_docking::load_dock_approach_yaml(approach_path);
  if (approach)
  {
    yaw_to_dock_rad_ = approach->yaw_to_dock_rad;
  }
  // dock_target = dock_calibration.yaml — that's the contact point.
  auto dock_xy = load_dock_xy(ctx->dock_calibration_path);
  if (dock_xy)
  {
    dock_target_x_ = dock_xy->first;
    dock_target_y_ = dock_xy->second;
  }

  start_time_ = std::chrono::steady_clock::now();
  confidence_loss_streak_start_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "FineDock: starting (crawl=%.2fm/s, k_lat=%.1f, k_yaw=%.1f, "
              "yaw_to_dock=%.1f°, target=(%.3f, %.3f))",
              crawl_speed_ms_, k_lateral_, k_yaw_,
              yaw_to_dock_rad_ * 180.0 / M_PI,
              dock_target_x_, dock_target_y_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FineDock::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Snapshot context state under lock (release before publishing).
  bool emergency = false;
  bool charging = false;
  bool match_received = false;
  bool match_trusted = false;
  geometry_msgs::msg::PoseWithCovarianceStamped match_pose;
  std::chrono::steady_clock::time_point match_at;
  mowgli_interfaces::msg::DockMatchConfidence match_conf;
  {
    std::lock_guard<std::mutex> lk(ctx->context_mutex);
    emergency = ctx->latest_emergency.active_emergency ||
                ctx->latest_emergency.latched_emergency;
    charging = ctx->latest_status.is_charging;
    match_received = ctx->latest_dock_match_pose_received;
    match_trusted = ctx->last_dock_match_trusted;
    match_pose = ctx->latest_dock_match_pose;
    match_at = ctx->last_dock_match_pose_at;
    match_conf = ctx->latest_dock_match_conf;
  }

  // R-9: emergency hold. onHalted in the BT framework will fire on the
  // outer halt, but the explicit zero-twist + FAILURE return here closes
  // the gap if the parent uses a non-reactive sequence.
  if (emergency)
  {
    publish_zero();
    RCLCPP_WARN(ctx->node->get_logger(), "FineDock: emergency — FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // R-7 success path.
  if (charging)
  {
    publish_zero();
    RCLCPP_INFO(ctx->node->get_logger(),
                "FineDock: is_charging=true — SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  const auto now = std::chrono::steady_clock::now();

  // Degraded-mode contract (WARNING-9; distinct from R-8): no /dock_match/pose
  // EVER received. Hold zero for up to no_pose_bail_s_, then FAILURE.
  if (!match_received)
  {
    publish_zero();
    const double elapsed = std::chrono::duration<double>(now - start_time_).count();
    if (elapsed > no_pose_bail_s_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "FineDock: no /dock_match/pose received within %.1fs — bailing",
                   no_pose_bail_s_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // Pose freshness — if it's been more than 2 s since last pose AND we
  // already passed the no_pose_bail_s_ window, treat as confidence-loss
  // surrogate (R-8 spirit).
  const double pose_age_s = std::chrono::duration<double>(now - match_at).count();
  if (pose_age_s > 2.0)
  {
    publish_zero();
    const double elapsed = std::chrono::duration<double>(now - start_time_).count();
    if (elapsed > no_pose_bail_s_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "FineDock: /dock_match/pose stale (age=%.1fs > 2.0s) and "
                   "elapsed=%.1fs past bail window — FAILURE",
                   pose_age_s, elapsed);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // R-8: trust-loss streak abort.
  if (!match_trusted)
  {
    if (!confidence_loss_streak_start_)
    {
      confidence_loss_streak_start_ = now;
      publish_zero();
      return BT::NodeStatus::RUNNING;
    }
    const double streak = std::chrono::duration<double>(
        now - *confidence_loss_streak_start_).count();
    if (streak >= confidence_loss_timeout_s_)
    {
      publish_zero();
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "LiDAR ICP confidence dropped: inlier_ratio=%.2f rmse=%.3f "
                   "— aborting fine dock", match_conf.inlier_ratio, match_conf.rmse_m);
      return BT::NodeStatus::FAILURE;
    }
    publish_zero();
    return BT::NodeStatus::RUNNING;
  }

  // Trusted — clear any prior streak.
  confidence_loss_streak_start_.reset();

  // ---- Closed-loop control --------------------------------------------------
  // Lateral_y error in dock-aligned frame: rotate (pose - target) by
  // -yaw_to_dock_rad to put it in dock-aligned coordinates, then take y.
  const double cur_x = match_pose.pose.pose.position.x;
  const double cur_y = match_pose.pose.pose.position.y;
  const double cur_yaw = tf2::getYaw(match_pose.pose.pose.orientation);

  const double dx = cur_x - dock_target_x_;
  const double dy = cur_y - dock_target_y_;
  const double cs = std::cos(-yaw_to_dock_rad_);
  const double sn = std::sin(-yaw_to_dock_rad_);
  const double lateral_y_err = sn * dx + cs * dy;

  // Yaw error: shortest angle from current yaw to yaw_to_dock_rad_.
  double yaw_err = yaw_to_dock_rad_ - cur_yaw;
  while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

  // Sign convention: positive lateral_y_err means robot is LEFT of the
  // approach line in the dock-aligned frame → steer right (negative
  // angular.z). Sim-test (Plan 02-08) catches sign errors before hardware.
  geometry_msgs::msg::Twist twist;
  twist.linear.x = crawl_speed_ms_;
  twist.angular.z = -(k_lateral_ * lateral_y_err) - (k_yaw_ * yaw_err);
  cmd_pub_->publish(twist);

  return BT::NodeStatus::RUNNING;
}

void FineDock::onHalted()
{
  // Critical for R-9 E-Stop semantics — drive cmd_vel to zero before yielding.
  publish_zero();
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (ctx && ctx->node)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "FineDock halted");
  }
}

// ---------------------------------------------------------------------------
// PreUndockClearanceCheck (SPEC R-12)
// ---------------------------------------------------------------------------

BT::NodeStatus PreUndockClearanceCheck::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  double min_undock_distance_m = 1.5;
  double rear_safety_buffer_m = 0.20;
  double rear_sector_half_deg = 30.0;
  double scan_max_age_s = 0.5;
  getInput("min_undock_distance_m", min_undock_distance_m);
  getInput("rear_safety_buffer_m", rear_safety_buffer_m);
  getInput("rear_sector_half_deg", rear_sector_half_deg);
  getInput("scan_max_age_s", scan_max_age_s);

  // Snapshot scan + receipt state under lock.
  sensor_msgs::msg::LaserScan scan;
  bool received = false;
  std::chrono::steady_clock::time_point scan_at;
  {
    std::lock_guard<std::mutex> lk(ctx->context_mutex);
    scan = ctx->latest_scan_kicp;
    received = ctx->latest_scan_kicp_received;
    scan_at = ctx->last_scan_kicp_at;
  }

  if (!received)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "PreUndockClearanceCheck: no /scan_kicp received — assuming unsafe, FAILURE");
    return BT::NodeStatus::FAILURE;
  }
  const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - scan_at).count();
  if (age > scan_max_age_s)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "PreUndockClearanceCheck: /scan_kicp age %.2fs > %.2fs — FAILURE",
                age, scan_max_age_s);
    return BT::NodeStatus::FAILURE;
  }

  const double half_rad = rear_sector_half_deg * M_PI / 180.0;
  const double required = min_undock_distance_m + rear_safety_buffer_m;
  double min_r = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double angle = scan.angle_min + i * scan.angle_increment;
    // angle == ±π is the rear (forward = +X = 0 rad on lidar_link_wheels;
    // YF500 has the LD19 mounted forward — see SPEC R-12 + AI #15 note).
    double delta = angle - M_PI;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    if (std::abs(delta) <= half_rad)
    {
      if (r < min_r) min_r = r;
    }
  }

  if (!std::isfinite(min_r))
  {
    // Empty rear sector (no valid returns) — log + treat as success
    // (genuinely no obstacle; rare in practice).
    RCLCPP_INFO(ctx->node->get_logger(),
                "PreUndockClearanceCheck: rear sector empty — SUCCESS");
    return BT::NodeStatus::SUCCESS;
  }

  if (min_r < required)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "PreUndockClearanceCheck: rear clearance %.2f m < required %.2f m — FAILURE",
                min_r, required);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(ctx->node->get_logger(),
              "PreUndockClearanceCheck: rear clearance %.2f m — SUCCESS", min_r);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// PostUndockRtkValidation (SPEC R-13)
// ---------------------------------------------------------------------------

BT::NodeStatus PostUndockRtkValidation::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  double warn_threshold_m = 0.5;
  double fail_threshold_m = 1.5;
  getInput("warn_threshold_m", warn_threshold_m);
  getInput("fail_threshold_m", fail_threshold_m);

  // Skip silently if not RTK_FIXED (gps_fix_type 4).
  uint8_t fix_type = 0;
  double gps_x = 0.0, gps_y = 0.0;
  {
    std::lock_guard<std::mutex> lk(ctx->context_mutex);
    fix_type = ctx->gps_fix_type;
    gps_x = ctx->gps_x;
    gps_y = ctx->gps_y;
  }
  if (fix_type < 4)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "PostUndockRtkValidation: not RTK_FIXED (fix_type=%u) — skipping",
                fix_type);
    return BT::NodeStatus::SUCCESS;
  }

  // Look up odom -> base_footprint (post-undock pose). The BT runs this
  // node AFTER CalibrateHeadingFromUndock; the EKF state reflects post-
  // undock. We compare odom-frame against gps for a consistency check.
  double odom_x = 0.0, odom_y = 0.0;
  try
  {
    auto tf = ctx->tf_buffer->lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    odom_x = tf.transform.translation.x;
    odom_y = tf.transform.translation.y;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "PostUndockRtkValidation: TF odom->base_footprint failed (%s) — "
                "cannot validate, returning SUCCESS",
                ex.what());
    return BT::NodeStatus::SUCCESS;
  }

  const double error_xy = std::hypot(gps_x - odom_x, gps_y - odom_y);

  if (error_xy <= warn_threshold_m)
  {
    return BT::NodeStatus::SUCCESS;
  }
  if (error_xy <= fail_threshold_m)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "PostUndockRtkValidation: odom-vs-GPS discrepancy %.2f m — "
                "dock_pose_suspect=true", error_xy);
    {
      std::lock_guard<std::mutex> lk(ctx->context_mutex);
      ctx->dock_pose_suspect = true;
    }
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(ctx->node->get_logger(),
               "PostUndockRtkValidation: odom-vs-GPS discrepancy %.2f m exceeds "
               "%.2f m threshold — FAILURE",
               error_xy, fail_threshold_m);
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Factory registration helper
// ---------------------------------------------------------------------------

void register_docking_nodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<RecordDockApproachPose>("RecordDockApproachPose");
  factory.registerNodeType<ApproachDock>("ApproachDock");
  factory.registerNodeType<FineDock>("FineDock");
  factory.registerNodeType<PreUndockClearanceCheck>("PreUndockClearanceCheck");
  factory.registerNodeType<PostUndockRtkValidation>("PostUndockRtkValidation");
}

}  // namespace mowgli_behavior
