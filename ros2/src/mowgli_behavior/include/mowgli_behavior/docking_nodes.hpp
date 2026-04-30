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

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /dock_robot action to dock the robot.
///
/// Input ports:
///   dock_id   (string) – named dock instance (e.g. "home_dock")
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class DockRobot : public BT::StatefulActionNode
{
public:
  using DockAction = nav2_msgs::action::DockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<DockAction>;

  DockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_id", "home_dock", "Named dock instance"),
            BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<DockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
};

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /undock_robot action to undock the robot.
///
/// Input ports:
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class UndockRobot : public BT::StatefulActionNode
{
public:
  using UndockAction = nav2_msgs::action::UndockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<UndockAction>;

  UndockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<UndockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
};

// ---------------------------------------------------------------------------
// RecordResumeUndockFailure
// ---------------------------------------------------------------------------

/// Increments the resume_undock_failures counter in BTContext.
/// Always returns SUCCESS so it can be placed inside any sequence.
class RecordResumeUndockFailure : public BT::SyncActionNode
{
public:
  RecordResumeUndockFailure(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ===========================================================================
// Phase 2 — LiDAR-based dock pose estimation BT nodes (Plan 02-06)
// ===========================================================================
//
// 5 new nodes implementing SPEC R-5..R-9 + R-11..R-13. These run alongside
// the legacy DockRobot/UndockRobot above; Plan 02-07 wires them into
// main_tree.xml and migrates the 4 DockRobot sites to ApproachDock+FineDock.
//
// CLAUDE.md AI #1 compliance: NONE of these BT nodes publish TF or feed
// robot_localization. RecordDockApproachPose writes a file. ApproachDock
// dispatches a NavigateToPose action goal. FineDock publishes ONLY to
// /cmd_vel_docking (twist_mux priority 15 per AI #13). PreUndockClearanceCheck
// and PostUndockRtkValidation are pure pre/post-condition checkers.

// ---------------------------------------------------------------------------
// RecordDockApproachPose (SPEC R-5 + R-11)
// ---------------------------------------------------------------------------

/// Persists the post-undock pose to dock_approach.yaml. Triggers SPEC R-11
/// dock_scan.pcd auto-refresh when meta age > 7 days AND the latest
/// /dock_match/confidence.inlier_ratio >= 0.95.
///
/// Source preference: trusted /dock_match/pose (within 1 s of arrival) →
/// map -> base_footprint TF (fallback). On both-fail, returns SUCCESS
/// without writing (operator can re-attempt; UndockSequence still completes).
class RecordDockApproachPose : public BT::SyncActionNode
{
public:
  RecordDockApproachPose(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>(
            "match_max_age_s", 1.0,
            "Max age (seconds) of /dock_match/pose to accept as 'trusted'"),
        BT::InputPort<int>(
            "refresh_age_days", 7,
            "Min PCD age (days) before R-11 auto-refresh becomes eligible"),
        BT::InputPort<double>(
            "refresh_min_inlier", 0.95,
            "Min /dock_match/confidence.inlier_ratio for the R-11 refresh trigger"),
    };
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// ApproachDock (SPEC R-6)
// ---------------------------------------------------------------------------

/// Loads dock_approach.yaml and dispatches a NavigateToPose action goal to
/// (dock_approach_x, dock_approach_y, dock_approach_yaw_to_dock_rad). Returns
/// SUCCESS on action SUCCEEDED (within 60 s), FAILURE on missing yaml /
/// rejected goal / aborted goal.
///
/// Replaces opennav_docking::SimpleChargingDock RPP approach for all 4 dock
/// sites in main_tree.xml (Plan 02-07 migration).
class ApproachDock : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ApproachDock(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>(
            "dock_approach_path", "",
            "Path to dock_approach.yaml; empty -> ctx->dock_approach_path"),
        BT::InputPort<double>(
            "timeout_sec", 60.0,
            "Wall-clock timeout for the NavigateToPose action (seconds)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::chrono::steady_clock::time_point start_time_;
  double timeout_sec_{60.0};
};

// ---------------------------------------------------------------------------
// FineDock (SPEC R-7, R-8, R-9)
// ---------------------------------------------------------------------------

/// Closed-loop crawl of the final 1.5 m under continuous /dock_match/pose
/// correction. Publishes /cmd_vel_docking (twist_mux priority 15 per AI #13).
///
/// SUCCESS on is_charging == true. FAILURE on:
///   * E-Stop (active or latched) — onHalted publishes zero twist (R-9)
///   * /dock_match/confidence.trusted == false for >= 1 s (R-8)
///   * No /dock_match/pose received within 5 s of onStart (degraded-mode
///     contract, distinct from R-8 — see WARNING-9)
///
/// Sign convention (Open Q5 resolution): positive lateral_y_err means
/// the robot is to the LEFT of the dock-aligned approach line; we steer
/// right (negative angular.z) to correct.
class FineDock : public BT::StatefulActionNode
{
public:
  FineDock(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>(
            "crawl_speed_ms", 0.05, "Forward crawl speed (m/s)"),
        BT::InputPort<double>(
            "k_lateral", 1.5, "P-gain on lateral_y error (rad/s per m)"),
        BT::InputPort<double>(
            "k_yaw", 2.0, "P-gain on yaw error (rad/s per rad)"),
        BT::InputPort<std::string>(
            "dock_approach_path", "",
            "Path to dock_approach.yaml; empty -> ctx->dock_approach_path"),
        BT::InputPort<double>(
            "confidence_loss_timeout_s", 1.0,
            "Trust-loss streak duration before FAILURE (R-8)"),
        BT::InputPort<double>(
            "no_pose_bail_s", 5.0,
            "No-pose grace period before FAILURE (degraded-mode contract)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publish_zero();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::optional<std::chrono::steady_clock::time_point> confidence_loss_streak_start_;
  std::chrono::steady_clock::time_point start_time_;
  double yaw_to_dock_rad_{0.0};
  double dock_target_x_{0.0};
  double dock_target_y_{0.0};
  double crawl_speed_ms_{0.05};
  double k_lateral_{1.5};
  double k_yaw_{2.0};
  double confidence_loss_timeout_s_{1.0};
  double no_pose_bail_s_{5.0};
};

// ---------------------------------------------------------------------------
// PreUndockClearanceCheck (SPEC R-12)
// ---------------------------------------------------------------------------

/// Pre-flight rear-sector free-space probe over /scan_kicp. The LiDAR
/// frame is `lidar_link_wheels` with forward = +X = 0 rad; rear = ±π.
/// The default rear sector is 180° ± 30° (60° arc). FAILURE when the
/// minimum range in that arc is below (min_undock_distance_m +
/// rear_safety_buffer_m), default 1.5 + 0.20 = 1.70 m.
class PreUndockClearanceCheck : public BT::SyncActionNode
{
public:
  PreUndockClearanceCheck(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>(
            "min_undock_distance_m", 1.5,
            "BackUp distance the upcoming UndockSequence will request"),
        BT::InputPort<double>(
            "rear_safety_buffer_m", 0.20,
            "Extra clearance margin beyond min_undock_distance_m"),
        BT::InputPort<double>(
            "rear_sector_half_deg", 30.0,
            "Half-angle of the rear sweep cone (degrees)"),
        BT::InputPort<double>(
            "scan_max_age_s", 0.5,
            "Max age of /scan_kicp before FAILURE (assume unsafe)"),
    };
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// PostUndockRtkValidation (SPEC R-13)
// ---------------------------------------------------------------------------

/// Cross-checks odom -> base_footprint at undock end against the latest
/// /gps/absolute_pose. Skipped silently when GPS is not RTK_FIXED. Three
/// thresholds (configurable via ports):
///   * error_xy <= warn_threshold_m (0.5 m default): SUCCESS silent
///   * warn_threshold_m < error_xy <= fail_threshold_m (1.5 m default):
///       SUCCESS + WARN log + sets ctx->dock_pose_suspect = true
///   * error_xy > fail_threshold_m: FAILURE (UndockSequence aborts;
///       operator alerted via outer BT)
class PostUndockRtkValidation : public BT::SyncActionNode
{
public:
  PostUndockRtkValidation(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("warn_threshold_m", 0.5),
        BT::InputPort<double>("fail_threshold_m", 1.5),
    };
  }

  BT::NodeStatus tick() override;
};

/// Helper that registers all 5 Phase-2 BT nodes with the BT factory. Called
/// from register_nodes.cpp::registerAllNodes alongside the legacy node
/// registrations.
void register_docking_nodes(BT::BehaviorTreeFactory& factory);

}  // namespace mowgli_behavior
