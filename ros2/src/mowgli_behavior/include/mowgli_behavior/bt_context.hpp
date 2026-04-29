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
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "mowgli_interfaces/msg/coverage_waypoint.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace mowgli_behavior
{

/// Shared context passed to all BehaviorTree nodes via the blackboard.
///
/// The main node keeps this struct alive and updates it from ROS2 topic
/// callbacks before each tree tick.  BT nodes retrieve a shared_ptr to
/// this struct with:
///
///   auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
struct BTContext
{
  /// ROS2 node used by action/service nodes to create clients.
  rclcpp::Node::SharedPtr node;

  // -----------------------------------------------------------------------
  // Latest sensor state (updated by topic subscribers in the main node)
  // -----------------------------------------------------------------------

  mowgli_interfaces::msg::Status latest_status;
  mowgli_interfaces::msg::Emergency latest_emergency;
  mowgli_interfaces::msg::Power latest_power;

  /// Timestamp of the last emergency message received.
  std::chrono::steady_clock::time_point last_emergency_time{};

  // -----------------------------------------------------------------------
  // Thread safety
  // -----------------------------------------------------------------------

  /// Mutex protecting fields written by subscriber callbacks and read by
  /// BT condition/action nodes.  Use std::lock_guard for RAII locking.
  mutable std::mutex context_mutex;

  // -----------------------------------------------------------------------
  // Command state (set by HighLevelControl service handler)
  // -----------------------------------------------------------------------

  /// Last command received via the ~/high_level_control service.
  /// Constants match HighLevelControl.srv (COMMAND_START=1, COMMAND_HOME=2,
  /// COMMAND_S1=3, COMMAND_S2=4, COMMAND_MANUAL_MOW=7,
  /// COMMAND_RESET_EMERGENCY=254, …).
  uint8_t current_command{0};

  // -----------------------------------------------------------------------
  // Derived / convenience fields (computed from latest_* messages)
  // -----------------------------------------------------------------------

  float battery_percent{100.0f};
  float gps_quality{0.0f};

  /// Latest GPS position in map frame (from /gps/absolute_pose)
  double gps_x{0.0};
  double gps_y{0.0};

  // -----------------------------------------------------------------------
  // GPS quality classification (derived from gps_quality / fix_type)
  // -----------------------------------------------------------------------

  /// GPS fix type: 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float
  uint8_t gps_fix_type{0};

  /// true when RTK fixed (fix_type >= 4 and gps_quality > 80%)
  bool gps_is_fixed{false};

  // -----------------------------------------------------------------------
  // Localization quality flags (set by boundary/replan monitors)
  // -----------------------------------------------------------------------

  /// Set to true when ObstacleTracker publishes updated obstacles that
  /// differ from the last coverage plan.
  bool replan_needed{false};

  /// Set to true when the robot is outside all allowed polygons.
  bool boundary_violation{false};

  /// Set to true when the robot is outside all allowed polygons by more
  /// than lethal_boundary_margin_m. Escalates the BoundaryGuard from
  /// "try to navigate back inside" to "emergency stop + wait for
  /// operator" — blade/motors past this margin can do real damage.
  bool lethal_boundary_violation{false};

  /// Current navigation mode: "precise" or "degraded"
  std::string current_nav_mode{"precise"};

  /// True if it was raining when the current mowing session started.
  /// Set by WasRainingAtStart, checked by IsNewRain.
  bool raining_at_mow_start{false};

  // -----------------------------------------------------------------------
  // Session-level counters (reset at mowing session start)
  // -----------------------------------------------------------------------

  /// Number of resume-undock failures this mowing session.  Prevents
  /// infinite dock/charge/undock cycles when undocking is mechanically broken.
  int resume_undock_failures{0};

  // -----------------------------------------------------------------------
  // Odom-frame snapshot for heading calibration during undock.
  //
  // RecordUndockStart writes the odom→base_footprint translation here just
  // before the BackUp behavior runs; CalibrateHeadingFromUndock diffs the
  // current odom→base_footprint against this snapshot to derive the actual
  // physical displacement (GPS-free). Raw GPS is intentionally avoided —
  // RTK Fixed→Float drops near the dock cause metre-scale jumps that
  // poisoned the previous GPS-based calculation (issue #73). Yaw is then
  // converted to map frame via the current map→odom rotation.
  // -----------------------------------------------------------------------
  double undock_start_x{0.0};
  double undock_start_y{0.0};
  bool undock_start_recorded{false};

  // -----------------------------------------------------------------------
  // Per-session flags reset by ClearCommand at session end
  // -----------------------------------------------------------------------

  /// True after any seeding node (CalibrateHeadingFromUndock or
  /// SeedYawFromMotion) has successfully published a set_pose to ekf_map
  /// during the current autonomous session. Prevents the forward-drive
  /// SeedYawFromMotion from re-triggering when the root ReactiveSequence
  /// halts MowingSequence (e.g., BoundaryGuard or GpsMode transition) and
  /// later re-enters it from the top.
  bool yaw_seeded_this_session{false};

  // -----------------------------------------------------------------------
  // Docking point (set from parameter or service call)
  // -----------------------------------------------------------------------

  double dock_x{0.0};
  double dock_y{0.0};
  double dock_yaw{0.0};

  // -----------------------------------------------------------------------
  // Coverage plan state (Plan 01-08).
  //
  // Single-shot pull-down model: PlanCoverageGoal calls
  // /coverage_planner_node/plan_coverage once at AUTONOMOUS branch entry
  // (D-04) and writes the resulting CoverageWaypoint[] here. FollowCoveragePlan
  // consumes the plan sequentially and delegates per-area resume state to
  // the planner via /coverage_planner_node/write_checkpoint
  // (RESEARCH §10 Q1 lock — BT NEVER touches the filesystem).
  //
  // Empty when no plan is loaded. Cleared by ClearCommand at session end.
  // -----------------------------------------------------------------------

  /// Coverage plan written by PlanCoverageGoal, consumed by FollowCoveragePlan.
  std::vector<mowgli_interfaces::msg::CoverageWaypoint> coverage_plan;

  /// Mow angle (degrees, in [0, 180)) actually applied by the planner — copied
  /// from PlanCoverage::Result::metadata.mow_angle_used_deg by PlanCoverageGoal
  /// after a successful goal. FollowCoveragePlan stamps this into every
  /// Checkpoint it dispatches via WriteCheckpoint.srv so the next plan's
  /// derive_mow_angle (R-9) sees the correct previous-angle on read-back.
  /// Defaults to 0.0 (= "no plan ran yet"); read by FollowCoveragePlan only
  /// when an actual plan was loaded into coverage_plan, so the default is
  /// never persisted to a real checkpoint file.
  double last_mow_angle_used_deg{0.0};

  // -----------------------------------------------------------------------
  // TF buffer (shared across all BT nodes)
  // -----------------------------------------------------------------------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // -----------------------------------------------------------------------
  // Shared helper node for service calls (avoids creating/destroying DDS
  // participants on every call — the main node is in rclcpp::spin so it
  // cannot be used directly with spin_until_future_complete).
  // -----------------------------------------------------------------------
  rclcpp::Node::SharedPtr helper_node;
};

}  // namespace mowgli_behavior
