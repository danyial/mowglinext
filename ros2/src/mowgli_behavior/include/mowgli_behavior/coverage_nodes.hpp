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
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/msg/coverage_waypoint.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "mowgli_interfaces/srv/write_checkpoint.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// should_blade_enable — pure helper, blade ON iff the segment_type is one of
// the three mowing segment types per CLAUDE.md "What NOT to Do" + plan 01-08
// task 2 safety contract. Free function so unit tests can reach it without
// spinning an action server. T-08-01 mitigation lives here.
// ---------------------------------------------------------------------------
inline bool should_blade_enable(const mowgli_interfaces::msg::CoverageWaypoint& wp)
{
  using CW = mowgli_interfaces::msg::CoverageWaypoint;
  return wp.segment_type == CW::SEGMENT_MOWING_BOUSTROPHEDON ||
         wp.segment_type == CW::SEGMENT_OUTLINE_WORKING_AREA ||
         wp.segment_type == CW::SEGMENT_OUTLINE_OBSTACLE;
}

// ---------------------------------------------------------------------------
// PlanCoverageGoal — sends one PlanCoverage.action goal to coverage_planner
// at AUTONOMOUS branch entry (D-04). Writes the resulting plan to
// BTContext::coverage_plan; FollowCoveragePlan consumes it sequentially.
//
// Returns:
//   SUCCESS  — plan received and stored in ctx->coverage_plan.
//   FAILURE  — action server unavailable, goal rejected, planner returned
//              success=false, or the plan was empty.
// ---------------------------------------------------------------------------
class PlanCoverageGoal : public BT::StatefulActionNode
{
public:
  using Action = mowgli_interfaces::action::PlanCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  PlanCoverageGoal(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  bool result_requested_ = false;
};

// ---------------------------------------------------------------------------
// FollowCoveragePlan — consumes BTContext::coverage_plan sequentially.
//
// Per waypoint segment_type (per Plan 01-08 D-03 dispatch table):
//   UNDOCK / TRANSIT / DOCK_APPROACH / DOCKING / RETURN_TO_DOCK
//     -> nav2_msgs/action/NavigateToPose, blade OFF
//   OUTLINE_WORKING_AREA / OUTLINE_OBSTACLE / MOWING_BOUSTROPHEDON
//     -> nav2_msgs/action/FollowPath with controller_id="FollowCoveragePath"
//        (FTCController, <10 mm tracking error), blade ON. Consecutive same-
//        segment-type vertices are collapsed into one FollowPath call:
//          - MOWING_BOUSTROPHEDON: pairs of 2 (one swath = one FollowPath).
//          - OUTLINE_*: a contiguous run of same-segment vertices = one
//            closed-loop FollowPath.
//
// After a successful MOWING_BOUSTROPHEDON pair or OUTLINE_* run, the node
// calls /coverage_planner_node/write_checkpoint (Plan 01-05's
// WriteCheckpoint.srv) — the BT NEVER touches the filesystem (RESEARCH §10
// Q1 lock).
//
// Safety-critical contract (T-08-03 mitigation, regression-tested in
// test_coverage_nodes.cpp): onHalted() MUST disable the blade unconditionally
// AND cancel any active sub-action goal. The firmware is the sole safety
// authority and will refuse the blade command when its own gates fire, but
// this software-side invariant is defense in depth.
// ---------------------------------------------------------------------------
class FollowCoveragePlan : public BT::StatefulActionNode
{
public:
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;
  using CoverageWaypoint = mowgli_interfaces::msg::CoverageWaypoint;

  // Internal state machine per Plan 01-08 RESEARCH §6.3.
  enum class InternalState
  {
    IDLE,
    SEND_BLADE,
    WAIT_BLADE,
    SEND_NAV_GOAL,
    WAIT_NAV,
    SEND_FTC_GOAL,
    WAIT_FTC,
    CHECKPOINT_WRITE,
    ADVANCE_WAYPOINT
  };

  FollowCoveragePlan(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  /// Fire-and-forget MowerControl (firmware is sole safety authority).
  /// Virtual so unit tests (test_coverage_nodes.cpp) can intercept calls
  /// without spinning up an in-process MowerControl service — same-process
  /// rclcpp service round-trips have proven flaky on Cyclone DDS.
  virtual void setBladeEnabled(bool enabled);

  /// Lifted from private to protected for unit-test injection of a synthetic
  /// coverage_plan_ (test_coverage_nodes.cpp WriteCheckpointAreaIndexTest).
  /// Production callers are still inside the class.
  std::vector<CoverageWaypoint> coverage_plan_;
  size_t current_waypoint_idx_{0};
  size_t group_end_idx_exclusive_{0};   // exclusive end of the active group

  /// Lifted from private to protected for the same reason. Drives the
  /// /coverage_planner_node/write_checkpoint RPC; tested via an in-process
  /// stub server (WriteCheckpointAreaIndexTest in test_coverage_nodes.cpp).
  void dispatch_checkpoint_write(size_t completed_end_idx_exclusive);

private:
  /// Determine the closing index of a same-segment-type group starting at
  /// `start_idx`. For MOWING_BOUSTROPHEDON, group exactly two waypoints
  /// (swath start + end). For OUTLINE_*, group all consecutive vertices
  /// with the same segment_type. For Nav2-dispatched segments, group of 1.
  size_t group_end_index(size_t start_idx) const;

  /// Build a nav_msgs/Path for FTCController from coverage_plan_ entries
  /// in the half-open range [start_idx, end_idx_exclusive).
  nav_msgs::msg::Path build_path_segment(size_t start_idx, size_t end_idx_exclusive) const;

  // Sub-action / service clients — created lazily on first use.
  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  rclcpp::Client<mowgli_interfaces::srv::WriteCheckpoint>::SharedPtr checkpoint_client_;

  // Active goal tracking.
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;

  // Blade state machine.
  static constexpr double kBladeSpinupDelaySec = 1.5;
  std::chrono::steady_clock::time_point blade_start_time_;
  bool blade_currently_enabled_ = false;

  InternalState state_{InternalState::IDLE};

  // Checkpoint pending future (we don't block, we WARN on failure).
  std::shared_future<mowgli_interfaces::srv::WriteCheckpoint::Response::SharedPtr>
      checkpoint_future_;
  bool checkpoint_in_flight_ = false;
};

}  // namespace mowgli_behavior
