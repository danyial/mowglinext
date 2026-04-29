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

#include "mowgli_behavior/coverage_nodes.hpp"

#include <chrono>

#include "action_msgs/msg/goal_status.hpp"

namespace mowgli_behavior
{

// ===========================================================================
// PlanCoverageGoal — request a coverage plan from coverage_planner_node.
// ===========================================================================

BT::NodeStatus PlanCoverageGoal::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Reset previous run state (StatefulActionNode reuses instances).
  goal_handle_.reset();
  result_requested_ = false;

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<Action>(
        ctx->node, "/coverage_planner_node/plan_coverage");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PlanCoverageGoal: /coverage_planner_node/plan_coverage not available");
    return BT::NodeStatus::FAILURE;
  }

  // Build the goal. SPEC R-2 schema (Plan 01-01).
  Action::Goal goal;

  // start_pose: leave header.frame_id empty so planner falls back to dock_pose.
  // Future enhancement: snapshot ctx->gps_x/gps_y here once a current-pose
  // tracker exists in BTContext.
  goal.start_pose.header.frame_id = "";

  goal.dock_pose.header.frame_id = "map";
  goal.dock_pose.header.stamp = ctx->node->now();
  goal.dock_pose.pose.position.x = ctx->dock_x;
  goal.dock_pose.pose.position.y = ctx->dock_y;
  goal.dock_pose.pose.position.z = 0.0;
  // Yaw -> quaternion for dock_pose.
  const double yaw = ctx->dock_yaw;
  goal.dock_pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.dock_pose.pose.orientation.w = std::cos(yaw / 2.0);

  // Auto-rotate (planner uses persisted last_mow_angle_deg + angle_increment).
  goal.mow_angle_offset_deg = -1.0F;

  // Resume flag: true after a charge cycle. The dock-detect / emergency-reset
  // logic sets ctx->resume_undock_failures > 0 only on successful resume.
  // Conservative default: always attempt to read checkpoint files so the
  // planner can resume mid-area if .kv exists. Missing files = fresh plan.
  goal.resume_from_checkpoint = true;

  goal_future_ = action_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "PlanCoverageGoal: sent goal (dock=(%.2f, %.2f, yaw=%.2f), auto-rotate, resume=true)",
              ctx->dock_x, ctx->dock_y, ctx->dock_yaw);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanCoverageGoal::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Phase 1: wait for goal acceptance.
  if (!goal_handle_)
  {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "PlanCoverageGoal: goal rejected by coverage_planner_node");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ctx->node->get_logger(), "PlanCoverageGoal: goal accepted");
  }

  // Phase 2: request the result future once.
  if (!result_requested_)
  {
    result_future_ = action_client_->async_get_result(goal_handle_);
    result_requested_ = true;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped = result_future_.get();

  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PlanCoverageGoal: action ended with code %d (not SUCCEEDED)",
                 static_cast<int>(wrapped.code));
    return BT::NodeStatus::FAILURE;
  }

  if (!wrapped.result || !wrapped.result->success)
  {
    const std::string err = (wrapped.result && !wrapped.result->error.human_readable.empty())
                                ? wrapped.result->error.human_readable
                                : std::string("(no error message)");
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PlanCoverageGoal: planner returned success=false: %s",
                 err.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (wrapped.result->plan.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PlanCoverageGoal: planner returned success=true but plan is empty");
    return BT::NodeStatus::FAILURE;
  }

  // Snapshot into BTContext blackboard. FollowCoveragePlan reads it on its
  // own onStart().
  ctx->coverage_plan = wrapped.result->plan;

  RCLCPP_INFO(ctx->node->get_logger(),
              "PlanCoverageGoal: received plan with %zu waypoints (mow_angle=%.1f deg)",
              ctx->coverage_plan.size(),
              wrapped.result->metadata.mow_angle_used_deg);

  return BT::NodeStatus::SUCCESS;
}

void PlanCoverageGoal::onHalted()
{
  if (goal_handle_ && action_client_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  result_requested_ = false;
}

// ===========================================================================
// FollowCoveragePlan — sequentially execute coverage_plan from blackboard.
// ===========================================================================

BT::NodeStatus FollowCoveragePlan::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->coverage_plan.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowCoveragePlan: ctx->coverage_plan empty (PlanCoverageGoal must run first)");
    return BT::NodeStatus::FAILURE;
  }

  // Snapshot — protect against concurrent blackboard writes.
  coverage_plan_ = ctx->coverage_plan;
  current_waypoint_idx_ = 0;
  group_end_idx_exclusive_ = 0;

  follow_handle_.reset();
  nav_handle_.reset();
  blade_currently_enabled_ = false;
  state_ = InternalState::IDLE;
  checkpoint_in_flight_ = false;

  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }
  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }

  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowCoveragePlan: /follow_path action server not available");
    setBladeEnabled(false);
    return BT::NodeStatus::FAILURE;
  }
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowCoveragePlan: /navigate_to_pose action server not available");
    setBladeEnabled(false);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowCoveragePlan: starting execution of %zu-waypoint plan",
              coverage_plan_.size());

  return BT::NodeStatus::RUNNING;
}

size_t FollowCoveragePlan::group_end_index(size_t start_idx) const
{
  using CW = mowgli_interfaces::msg::CoverageWaypoint;
  if (start_idx >= coverage_plan_.size()) return start_idx;

  const auto seg = coverage_plan_[start_idx].segment_type;

  // MOWING_BOUSTROPHEDON: pair of two (swath start + end) per RESEARCH §10 Q3.
  if (seg == CW::SEGMENT_MOWING_BOUSTROPHEDON)
  {
    size_t end = start_idx + 1;
    if (end < coverage_plan_.size() && coverage_plan_[end].segment_type == seg)
    {
      ++end;
    }
    return end;  // exclusive
  }

  // OUTLINE_*: contiguous run of same-segment-type vertices = one closed loop.
  if (seg == CW::SEGMENT_OUTLINE_WORKING_AREA || seg == CW::SEGMENT_OUTLINE_OBSTACLE)
  {
    size_t end = start_idx + 1;
    while (end < coverage_plan_.size() && coverage_plan_[end].segment_type == seg)
    {
      ++end;
    }
    return end;
  }

  // Nav2-dispatched (UNDOCK / TRANSIT / DOCK_APPROACH / DOCKING / RETURN_TO_DOCK):
  // one waypoint per call.
  return start_idx + 1;
}

nav_msgs::msg::Path FollowCoveragePlan::build_path_segment(size_t start_idx,
                                                            size_t end_idx_exclusive) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  if (start_idx < coverage_plan_.size())
  {
    path.header.stamp = coverage_plan_[start_idx].pose.header.stamp;
  }
  path.poses.reserve(end_idx_exclusive - start_idx);
  for (size_t i = start_idx; i < end_idx_exclusive && i < coverage_plan_.size(); ++i)
  {
    path.poses.push_back(coverage_plan_[i].pose);
  }
  return path;
}

void FollowCoveragePlan::dispatch_checkpoint_write(size_t completed_end_idx_exclusive)
{
  using CW = mowgli_interfaces::msg::CoverageWaypoint;
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (completed_end_idx_exclusive == 0 || completed_end_idx_exclusive > coverage_plan_.size())
  {
    return;
  }

  if (!checkpoint_client_)
  {
    checkpoint_client_ = ctx->node->create_client<mowgli_interfaces::srv::WriteCheckpoint>(
        "/coverage_planner_node/write_checkpoint");
  }

  if (!checkpoint_client_->wait_for_service(std::chrono::milliseconds(200)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "FollowCoveragePlan: WriteCheckpoint service unavailable, skipping (next "
                "successful checkpoint is the recovery point)");
    return;
  }

  // Build a Checkpoint reflecting the just-completed segment. The planner
  // owns filesystem I/O — we hand it a Checkpoint message and it writes the
  // <areas_dir>/coverage_<area_index>.kv file atomically (Plan 01-05).
  const auto& last_wp = coverage_plan_[completed_end_idx_exclusive - 1];

  auto req = std::make_shared<mowgli_interfaces::srv::WriteCheckpoint::Request>();
  req->checkpoint.area_index = last_wp.sequence_id;  // best-effort, planner owns the canonical key
  req->checkpoint.current_outline_index = 0;
  req->checkpoint.current_swath_index = static_cast<uint32_t>(last_wp.sequence_id);
  req->checkpoint.swath_direction =
      mowgli_interfaces::msg::Checkpoint::SWATH_DIRECTION_FORWARD;
  req->checkpoint.last_completed_swath_index = static_cast<uint32_t>(last_wp.sequence_id);
  req->checkpoint.next_open_swath_index = static_cast<uint32_t>(last_wp.sequence_id) + 1u;
  req->checkpoint.last_mow_angle_deg = 0.0;  // Planner derives this from its own state.
  req->checkpoint.last_swath_endpoint = last_wp.pose.pose;

  // Tag for log/diagnostic clarity.
  if (last_wp.segment_type == CW::SEGMENT_OUTLINE_WORKING_AREA ||
      last_wp.segment_type == CW::SEGMENT_OUTLINE_OBSTACLE)
  {
    req->checkpoint.current_outline_index =
        static_cast<uint32_t>(completed_end_idx_exclusive - 1);
  }

  // Fire-and-forget: we only WARN on failure (Q1 lock — next successful
  // checkpoint is the recovery point). Discard the future immediately;
  // the service handler completes asynchronously inside the planner.
  checkpoint_client_->async_send_request(req);

  RCLCPP_DEBUG(ctx->node->get_logger(),
               "FollowCoveragePlan: dispatched checkpoint after waypoint %zu",
               completed_end_idx_exclusive - 1);
}

BT::NodeStatus FollowCoveragePlan::onRunning()
{
  using CW = mowgli_interfaces::msg::CoverageWaypoint;
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Plan exhausted -> SUCCESS, blade off.
  if (current_waypoint_idx_ >= coverage_plan_.size())
  {
    setBladeEnabled(false);
    blade_currently_enabled_ = false;
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowCoveragePlan: plan exhausted (%zu waypoints)",
                coverage_plan_.size());
    return BT::NodeStatus::SUCCESS;
  }

  const auto& wp = coverage_plan_[current_waypoint_idx_];
  const bool target_blade = should_blade_enable(wp);

  switch (state_)
  {
    case InternalState::IDLE:
    {
      // Determine group boundaries for the active segment_type.
      group_end_idx_exclusive_ = group_end_index(current_waypoint_idx_);

      // Decide blade transition.
      if (target_blade != blade_currently_enabled_)
      {
        setBladeEnabled(target_blade);
        blade_currently_enabled_ = target_blade;
        blade_start_time_ = std::chrono::steady_clock::now();
        state_ = InternalState::WAIT_BLADE;
      } else {
        // No transition needed -> dispatch immediately.
        state_ = (wp.segment_type == CW::SEGMENT_MOWING_BOUSTROPHEDON ||
                  wp.segment_type == CW::SEGMENT_OUTLINE_WORKING_AREA ||
                  wp.segment_type == CW::SEGMENT_OUTLINE_OBSTACLE)
                     ? InternalState::SEND_FTC_GOAL
                     : InternalState::SEND_NAV_GOAL;
      }
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::WAIT_BLADE:
    {
      // Only wait the spinup delay when turning blade ON (RPM stabilization
      // before the path begins). Turning blade OFF is immediate — the firmware
      // can stop the blade at any time and the path doesn't need to wait.
      if (blade_currently_enabled_)
      {
        const auto elapsed = std::chrono::steady_clock::now() - blade_start_time_;
        if (elapsed < std::chrono::duration<double>(kBladeSpinupDelaySec))
        {
          return BT::NodeStatus::RUNNING;
        }
      }
      state_ = (wp.segment_type == CW::SEGMENT_MOWING_BOUSTROPHEDON ||
                wp.segment_type == CW::SEGMENT_OUTLINE_WORKING_AREA ||
                wp.segment_type == CW::SEGMENT_OUTLINE_OBSTACLE)
                   ? InternalState::SEND_FTC_GOAL
                   : InternalState::SEND_NAV_GOAL;
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::SEND_NAV_GOAL:
    {
      Nav2Navigate::Goal goal;
      goal.pose = wp.pose;
      // Ensure header.frame_id is "map" — planner contract guarantees this
      // but be explicit for downstream Nav2.
      if (goal.pose.header.frame_id.empty())
      {
        goal.pose.header.frame_id = "map";
      }
      nav_handle_.reset();
      nav_future_ = nav_client_->async_send_goal(goal);
      RCLCPP_INFO(ctx->node->get_logger(),
                  "FollowCoveragePlan: NavigateToPose seg=%u idx=%zu pos=(%.2f, %.2f)",
                  wp.segment_type, current_waypoint_idx_,
                  goal.pose.pose.position.x, goal.pose.pose.position.y);
      state_ = InternalState::WAIT_NAV;
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::WAIT_NAV:
    {
      if (!nav_handle_)
      {
        if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        {
          return BT::NodeStatus::RUNNING;
        }
        nav_handle_ = nav_future_.get();
        if (!nav_handle_)
        {
          RCLCPP_ERROR(ctx->node->get_logger(),
                       "FollowCoveragePlan: NavigateToPose goal rejected at idx=%zu",
                       current_waypoint_idx_);
          setBladeEnabled(false);
          blade_currently_enabled_ = false;
          return BT::NodeStatus::FAILURE;
        }
      }
      auto status = nav_handle_->get_status();
      if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        nav_handle_.reset();
        // Nav2 segments do NOT trigger a checkpoint write — only completed
        // mowing/outline groups do (they're the meaningful recovery points).
        state_ = InternalState::ADVANCE_WAYPOINT;
        return BT::NodeStatus::RUNNING;
      }
      if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
          status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "FollowCoveragePlan: NavigateToPose aborted/canceled at idx=%zu",
                    current_waypoint_idx_);
        nav_handle_.reset();
        setBladeEnabled(false);
        blade_currently_enabled_ = false;
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::SEND_FTC_GOAL:
    {
      Nav2FollowPath::Goal goal;
      goal.path = build_path_segment(current_waypoint_idx_, group_end_idx_exclusive_);
      goal.controller_id = "FollowCoveragePath";   // FTCController per CLAUDE.md invariant #8
      goal.goal_checker_id = "coverage_goal_checker";
      follow_handle_.reset();
      follow_future_ = follow_client_->async_send_goal(goal);
      RCLCPP_INFO(ctx->node->get_logger(),
                  "FollowCoveragePlan: FollowPath seg=%u idx=[%zu,%zu) poses=%zu "
                  "controller=FollowCoveragePath",
                  wp.segment_type, current_waypoint_idx_, group_end_idx_exclusive_,
                  goal.path.poses.size());
      state_ = InternalState::WAIT_FTC;
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::WAIT_FTC:
    {
      if (!follow_handle_)
      {
        if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        {
          return BT::NodeStatus::RUNNING;
        }
        follow_handle_ = follow_future_.get();
        if (!follow_handle_)
        {
          RCLCPP_ERROR(ctx->node->get_logger(),
                       "FollowCoveragePlan: FollowPath goal rejected at idx=%zu",
                       current_waypoint_idx_);
          setBladeEnabled(false);
          blade_currently_enabled_ = false;
          return BT::NodeStatus::FAILURE;
        }
      }
      auto status = follow_handle_->get_status();
      if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        follow_handle_.reset();
        state_ = InternalState::CHECKPOINT_WRITE;
        return BT::NodeStatus::RUNNING;
      }
      if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
          status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "FollowCoveragePlan: FollowPath aborted/canceled at idx=%zu",
                    current_waypoint_idx_);
        follow_handle_.reset();
        setBladeEnabled(false);
        blade_currently_enabled_ = false;
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::CHECKPOINT_WRITE:
    {
      // Q1 lock: BT delegates checkpoint writes to the planner via service.
      // Fire-and-forget — we WARN on failure but do NOT halt the plan.
      dispatch_checkpoint_write(group_end_idx_exclusive_);
      state_ = InternalState::ADVANCE_WAYPOINT;
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::ADVANCE_WAYPOINT:
    {
      current_waypoint_idx_ = group_end_idx_exclusive_;
      state_ = InternalState::IDLE;
      return BT::NodeStatus::RUNNING;
    }

    case InternalState::SEND_BLADE:
    {
      // Currently unused — blade transitions go directly via WAIT_BLADE.
      // Reserved for future SEND_BLADE -> WAIT_BLADE split if we need to
      // observe the MowerControl future before timing the spinup delay.
      state_ = InternalState::WAIT_BLADE;
      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void FollowCoveragePlan::onHalted()
{
  // Safety-critical contract (T-08-03 mitigation, regression-tested in
  // test_coverage_nodes.cpp::OnHaltedDisablesBlade): the blade MUST be
  // disabled unconditionally and any active sub-action goal MUST be canceled.
  if (follow_handle_ && follow_client_)
  {
    follow_client_->async_cancel_goal(follow_handle_);
  }
  follow_handle_.reset();

  if (nav_handle_ && nav_client_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();

  setBladeEnabled(false);  // ALWAYS — see contract above.
  blade_currently_enabled_ = false;

  state_ = InternalState::IDLE;
}

void FollowCoveragePlan::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (!blade_client_)
  {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }
  if (!blade_client_->wait_for_service(std::chrono::milliseconds(200)))
  {
    return;
  }

  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  // Fire-and-forget — firmware is sole safety authority and decides whether
  // to actually run/stop the blade based on its own gates (CLAUDE.md Safety).
  blade_client_->async_send_request(req);
}

}  // namespace mowgli_behavior
