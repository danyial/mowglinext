// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

// CoveragePlannerNode owns:
//   - rclcpp_action server for /coverage_planner_node/plan_coverage
//   - rclcpp::Client<GetAllAreas> for the snapshot pull from map_server
//   - rclcpp::Service<WriteCheckpoint> for BT-delegates-IO-to-planner (Plan 01-01 lock)
//
// Plan 01-05 landed the skeleton: parameter declaration + validation, the
// action plumbing, the GetAllAreas client, and the WriteCheckpoint handler
// backed by mowgli_geometry::atomic_write. Plan 01-07 fills in the
// validator pipeline + plan builder; execute() now runs the full SPEC R-12
// pipeline end-to-end.

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mowgli_interfaces/action/plan_coverage.hpp>
#include <mowgli_interfaces/srv/get_all_areas.hpp>
#include <mowgli_interfaces/srv/write_checkpoint.hpp>

#include "mowgli_coverage_planner/plan_context.hpp"

namespace mowgli_coverage_planner
{

class CoveragePlannerNode : public rclcpp::Node
{
public:
  using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanCoverage>;

  explicit CoveragePlannerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ---- Action server callbacks -------------------------------------------
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const PlanCoverage::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(std::shared_ptr<GoalHandle> goal_handle);

  // GetAllAreas RPC; populates ctx.areas. Returns false on timeout / failure
  // and writes ERROR_INTERNAL into ctx.error.
  bool fetch_all_areas(PlanContext& ctx,
                       const std::shared_ptr<GoalHandle>& goal_handle);

  // ---- WriteCheckpoint service handler ----------------------------------
  void on_write_checkpoint(
      const mowgli_interfaces::srv::WriteCheckpoint::Request::SharedPtr req,
      mowgli_interfaces::srv::WriteCheckpoint::Response::SharedPtr res);

  // ---- Parameter helpers -------------------------------------------------
  void load_robot_geometry();
  /// Returns true iff robot_length > 0, robot_width > 0, tool_width > 0
  /// (per D-09). Logs the offending values when false.
  bool validate_robot_geometry();

  // ---- Members -----------------------------------------------------------
  rclcpp_action::Server<PlanCoverage>::SharedPtr action_server_;
  rclcpp::Client<mowgli_interfaces::srv::GetAllAreas>::SharedPtr
      get_all_areas_client_;
  rclcpp::Service<mowgli_interfaces::srv::WriteCheckpoint>::SharedPtr
      write_checkpoint_srv_;

  RobotGeometry robot_;
  bool robot_valid_{false};
  std::string areas_dir_;

  /// Single-goal-at-a-time guard. handle_goal REJECTs while this is true;
  /// execute() clears it on every exit path.
  std::atomic<bool> planning_active_{false};
};

}  // namespace mowgli_coverage_planner
