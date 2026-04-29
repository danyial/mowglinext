// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_coverage_planner/coverage_planner_node.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mowgli_interfaces/action/plan_coverage.hpp>
#include <mowgli_interfaces/msg/plan_error.hpp>
#include <mowgli_interfaces/srv/get_all_areas.hpp>
#include <mowgli_interfaces/srv/write_checkpoint.hpp>

#include "mowgli_coverage_planner/checkpoint_io.hpp"
#include "mowgli_coverage_planner/plan_builder.hpp"
#include "mowgli_coverage_planner/plan_context.hpp"
#include "mowgli_coverage_planner/validators.hpp"

// Plan 01-05 ships the skeleton wiring. The Checkpoint key=value serializer
// + WriteCheckpoint handler implementation lands in checkpoint_io.cpp and
// is invoked from on_write_checkpoint() below. Plan 01-07 lands validators
// + plan builder.

namespace mowgli_coverage_planner
{

namespace
{
using PlanError = mowgli_interfaces::msg::PlanError;
}  // namespace

CoveragePlannerNode::CoveragePlannerNode(const rclcpp::NodeOptions& options)
    : Node("coverage_planner_node", options)
{
  // -------------------------------------------------------------------------
  // Declare ALL parameters (per .claude/rules/ros2.md — no undeclared access).
  // robot_geometry.* mirrors mowgli_robot.yaml:robot_geometry (Plan 01-03).
  // outline_offset / strip_overlap / outline_passes / angle_increment_deg are
  // planner-specific knobs; areas_dir matches the existing map_server install.
  // -------------------------------------------------------------------------
  declare_parameter<double>("robot_geometry.robot_length", 0.60);
  declare_parameter<double>("robot_geometry.robot_width", 0.40);
  declare_parameter<double>("robot_geometry.drive_axis_x_offset", -0.20);
  declare_parameter<double>("robot_geometry.drive_axis_y_offset", 0.0);
  declare_parameter<double>("robot_geometry.blade_x_offset", 0.25);
  declare_parameter<double>("robot_geometry.blade_y_offset", 0.0);
  declare_parameter<double>("robot_geometry.tool_width", 0.18);

  declare_parameter<double>("outline_offset", 0.05);
  declare_parameter<double>("strip_overlap", 0.02);
  declare_parameter<int>("outline_passes", 2);
  declare_parameter<double>("angle_increment_deg", 30.0);
  declare_parameter<std::string>("areas_dir", "/ros2_ws/maps");

  load_robot_geometry();
  robot_valid_ = validate_robot_geometry();
  areas_dir_ = get_parameter("areas_dir").as_string();

  // -------------------------------------------------------------------------
  // Action server — `~/plan_coverage` resolves to /<node-name>/plan_coverage,
  // matching the GUI's expectation of /coverage_planner_node/plan_coverage.
  // (Bare "plan_coverage" — without the `~/` prefix — would land at root
  // namespace `/plan_coverage`, which is not what the GUI expects.)
  // -------------------------------------------------------------------------
  using std::placeholders::_1;
  using std::placeholders::_2;

  action_server_ = rclcpp_action::create_server<PlanCoverage>(
      this,
      "~/plan_coverage",
      std::bind(&CoveragePlannerNode::handle_goal, this, _1, _2),
      std::bind(&CoveragePlannerNode::handle_cancel, this, _1),
      std::bind(&CoveragePlannerNode::handle_accepted, this, _1));

  // -------------------------------------------------------------------------
  // GetAllAreas client — IPC to /map_server_node/get_all_areas (Plan 01-01).
  // -------------------------------------------------------------------------
  get_all_areas_client_ = create_client<mowgli_interfaces::srv::GetAllAreas>(
      "/map_server_node/get_all_areas");

  // -------------------------------------------------------------------------
  // WriteCheckpoint service — BT FollowCoveragePlan delegates checkpoint IO
  // to this node (Plan 01-01 RESEARCH §10 Q1 lock). Resolves to
  // /coverage_planner_node/write_checkpoint.
  // -------------------------------------------------------------------------
  write_checkpoint_srv_ =
      create_service<mowgli_interfaces::srv::WriteCheckpoint>(
          "~/write_checkpoint",
          std::bind(&CoveragePlannerNode::on_write_checkpoint, this, _1, _2));

  RCLCPP_INFO(get_logger(),
              "coverage_planner_node ready: areas_dir=%s, robot_valid=%s",
              areas_dir_.c_str(),
              robot_valid_ ? "true" : "false");
}

// ---------------------------------------------------------------------------
// Parameter helpers
// ---------------------------------------------------------------------------

void CoveragePlannerNode::load_robot_geometry()
{
  robot_.footprint.robot_length =
      get_parameter("robot_geometry.robot_length").as_double();
  robot_.footprint.robot_width =
      get_parameter("robot_geometry.robot_width").as_double();
  robot_.footprint.drive_axis_x_offset =
      get_parameter("robot_geometry.drive_axis_x_offset").as_double();
  robot_.footprint.drive_axis_y_offset =
      get_parameter("robot_geometry.drive_axis_y_offset").as_double();
  robot_.blade_x_offset =
      get_parameter("robot_geometry.blade_x_offset").as_double();
  robot_.blade_y_offset =
      get_parameter("robot_geometry.blade_y_offset").as_double();
  robot_.tool_width =
      get_parameter("robot_geometry.tool_width").as_double();

  robot_.outline_offset = get_parameter("outline_offset").as_double();
  robot_.strip_overlap = get_parameter("strip_overlap").as_double();
  const int passes = get_parameter("outline_passes").as_int();
  robot_.outline_passes =
      static_cast<std::uint32_t>(passes > 0 ? passes : 1);
  robot_.angle_increment_deg =
      get_parameter("angle_increment_deg").as_double();
}

bool CoveragePlannerNode::validate_robot_geometry()
{
  const bool ok = robot_.footprint.robot_length > 0.0 &&
                  robot_.footprint.robot_width > 0.0 &&
                  robot_.tool_width > 0.0;
  if (!ok)
  {
    RCLCPP_ERROR(get_logger(),
                 "Invalid robot_geometry: robot_length=%.3f, robot_width=%.3f, "
                 "tool_width=%.3f (all three must be > 0)",
                 robot_.footprint.robot_length,
                 robot_.footprint.robot_width,
                 robot_.tool_width);
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Action server callbacks (RESEARCH §6.1)
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CoveragePlannerNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const PlanCoverage::Goal> /*goal*/)
{
  if (planning_active_.load())
  {
    RCLCPP_WARN(get_logger(),
                "Rejecting PlanCoverage goal: another plan is already in flight");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CoveragePlannerNode::handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "PlanCoverage cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoveragePlannerNode::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
  planning_active_.store(true);
  // Detached worker thread per RESEARCH §6.1 — the executor stays responsive
  // for cancel callbacks while planning runs.
  std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
}

// ---------------------------------------------------------------------------
// fetch_all_areas — synchronous-with-cancel poll on /map_server_node/get_all_areas.
// ---------------------------------------------------------------------------

bool CoveragePlannerNode::fetch_all_areas(
    PlanContext& ctx, const std::shared_ptr<GoalHandle>& goal_handle)
{
  if (!get_all_areas_client_->wait_for_service(std::chrono::seconds(5)))
  {
    PlanError err;
    err.error_code = PlanError::ERROR_INTERNAL;
    err.human_readable = "GetAllAreas service not available";
    ctx.error = err;
    RCLCPP_ERROR(get_logger(), "%s", err.human_readable.c_str());
    return false;
  }

  auto req = std::make_shared<mowgli_interfaces::srv::GetAllAreas::Request>();
  auto fut = get_all_areas_client_->async_send_request(req);

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok())
  {
    if (goal_handle && goal_handle->is_canceling())
    {
      PlanError err;
      err.error_code = PlanError::ERROR_INTERNAL;
      err.human_readable = "GetAllAreas cancelled";
      ctx.error = err;
      return false;
    }
    if (fut.wait_for(std::chrono::milliseconds(20)) ==
        std::future_status::ready)
    {
      ctx.areas = fut.get()->areas;
      return true;
    }
    if (std::chrono::steady_clock::now() > deadline)
    {
      break;
    }
  }

  PlanError err;
  err.error_code = PlanError::ERROR_INTERNAL;
  err.human_readable = "GetAllAreas timed out";
  ctx.error = err;
  RCLCPP_ERROR(get_logger(), "%s", err.human_readable.c_str());
  return false;
}

// ---------------------------------------------------------------------------
// execute() — phase pipeline.
//   1. Robot-geometry validation (D-09).
//   2. Snapshot pull of all areas via /map_server_node/get_all_areas.
//   3. Cancel-fast.
//   4. ERROR_NO_AREAS guard (SPEC R-12).
//   5. Pre-geometry ValidatorPipeline (SPEC R-12 fail-fast).
//   6. PlanBuilder: UNDOCK + per-area outlines/sweep/narrow + dock segments.
//   7. Post-geometry ValidatorPipeline (T-07-07 / T-07-08 mitigations).
//   8. Build result + PlanMetadata; succeed.
// ---------------------------------------------------------------------------

void CoveragePlannerNode::execute(std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<PlanCoverage::Result>();
  auto feedback = std::make_shared<PlanCoverage::Feedback>();

  PlanContext ctx;
  ctx.goal = *goal_handle->get_goal();
  ctx.robot = robot_;
  ctx.areas_dir = areas_dir_;

  // 1. Robot-geometry validation (D-09). Reject any plan request when the
  //    chassis dimensions are unusable.
  if (!robot_valid_)
  {
    result->success = false;
    PlanError err;
    err.error_code = PlanError::ERROR_INTERNAL;
    err.human_readable = "Invalid robot_geometry parameters";
    result->error = err;
    goal_handle->succeed(result);
    planning_active_.store(false);
    return;
  }

  // 2. Snapshot pull of all areas from map_server.
  feedback->progress_percent = 0.0f;
  feedback->phase = "areas_loaded";
  goal_handle->publish_feedback(feedback);

  if (!fetch_all_areas(ctx, goal_handle))
  {
    result->success = false;
    result->error = ctx.error.value_or(PlanError{});
    goal_handle->succeed(result);
    planning_active_.store(false);
    return;
  }

  // 3. Cancel-fast.
  if (goal_handle->is_canceling())
  {
    goal_handle->canceled(result);
    planning_active_.store(false);
    return;
  }

  // 4. Empty-input guard (SPEC R-12, ERROR_NO_AREAS).
  if (ctx.areas.empty())
  {
    result->success = false;
    PlanError err;
    err.error_code = PlanError::ERROR_NO_AREAS;
    err.failed_validation_point = 0;
    err.human_readable = "No mowing areas defined";
    result->error = err;
    goal_handle->succeed(result);
    planning_active_.store(false);
    return;
  }

  // 5. Pre-geometry validation pipeline (SPEC R-12, fail-fast).
  ValidatorPipeline pre_geom;
  pre_geom.add_pre_geometry_validators();
  if (auto err = pre_geom.run(ctx))
  {
    result->success = false;
    result->error = *err;
    RCLCPP_WARN(get_logger(), "Pre-geometry validation rejected plan: %s",
                err->human_readable.c_str());
    goal_handle->succeed(result);
    planning_active_.store(false);
    return;
  }

  // 6. Plan build: UNDOCK -> per-area outlines + sweep + narrow strategies
  //    -> RETURN_TO_DOCK -> DOCK_APPROACH -> DOCKING.
  PlanBuilder builder(robot_, areas_dir_);

  feedback->progress_percent = 25.0F;
  feedback->phase = "outlines_generated";
  goal_handle->publish_feedback(feedback);

  if (!builder.build(ctx))
  {
    result->success = false;
    result->error = ctx.error.value_or(PlanError{});
    RCLCPP_WARN(get_logger(), "PlanBuilder failed: %s",
                result->error.human_readable.c_str());
    goal_handle->succeed(result);
    planning_active_.store(false);
    return;
  }

  feedback->progress_percent = 75.0F;
  feedback->phase = "swaths_generated";
  goal_handle->publish_feedback(feedback);

  // 7. Post-geometry validation pipeline (SPEC R-12 + T-07-07 / T-07-08).
  ValidatorPipeline post_geom;
  post_geom.add_post_geometry_validators();
  if (auto err = post_geom.run(ctx))
  {
    result->success = false;
    result->error = *err;
    RCLCPP_WARN(get_logger(), "Post-geometry validation rejected plan: %s",
                err->human_readable.c_str());
    goal_handle->succeed(result);
    planning_active_.store(false);
    return;
  }

  feedback->progress_percent = 95.0F;
  feedback->phase = "validation_passed";
  goal_handle->publish_feedback(feedback);

  // 8. Build the result + metadata.
  result->success = true;
  result->plan = ctx.plan;
  result->metadata.mow_angle_used_deg = ctx.mow_angle_used_deg;
  result->metadata.outline_passes_used = robot_.outline_passes;
  result->metadata.path_spacing_used = robot_.tool_width - robot_.strip_overlap;
  result->metadata.processed_area_indices = ctx.processed_area_indices;
  result->metadata.skipped_area_indices = ctx.skipped_area_indices;
  result->metadata.skip_reasons = ctx.skip_reasons;
  result->metadata.warnings = ctx.warnings;
  // checkpoint_seed: zero-initialized for fresh runs; on resume, the caller
  // (BT FollowCoveragePlan) will read the per-area .kv files itself before
  // executing each area.
  goal_handle->succeed(result);
  planning_active_.store(false);
}

// ---------------------------------------------------------------------------
// on_write_checkpoint — BT FollowCoveragePlan calls this after each
// completed swath (Plan 01-01 RESEARCH §10 Q1 lock). The handler delegates
// path construction + atomic write to checkpoint_io.cpp's
// write_checkpoint_file(), which in turn calls
// mowgli_geometry::atomic_write (Plan 01-02). T-05-01 / T-05-02 mitigations
// live in write_checkpoint_file (digit-only filename interpolation +
// non-finite value rejection).
// ---------------------------------------------------------------------------

void CoveragePlannerNode::on_write_checkpoint(
    const mowgli_interfaces::srv::WriteCheckpoint::Request::SharedPtr req,
    mowgli_interfaces::srv::WriteCheckpoint::Response::SharedPtr res)
{
  std::string err;
  if (!write_checkpoint_file(areas_dir_, req->checkpoint, &err))
  {
    res->success = false;
    res->error_message = err;
    RCLCPP_ERROR(get_logger(), "WriteCheckpoint failed: %s", err.c_str());
    return;
  }
  res->success = true;
  res->error_message.clear();
  RCLCPP_INFO(get_logger(),
              "WriteCheckpoint: persisted area_index=%u",
              req->checkpoint.area_index);
}

}  // namespace mowgli_coverage_planner
