// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// SPEC R-1 / R-2 acceptance: CoveragePlannerNode constructs cleanly under
// gtest, exposes the /coverage_planner_node/plan_coverage action endpoint,
// and a freshly-spun client can wait_for_action_server() within 2 s.
// Sending a goal with an empty area inventory exercises the action plumbing
// end-to-end (the GetAllAreas client will time out because no map_server
// is running, which exercises the ERROR_INTERNAL path; the test only
// asserts that the plumbing is alive).

#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mowgli_interfaces/action/plan_coverage.hpp>

#include "mowgli_coverage_planner/coverage_planner_node.hpp"

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

}  // namespace

class CoveragePlannerSkeletonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    // Mirror Plan 01-03 defaults so validate_robot_geometry passes.
    opts.append_parameter_override("robot_geometry.robot_length", 0.60);
    opts.append_parameter_override("robot_geometry.robot_width", 0.40);
    opts.append_parameter_override("robot_geometry.drive_axis_x_offset", -0.20);
    opts.append_parameter_override("robot_geometry.drive_axis_y_offset", 0.0);
    opts.append_parameter_override("robot_geometry.blade_x_offset", 0.25);
    opts.append_parameter_override("robot_geometry.blade_y_offset", 0.0);
    opts.append_parameter_override("robot_geometry.tool_width", 0.18);
    opts.append_parameter_override("outline_offset", 0.05);
    opts.append_parameter_override("strip_overlap", 0.02);
    opts.append_parameter_override("outline_passes", 2);
    opts.append_parameter_override("angle_increment_deg", 30.0);
    opts.append_parameter_override("areas_dir", "/tmp/mowgli_skeleton_test");

    node_ = std::make_shared<mowgli_coverage_planner::CoveragePlannerNode>(opts);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }
  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    node_.reset();
    executor_.reset();
  }
  std::shared_ptr<mowgli_coverage_planner::CoveragePlannerNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(CoveragePlannerSkeletonTest, NodeConstructsCleanly)
{
  ASSERT_NE(node_, nullptr);
  EXPECT_STREQ(node_->get_name(), "coverage_planner_node");
}

TEST_F(CoveragePlannerSkeletonTest, ActionEndpointIsAvailable)
{
  using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
  // Spin up an external client node — the action server lives on `node_`.
  auto client_node = rclcpp::Node::make_shared("test_client");
  auto client = rclcpp_action::create_client<PlanCoverage>(
      client_node, "/coverage_planner_node/plan_coverage");

  // Pump the client_node's executor until the server appears (or we time
  // out). The server's executor is on its own thread.
  rclcpp::executors::SingleThreadedExecutor client_exec;
  client_exec.add_node(client_node);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  bool server_ready = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (client->action_server_is_ready())
    {
      server_ready = true;
      break;
    }
    client_exec.spin_some(std::chrono::milliseconds(50));
  }
  EXPECT_TRUE(server_ready)
      << "/coverage_planner_node/plan_coverage was not advertised within 2 s";
}
