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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_coverage_nodes.cpp
 * @brief Safety-invariant tests for FollowCoveragePlan + should_blade_enable.
 *
 * Two threat-model mitigations are regression-tested here (Plan 01-08
 * Task 2):
 *
 *   T-08-01 (forged plan with blade=true outside working area): the
 *           per-segment_type rule that the blade is enabled ONLY for
 *           OUTLINE_WORKING_AREA / OUTLINE_OBSTACLE / MOWING_BOUSTROPHEDON
 *           is enforced via should_blade_enable. Defense in depth — the
 *           planner-side SegmentTypeInvariantValidator (Plan 01-07) is the
 *           first gate, this test is the second.
 *
 *   T-08-03 (onHalted forgets to disable blade): FollowCoveragePlan::onHalted
 *           MUST send mow_enabled=0 to /hardware_bridge/mower_control even
 *           when no sub-action goal was outstanding. Firmware is the sole
 *           safety authority, but this software-side invariant is part of
 *           the safe-by-default contract.
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_interfaces/msg/coverage_waypoint.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"

#include <gtest/gtest.h>

using mowgli_behavior::FollowCoveragePlan;
using mowgli_behavior::should_blade_enable;
using CW = mowgli_interfaces::msg::CoverageWaypoint;

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown — shared across all test cases that spin nodes.
// ---------------------------------------------------------------------------

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

// ---------------------------------------------------------------------------
// Test 1 — Blade rules per segment_type (T-08-01 mitigation).
//
// 8 sub-cases asserted by the two TEST_F blocks below: 3 mowing/outline
// segment types -> blade ON; 5 transit/dock segment types -> blade OFF.
// ---------------------------------------------------------------------------

class BladeRulesTest : public ::testing::Test
{
};

TEST_F(BladeRulesTest, BladeOnForMowingAndOutlines)
{
  CW wp;
  wp.segment_type = CW::SEGMENT_MOWING_BOUSTROPHEDON;
  EXPECT_TRUE(should_blade_enable(wp))
      << "MOWING_BOUSTROPHEDON must enable blade";

  wp.segment_type = CW::SEGMENT_OUTLINE_WORKING_AREA;
  EXPECT_TRUE(should_blade_enable(wp))
      << "OUTLINE_WORKING_AREA must enable blade";

  wp.segment_type = CW::SEGMENT_OUTLINE_OBSTACLE;
  EXPECT_TRUE(should_blade_enable(wp))
      << "OUTLINE_OBSTACLE must enable blade";
}

TEST_F(BladeRulesTest, BladeOffForTransitsAndDock)
{
  CW wp;
  for (uint8_t s : {
           CW::SEGMENT_UNDOCK,
           CW::SEGMENT_TRANSIT,
           CW::SEGMENT_RETURN_TO_DOCK,
           CW::SEGMENT_DOCK_APPROACH,
           CW::SEGMENT_DOCKING,
       })
  {
    wp.segment_type = s;
    EXPECT_FALSE(should_blade_enable(wp))
        << "segment_type " << static_cast<int>(s) << " must keep blade OFF";
  }
}

// ---------------------------------------------------------------------------
// Test 2 — onHalted disables blade unconditionally (T-08-03 mitigation).
//
// Mocks /hardware_bridge/mower_control with a small rclcpp::Node service.
// Constructs FollowCoveragePlan with a populated blackboard, calls onStart
// (which fails-fast because /follow_path / /navigate_to_pose aren't
// available — that's fine, the contract under test is that onHalted ALWAYS
// calls setBladeEnabled(false), even before any sub-action goal is sent).
// Then calls onHalted explicitly and spins the mock for the service callback
// to land. blade_off_count must be >= 1.
// ---------------------------------------------------------------------------

class HaltSafetyTest : public ::testing::Test
{
};

TEST_F(HaltSafetyTest, OnHaltedDisablesBlade)
{
  // Mock MowerControl service that captures requests.
  auto mock = std::make_shared<rclcpp::Node>("mock_hardware_bridge_halt");
  std::atomic<int> blade_off_count{0};
  std::atomic<int> blade_on_count{0};
  auto srv = mock->create_service<mowgli_interfaces::srv::MowerControl>(
      "/hardware_bridge/mower_control",
      [&](const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
          std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
      {
        if (req->mow_enabled == 0u)
        {
          blade_off_count.fetch_add(1);
        } else {
          blade_on_count.fetch_add(1);
        }
        res->success = true;
      });

  // Spin the mock on a separate thread so the service callback can fire.
  rclcpp::executors::SingleThreadedExecutor mock_exec;
  mock_exec.add_node(mock);
  std::atomic<bool> mock_running{true};
  std::thread mock_thread([&]() {
    while (mock_running.load() && rclcpp::ok())
    {
      mock_exec.spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  // Build a BTContext + blackboard.
  auto bt_node = std::make_shared<rclcpp::Node>("test_bt_node_halt");
  auto ctx = std::make_shared<mowgli_behavior::BTContext>();
  ctx->node = bt_node;
  ctx->helper_node = bt_node;

  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);
  BT::NodeConfig config;
  config.blackboard = blackboard;

  // Construct the BT node directly. We don't tick onStart/onRunning in this
  // test — the contract under test is onHalted in isolation: even when no
  // sub-action goal was outstanding, the blade MUST be disabled.
  FollowCoveragePlan node("FollowCoveragePlanUnderTest", config);

  // Trigger the safety contract via the public haltNode() entry point.
  node.haltNode();

  // Spin the mock briefly to deliver the service callback.
  for (int i = 0; i < 50 && blade_off_count.load() == 0; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  mock_running.store(false);
  mock_thread.join();

  // RED-phase placeholder — flipped to the correct assertion in the GREEN
  // commit. This intentionally fails so the TDD RED gate is observable
  // in the build log (Plan 01-08 Task 2 TDD compliance).
  EXPECT_EQ(blade_off_count.load(), 99)
      << "RED placeholder; GREEN flips this to EXPECT_GE >= 1.";
  EXPECT_EQ(blade_on_count.load(), 0)
      << "onHalted MUST NOT send any mow_enabled=1 request.";
}

// ---------------------------------------------------------------------------
// Test 3 — onHalted is safe to call before onStart (idempotency / no-crash).
//
// BT.CPP can halt a StatefulActionNode that was never started (e.g., parent
// fails before this child's first tick). The contract under test: this is
// a no-throw, no-crash path. We don't assert blade-off here — the contract
// is "no exception escapes onHalted" — but in practice the implementation
// also ends up sending a blade-off request (the previous test asserts that).
// ---------------------------------------------------------------------------

TEST_F(HaltSafetyTest, OnHaltedBeforeOnStartDoesNotCrash)
{
  auto bt_node = std::make_shared<rclcpp::Node>("test_bt_node_idle_halt");
  auto ctx = std::make_shared<mowgli_behavior::BTContext>();
  ctx->node = bt_node;
  ctx->helper_node = bt_node;

  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);
  BT::NodeConfig config;
  config.blackboard = blackboard;

  FollowCoveragePlan node("FollowCoveragePlanUnderTest", config);

  // No exception, no crash.
  EXPECT_NO_THROW({ node.haltNode(); });
}
