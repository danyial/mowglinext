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
 *           MUST call setBladeEnabled(false) even when no sub-action goal
 *           was outstanding. Firmware is the sole safety authority, but
 *           this software-side invariant is part of the safe-by-default
 *           contract.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_interfaces/msg/coverage_waypoint.hpp"

using mowgli_behavior::FollowCoveragePlan;
using mowgli_behavior::should_blade_enable;
using CW = mowgli_interfaces::msg::CoverageWaypoint;

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown — needed because BTContext owns an rclcpp::Node.
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
// 8 sub-cases asserted across the two TEST_F blocks: 3 mowing/outline
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
// Subclasses FollowCoveragePlan and overrides setBladeEnabled with a counter
// hook. This sidesteps DDS-level service round-trips (which have proven
// flaky in single-process test setups) and tests the software contract
// directly: when onHalted runs, setBladeEnabled(false) is invoked. This is
// the actual safety-critical assertion — the firmware is the sole safety
// authority, and the BT's job is to send the disable command. Whether it
// physically reaches the firmware is the firmware's concern; whether the
// command is _emitted at all_ is what this test asserts.
// ---------------------------------------------------------------------------

namespace
{
class FollowCoveragePlanUnderTest : public FollowCoveragePlan
{
public:
  using FollowCoveragePlan::FollowCoveragePlan;

  std::atomic<int> blade_off_calls{0};
  std::atomic<int> blade_on_calls{0};
  std::vector<bool> call_log;
  std::mutex call_log_mutex;

  // Public proxy so the test can drive the safety-critical codepath
  // without depending on BT.CPP's StatefulActionNode lifecycle (which
  // skips onHalted when the node never reached RUNNING). The contract
  // we need to assert is "onHalted -> setBladeEnabled(false)"; whether
  // BT.CPP routes the haltNode() call depends on prior status, which
  // is orthogonal to the safety invariant.
  void invokeOnHaltedDirectly()
  {
    onHalted();
  }

protected:
  void setBladeEnabled(bool enabled) override
  {
    {
      std::lock_guard<std::mutex> g(call_log_mutex);
      call_log.push_back(enabled);
    }
    if (enabled)
    {
      blade_on_calls.fetch_add(1);
    } else {
      blade_off_calls.fetch_add(1);
    }
    // NB: deliberately does NOT call into the production setBladeEnabled —
    // the contract under test is at the software boundary inside the BT,
    // not at the DDS edge. The firmware is the sole safety authority.
  }
};
}  // namespace

class HaltSafetyTest : public ::testing::Test
{
};

TEST_F(HaltSafetyTest, OnHaltedDisablesBlade)
{
  // Build a minimal BTContext + blackboard so the test subclass can be
  // constructed. The override of setBladeEnabled does not access ctx->node
  // (it captures call counts directly) so the node need not be spun.
  auto bt_node = std::make_shared<rclcpp::Node>("test_bt_node_halt");
  auto ctx = std::make_shared<mowgli_behavior::BTContext>();
  ctx->node = bt_node;
  ctx->helper_node = bt_node;

  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);
  BT::NodeConfig config;
  config.blackboard = blackboard;

  FollowCoveragePlanUnderTest node("FollowCoveragePlanUnderTest", config);

  // Drive onHalted directly via the test proxy — this is the safety-critical
  // codepath that BatteryGuard / RainGuard rely on at runtime.
  node.invokeOnHaltedDirectly();

  EXPECT_GE(node.blade_off_calls.load(), 1)
      << "onHalted MUST call setBladeEnabled(false) at least once "
         "(T-08-03 safety-critical contract).";
  EXPECT_EQ(node.blade_on_calls.load(), 0)
      << "onHalted MUST NOT call setBladeEnabled(true).";
}

// ---------------------------------------------------------------------------
// Test 3 — onHalted is safe to call before onStart (idempotency / no-crash).
//
// BT.CPP can halt a StatefulActionNode that was never started (e.g., a
// parent ReactiveSequence's higher-priority condition fires before this
// child has ticked once). The contract under test: this is a no-throw,
// no-crash path. The OnHaltedDisablesBlade test above asserts the blade-off
// invariant in this same configuration; this test only adds the no-throw
// guarantee for clarity.
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

  FollowCoveragePlanUnderTest node("FollowCoveragePlanUnderTest", config);

  // Calling onHalted directly — even with no prior onStart and no active
  // sub-action goal, the cancel-and-blade-off codepath must not throw.
  EXPECT_NO_THROW({ node.invokeOnHaltedDirectly(); });
}
