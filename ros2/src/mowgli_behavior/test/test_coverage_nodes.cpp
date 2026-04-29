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
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "behaviortree_cpp/bt_factory.h"

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_interfaces/msg/checkpoint.hpp"
#include "mowgli_interfaces/msg/coverage_waypoint.hpp"
#include "mowgli_interfaces/srv/write_checkpoint.hpp"

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

// ---------------------------------------------------------------------------
// Tests 4-6 — WriteCheckpoint integration: BT-stamps-canonical-area-index
// (R-9 / R-11 production-path regression test, fixes CR-01).
//
// Spins a real rclcpp service server in the SAME process to capture
// WriteCheckpoint requests as the BT dispatches them. The DDS edge IS
// exercised here — unlike the HaltSafety tests which override at the C++
// level — because dispatch_checkpoint_write() consumes only the rclcpp client
// on its node, and we own that node.
//
// The pattern that works around the 01-08 single-process Cyclone DDS
// flakiness: ONE shared rclcpp::Node for both client and server (so the
// transport layer is not crossed at all — the Cyclone shm fast path is used),
// SingleThreadedExecutor on a worker thread, deterministic wait_for_service
// + 250 ms post-dispatch settle wait. Mirrors the in-process pattern in
// mowgli_coverage_planner's test_coverage_planner_skeleton.cpp.
// ---------------------------------------------------------------------------

namespace
{
/// Test subclass exposing dispatch_checkpoint_write so the test can drive it
/// directly. Same private→protected lift idea as FollowCoveragePlanUnderTest.
class FollowCoveragePlanIntegrationTest : public FollowCoveragePlan
{
public:
  using FollowCoveragePlan::FollowCoveragePlan;

  void invokeDispatchCheckpointWrite(size_t completed_end_idx_exclusive)
  {
    dispatch_checkpoint_write(completed_end_idx_exclusive);
  }

  void setCoveragePlan(std::vector<CW> plan)
  {
    coverage_plan_ = std::move(plan);
  }

  // Override setBladeEnabled to count blade-OFF transitions. No assertions
  // yet in these three tests, but future fixture-mates can use the counter
  // to verify blade-cleanup contracts on dispatch paths.
  std::atomic<int> blade_off_calls_{0};

protected:
  void setBladeEnabled(bool enabled) override
  {
    if (!enabled) {
      ++blade_off_calls_;
    }
  }
};
}  // namespace

class WriteCheckpointAreaIndexTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bt_node_ = std::make_shared<rclcpp::Node>(
        "test_bt_node_writecheckpoint",
        rclcpp::NodeOptions{});
    ctx_ = std::make_shared<mowgli_behavior::BTContext>();
    ctx_->node = bt_node_;
    ctx_->helper_node = bt_node_;
    ctx_->last_mow_angle_used_deg = 42.5;  // distinguishable from 0.0

    // Stub WriteCheckpoint server — captures requests into captured_.
    captured_.clear();
    using mowgli_interfaces::srv::WriteCheckpoint;
    server_ = bt_node_->create_service<WriteCheckpoint>(
        "/coverage_planner_node/write_checkpoint",
        [this](const std::shared_ptr<WriteCheckpoint::Request> req,
               std::shared_ptr<WriteCheckpoint::Response> res) {
          {
            std::lock_guard<std::mutex> lk(captured_mutex_);
            captured_.push_back(req->checkpoint);
          }
          res->success = true;
          res->error_message.clear();
        });

    // Spin on a worker thread.
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(bt_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    // Build the standard coverage plan fixture.
    plan_ = build_synthetic_plan();

    // Build a blackboard so the test subclass can read ctx.
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("context", ctx_);
    config_.blackboard = blackboard_;
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    server_.reset();
    bt_node_.reset();
    ctx_.reset();
  }

  std::vector<CW> build_synthetic_plan()
  {
    constexpr std::uint32_t kNoArea = std::numeric_limits<std::uint32_t>::max();
    std::vector<CW> p;
    auto mk = [](std::uint32_t seq, std::uint32_t area, std::uint8_t st) {
      CW wp;
      wp.sequence_id = seq;
      wp.area_index = area;
      wp.segment_type = st;
      wp.pose.header.frame_id = "map";
      wp.pose.pose.position.x = static_cast<double>(seq);
      wp.pose.pose.orientation.w = 1.0;
      return wp;
    };
    // 6 waypoints: 0-1 UNDOCK (no area), 2-3 MOWING for area 1, 4-5 dock segs.
    p.push_back(mk(0, kNoArea, CW::SEGMENT_UNDOCK));
    p.push_back(mk(1, kNoArea, CW::SEGMENT_UNDOCK));
    p.push_back(mk(2, 1u,      CW::SEGMENT_MOWING_BOUSTROPHEDON));
    p.push_back(mk(3, 1u,      CW::SEGMENT_MOWING_BOUSTROPHEDON));
    p.push_back(mk(4, kNoArea, CW::SEGMENT_DOCK_APPROACH));
    p.push_back(mk(5, kNoArea, CW::SEGMENT_DOCKING));
    return p;
  }

  std::size_t captured_size() const
  {
    std::lock_guard<std::mutex> lk(captured_mutex_);
    return captured_.size();
  }

  // Wait up to `timeout` for captured_size() to reach `expected`. Returns
  // true if reached, false on timeout. Polls every 25 ms.
  bool wait_for_captured(std::size_t expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (captured_size() >= expected) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return captured_size() >= expected;
  }

  rclcpp::Node::SharedPtr bt_node_;
  std::shared_ptr<mowgli_behavior::BTContext> ctx_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Service<mowgli_interfaces::srv::WriteCheckpoint>::SharedPtr server_;
  std::vector<CW> plan_;
  BT::Blackboard::Ptr blackboard_;
  BT::NodeConfig config_;

  mutable std::mutex captured_mutex_;
  std::vector<mowgli_interfaces::msg::Checkpoint> captured_;
};

TEST_F(WriteCheckpointAreaIndexTest, PersistsCanonicalAreaIndex)
{
  FollowCoveragePlanIntegrationTest node("FollowCoveragePlanIntegrationTest", config_);
  node.setCoveragePlan(plan_);

  // Last completed waypoint = index 3 (second MOWING for area 1).
  node.invokeDispatchCheckpointWrite(/*completed_end_idx_exclusive=*/4);

  ASSERT_TRUE(wait_for_captured(1u, std::chrono::milliseconds(2000)))
      << "no WriteCheckpoint request captured within 2 s";

  std::lock_guard<std::mutex> lk(captured_mutex_);
  ASSERT_EQ(captured_.size(), 1u);
  EXPECT_EQ(captured_[0].area_index, 1u)
      << "BT must stamp the canonical area_index, not sequence_id";
  EXPECT_NE(captured_[0].area_index, 3u)
      << "BT must not echo sequence_id (3) into area_index";
  EXPECT_DOUBLE_EQ(captured_[0].last_mow_angle_deg, 42.5)
      << "BT must propagate ctx->last_mow_angle_used_deg, not 0.0";
}

TEST_F(WriteCheckpointAreaIndexTest, SkipsSentinelAreaIndex)
{
  FollowCoveragePlanIntegrationTest node("FollowCoveragePlanIntegrationTest", config_);
  node.setCoveragePlan(plan_);

  // Last completed waypoint = index 1 (second UNDOCK with area_index=UINT32_MAX).
  node.invokeDispatchCheckpointWrite(/*completed_end_idx_exclusive=*/2);

  // Wait 500 ms — even a slow service round-trip would have arrived by then.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_EQ(captured_size(), 0u)
      << "BT must NOT dispatch a checkpoint write for sentinel area_index "
         "(UINT32_MAX) — dock/undock segments have no canonical area";
}

TEST_F(WriteCheckpointAreaIndexTest, DoesNotEchoSequenceIdAsAreaIndex)
{
  // Fresh plan where the offending pre-fix bug would surface as area_index=42.
  std::vector<CW> p;
  CW wp;
  wp.sequence_id = 42u;
  wp.area_index = 1u;
  wp.segment_type = CW::SEGMENT_OUTLINE_WORKING_AREA;
  wp.pose.header.frame_id = "map";
  wp.pose.pose.orientation.w = 1.0;
  p.push_back(wp);

  FollowCoveragePlanIntegrationTest node("FollowCoveragePlanIntegrationTest", config_);
  node.setCoveragePlan(p);

  node.invokeDispatchCheckpointWrite(/*completed_end_idx_exclusive=*/1);

  ASSERT_TRUE(wait_for_captured(1u, std::chrono::milliseconds(2000)));
  std::lock_guard<std::mutex> lk(captured_mutex_);
  EXPECT_EQ(captured_[0].area_index, 1u)
      << "regression guard for CR-01: area_index must come from "
         "wp.area_index, not wp.sequence_id";
}
