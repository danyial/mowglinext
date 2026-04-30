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
 * @file test_docking_nodes.cpp
 * @brief gtests for the 5 Phase-2 LiDAR-dock BT nodes (Plan 02-06).
 *
 * Pins SPEC R-5..R-9 + R-11..R-13 contracts at the unit-test level. The
 * production /dock_match/* + /scan_kicp surfaces are mocked by writing
 * directly into BTContext (the same struct the live behavior_tree_node
 * subscribers populate at runtime), so these tests exercise the BT
 * contract surface without standing up the full DDS subscription stack.
 *
 * Test cases (all 13 frozen contracts):
 *   1.  RecordDockApproachPosePrefersLidarMatch        (R-5)
 *   2.  RecordDockApproachPoseFallsBackToTf            (R-5)
 *   3.  RecordDockApproachPoseAutoRefreshTriggers      (R-11)
 *   4.  RecordDockApproachPoseAutoRefreshSkipsLowConfidence  (R-11)
 *   5.  ApproachDockFailsOnMissingYaml                 (R-6)
 *   6.  ApproachDockSendsCorrectGoal                   (R-6)
 *   7.  FineDockSucceedsOnIsCharging                   (R-7)
 *   8.  FineDockAbortsOnConfidenceLoss                 (R-8)
 *   9.  FineDockHaltsOnEmergency                       (R-9)
 *   10. PreUndockClearanceFailsOnRearObstacle          (R-12)
 *   11. PreUndockClearanceSucceedsWhenClear            (R-12)
 *   12. PostUndockRtkValidationThresholds              (R-13)
 *   13. FineDockBailsAfter5sNoPose                     (WARNING-9 degraded mode)
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "behaviortree_cpp/bt_factory.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/docking_nodes.hpp"
#include "mowgli_interfaces/msg/dock_match_confidence.hpp"

using mowgli_behavior::ApproachDock;
using mowgli_behavior::BTContext;
using mowgli_behavior::FineDock;
using mowgli_behavior::PostUndockRtkValidation;
using mowgli_behavior::PreUndockClearanceCheck;
using mowgli_behavior::RecordDockApproachPose;

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
// Helpers
// ---------------------------------------------------------------------------

namespace
{

std::string make_tmp_dir()
{
  // /tmp/docking_nodes_<pid>_<counter>/ — unique per test, cleaned in TearDown.
  static std::atomic<int> counter{0};
  std::string p = "/tmp/docking_nodes_" + std::to_string(::getpid()) + "_" +
                  std::to_string(counter.fetch_add(1));
  std::filesystem::create_directories(p);
  return p;
}

void write_dock_calibration_yaml(const std::string& path, double dx, double dy, double dyaw)
{
  std::ofstream f(path);
  f << "dock_pose_x: " << dx << "\n";
  f << "dock_pose_y: " << dy << "\n";
  f << "dock_pose_yaw_rad: " << dyaw << "\n";
}

std::shared_ptr<BTContext> make_ctx(const std::shared_ptr<rclcpp::Node>& node)
{
  auto ctx = std::make_shared<BTContext>();
  ctx->node = node;
  ctx->helper_node = node;
  ctx->tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  ctx->tf_listener = std::make_shared<tf2_ros::TransformListener>(*ctx->tf_buffer);
  return ctx;
}

BT::NodeConfig make_config(const std::shared_ptr<BTContext>& ctx,
                           BT::Blackboard::Ptr& bb_out)
{
  bb_out = BT::Blackboard::create();
  bb_out->set("context", ctx);
  BT::NodeConfig cfg;
  cfg.blackboard = bb_out;
  return cfg;
}

void publish_static_tf(const std::shared_ptr<rclcpp::Node>& node,
                       std::shared_ptr<tf2_ros::Buffer>& buf,
                       const std::string& parent,
                       const std::string& child,
                       double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped tfm;
  tfm.header.stamp = node->now();
  tfm.header.frame_id = parent;
  tfm.child_frame_id = child;
  tfm.transform.translation.x = x;
  tfm.transform.translation.y = y;
  tfm.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  tfm.transform.rotation.x = q.x();
  tfm.transform.rotation.y = q.y();
  tfm.transform.rotation.z = q.z();
  tfm.transform.rotation.w = q.w();
  buf->setTransform(tfm, "test_authority", /*is_static=*/true);
}

}  // namespace

// ---------------------------------------------------------------------------
// Fixture — common BTContext + tmpdir for all tests.
// ---------------------------------------------------------------------------

class DockingNodesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_docking_nodes_node");
    ctx_ = make_ctx(node_);
    tmpdir_ = make_tmp_dir();
    ctx_->dock_approach_path = tmpdir_ + "/dock_approach.yaml";
    ctx_->dock_calibration_path = tmpdir_ + "/dock_calibration.yaml";
    ctx_->dock_scan_path = tmpdir_ + "/dock_scan.pcd";
    ctx_->dock_scan_meta_path = tmpdir_ + "/dock_scan_meta.yaml";
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmpdir_, ec);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<BTContext> ctx_;
  std::string tmpdir_;
};

// ---------------------------------------------------------------------------
// 1. RecordDockApproachPose — prefers /dock_match/pose when trusted+fresh
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, RecordDockApproachPosePrefersLidarMatch)
{
  // dock at (3, 4); robot is at (1, 2) per /dock_match/pose.
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 3.0, 4.0, 0.0);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 1.0;
  pose.pose.pose.position.y = 2.0;
  ctx_->latest_dock_match_pose = pose;
  ctx_->latest_dock_match_pose_received = true;
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();
  ctx_->last_dock_match_trusted = true;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  RecordDockApproachPose n("RecordDockApproachPose", cfg);

  EXPECT_EQ(n.tick(), BT::NodeStatus::SUCCESS);

  // Read back dock_approach.yaml — flat key=value, parse simply.
  std::ifstream f(ctx_->dock_approach_path);
  ASSERT_TRUE(f.good()) << "dock_approach.yaml not written";
  std::string content((std::istreambuf_iterator<char>(f)),
                      std::istreambuf_iterator<char>());

  EXPECT_NE(content.find("source: lidar"), std::string::npos)
      << "expected source=lidar, got: " << content;
  // yaw_to_dock = atan2(4-2, 3-1) = atan2(2, 2) = π/4 ≈ 0.7854
  // Just look for a numeric token near that value — flat-yaml tolerant.
  EXPECT_NE(content.find("0.7853"), std::string::npos)
      << "expected yaw_to_dock_rad ≈ 0.7853, got: " << content;
}

// ---------------------------------------------------------------------------
// 2. RecordDockApproachPose — falls back to TF when /dock_match not trusted
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, RecordDockApproachPoseFallsBackToTf)
{
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 3.0, 4.0, 0.0);

  // No trusted /dock_match — but TF can resolve map -> base_footprint.
  publish_static_tf(node_, ctx_->tf_buffer, "map", "base_footprint", 1.5, 2.5, 0.0);
  ctx_->last_dock_match_trusted = false;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  RecordDockApproachPose n("RecordDockApproachPose", cfg);

  EXPECT_EQ(n.tick(), BT::NodeStatus::SUCCESS);

  std::ifstream f(ctx_->dock_approach_path);
  ASSERT_TRUE(f.good());
  std::string content((std::istreambuf_iterator<char>(f)),
                      std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("source: tf"), std::string::npos)
      << "expected source=tf, got: " << content;
  EXPECT_NE(content.find("1.500"), std::string::npos);
  EXPECT_NE(content.find("2.500"), std::string::npos);
}

// ---------------------------------------------------------------------------
// 3. RecordDockApproachPose — auto-refresh fires on 8d-old meta + high inlier
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, RecordDockApproachPoseAutoRefreshTriggers)
{
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 3.0, 4.0, 0.0);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 1.0;
  pose.pose.pose.position.y = 2.0;
  ctx_->latest_dock_match_pose = pose;
  ctx_->latest_dock_match_pose_received = true;
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();
  ctx_->last_dock_match_trusted = true;

  // 8 days ago — well past the 7d threshold.
  std::ofstream meta(ctx_->dock_scan_meta_path);
  meta << "dock_scan_pcd_path: " << ctx_->dock_scan_path << "\n";
  meta << "dock_pose_x: 3.0\n";
  meta << "dock_pose_y: 4.0\n";
  meta << "dock_pose_yaw_rad: 0.0\n";
  meta << "sensor_extrinsic_x: 0.0\n";
  meta << "sensor_extrinsic_y: 0.0\n";
  meta << "sensor_extrinsic_yaw_rad: 0.0\n";
  meta << "point_count: 100\n";
  meta << "captured_at: 2026-04-21T00:00:00Z\n";
  meta << "fix_type: RTK_FIXED\n";
  meta.close();

  // Stub PCD so the node has something to overwrite.
  std::ofstream pcd(ctx_->dock_scan_path);
  pcd << "stub\n";
  pcd.close();

  // Inlier ratio 0.97 >= 0.95 — refresh should trigger.
  ctx_->latest_dock_match_conf.inlier_ratio = 0.97f;
  ctx_->latest_dock_match_conf.rmse_m = 0.02f;
  ctx_->latest_dock_match_conf.trusted = true;

  // Synthetic /scan_kicp — simple ring of 36 points at radius 1 m.
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link_wheels";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = 2.0 * M_PI / 36.0;
  scan.range_min = 0.05;
  scan.range_max = 12.0;
  scan.ranges.assign(36, 1.0f);
  ctx_->latest_scan_kicp = scan;
  ctx_->latest_scan_kicp_received = true;
  ctx_->last_scan_kicp_at = std::chrono::steady_clock::now();

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  RecordDockApproachPose n("RecordDockApproachPose", cfg);
  EXPECT_EQ(n.tick(), BT::NodeStatus::SUCCESS);

  // Meta should have been rewritten with a fresher captured_at — load and
  // expect a year that is >= 2026 (smoke check; we don't pin the day).
  std::ifstream meta_after(ctx_->dock_scan_meta_path);
  ASSERT_TRUE(meta_after.good());
  std::string meta_content((std::istreambuf_iterator<char>(meta_after)),
                           std::istreambuf_iterator<char>());
  EXPECT_NE(meta_content.find("captured_at: 2026"), std::string::npos)
      << "expected refreshed captured_at, got: " << meta_content;
  // The captured_at MUST NOT still be the stale 04-21.
  EXPECT_EQ(meta_content.find("captured_at: 2026-04-21"), std::string::npos)
      << "captured_at was not refreshed: " << meta_content;
}

// ---------------------------------------------------------------------------
// 4. RecordDockApproachPose — skips refresh when inlier_ratio < 0.95
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, RecordDockApproachPoseAutoRefreshSkipsLowConfidence)
{
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 3.0, 4.0, 0.0);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 1.0;
  pose.pose.pose.position.y = 2.0;
  ctx_->latest_dock_match_pose = pose;
  ctx_->latest_dock_match_pose_received = true;
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();
  ctx_->last_dock_match_trusted = true;

  // 8d-old meta same as test 3.
  std::ofstream meta(ctx_->dock_scan_meta_path);
  meta << "dock_scan_pcd_path: " << ctx_->dock_scan_path << "\n";
  meta << "dock_pose_x: 3.0\n";
  meta << "dock_pose_y: 4.0\n";
  meta << "dock_pose_yaw_rad: 0.0\n";
  meta << "sensor_extrinsic_x: 0.0\n";
  meta << "sensor_extrinsic_y: 0.0\n";
  meta << "sensor_extrinsic_yaw_rad: 0.0\n";
  meta << "point_count: 100\n";
  meta << "captured_at: 2026-04-21T00:00:00Z\n";
  meta << "fix_type: RTK_FIXED\n";
  meta.close();

  // inlier_ratio < 0.95 — refresh MUST be skipped.
  ctx_->latest_dock_match_conf.inlier_ratio = 0.85f;
  ctx_->latest_dock_match_conf.rmse_m = 0.04f;
  ctx_->latest_dock_match_conf.trusted = true;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  RecordDockApproachPose n("RecordDockApproachPose", cfg);
  EXPECT_EQ(n.tick(), BT::NodeStatus::SUCCESS);

  std::ifstream meta_after(ctx_->dock_scan_meta_path);
  ASSERT_TRUE(meta_after.good());
  std::string meta_content((std::istreambuf_iterator<char>(meta_after)),
                           std::istreambuf_iterator<char>());
  EXPECT_NE(meta_content.find("captured_at: 2026-04-21"), std::string::npos)
      << "captured_at was wrongly refreshed despite low inlier_ratio: "
      << meta_content;
}

// ---------------------------------------------------------------------------
// 5. ApproachDock — FAILURE on missing dock_approach.yaml
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, ApproachDockFailsOnMissingYaml)
{
  // Do NOT write dock_approach.yaml. ApproachDock onStart must FAIL fast.
  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  bb->set("dock_approach_path", ctx_->dock_approach_path);

  ApproachDock n("ApproachDock", cfg);
  EXPECT_EQ(n.onStart(), BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// 6. ApproachDock — sends correct goal payload
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, ApproachDockSendsCorrectGoal)
{
  // Write a valid dock_approach.yaml first.
  std::ofstream f(ctx_->dock_approach_path);
  f << "dock_approach_x: 1.234\n";
  f << "dock_approach_y: 5.678\n";
  f << "dock_approach_yaw_to_dock_rad: 0.500000\n";
  f << "source: tf\n";
  f << "captured_at: 2026-04-29T12:00:00Z\n";
  f.close();

  // In-process NavigateToPose action server that captures the goal payload.
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  std::atomic<bool> goal_received{false};
  std::atomic<double> recv_x{0.0};
  std::atomic<double> recv_y{0.0};
  std::atomic<double> recv_yaw{0.0};
  std::string recv_frame;
  std::mutex recv_mutex;

  auto server_node = std::make_shared<rclcpp::Node>("nav_to_pose_test_server");
  auto action_server = rclcpp_action::create_server<NavigateToPose>(
      server_node, "/navigate_to_pose",
      [&](const rclcpp_action::GoalUUID&,
          std::shared_ptr<const NavigateToPose::Goal> goal)
      {
        recv_x.store(goal->pose.pose.position.x);
        recv_y.store(goal->pose.pose.position.y);
        recv_yaw.store(tf2::getYaw(goal->pose.pose.orientation));
        {
          std::lock_guard<std::mutex> lk(recv_mutex);
          recv_frame = goal->pose.header.frame_id;
        }
        goal_received.store(true);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [](const std::shared_ptr<GoalHandle> gh)
      {
        // Immediately succeed.
        std::thread(
            [gh]()
            {
              auto result = std::make_shared<NavigateToPose::Result>();
              gh->succeed(result);
            })
            .detach();
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(server_node);
  std::atomic<bool> stop{false};
  std::thread spinner([&]() {
    while (!stop.load()) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  bb->set("dock_approach_path", ctx_->dock_approach_path);

  ApproachDock n("ApproachDock", cfg);
  auto onstart_status = n.onStart();
  EXPECT_EQ(onstart_status, BT::NodeStatus::RUNNING);

  // Poll until the server receives the goal (max 2 s).
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!goal_received.load() && std::chrono::steady_clock::now() < deadline)
  {
    n.onRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  stop.store(true);
  spinner.join();

  ASSERT_TRUE(goal_received.load()) << "Goal never received by mock action server";
  EXPECT_NEAR(recv_x.load(), 1.234, 1e-5);
  EXPECT_NEAR(recv_y.load(), 5.678, 1e-5);
  EXPECT_NEAR(recv_yaw.load(), 0.5, 1e-5);
  {
    std::lock_guard<std::mutex> lk(recv_mutex);
    EXPECT_EQ(recv_frame, "map");
  }
}

// ---------------------------------------------------------------------------
// 7. FineDock — SUCCESS when is_charging engages
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, FineDockSucceedsOnIsCharging)
{
  // Write a dock_approach so onStart can load it.
  std::ofstream f(ctx_->dock_approach_path);
  f << "dock_approach_x: 0.0\n";
  f << "dock_approach_y: 0.0\n";
  f << "dock_approach_yaw_to_dock_rad: 0.0\n";
  f << "source: tf\n";
  f << "captured_at: 2026-04-29T12:00:00Z\n";
  f.close();

  // Also write dock_calibration.yaml — FineDock loads dock target.
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 1.0, 0.0, 0.0);

  // /dock_match/pose flowing + trusted.
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 0.0;
  pose.pose.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  pose.pose.pose.orientation.x = q.x();
  pose.pose.pose.orientation.y = q.y();
  pose.pose.pose.orientation.z = q.z();
  pose.pose.pose.orientation.w = q.w();
  ctx_->latest_dock_match_pose = pose;
  ctx_->latest_dock_match_pose_received = true;
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();
  ctx_->latest_dock_match_conf.trusted = true;
  ctx_->last_dock_match_trusted = true;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  bb->set("dock_approach_path", ctx_->dock_approach_path);

  FineDock n("FineDock", cfg);
  EXPECT_EQ(n.onStart(), BT::NodeStatus::RUNNING);

  // Tick once with !is_charging — RUNNING.
  ctx_->latest_status.is_charging = false;
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::RUNNING);

  // Now flip is_charging — should SUCCEED on next tick.
  ctx_->latest_status.is_charging = true;
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// 8. FineDock — aborts within 1 s of trusted=false (R-8)
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, FineDockAbortsOnConfidenceLoss)
{
  std::ofstream f(ctx_->dock_approach_path);
  f << "dock_approach_x: 0.0\n";
  f << "dock_approach_y: 0.0\n";
  f << "dock_approach_yaw_to_dock_rad: 0.0\n";
  f << "source: tf\n";
  f << "captured_at: 2026-04-29T12:00:00Z\n";
  f.close();
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 1.0, 0.0, 0.0);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  ctx_->latest_dock_match_pose = pose;
  ctx_->latest_dock_match_pose_received = true;
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();
  ctx_->latest_dock_match_conf.trusted = true;
  ctx_->last_dock_match_trusted = true;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  bb->set("dock_approach_path", ctx_->dock_approach_path);

  FineDock n("FineDock", cfg);
  EXPECT_EQ(n.onStart(), BT::NodeStatus::RUNNING);

  // First tick: trusted — RUNNING.
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::RUNNING);

  // Drop trust. Streak starts now. After >= 1 s of trusted=false, FAILURE.
  ctx_->last_dock_match_trusted = false;
  ctx_->latest_dock_match_conf.trusted = false;
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();

  // First low-trust tick — should NOT yet fail (streak just started).
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::RUNNING);

  // Sleep 1100 ms then refresh pose age so the freshness gate doesn't bail.
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();

  // Now trust-loss streak >= 1s — must FAILURE.
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// 9. FineDock — FAILURE on emergency (R-9)
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, FineDockHaltsOnEmergency)
{
  std::ofstream f(ctx_->dock_approach_path);
  f << "dock_approach_x: 0.0\n";
  f << "dock_approach_y: 0.0\n";
  f << "dock_approach_yaw_to_dock_rad: 0.0\n";
  f << "source: tf\n";
  f << "captured_at: 2026-04-29T12:00:00Z\n";
  f.close();
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 1.0, 0.0, 0.0);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  ctx_->latest_dock_match_pose = pose;
  ctx_->latest_dock_match_pose_received = true;
  ctx_->last_dock_match_pose_at = std::chrono::steady_clock::now();
  ctx_->latest_dock_match_conf.trusted = true;
  ctx_->last_dock_match_trusted = true;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  bb->set("dock_approach_path", ctx_->dock_approach_path);

  FineDock n("FineDock", cfg);
  EXPECT_EQ(n.onStart(), BT::NodeStatus::RUNNING);

  ctx_->latest_emergency.active_emergency = true;
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// 10. PreUndockClearanceCheck — FAILURE on rear obstacle
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, PreUndockClearanceFailsOnRearObstacle)
{
  // 60-ray scan; ray 30 (angle = 0 - π + 30·dθ where dθ = 2π/60 = 0.1047 → angle ≈ π).
  // Use angle_min=-π, increment=2π/60. Index for angle=π is 60 (wraps); use
  // index 30 → angle = -π + 30·(2π/60) = 0. Adjust: place obstacle at index where
  // angle ≈ ±π. Easier: index 0 = -π is the rear.
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link_wheels";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = 2.0 * M_PI / 60.0;
  scan.range_min = 0.05;
  scan.range_max = 12.0;
  scan.ranges.assign(60, 5.0f);
  // Rear = ±π. Index 0 -> angle=-π; place a 0.5 m return there.
  scan.ranges[0] = 0.5f;
  scan.ranges[59] = 0.5f;
  ctx_->latest_scan_kicp = scan;
  ctx_->latest_scan_kicp_received = true;
  ctx_->last_scan_kicp_at = std::chrono::steady_clock::now();

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  PreUndockClearanceCheck n("PreUndockClearanceCheck", cfg);
  EXPECT_EQ(n.tick(), BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// 11. PreUndockClearanceCheck — SUCCESS when rear is clear
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, PreUndockClearanceSucceedsWhenClear)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link_wheels";
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = 2.0 * M_PI / 60.0;
  scan.range_min = 0.05;
  scan.range_max = 12.0;
  scan.ranges.assign(60, 5.0f);  // Plenty of clearance everywhere.
  ctx_->latest_scan_kicp = scan;
  ctx_->latest_scan_kicp_received = true;
  ctx_->last_scan_kicp_at = std::chrono::steady_clock::now();

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  PreUndockClearanceCheck n("PreUndockClearanceCheck", cfg);
  EXPECT_EQ(n.tick(), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// 12. PostUndockRtkValidation — three thresholds
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, PostUndockRtkValidationThresholds)
{
  // Set up odom -> base_footprint at (0, 0) and gps at (0.3, 0).
  publish_static_tf(node_, ctx_->tf_buffer, "odom", "base_footprint", 0.0, 0.0, 0.0);
  ctx_->gps_fix_type = 4;  // RTK_FIXED
  ctx_->gps_x = 0.3;
  ctx_->gps_y = 0.0;
  ctx_->dock_pose_suspect = false;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  PostUndockRtkValidation n("PostUndockRtkValidation", cfg);
  EXPECT_EQ(n.tick(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx_->dock_pose_suspect) << "0.3 m discrepancy must NOT flag suspect";

  // 0.7 m -> SUCCESS + suspect=true
  ctx_->gps_x = 0.7;
  EXPECT_EQ(n.tick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx_->dock_pose_suspect) << "0.7 m discrepancy must flag suspect";

  // 2.0 m -> FAILURE
  ctx_->gps_x = 2.0;
  EXPECT_EQ(n.tick(), BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// 13. FineDock — bails after 5 s with no /dock_match/pose ever received
//     (WARNING-9 degraded-mode contract; distinct from R-8 trust-loss abort)
// ---------------------------------------------------------------------------

TEST_F(DockingNodesTest, FineDockBailsAfter5sNoPose)
{
  std::ofstream f(ctx_->dock_approach_path);
  f << "dock_approach_x: 0.0\n";
  f << "dock_approach_y: 0.0\n";
  f << "dock_approach_yaw_to_dock_rad: 0.0\n";
  f << "source: tf\n";
  f << "captured_at: 2026-04-29T12:00:00Z\n";
  f.close();
  write_dock_calibration_yaml(ctx_->dock_calibration_path, 1.0, 0.0, 0.0);

  // No /dock_match/pose ever — latest_dock_match_pose_received stays false.
  ctx_->latest_dock_match_pose_received = false;
  ctx_->last_dock_match_trusted = false;

  BT::Blackboard::Ptr bb;
  auto cfg = make_config(ctx_, bb);
  bb->set("dock_approach_path", ctx_->dock_approach_path);

  FineDock n("FineDock", cfg);
  EXPECT_EQ(n.onStart(), BT::NodeStatus::RUNNING);

  // First tick — under 5 s: stays RUNNING.
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::RUNNING);

  // Sleep > 5 s — node must FAILURE on next tick.
  std::this_thread::sleep_for(std::chrono::milliseconds(5100));
  EXPECT_EQ(n.onRunning(), BT::NodeStatus::FAILURE);
}
