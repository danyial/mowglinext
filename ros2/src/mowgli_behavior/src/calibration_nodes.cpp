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

#include "mowgli_behavior/calibration_nodes.hpp"

#include <cmath>
#include <optional>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.h"

namespace mowgli_behavior
{

namespace
{

// Fill the covariance block for a yaw-plus-xy seed: tight trust on the
// states we want to set, effectively infinite variance on the states we
// want the filter to keep from its prior.
void set_seed_covariance(geometry_msgs::msg::PoseWithCovarianceStamped& msg,
                         double yaw_var)
{
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0]  = 0.01;      // x
  msg.pose.covariance[7]  = 0.01;      // y
  msg.pose.covariance[14] = 1e6;       // z (keep prior)
  msg.pose.covariance[21] = 1e6;       // roll (keep prior)
  msg.pose.covariance[28] = 1e6;       // pitch (keep prior)
  msg.pose.covariance[35] = yaw_var;
}

// Seed ekf_odom with the same yaw we are pushing into ekf_map, at odom
// origin. Keeps odom→base yaw aligned with map→base yaw so that the
// lever-arm correction in navsat_to_absolute_pose_node rotates the
// antenna→base offset by the true robot heading. Without this, the two
// filter yaws drift apart and /gps/pose_cov ends up ~0.55 m off truth.
// Position is reset to (0, 0) — harmless because the robot is stationary
// during seeds that matter (dock) and because ClearCostmap runs right
// after so no odom-frame obstacles are lost.
void publish_odom_yaw_seed(
    const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr& pub,
    const rclcpp::Time& stamp,
    const tf2::Quaternion& q,
    double yaw_var)
{
  geometry_msgs::msg::PoseWithCovarianceStamped odom_seed;
  odom_seed.header.stamp = stamp;
  odom_seed.header.frame_id = "odom";
  odom_seed.pose.pose.position.x = 0.0;
  odom_seed.pose.pose.position.y = 0.0;
  odom_seed.pose.pose.orientation.x = q.x();
  odom_seed.pose.pose.orientation.y = q.y();
  odom_seed.pose.pose.orientation.z = q.z();
  odom_seed.pose.pose.orientation.w = q.w();
  set_seed_covariance(odom_seed, yaw_var);
  pub->publish(odom_seed);
}

}  // namespace

namespace
{

// Look up odom→base_footprint translation. Returns nullopt and logs on
// failure so callers can fall back. Uses TimePointZero for the latest
// available transform (ekf_odom publishes at 25 Hz so the latest is
// always fresh; specifying ctx->node->now() can race the publisher and
// throw ExtrapolationException right after boot).
std::optional<std::pair<double, double>> lookup_odom_xy(
    const std::shared_ptr<BTContext>& ctx, const char* who)
{
  try
  {
    auto tf = ctx->tf_buffer->lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    return std::make_pair(tf.transform.translation.x,
                          tf.transform.translation.y);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "%s: TF lookup odom→base_footprint failed: %s",
                who, ex.what());
    return std::nullopt;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// RecordUndockStart
// ---------------------------------------------------------------------------

BT::NodeStatus RecordUndockStart::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Snapshot odom-frame position, NOT raw GPS. odom→base_footprint comes
  // from ekf_odom which fuses wheels + gyro only — it cannot jump the way
  // RTK Fixed→Float transitions at the dock can jump /gps/absolute_pose
  // (issue #73). The odom-frame displacement is the true physical motion;
  // we convert the resulting heading to map frame via map→odom rotation
  // in CalibrateHeadingFromUndock below.
  auto xy = lookup_odom_xy(ctx, "RecordUndockStart");
  if (!xy)
  {
    // No TF yet — refuse to record so CalibrateHeadingFromUndock
    // skips with "no undock_start recorded" and we fall back to the
    // dock_yaw seed alone. Returning FAILURE would propagate through
    // UndockSequence and force a retry that won't help (TF still not
    // ready), so SUCCESS is the safer choice.
    return BT::NodeStatus::SUCCESS;
  }
  ctx->undock_start_x = xy->first;
  ctx->undock_start_y = xy->second;
  ctx->undock_start_recorded = true;
  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordUndockStart: odom_pos=(%.3f, %.3f)",
              ctx->undock_start_x,
              ctx->undock_start_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock
// ---------------------------------------------------------------------------

BT::NodeStatus CalibrateHeadingFromUndock::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->undock_start_recorded)
  {
    // RecordUndockStart never fired (e.g., fresh retry after a failed
    // undock where the node was halted). Report SUCCESS rather than
    // FAILURE so the rest of the sequence runs; the dock_yaw injection
    // (hardware_bridge → ekf_map via dock_yaw_to_set_pose) is still
    // active and will have seeded the filter from the charging state.
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: no undock_start recorded, "
                "skipping (relying on dock_yaw seed).");
    return BT::NodeStatus::SUCCESS;
  }

  // Minimum displacement (in odom frame, GPS-free) to refine yaw. With
  // wheel encoders + gyro the σ_pos over a 1.5 m straight-line BackUp is
  // a few cm at most, so 0.20 m gives σ_yaw ≈ atan2(2·0.05, 0.20) ≈ 27° —
  // dominant over our seed-floor σ=10° at the lower bound. Larger BackUps
  // sharpen this naturally.
  double min_displacement = 0.20;
  getInput("min_displacement_m", min_displacement);

  auto xy_now = lookup_odom_xy(ctx, "CalibrateHeadingFromUndock");
  if (!xy_now)
  {
    ctx->undock_start_recorded = false;
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: TF unavailable — skipping "
                "yaw refinement, keeping dock_yaw seed.");
    ctx->yaw_seeded_this_session = true;
    return BT::NodeStatus::SUCCESS;
  }
  const double dx = xy_now->first - ctx->undock_start_x;
  const double dy = xy_now->second - ctx->undock_start_y;
  const double dist = std::hypot(dx, dy);

  if (dist < min_displacement)
  {
    const bool still_on_dock = ctx->latest_power.charger_enabled;
    ctx->undock_start_recorded = false;
    if (still_on_dock)
    {
      // BackUp reported complete but odom barely moved AND the dock still
      // charges — robot is genuinely stuck on the dock latch. Fail so
      // UndockSequence fails and the outer ReactiveSequence retries.
      RCLCPP_WARN(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: odom displacement %.3fm below min %.3fm "
                  "AND is_charging=true — robot stuck on dock, retrying undock.",
                  dist, min_displacement);
      return BT::NodeStatus::FAILURE;
    }
    // Partial undock: odom moved less than expected (wheel slip on the dock
    // ramp is common) but charging has dropped, so the robot IS off the
    // dock. The displacement is too short to refine yaw reliably — trust
    // the dock_yaw injected by dock_yaw_to_set_pose while still on the
    // dock and continue with the rest of the session.
    RCLCPP_INFO(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: partial undock (%.3fm < %.3fm) but "
                "charging dropped — keeping dock_yaw seed, skipping refinement.",
                dist, min_displacement);
    ctx->yaw_seeded_this_session = true;
    return BT::NodeStatus::SUCCESS;
  }

  // Robot moved backward during the BackUp. Motion vector (dx, dy) points
  // OPPOSITE to the robot's heading, so heading_in_odom = atan2(-dy, -dx).
  const double yaw_odom = std::atan2(-dy, -dx);

  // Convert odom-frame yaw to map-frame yaw via the current map→odom
  // rotation. Both EKFs were seeded to dock_yaw so this rotation is
  // typically near zero, but a few seconds of GPS innovations during
  // BackUp can rotate it by a degree or two — accounting for it keeps
  // the seed honest. Falls back to identity if TF is briefly unavailable.
  double map_to_odom_yaw = 0.0;
  try
  {
    auto map_to_odom = ctx->tf_buffer->lookupTransform(
        "map", "odom", tf2::TimePointZero, tf2::durationFromSec(0.2));
    map_to_odom_yaw = tf2::getYaw(map_to_odom.transform.rotation);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: map→odom lookup failed (%s) — "
                "assuming identity rotation for yaw seed.",
                ex.what());
  }
  const double yaw_map = yaw_odom + map_to_odom_yaw;

  // Position seed: prefer the EKF's current map→base_footprint estimate
  // (TF) over raw GPS (which may have just jumped). If TF is unavailable
  // for any reason, fall back to ctx->gps_x/y so we still publish a seed.
  double seed_x = ctx->gps_x;
  double seed_y = ctx->gps_y;
  try
  {
    auto map_to_base = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.2));
    seed_x = map_to_base.transform.translation.x;
    seed_y = map_to_base.transform.translation.y;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: map→base_footprint lookup "
                "failed (%s) — falling back to /gps/absolute_pose for "
                "position seed.",
                ex.what());
  }

  if (!set_pose_pub_)
  {
    auto qos = rclcpp::QoS(1).reliable();
    set_pose_pub_ = ctx->node->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf_map_node/set_pose", qos);
    set_pose_odom_pub_ = ctx->node->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/set_pose", qos);
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_map);

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = seed_x;
  seed.pose.pose.position.y = seed_y;
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  // σ ≈ atan(2·σ_pos / displacement). Use σ_pos = 5 cm for wheel+gyro
  // dead-reckoning over 1–2 m. Floor σ ≈ 1.3° on long undocks.
  const double sigma_yaw = std::atan2(2.0 * 0.05, std::max(dist, 0.05));
  const double yaw_var = std::max(sigma_yaw * sigma_yaw, 5e-4);
  set_seed_covariance(seed, yaw_var);
  set_pose_pub_->publish(seed);
  publish_odom_yaw_seed(set_pose_odom_pub_, seed.header.stamp, q, yaw_var);

  ctx->undock_start_recorded = false;
  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "CalibrateHeadingFromUndock: odom_dist=%.3fm yaw_odom=%.1f° "
              "map→odom=%.1f° yaw_map=%.1f° pos=(%.3f, %.3f) — set_pose published.",
              dist,
              yaw_odom * 180.0 / M_PI,
              map_to_odom_yaw * 180.0 / M_PI,
              yaw_map * 180.0 / M_PI,
              seed_x, seed_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// SeedYawFromMotion
// ---------------------------------------------------------------------------

void SeedYawFromMotion::publish_forward(const rclcpp::Node::SharedPtr& node,
                                        double speed)
{
  geometry_msgs::msg::TwistStamped cmd{};
  cmd.header.stamp = node->now();
  cmd.header.frame_id = "base_footprint";
  cmd.twist.linear.x = speed;
  cmd_pub_->publish(cmd);
}

void SeedYawFromMotion::publish_zero(const rclcpp::Node::SharedPtr& node)
{
  publish_forward(node, 0.0);
}

BT::NodeStatus SeedYawFromMotion::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->yaw_seeded_this_session)
  {
    // Already seeded this session — don't re-drive on ReactiveSequence
    // halt/resume cycles.
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "SeedYawFromMotion: yaw already seeded this session, "
                 "skipping forward drive.");
    return BT::NodeStatus::SUCCESS;
  }

  if (!cmd_pub_)
  {
    cmd_pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_teleop", 10);
  }
  if (!set_pose_pub_)
  {
    auto qos = rclcpp::QoS(1).reliable();
    set_pose_pub_ = ctx->node->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf_map_node/set_pose", qos);
    set_pose_odom_pub_ = ctx->node->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/set_pose", qos);
  }

  distance_m_ = 1.0;
  speed_ms_ = 0.2;
  timeout_sec_ = 30.0;
  min_displacement_m_ = 0.5;
  getInput("distance_m", distance_m_);
  getInput("speed_ms", speed_ms_);
  getInput("timeout_sec", timeout_sec_);
  getInput("min_displacement_m", min_displacement_m_);

  if (!ctx->gps_is_fixed)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: no RTK fix at start (fix_type=%u), "
                "aborting seed.",
                ctx->gps_fix_type);
    return BT::NodeStatus::FAILURE;
  }

  x0_ = ctx->gps_x;
  y0_ = ctx->gps_y;
  start_time_ = ctx->node->now();

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: start pos=(%.3f, %.3f), drive %.2fm fwd at %.2fm/s",
              x0_, y0_, distance_m_, speed_ms_);
  publish_forward(ctx->node, speed_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SeedYawFromMotion::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->latest_emergency.active_emergency ||
      ctx->latest_emergency.latched_emergency)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: emergency during seed drive, aborting.");
    return BT::NodeStatus::FAILURE;
  }

  const double elapsed = (ctx->node->now() - start_time_).seconds();
  if (elapsed > timeout_sec_)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: timeout after %.1fs, aborting.", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  const double dx = ctx->gps_x - x0_;
  const double dy = ctx->gps_y - y0_;
  const double dist = std::hypot(dx, dy);

  if (dist < distance_m_)
  {
    publish_forward(ctx->node, speed_ms_);
    return BT::NodeStatus::RUNNING;
  }

  publish_zero(ctx->node);

  if (dist < min_displacement_m_)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: displacement %.3fm below min %.3fm, "
                "seed invalid.",
                dist, min_displacement_m_);
    return BT::NodeStatus::FAILURE;
  }

  const double yaw = std::atan2(dy, dx);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = ctx->gps_x;
  seed.pose.pose.position.y = ctx->gps_y;
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  set_seed_covariance(seed, 5e-3);  // ~4° σ
  set_pose_pub_->publish(seed);
  publish_odom_yaw_seed(set_pose_odom_pub_, seed.header.stamp, q, 5e-3);

  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: dist=%.3fm yaw_seed=%.1f° pos=(%.3f, %.3f) — "
              "set_pose published.",
              dist, yaw * 180.0 / M_PI, ctx->gps_x, ctx->gps_y);
  return BT::NodeStatus::SUCCESS;
}

void SeedYawFromMotion::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (cmd_pub_)
  {
    publish_zero(ctx->node);
  }
}

}  // namespace mowgli_behavior
