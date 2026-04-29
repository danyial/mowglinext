// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Plan 02-04 SPEC R-2 + R-3 entry point: spawns the dock_scan_match
// rclcpp node and spins until shutdown. Single-threaded executor (default
// rclcpp::spin); the TF distance gate keeps the duty cycle low enough
// that we don't need a MultiThreadedExecutor on the Pi5 (Pitfall 5).
//
// Architecture Invariant #1 reminder: this binary publishes /dock_match/
// pose + /dock_match/confidence ONLY. No TF broadcaster, no /odom*
// publisher, no robot_localization output subscription. Plan 02-05 wires
// /dock_match/pose into the EKF seed cascade via /set_pose, never via TF.

#include "mowgli_lidar_docking/dock_scan_match_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_lidar_docking::DockScanMatchNode>());
  rclcpp::shutdown();
  return 0;
}
