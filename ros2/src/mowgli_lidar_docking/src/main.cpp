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
  // GH #78: surface construction / runtime exceptions before libc's
  // abort handler eats them — without this catch the only signal we
  // get is SIGABRT with no message, which is essentially undebuggable
  // from `docker logs` output. The DockScanMatchNode constructor
  // already catches around try_load_matcher_files; this is the outer
  // safety net for anything else (rclcpp internals, allocator, etc.).
  try
  {
    rclcpp::spin(std::make_shared<mowgli_lidar_docking::DockScanMatchNode>());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_FATAL(rclcpp::get_logger("dock_scan_match"),
                 "dock_scan_match terminating on unhandled exception: %s",
                 ex.what());
    rclcpp::shutdown();
    return 1;
  }
  catch (...)
  {
    RCLCPP_FATAL(rclcpp::get_logger("dock_scan_match"),
                 "dock_scan_match terminating on non-std exception");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
