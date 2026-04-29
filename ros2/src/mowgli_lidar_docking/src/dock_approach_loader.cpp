// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_lidar_docking/dock_approach_loader.hpp"

#include <mowgli_geometry/atomic_write.hpp>

#include <iomanip>
#include <sstream>

namespace mowgli_lidar_docking
{

bool save_dock_approach_yaml(const std::string& path, const DockApproach& approach)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(6);
  os << "dock_approach_x: " << approach.x << "\n";
  os << "dock_approach_y: " << approach.y << "\n";
  os << "dock_approach_yaw_to_dock_rad: " << approach.yaw_to_dock_rad << "\n";
  os << "source: " << approach.source << "\n";
  os << "captured_at: " << approach.captured_at << "\n";
  return mowgli_geometry::atomic_write(path, os.str());
}

}  // namespace mowgli_lidar_docking
