// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SPDX-License-Identifier: GPL-3.0
//
// Implementation notes:
//
//   * D-06 mandates PCL ASCII (binary=false). pcl::io::savePCDFile defaults
//     to ASCII when its `binary_mode` argument is left at the default false,
//     but we pass it explicitly for grep-ability.
//
//   * The atomic variant renders the same PCL ASCII format manually into a
//     std::string and hands it to mowgli_geometry::atomic_write. We don't
//     route the binary file descriptor through atomic_write because PCL's
//     savePCDFile owns its own file handle and there is no public hook to
//     redirect it to a writable string buffer in the Kilted apt build of
//     PCL. The hand-rolled format is the canonical PCD v0.7 ASCII layout
//     (FIELDS x y z / SIZE 4 4 4 / TYPE F F F F → wait, three Fs for x/y/z;
//     COUNT 1 1 1; WIDTH N; HEIGHT 1; VIEWPOINT 0 0 0 1 0 0 0; POINTS N;
//     DATA ascii) which both pcl_viewer and load_dock_scan_pcd parse.

#include "mowgli_lidar_docking/dock_scan_io.hpp"

#include <mowgli_geometry/atomic_write.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iomanip>
#include <sstream>

namespace mowgli_lidar_docking
{

namespace
{

/// Convert the input vector into a pcl::PointCloud<pcl::PointXYZ> with
/// width=N, height=1, is_dense=true. Used by the non-atomic save path.
pcl::PointCloud<pcl::PointXYZ> to_pcl_cloud(
    const std::vector<Eigen::Vector3d>& points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    cloud.points[i].x = static_cast<float>(points[i].x());
    cloud.points[i].y = static_cast<float>(points[i].y());
    cloud.points[i].z = static_cast<float>(points[i].z());
  }
  return cloud;
}

/// Render `points` as canonical PCD v0.7 ASCII into a std::string.
/// Used by the atomic save path so we can hand the rendered content to
/// mowgli_geometry::atomic_write (temp file + fsync + rename + dir fsync).
std::string render_pcd_ascii(const std::vector<Eigen::Vector3d>& points)
{
  const std::size_t n = points.size();
  std::ostringstream os;
  os << "# .PCD v0.7 - Point Cloud Data file format\n"
     << "VERSION 0.7\n"
     << "FIELDS x y z\n"
     << "SIZE 4 4 4\n"
     << "TYPE F F F\n"
     << "COUNT 1 1 1\n"
     << "WIDTH " << n << "\n"
     << "HEIGHT 1\n"
     << "VIEWPOINT 0 0 0 1 0 0 0\n"
     << "POINTS " << n << "\n"
     << "DATA ascii\n";
  os << std::fixed << std::setprecision(6);
  for (const auto& p : points)
  {
    // PCL ASCII writes float32 internally; we keep 6 decimals so a
    // round-trip via load_dock_scan_pcd matches to ~1e-4 m (PCL's float
    // truncation dominates the error).
    os << static_cast<float>(p.x()) << " "
       << static_cast<float>(p.y()) << " "
       << static_cast<float>(p.z()) << "\n";
  }
  return os.str();
}

}  // namespace

bool save_dock_scan_pcd(const std::string& path,
                        const std::vector<Eigen::Vector3d>& points)
{
  const auto cloud = to_pcl_cloud(points);
  // binary_mode = false → PCL ASCII per D-06.
  const int rc = pcl::io::savePCDFile(path, cloud, false);
  return rc == 0;
}

bool save_dock_scan_pcd_atomic(const std::string& path,
                               const std::vector<Eigen::Vector3d>& points)
{
  const std::string content = render_pcd_ascii(points);
  return mowgli_geometry::atomic_write(path, content);
}

bool load_dock_scan_pcd(const std::string& path,
                        std::vector<Eigen::Vector3d>& out_points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const int rc = pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud);
  if (rc != 0) return false;

  out_points.clear();
  out_points.reserve(cloud.points.size());
  for (const auto& p : cloud.points)
  {
    out_points.emplace_back(static_cast<double>(p.x),
                            static_cast<double>(p.y),
                            static_cast<double>(p.z));
  }
  return true;
}

}  // namespace mowgli_lidar_docking
