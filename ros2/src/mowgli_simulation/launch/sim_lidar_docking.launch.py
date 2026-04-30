# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
sim_lidar_docking.launch.py — Plan 02-08 sim composition.

Brings up the FineDock pipeline in headless Gazebo:
  1. The existing simulation.launch.py (world + spawn + ros_gz_bridge + RViz).
  2. The synthetic /scan_kicp publisher (D-14) — replaces the LD19 in sim.
  3. dock_scan_match (Plan 02-04) — consumes /scan_kicp + the pre-baked
     test PCD and publishes /dock_match/pose + /dock_match/confidence.

The pre-baked sim PCD ships with the package as
  test_data/sim_dock_scan.pcd  + test_data/sim_dock_scan_meta.yaml
so dock_scan_match starts in non-degraded mode immediately. The launch
file copies them into the maps directory the production node reads from
(`/ros2_ws/maps/` by default; configurable via the `maps_dir` argument).

WARNING — sim-only: this launch file MUST NOT be used on Pi5 hardware.
Production deployments use `mowgli_bringup/launch/full_system.launch.py`
which composes the real LD19 driver + kinematic_icp_scan_frame_relay.
Running both publishers on /scan_kicp would have the DDS layer interleave
their messages and break dock_scan_match's voxel-map registration.
"""

from __future__ import annotations

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _stage_pre_baked_pcd(context):
    """Copy the committed test_data/{sim_dock_scan.pcd,meta.yaml} into the
    operator's maps_dir at the canonical names dock_scan_match expects.
    Done at launch time (rather than baked into the install share) so
    operators can override via `maps_dir:=...` at the command line."""
    sim_share = get_package_share_directory("mowgli_simulation")
    maps_dir = LaunchConfiguration("maps_dir").perform(context)
    os.makedirs(maps_dir, exist_ok=True)
    src_pcd = os.path.join(sim_share, "test_data", "sim_dock_scan.pcd")
    src_meta = os.path.join(sim_share, "test_data", "sim_dock_scan_meta.yaml")
    dst_pcd = os.path.join(maps_dir, "dock_scan.pcd")
    dst_meta = os.path.join(maps_dir, "dock_scan_meta.yaml")
    if os.path.isfile(src_pcd):
        shutil.copy2(src_pcd, dst_pcd)
    if os.path.isfile(src_meta):
        shutil.copy2(src_meta, dst_meta)
    return []


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory("mowgli_simulation")

    # --- launch arguments ----------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="garden",
        description="Gazebo world name (forwarded to simulation.launch.py).",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo without GUI (CI default).",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Skip RViz in CI by default.",
    )
    dock_pose_x_arg = DeclareLaunchArgument(
        "dock_pose_x",
        default_value="5.0",
        description="Sim dock anchor x in map frame (synthetic publisher param).",
    )
    dock_pose_y_arg = DeclareLaunchArgument(
        "dock_pose_y",
        default_value="5.0",
        description="Sim dock anchor y in map frame.",
    )
    dock_pose_yaw_arg = DeclareLaunchArgument(
        "dock_pose_yaw_rad",
        default_value="0.0",
        description="Sim dock anchor yaw (rad). +x of dock-local frame points away from the robot.",
    )
    maps_dir_arg = DeclareLaunchArgument(
        "maps_dir",
        default_value="/ros2_ws/maps",
        description="Directory the matcher reads dock_scan.pcd / dock_scan_meta.yaml from.",
    )

    # --- 1. existing simulation stack ---------------------------------
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    # --- 2. synthetic /scan_kicp publisher ----------------------------
    synthetic_publisher = Node(
        package="mowgli_simulation",
        executable="synthetic_scan_kicp_publisher.py",
        name="synthetic_scan_kicp_publisher",
        output="screen",
        parameters=[
            {
                "output_topic": "/scan_kicp",
                "output_frame": "lidar_link_wheels",
                "publish_rate_hz": 10.0,
                "dock_pose_x": LaunchConfiguration("dock_pose_x"),
                "dock_pose_y": LaunchConfiguration("dock_pose_y"),
                "dock_pose_yaw_rad": LaunchConfiguration("dock_pose_yaw_rad"),
                "use_sim_time": True,
            }
        ],
    )

    # --- 3. dock_scan_match (Plan 02-04) ------------------------------
    # Pre-baked PCD is copied via an OpaqueFunction below before this
    # node starts.
    dock_scan_match = Node(
        package="mowgli_lidar_docking",
        executable="dock_scan_match",
        name="dock_scan_match",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("mowgli_lidar_docking"),
                "config",
                "dock_scan_match.yaml",
            ),
            {
                "use_sim_time": True,
                # Matches what the launch arg passes to _stage_pre_baked_pcd.
                "dock_scan_pcd_path": [
                    LaunchConfiguration("maps_dir"),
                    "/dock_scan.pcd",
                ],
                "dock_scan_meta_path": [
                    LaunchConfiguration("maps_dir"),
                    "/dock_scan_meta.yaml",
                ],
            },
        ],
    )

    # OpaqueFunction runs at launch-time after all DeclareLaunchArgument
    # are resolved; this is the right hook to stage the test PCD.
    stage_pcd = OpaqueFunction(function=_stage_pre_baked_pcd)

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            use_rviz_arg,
            dock_pose_x_arg,
            dock_pose_y_arg,
            dock_pose_yaw_arg,
            maps_dir_arg,
            stage_pcd,
            simulation_launch,
            synthetic_publisher,
            dock_scan_match,
        ]
    )
