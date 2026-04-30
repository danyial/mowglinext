#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
synthetic_scan_kicp_publisher.py — sim-only stand-in for the real LD19
LiDAR's `/scan_kicp` stream.

Plan 02-08 D-14 deliverable. Runs ONLY inside the simulation stack. In
production (Pi5 ARM64 deploy) this node MUST NOT be launched: the real
`kinematic_icp_scan_frame_relay` already publishes `/scan_kicp` from the
LD19, and a second publisher on the same topic would interleave at the
DDS layer and break ICP.

Pipeline contract (mirrors `kinematic_icp_scan_frame_relay`):
    Topic         : `/scan_kicp`           (configurable via `output_topic`)
    Type          : `sensor_msgs/LaserScan`
    QoS           : SensorDataQoS — best-effort, volatile, depth=10
    header.frame_id: `lidar_link_wheels`   (the parallel-tree sensor frame
                     mirrored by `kinematic_icp_scan_frame_relay`; this is
                     what `dock_scan_match` and the kinematic_icp pipeline
                     subscribe under in production. Keeping the SAME frame
                     here means consumers see no behaviour difference
                     between the real and synthetic scan source.)

Geometry source:
    A V-funnel approximating the YF500 dock structure is hard-coded in
    `_dock_segments()`. The funnel is parametrised by the dock pose
    (`dock_pose_x`, `dock_pose_y`, `dock_pose_yaw_rad` ROS params): one
    base segment 0.40 m wide and two arm segments 0.60 m long flaring at
    ±20° from forward. Optional additional polygons describing world
    obstacles can be loaded from `world_obstacles_yaml` (a flat key=value
    file: each line `obstacle_<i>: x1,y1;x2,y2;...`).

Tick:
    On each timer firing the node looks up the robot pose via
    TF (`map -> base_footprint_wheels` if available, else `map ->
    base_footprint`), ray-casts 360 evenly-spaced bearings over
    `[angle_min, angle_max]` against the dock + obstacle segments,
    applies Gaussian noise (σ=0.005 m, LD19-class), and publishes the
    `LaserScan`.

Consumers (sim path):
    - `dock_scan_capture` (Plan 02-03): captures a baseline PCD, but in
      sim we ship a pre-baked PCD at `test_data/sim_dock_scan.pcd` so
      `dock_scan_match` starts in non-degraded mode without a calibration
      drive.
    - `dock_scan_match` (Plan 02-04): consumes `/scan_kicp` for the live
      ICP step that produces `/dock_match/pose` + `/dock_match/confidence`.
    - `kinematic_icp_pipeline_node`: would consume too if you launched
      it in sim, but we omit it from `sim_lidar_docking.launch.py` since
      we just want to exercise the dock-match path.
"""

from __future__ import annotations

import argparse
import math
import os
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


# ---------------------------------------------------------------------------
# Scanner constants — matched to LD19 datasheet so dock_scan_match's
# angular & range gating behaves identically to a real Pi5 deployment.
# Source: docs/sensor/lidar/LD19.md (LD19 / LDLIDAR_LD19 product spec).
# ---------------------------------------------------------------------------

LD19_ANGLE_MIN_RAD: float = -math.pi             # full 360°
LD19_ANGLE_MAX_RAD: float = math.pi
LD19_NUM_RAYS: int = 360                          # 1° resolution; matches the
                                                  # downsampled scan that the
                                                  # ros_gz_bridge emits in sim.
LD19_RANGE_MIN_M: float = 0.02                    # LD19 minimum range
LD19_RANGE_MAX_M: float = 12.0                    # LD19 maximum range
LD19_RANGE_NOISE_SIGMA_M: float = 0.005           # ~5 mm 1σ noise floor
LD19_PUBLISH_RATE_HZ: float = 10.0                # full revolution at 10 Hz


# ---------------------------------------------------------------------------
# Dock V-funnel geometry — a 3-segment polyline that approximates the
# YF500 dock's funnel arms. Numbers chosen to match the real geometry
# the operator measures with calipers; tweak in one place if the
# YF500-clone shape changes.
# ---------------------------------------------------------------------------

DOCK_BASE_HALF_WIDTH_M: float = 0.20              # 0.40 m base across
DOCK_ARM_LENGTH_M: float = 0.60                   # arms 60 cm long
DOCK_ARM_FLARE_RAD: float = math.radians(20.0)    # ±20° from forward


@dataclass(frozen=True)
class Segment2D:
    """An immutable line segment (a, b) in the map frame."""

    ax: float
    ay: float
    bx: float
    by: float


# ---------------------------------------------------------------------------
# Ray / segment intersection — closed-form, parametric form.
# Used per-tick per-ray in the inner loop; kept dependency-free so the
# smoke pytest can import this module without ROS2 active.
# ---------------------------------------------------------------------------


def ray_segment_intersection(
    rx: float,
    ry: float,
    rdx: float,
    rdy: float,
    seg: Segment2D,
) -> Optional[float]:
    """Return ray-parameter `t` (== range, since (rdx, rdy) is unit-length)
    at the first intersection of ray (rx, ry, rdx, rdy) with segment seg,
    or None if no intersection.

    Solves the 2x2 system:
        rx + t * rdx = ax + s * (bx - ax)
        ry + t * rdy = ay + s * (by - ay)
    for (t >= 0, 0 <= s <= 1). Numerically robust under near-parallel
    edge cases via a small denominator epsilon.
    """
    sx = seg.bx - seg.ax
    sy = seg.by - seg.ay
    denom = rdx * (-sy) - rdy * (-sx)
    if abs(denom) < 1e-12:
        return None  # parallel
    dx = seg.ax - rx
    dy = seg.ay - ry
    t = (dx * (-sy) - dy * (-sx)) / denom
    s = (rdx * dy - rdy * dx) / denom
    if t < 0.0:
        return None
    if s < 0.0 or s > 1.0:
        return None
    return t


# ---------------------------------------------------------------------------
# Dock V-funnel construction. Returned in the MAP frame; ray casting then
# transforms ray origin into the same frame so we don't have to apply
# any inverse rotation per tick.
# ---------------------------------------------------------------------------


def build_dock_segments(
    dock_x: float, dock_y: float, dock_yaw: float
) -> List[Segment2D]:
    """Return the 3 segments that make up the YF500 V-funnel:
    a base bar across the dock front + two arms flaring forward.

    Segments are placed in the MAP frame given the dock anchor (its
    `base_footprint`-equivalent at the rim of the funnel) and yaw."""
    cos_y = math.cos(dock_yaw)
    sin_y = math.sin(dock_yaw)

    def to_map(local_x: float, local_y: float) -> Tuple[float, float]:
        return (
            dock_x + local_x * cos_y - local_y * sin_y,
            dock_y + local_x * sin_y + local_y * cos_y,
        )

    # Base: from (-half, 0) to (+half, 0) in dock-local frame. The dock's
    # local +x points away from the robot (into the dock) so the base sits
    # at local x=0; arms extend forward along +x.
    base_a = to_map(0.0, -DOCK_BASE_HALF_WIDTH_M)
    base_b = to_map(0.0, +DOCK_BASE_HALF_WIDTH_M)

    # Left arm: from base-b along +flare from forward axis.
    left_arm_end = to_map(
        DOCK_ARM_LENGTH_M * math.cos(DOCK_ARM_FLARE_RAD),
        +DOCK_BASE_HALF_WIDTH_M
        + DOCK_ARM_LENGTH_M * math.sin(DOCK_ARM_FLARE_RAD),
    )

    # Right arm: from base-a along -flare from forward axis.
    right_arm_end = to_map(
        DOCK_ARM_LENGTH_M * math.cos(DOCK_ARM_FLARE_RAD),
        -DOCK_BASE_HALF_WIDTH_M
        - DOCK_ARM_LENGTH_M * math.sin(DOCK_ARM_FLARE_RAD),
    )

    return [
        Segment2D(base_a[0], base_a[1], base_b[0], base_b[1]),
        Segment2D(base_b[0], base_b[1], left_arm_end[0], left_arm_end[1]),
        Segment2D(base_a[0], base_a[1], right_arm_end[0], right_arm_end[1]),
    ]


def parse_world_obstacles_yaml(path: str) -> List[Segment2D]:
    """Load obstacle polygons from a flat `key: x1,y1;x2,y2;...` file
    (mowgli_geometry key=value style — see `dock_scan_capture.py` /
    `mowgli_geometry::key_value_parser` for the canonical loader). Each
    polygon is closed (last vertex connects to first). Returns the
    flattened segment list. Returns [] on missing file or parse failure;
    the publisher continues with dock-only geometry, so the test stays
    green even without the optional file."""
    if not path or not os.path.isfile(path):
        return []
    segments: List[Segment2D] = []
    try:
        with open(path) as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" not in line:
                    continue
                _, vertex_str = line.split(":", 1)
                vertex_pairs = [v.strip() for v in vertex_str.strip().split(";") if v.strip()]
                pts: List[Tuple[float, float]] = []
                for pair in vertex_pairs:
                    parts = pair.split(",")
                    if len(parts) != 2:
                        continue
                    pts.append((float(parts[0]), float(parts[1])))
                if len(pts) >= 2:
                    for i in range(len(pts)):
                        a = pts[i]
                        b = pts[(i + 1) % len(pts)]
                        segments.append(Segment2D(a[0], a[1], b[0], b[1]))
    except (OSError, ValueError):
        return []
    return segments


# ---------------------------------------------------------------------------
# Pure-Python scan synthesis — exposed at module scope so the smoke
# pytest can call it without spinning the ROS2 node.
# ---------------------------------------------------------------------------


def synthesize_scan_ranges(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    segments: List[Segment2D],
    *,
    angle_min: float = LD19_ANGLE_MIN_RAD,
    angle_max: float = LD19_ANGLE_MAX_RAD,
    num_rays: int = LD19_NUM_RAYS,
    range_min: float = LD19_RANGE_MIN_M,
    range_max: float = LD19_RANGE_MAX_M,
    noise_sigma: float = LD19_RANGE_NOISE_SIGMA_M,
    rng: Optional[random.Random] = None,
) -> List[float]:
    """Ray-cast the world for one full LaserScan revolution. Misses
    return float('inf') (LD19 / sim convention; downstream code already
    filters with `math.isfinite`)."""
    if rng is None:
        rng = random.Random(0)
    angle_step = (angle_max - angle_min) / max(num_rays - 1, 1)
    ranges: List[float] = []
    cos_robot = math.cos(robot_yaw)
    sin_robot = math.sin(robot_yaw)
    for i in range(num_rays):
        # Bearing in robot frame -> direction in map frame.
        bearing = angle_min + i * angle_step
        local_dx = math.cos(bearing)
        local_dy = math.sin(bearing)
        rdx = cos_robot * local_dx - sin_robot * local_dy
        rdy = sin_robot * local_dx + cos_robot * local_dy
        best: Optional[float] = None
        for seg in segments:
            t = ray_segment_intersection(robot_x, robot_y, rdx, rdy, seg)
            if t is None:
                continue
            if t < range_min or t > range_max:
                continue
            if best is None or t < best:
                best = t
        if best is None:
            ranges.append(float("inf"))
        else:
            r = best + rng.gauss(0.0, noise_sigma)
            if r < range_min:
                r = range_min
            elif r > range_max:
                r = range_max
            ranges.append(r)
    return ranges


# ---------------------------------------------------------------------------
# rclpy node
# ---------------------------------------------------------------------------


class SyntheticScanKicpPublisher(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_scan_kicp_publisher")

        # ROS2 params with defaults aligned to small_garden.sdf dock placement.
        self.declare_parameter("output_topic", "/scan_kicp")
        self.declare_parameter("output_frame", "lidar_link_wheels")
        self.declare_parameter("robot_pose_lookup_target", "base_footprint_wheels")
        self.declare_parameter("robot_pose_lookup_source", "map")
        self.declare_parameter("publish_rate_hz", LD19_PUBLISH_RATE_HZ)
        self.declare_parameter("dock_pose_x", 5.0)
        self.declare_parameter("dock_pose_y", 5.0)
        self.declare_parameter("dock_pose_yaw_rad", 0.0)
        self.declare_parameter("world_obstacles_yaml", "")
        self.declare_parameter("noise_sigma_m", LD19_RANGE_NOISE_SIGMA_M)
        # Static fallback robot pose used when TF lookup fails (e.g. in CI
        # before Gazebo's clock has fully started). Lets the node emit
        # plausible scans immediately so dock_scan_match doesn't sit
        # waiting on the publisher.
        self.declare_parameter("fallback_robot_x", 3.5)
        self.declare_parameter("fallback_robot_y", 5.0)
        self.declare_parameter("fallback_robot_yaw_rad", 0.0)

        self._output_topic: str = self.get_parameter("output_topic").value
        self._output_frame: str = self.get_parameter("output_frame").value
        self._tf_target: str = self.get_parameter("robot_pose_lookup_target").value
        self._tf_source: str = self.get_parameter("robot_pose_lookup_source").value
        rate_hz: float = float(self.get_parameter("publish_rate_hz").value)
        self._dock_x: float = float(self.get_parameter("dock_pose_x").value)
        self._dock_y: float = float(self.get_parameter("dock_pose_y").value)
        self._dock_yaw: float = float(self.get_parameter("dock_pose_yaw_rad").value)
        self._noise_sigma: float = float(self.get_parameter("noise_sigma_m").value)
        self._fallback_x: float = float(self.get_parameter("fallback_robot_x").value)
        self._fallback_y: float = float(self.get_parameter("fallback_robot_y").value)
        self._fallback_yaw: float = float(self.get_parameter("fallback_robot_yaw_rad").value)

        # Build geometry once at startup. Reload via service is not needed —
        # in sim the dock pose doesn't move during a run.
        self._segments: List[Segment2D] = build_dock_segments(
            self._dock_x, self._dock_y, self._dock_yaw
        )
        obstacles_path: str = self.get_parameter("world_obstacles_yaml").value
        extra = parse_world_obstacles_yaml(obstacles_path)
        if extra:
            self.get_logger().info(
                f"Loaded {len(extra)} obstacle segments from {obstacles_path}"
            )
            self._segments.extend(extra)

        # SensorDataQoS — same as kinematic_icp_scan_frame_relay so
        # subscribers behave identically.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, self._output_topic, sensor_qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._rng = random.Random(42)

        period = 1.0 / max(rate_hz, 0.1)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            "synthetic_scan_kicp_publisher ready:\n"
            f"  topic           : {self._output_topic}\n"
            f"  frame_id        : {self._output_frame}\n"
            f"  rate            : {rate_hz:.1f} Hz\n"
            f"  dock @          : ({self._dock_x:.2f}, {self._dock_y:.2f},"
            f" yaw {math.degrees(self._dock_yaw):.1f} deg)\n"
            f"  segments        : {len(self._segments)} "
            "(dock funnel + world obstacles)"
        )

    # ------------------------------------------------------------------

    def _lookup_robot_pose(self) -> Tuple[float, float, float]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._tf_source, self._tf_target, rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            return x, y, yaw
        except TransformException:
            return self._fallback_x, self._fallback_y, self._fallback_yaw

    def _tick(self) -> None:
        rx, ry, ryaw = self._lookup_robot_pose()
        ranges = synthesize_scan_ranges(
            rx,
            ry,
            ryaw,
            self._segments,
            noise_sigma=self._noise_sigma,
            rng=self._rng,
        )
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._output_frame
        msg.angle_min = LD19_ANGLE_MIN_RAD
        msg.angle_max = LD19_ANGLE_MAX_RAD
        msg.angle_increment = (LD19_ANGLE_MAX_RAD - LD19_ANGLE_MIN_RAD) / max(
            LD19_NUM_RAYS - 1, 1
        )
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / LD19_PUBLISH_RATE_HZ
        msg.range_min = LD19_RANGE_MIN_M
        msg.range_max = LD19_RANGE_MAX_M
        msg.ranges = [float(r) for r in ranges]
        msg.intensities = []
        self._pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(argv: Optional[List[str]] = None) -> None:
    # argparse here only so users can `python3 synthetic_scan_kicp_publisher.py
    # --help` and see the ROS2 params; rclpy.init reads ROS args directly.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_known_args(argv)
    rclpy.init(args=argv)
    node = SyntheticScanKicpPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
