#!/usr/bin/env python3
"""
dock_scan_capture - one-shot LiDAR snapshot to /ros2_ws/maps/dock_scan.pcd
                    + dock_scan_meta.yaml.

Called by calibrate_imu_yaw_node after a successful dock-yaw calibration so
the operator's existing GUI calibration button also produces the dock-scan
the dock_scan_match node (mowgli_lidar_docking, Plan 02-04) needs at
startup. SPEC R-1.

Format contracts (frozen by Plan 02-02):
- .pcd  PCL ASCII v0.7 (D-06).  Mirrors mowgli_lidar_docking::save_dock_scan_pcd
        byte-for-byte; PCL's loadPCDFile<pcl::PointXYZ> parses what this
        writes.
- .yaml flat key=value (D-17).   Read by
        mowgli_lidar_docking::load_dock_scan_meta_yaml via the
        line-anchored mowgli_geometry::key_value_parser.

Library, not a node. The function returns; the caller continues its flow.
No TF / pose / topic publish. Only one short-lived subscription.

Architecture invariants honoured (see CLAUDE.md):
- AI #1: capture reads /scan_kicp and (optionally) the parallel TF tree
  `base_footprint_wheels -> lidar_link_wheels`. It never publishes TF, pose,
  or any topic. The fused robot_localization state is not consulted.
- "Do NOT" list: never feeds anything back into robot_localization. The
  produced .pcd is consumed by Plan 02-04's matcher which publishes a
  PoseWithCovarianceStamped on /dock_match/pose, not by the EKF.
"""

import math
import os
import tempfile
from datetime import datetime, timezone

import rclpy
import rclpy.duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# LaserScan -> XY conversion
# ---------------------------------------------------------------------------


def _laserscan_to_xy(scan: LaserScan):
    """Convert LaserScan ranges to list[(x, y)] in the scan frame.

    Skips NaN / Inf ranges and ranges outside [range_min, range_max] (LD19
    emits sentinel values on no-return rays). Mirrors the math
    laser_geometry::LaserProjection::projectLaser does on the C++ side.
    """
    out = []
    angle = float(scan.angle_min)
    inc = float(scan.angle_increment)
    rmin = float(scan.range_min)
    rmax = float(scan.range_max)
    for r in scan.ranges:
        rf = float(r)
        if math.isfinite(rf) and rmin <= rf <= rmax:
            out.append((rf * math.cos(angle), rf * math.sin(angle)))
        angle += inc
    return out


# ---------------------------------------------------------------------------
# Voxel merge (Pitfall 2: 5-frame averaging cancels wheel-position noise)
# ---------------------------------------------------------------------------


def _voxel_merge(points, voxel_size: float):
    """Group points into a 2D voxel grid; return one centroid per voxel.

    Same idea as kiss_icp::VoxelHashMap (RESEARCH §Pitfall 2): bin
    coordinates by floor(x / voxel_size), keep the centroid as the
    representative. Implemented in pure Python for the rclpy library
    so we don't add a numpy build-time dep on the test path.
    """
    if voxel_size <= 0.0:
        return list(points)
    buckets = {}
    for x, y in points:
        kx = int(round(x / voxel_size))
        ky = int(round(y / voxel_size))
        b = buckets.get((kx, ky))
        if b is None:
            buckets[(kx, ky)] = [x, y, 1]
        else:
            b[0] += x
            b[1] += y
            b[2] += 1
    out = []
    for (sx, sy, n) in buckets.values():
        out.append((sx / n, sy / n))
    return out


# ---------------------------------------------------------------------------
# Atomic write (T-03-04 mitigation: tempfile + os.rename + os.fsync)
# ---------------------------------------------------------------------------


def _atomic_write(path: str, content: str) -> bool:
    """tempfile + fsync + os.rename atomic write. Returns False on error.

    POSIX guarantees os.rename is atomic on the same filesystem, so a
    concurrent reader (Plan 02-04 dock_scan_match) sees either the old
    bytes or the new bytes - never partial.
    """
    parent = os.path.dirname(path) or "."
    tmp_path = None
    try:
        os.makedirs(parent, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            mode="w",
            dir=parent,
            delete=False,
            prefix=".tmp_",
            suffix=os.path.basename(path),
        ) as fh:
            fh.write(content)
            fh.flush()
            os.fsync(fh.fileno())
            tmp_path = fh.name
        os.rename(tmp_path, path)
        return True
    except OSError:
        if tmp_path is not None:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
        return False


# ---------------------------------------------------------------------------
# PCL ASCII v0.7 PCD renderer (D-06)
# ---------------------------------------------------------------------------


def _format_pcd_ascii(points) -> str:
    """Render PCL ASCII PCD v0.7. Matches mowgli_lidar_docking::save_dock_scan_pcd
    byte-for-byte: same header lines in the same order, same float format
    (6 decimals, space-separated, one point per line, z always 0.0)."""
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )
    body_lines = [f"{x:.6f} {y:.6f} 0.000000\n" for (x, y) in points]
    return header + "".join(body_lines)


# ---------------------------------------------------------------------------
# D-17 dock_scan_meta.yaml renderer
# ---------------------------------------------------------------------------


def _format_dock_scan_meta(
    pcd_path: str,
    dock_pose_x: float,
    dock_pose_y: float,
    dock_pose_yaw_rad: float,
    sensor_x: float,
    sensor_y: float,
    sensor_yaw_rad: float,
    point_count: int,
    captured_at: str,
    fix_type: str,
) -> str:
    """D-17 schema; flat key=value, line-anchored, C++ loader-compatible.

    The mowgli_geometry::key_value_parser anchors on `\\n + key:` so all
    keys MUST be at column 0. No nested 'dock_scan_meta:' wrapper, no
    indentation, no yaml.safe_dump (which would re-indent and break the
    parser).
    """
    return (
        f"dock_scan_pcd_path: {pcd_path}\n"
        f"dock_pose_x: {dock_pose_x:.6f}\n"
        f"dock_pose_y: {dock_pose_y:.6f}\n"
        f"dock_pose_yaw_rad: {dock_pose_yaw_rad:.6f}\n"
        f"sensor_extrinsic_x: {sensor_x:.6f}\n"
        f"sensor_extrinsic_y: {sensor_y:.6f}\n"
        f"sensor_extrinsic_yaw_rad: {sensor_yaw_rad:.6f}\n"
        f"point_count: {point_count}\n"
        f"captured_at: {captured_at}\n"
        f"fix_type: {fix_type}\n"
    )


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def capture_and_save(
    node,
    dock_pose_x: float,
    dock_pose_y: float,
    dock_pose_yaw_rad: float,
    is_charging_gate: bool,
    wheel_vx: float,
    *,
    scan_kicp_topic: str = "/scan_kicp",
    pcd_path: str = "/ros2_ws/maps/dock_scan.pcd",
    meta_path: str = "/ros2_ws/maps/dock_scan_meta.yaml",
    n_frames: int = 5,
    voxel_size: float = 0.05,
    capture_timeout_s: float = 10.0,
    fix_type: str = "UNKNOWN",
    sensor_extrinsic_xyz_rad=(0.0, 0.0, 0.0),
    logger=None,
) -> bool:
    """Capture N LaserScan frames from `scan_kicp_topic`, voxel-merge,
    and persist as a PCL ASCII PCD + flat key=value YAML meta.

    Returns True on success, False on any failure (gate denied, timeout,
    write error). The caller (typically calibrate_imu_yaw_node) is
    responsible for surfacing the result; this function only logs.

    Gating per RESEARCH Pitfall 2:
      - is_charging_gate AND |wheel_vx| < 0.02 m/s.

    The function does NOT subscribe to /status or /wheel_odom directly so
    it stays unit-testable without standing up the safety topics.
    """
    log = logger or node.get_logger()

    # -- gating per RESEARCH Pitfall 2 --
    if not is_charging_gate:
        log.warn(
            "dock_scan_capture: is_charging_gate is False (robot not on dock); "
            "refusing capture"
        )
        return False
    if abs(float(wheel_vx)) >= 0.02:
        log.warn(
            f"dock_scan_capture: |wheel_vx|={abs(float(wheel_vx)):.3f} m/s "
            ">= 0.02; refusing capture (robot not stationary)"
        )
        return False

    # -- collect up to n_frames LaserScan messages --
    frames: list[LaserScan] = []

    def _on_scan(msg: LaserScan):
        if len(frames) < n_frames:
            frames.append(msg)

    sub = node.create_subscription(
        LaserScan, scan_kicp_topic, _on_scan, qos_profile_sensor_data
    )

    deadline = node.get_clock().now() + rclpy.duration.Duration(
        seconds=float(capture_timeout_s)
    )
    while (
        rclpy.ok()
        and len(frames) < n_frames
        and node.get_clock().now() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        node.destroy_subscription(sub)
    except Exception:
        pass

    if not frames:
        log.warn(
            f"dock_scan_capture: no {scan_kicp_topic} messages within "
            f"{capture_timeout_s:.1f}s; aborting (no files written)"
        )
        return False

    log.info(
        f"dock_scan_capture: collected {len(frames)} {scan_kicp_topic} "
        f"frame(s); voxel-merging at {voxel_size:.3f} m"
    )

    # -- merge --
    all_pts = []
    for s in frames:
        all_pts.extend(_laserscan_to_xy(s))
    merged = _voxel_merge(all_pts, voxel_size)
    if not merged:
        log.warn(
            "dock_scan_capture: no valid points after voxel merge; aborting"
        )
        return False

    # -- write PCD atomically --
    pcd_content = _format_pcd_ascii(merged)
    if not _atomic_write(pcd_path, pcd_content):
        log.error(f"dock_scan_capture: failed to write PCD to {pcd_path}")
        return False

    # -- write meta atomically --
    sx, sy, syaw = sensor_extrinsic_xyz_rad
    captured_at = datetime.now(tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    meta_content = _format_dock_scan_meta(
        pcd_path=pcd_path,
        dock_pose_x=float(dock_pose_x),
        dock_pose_y=float(dock_pose_y),
        dock_pose_yaw_rad=float(dock_pose_yaw_rad),
        sensor_x=float(sx),
        sensor_y=float(sy),
        sensor_yaw_rad=float(syaw),
        point_count=len(merged),
        captured_at=captured_at,
        fix_type=str(fix_type),
    )
    if not _atomic_write(meta_path, meta_content):
        log.error(f"dock_scan_capture: failed to write meta to {meta_path}")
        # Best-effort cleanup of orphan PCD: leaving a PCD with no meta
        # would let Plan 02-04's matcher load stale geometry. Better to
        # roll back so the next capture starts from a known-clean state.
        try:
            os.unlink(pcd_path)
        except OSError:
            pass
        return False

    log.info(
        f"dock_scan_capture: wrote {len(merged)} points -> {pcd_path}; "
        f"meta -> {meta_path}; "
        f"dock=({float(dock_pose_x):+.3f}, {float(dock_pose_y):+.3f}, "
        f"{math.degrees(float(dock_pose_yaw_rad)):+.1f} deg); "
        f"fix={fix_type}"
    )
    return True
