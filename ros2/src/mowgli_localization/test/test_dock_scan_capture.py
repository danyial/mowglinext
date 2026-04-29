#!/usr/bin/env python3
"""
test_dock_scan_capture - pytest harness for dock_scan_capture library.

Covers SPEC R-1 / Pitfall 2 / Pitfall 7 from
.planning/phases/02-lidar-dock-pose-estimation/02-RESEARCH.md:
- 5 LaserScan frames are voxel-merged into one PCD.
- Output PCD is PCL ASCII v0.7 (D-06) - byte-for-byte the same header
  mowgli_lidar_docking::save_dock_scan_pcd writes.
- Output meta is flat key=value (D-17) - readable by
  mowgli_lidar_docking::load_dock_scan_meta_yaml.
- Capture is gated on (is_charging_gate AND |wheel_vx| < 0.02).
- Capture returns False on /scan_kicp timeout; no files written.
- Atomic write: no .tmp_* orphan after success.
- All 10 D-17 keys present in meta with the right format.

The tests stand up a tiny rclpy node + a synthetic /scan_kicp publisher
in the same process, drive the capture function via
rclpy.spin_once, then assert the written files. rclpy is required at
runtime; on hosts without it, the whole module is skipped.
"""

import math
import os
import re
import shutil
import tempfile
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
LaserScan = pytest.importorskip("sensor_msgs.msg").LaserScan
qos_profile_sensor_data = pytest.importorskip(
    "rclpy.qos"
).qos_profile_sensor_data

# dock_scan_capture lives next to calibrate_imu_yaw_node in scripts/.
# colcon installs both into lib/mowgli_localization/, so during
# colcon-test the package directory is on PYTHONPATH and a flat import
# works. For a host pytest run, the conftest.py in this directory
# prepends scripts/ to sys.path.
import dock_scan_capture  # noqa: E402  (after importorskip block)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _build_synthetic_scan(
    n_rays: int = 360,
    constant_range: float = 1.0,
    range_min: float = 0.05,
    range_max: float = 8.0,
) -> LaserScan:
    scan = LaserScan()
    scan.header.frame_id = "lidar_link_wheels"
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2.0 * math.pi) / n_rays
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = range_min
    scan.range_max = range_max
    scan.ranges = [constant_range] * n_rays
    scan.intensities = []
    return scan


def _parse_pcd_ascii(path: str):
    with open(path, "r") as fh:
        lines = fh.read().splitlines()
    width = None
    points = []
    in_data = False
    for ln in lines:
        if ln.startswith("WIDTH "):
            width = int(ln.split()[1])
        if in_data:
            parts = ln.split()
            if len(parts) >= 3:
                points.append(tuple(float(p) for p in parts[:3]))
        if ln.strip() == "DATA ascii":
            in_data = True
    return width, points


# ---------------------------------------------------------------------------
# Fixture
# ---------------------------------------------------------------------------


@pytest.fixture
def rclpy_runtime():
    rclpy.init(args=None)
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass


@pytest.fixture
def tmp_maps(tmp_path):
    pcd = str(tmp_path / "dock_scan.pcd")
    meta = str(tmp_path / "dock_scan_meta.yaml")
    yield pcd, meta


@pytest.fixture
def synthetic_publisher(rclpy_runtime):
    """Spawn a thread that publishes synthetic /scan_kicp at 20 Hz.

    Tests trigger this fixture only when they want frames to arrive.
    Yield a stop callable; the fixture tears the publisher down on exit.
    """
    pub_node = rclpy.create_node("synthetic_scan_kicp_publisher_test")
    pub = pub_node.create_publisher(
        LaserScan, "/scan_kicp", qos_profile_sensor_data
    )
    stop_evt = threading.Event()

    def _spin():
        while not stop_evt.is_set() and rclpy.ok():
            scan = _build_synthetic_scan()
            scan.header.stamp = pub_node.get_clock().now().to_msg()
            try:
                pub.publish(scan)
            except Exception:
                break
            time.sleep(0.05)

    t = threading.Thread(target=_spin, daemon=True)
    t.start()
    yield
    stop_evt.set()
    t.join(timeout=1.0)
    pub_node.destroy_node()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_capture_writes_pcd_and_meta(rclpy_runtime, tmp_maps, synthetic_publisher):
    pcd_path, meta_path = tmp_maps
    cap_node = rclpy.create_node("capture_test_node_writes")

    ok = dock_scan_capture.capture_and_save(
        cap_node,
        dock_pose_x=1.234567,
        dock_pose_y=-2.345678,
        dock_pose_yaw_rad=0.5,
        is_charging_gate=True,
        wheel_vx=0.0,
        scan_kicp_topic="/scan_kicp",
        pcd_path=pcd_path,
        meta_path=meta_path,
        n_frames=5,
        voxel_size=0.05,
        capture_timeout_s=5.0,
        fix_type="RTK_FIXED",
        sensor_extrinsic_xyz_rad=(0.10, 0.0, 0.0),
    )

    cap_node.destroy_node()

    assert ok is True
    assert os.path.exists(pcd_path)
    assert os.path.exists(meta_path)

    width, points = _parse_pcd_ascii(pcd_path)
    assert width is not None and width >= 1
    assert len(points) == width

    with open(meta_path, "r") as fh:
        meta = fh.read()
    # Byte-exact line for the C++ loader contract (D-17).
    assert "dock_pose_x: 1.234567\n" in meta
    assert "dock_pose_y: -2.345678\n" in meta
    assert f"dock_scan_pcd_path: {pcd_path}\n" in meta
    assert "fix_type: RTK_FIXED\n" in meta


def test_capture_returns_false_on_timeout(rclpy_runtime, tmp_maps):
    pcd_path, meta_path = tmp_maps
    cap_node = rclpy.create_node("capture_test_node_timeout")

    t0 = time.monotonic()
    ok = dock_scan_capture.capture_and_save(
        cap_node,
        dock_pose_x=0.0,
        dock_pose_y=0.0,
        dock_pose_yaw_rad=0.0,
        is_charging_gate=True,
        wheel_vx=0.0,
        scan_kicp_topic="/scan_kicp_does_not_exist",
        pcd_path=pcd_path,
        meta_path=meta_path,
        n_frames=5,
        capture_timeout_s=1.0,
    )
    elapsed = time.monotonic() - t0

    cap_node.destroy_node()

    assert ok is False
    # 1 s budget + slack; must NOT exceed an absurd amount.
    assert elapsed < 5.0
    assert not os.path.exists(pcd_path)
    assert not os.path.exists(meta_path)


def test_capture_voxel_merge_reduces_points(
    rclpy_runtime, tmp_maps, synthetic_publisher
):
    """5 identical scans of N_RAYS rays each should NOT produce 5*N_RAYS
    points - voxel merge collapses identical bins."""
    pcd_path, meta_path = tmp_maps
    cap_node = rclpy.create_node("capture_test_node_voxel")

    ok = dock_scan_capture.capture_and_save(
        cap_node,
        dock_pose_x=0.0,
        dock_pose_y=0.0,
        dock_pose_yaw_rad=0.0,
        is_charging_gate=True,
        wheel_vx=0.0,
        scan_kicp_topic="/scan_kicp",
        pcd_path=pcd_path,
        meta_path=meta_path,
        n_frames=5,
        voxel_size=0.05,
        capture_timeout_s=5.0,
    )
    cap_node.destroy_node()

    assert ok is True
    width, _ = _parse_pcd_ascii(pcd_path)
    # 5 frames * 360 rays = 1800 raw; merged should be << 1800.
    assert width < 1800, f"voxel merge did not reduce points: {width} >= 1800"


def test_capture_atomicity_no_orphan_tmpfile(
    rclpy_runtime, tmp_maps, synthetic_publisher
):
    pcd_path, meta_path = tmp_maps
    cap_node = rclpy.create_node("capture_test_node_atomic")

    ok = dock_scan_capture.capture_and_save(
        cap_node,
        dock_pose_x=0.0,
        dock_pose_y=0.0,
        dock_pose_yaw_rad=0.0,
        is_charging_gate=True,
        wheel_vx=0.0,
        scan_kicp_topic="/scan_kicp",
        pcd_path=pcd_path,
        meta_path=meta_path,
        n_frames=3,
        capture_timeout_s=5.0,
    )
    cap_node.destroy_node()

    assert ok is True
    parent = os.path.dirname(pcd_path)
    orphans = [n for n in os.listdir(parent) if n.startswith(".tmp_")]
    assert orphans == [], f"orphan tempfiles after success: {orphans}"


def test_meta_keys_all_present(rclpy_runtime, tmp_maps, synthetic_publisher):
    """All 10 D-17 keys must appear exactly once with valid formats."""
    pcd_path, meta_path = tmp_maps
    cap_node = rclpy.create_node("capture_test_node_meta_keys")

    ok = dock_scan_capture.capture_and_save(
        cap_node,
        dock_pose_x=0.0,
        dock_pose_y=0.0,
        dock_pose_yaw_rad=0.0,
        is_charging_gate=True,
        wheel_vx=0.0,
        scan_kicp_topic="/scan_kicp",
        pcd_path=pcd_path,
        meta_path=meta_path,
        n_frames=3,
        capture_timeout_s=5.0,
    )
    cap_node.destroy_node()

    assert ok is True
    with open(meta_path, "r") as fh:
        meta = fh.read()

    required_float_keys = [
        "dock_pose_x",
        "dock_pose_y",
        "dock_pose_yaw_rad",
        "sensor_extrinsic_x",
        "sensor_extrinsic_y",
        "sensor_extrinsic_yaw_rad",
    ]
    for k in required_float_keys:
        m = re.search(rf"^{k}: (-?\d+\.\d{{6}})$", meta, re.MULTILINE)
        assert m, f"key {k} missing or wrong format in meta:\n{meta}"

    # int point_count
    m = re.search(r"^point_count: (\d+)$", meta, re.MULTILINE)
    assert m, "point_count missing or non-int"
    assert int(m.group(1)) >= 1

    # ISO-8601 captured_at
    m = re.search(
        r"^captured_at: (\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z)$",
        meta,
        re.MULTILINE,
    )
    assert m, "captured_at missing or wrong format"

    # string fix_type and dock_scan_pcd_path - present (and not empty)
    assert re.search(r"^fix_type: \S+$", meta, re.MULTILINE)
    assert re.search(r"^dock_scan_pcd_path: \S+$", meta, re.MULTILINE)


def test_capture_refuses_when_not_charging(rclpy_runtime, tmp_maps):
    pcd_path, meta_path = tmp_maps
    cap_node = rclpy.create_node("capture_test_node_gate_charge")

    ok = dock_scan_capture.capture_and_save(
        cap_node,
        dock_pose_x=0.0,
        dock_pose_y=0.0,
        dock_pose_yaw_rad=0.0,
        is_charging_gate=False,  # <-- gate trips
        wheel_vx=0.0,
        pcd_path=pcd_path,
        meta_path=meta_path,
    )

    cap_node.destroy_node()
    assert ok is False
    assert not os.path.exists(pcd_path)
    assert not os.path.exists(meta_path)


def test_capture_refuses_when_moving(rclpy_runtime, tmp_maps):
    pcd_path, meta_path = tmp_maps
    cap_node = rclpy.create_node("capture_test_node_gate_vx")

    ok = dock_scan_capture.capture_and_save(
        cap_node,
        dock_pose_x=0.0,
        dock_pose_y=0.0,
        dock_pose_yaw_rad=0.0,
        is_charging_gate=True,
        wheel_vx=0.05,  # <-- 5 cm/s > 2 cm/s gate
        pcd_path=pcd_path,
        meta_path=meta_path,
    )

    cap_node.destroy_node()
    assert ok is False
    assert not os.path.exists(pcd_path)
    assert not os.path.exists(meta_path)
