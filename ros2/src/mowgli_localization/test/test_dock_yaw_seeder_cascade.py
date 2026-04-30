#!/usr/bin/env python3
"""
test_dock_yaw_seeder_cascade - pytest harness for the 3-source EKF
seeder cascade in dock_yaw_to_set_pose.py.

Covers SPEC R-4 (Phase 2 Plan 02-05):
  Cascade priority is /dock_match/pose (LiDAR, when trusted) >
  dock_calibration.yaml (file) > /gnss/heading (live).

The 8 cases below exercise:

  - 6 truth-table transitions of _resolve_yaw_source()
  - 1 debounce regression on _try_publish() (cascade source must not
    bypass the existing 1 Hz throttle)
  - 1 AUTONOMOUS gate regression (HIGH_LEVEL_STATE_AUTONOMOUS must
    suppress publish even when source==lidar)

Tests stand up a real DockYawToSetPose node and drive
_resolve_yaw_source / _try_publish by attribute assignment plus
mocked publishers. rclpy is required at runtime; on hosts without
it the whole module is skipped via pytest.importorskip.
"""

import math
import time
from unittest import mock

import pytest

rclpy = pytest.importorskip("rclpy")
PoseWithCovarianceStamped = pytest.importorskip(
    "geometry_msgs.msg"
).PoseWithCovarianceStamped
DockMatchConfidence = pytest.importorskip(
    "mowgli_interfaces.msg"
).DockMatchConfidence
Imu = pytest.importorskip("sensor_msgs.msg").Imu

# dock_yaw_to_set_pose lives next to calibrate_imu_yaw_node in scripts/.
# conftest.py prepends scripts/ to sys.path so the flat import works.
import dock_yaw_to_set_pose  # noqa: E402  (after importorskip block)


# ---------------------------------------------------------------------------
# Fixtures
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
def seeder_node(rclpy_runtime):
    """Construct a DockYawToSetPose with mocked publishers so tests can
    invoke _try_publish without a live EKF subscriber."""
    node = dock_yaw_to_set_pose.DockYawToSetPose()
    # Replace publishers with MagicMock to count publish() calls.
    node._pub_map = mock.MagicMock()
    node._pub_odom = mock.MagicMock()
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _yaw_to_quaternion(yaw_rad: float):
    """Build a sensor_msgs/Imu.orientation-style quaternion for a yaw."""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.z = math.sin(yaw_rad / 2.0)
    return q


def _make_lidar_pose(yaw_rad: float, var: float = 0.001):
    """Build a /dock_match/pose message with the given yaw + variance."""
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.orientation = _yaw_to_quaternion(yaw_rad)
    cov = list(msg.pose.covariance)
    cov[35] = var
    msg.pose.covariance = cov
    return msg


def _make_heading_imu(yaw_rad: float):
    msg = Imu()
    msg.header.frame_id = "map"
    msg.orientation = _yaw_to_quaternion(yaw_rad)
    return msg


def _seed_lidar_match(node, yaw_rad, age_s, trusted, var=0.001):
    """Set the /dock_match/* state on the node directly."""
    node._latest_dock_match_pose = _make_lidar_pose(yaw_rad, var=var)
    # Subtract age_s from now() to simulate a stale-or-fresh arrival.
    now = node.get_clock().now()
    from rclpy.duration import Duration
    node._latest_dock_match_pose_at = now - Duration(seconds=age_s)
    node._latest_dock_match_trusted = trusted


# ---------------------------------------------------------------------------
# 6 truth-table cases for _resolve_yaw_source()
# ---------------------------------------------------------------------------


def test_cascade_case_1_trusted_lidar_wins_over_file_and_heading(seeder_node):
    """Case 1: trusted lidar present, file present, heading present -> lidar."""
    _seed_lidar_match(seeder_node, yaw_rad=0.5, age_s=0.1, trusted=True)
    seeder_node._file_yaw_rad = 0.2
    seeder_node._file_yaw_var = 0.03
    seeder_node._latest_heading = _make_heading_imu(0.1)
    source, yaw, var = seeder_node._resolve_yaw_source()
    assert source == "lidar", f"expected lidar, got {source}"
    assert math.isclose(yaw, 0.5, abs_tol=1e-3)
    assert var > 0.0


def test_cascade_case_2_trusted_lidar_wins_when_file_and_heading_absent(seeder_node):
    """Case 2: trusted lidar present, file absent, heading absent -> lidar."""
    _seed_lidar_match(seeder_node, yaw_rad=0.7, age_s=0.1, trusted=True)
    seeder_node._file_yaw_rad = None
    seeder_node._file_yaw_var = None
    seeder_node._latest_heading = None
    source, yaw, var = seeder_node._resolve_yaw_source()
    assert source == "lidar"
    assert math.isclose(yaw, 0.7, abs_tol=1e-3)


def test_cascade_case_3_untrusted_lidar_falls_to_file(seeder_node):
    """Case 3: untrusted lidar, file present, heading present -> file."""
    _seed_lidar_match(seeder_node, yaw_rad=0.5, age_s=0.1, trusted=False)
    seeder_node._file_yaw_rad = 0.25
    seeder_node._file_yaw_var = 0.03
    seeder_node._latest_heading = _make_heading_imu(0.1)
    source, yaw, var = seeder_node._resolve_yaw_source()
    assert source == "file", f"expected file, got {source}"
    assert math.isclose(yaw, 0.25, abs_tol=1e-6)
    assert math.isclose(var, 0.03, abs_tol=1e-6)


def test_cascade_case_4_untrusted_lidar_no_file_falls_to_heading(seeder_node):
    """Case 4: untrusted lidar, no file, heading present -> /gnss/heading."""
    _seed_lidar_match(seeder_node, yaw_rad=0.5, age_s=0.1, trusted=False)
    seeder_node._file_yaw_rad = None
    seeder_node._file_yaw_var = None
    seeder_node._latest_heading = _make_heading_imu(0.42)
    source, yaw, var = seeder_node._resolve_yaw_source()
    assert source == "/gnss/heading", f"expected heading, got {source}"
    assert math.isclose(yaw, 0.42, abs_tol=1e-3)


def test_cascade_case_5_stale_trusted_lidar_falls_to_file(seeder_node):
    """Case 5: trusted lidar 5 s old (stale) -> file wins."""
    _seed_lidar_match(seeder_node, yaw_rad=0.9, age_s=5.0, trusted=True)
    seeder_node._file_yaw_rad = 0.25
    seeder_node._file_yaw_var = 0.03
    seeder_node._latest_heading = _make_heading_imu(0.1)
    source, yaw, var = seeder_node._resolve_yaw_source()
    assert source == "file", f"stale lidar must fall through, got {source}"
    assert math.isclose(yaw, 0.25, abs_tol=1e-6)


def test_cascade_case_6_no_sources_returns_none(seeder_node):
    """Case 6: no lidar, no file, no heading -> source='none' (skip)."""
    seeder_node._latest_dock_match_pose = None
    seeder_node._latest_dock_match_trusted = False
    seeder_node._latest_dock_match_pose_at = None
    seeder_node._file_yaw_rad = None
    seeder_node._file_yaw_var = None
    seeder_node._latest_heading = None
    source, _yaw, _var = seeder_node._resolve_yaw_source()
    assert source == "none", f"expected 'none', got {source}"


# ---------------------------------------------------------------------------
# Case 7: debounce / throttle regression
# ---------------------------------------------------------------------------


def test_cascade_case_7_lidar_does_not_bypass_throttle(seeder_node):
    """Case 7: two _try_publish() calls within the throttle window must
    only publish once, even when the cascade resolves to 'lidar'.

    The existing 1 Hz throttle (_min_publish_period) is non-negotiable —
    extending the cascade must not regress this gate.
    """
    # Set up so cascade resolves to 'lidar' and publish-prerequisites are met.
    _seed_lidar_match(seeder_node, yaw_rad=0.5, age_s=0.1, trusted=True)
    seeder_node._file_yaw_rad = None
    seeder_node._file_yaw_var = None
    seeder_node._latest_heading = None
    # Provide GPS so _try_publish does not early-return on missing position.
    from mowgli_interfaces.msg import AbsolutePose
    gps = AbsolutePose()
    gps.pose.pose.position.x = 1.0
    gps.pose.pose.position.y = 2.0
    seeder_node._latest_gps = gps
    seeder_node._high_level_state = 1  # IDLE — allowed
    seeder_node._need_to_publish = True

    seeder_node._try_publish()
    seeder_node._try_publish()  # immediately again, throttled

    # _pub_map and _pub_odom each should have been called once.
    assert seeder_node._pub_map.publish.call_count == 1, (
        f"expected 1 map publish, got {seeder_node._pub_map.publish.call_count}"
    )
    assert seeder_node._pub_odom.publish.call_count == 1


# ---------------------------------------------------------------------------
# Case 8: HIGH_LEVEL_STATE_AUTONOMOUS gate regression
# ---------------------------------------------------------------------------


def test_cascade_case_8_autonomous_state_suppresses_lidar_seed(seeder_node):
    """Case 8: HIGH_LEVEL_STATE_AUTONOMOUS (=2) must block /set_pose
    even when cascade resolves to 'lidar'.

    Issue #73 root-cause: any seed during AUTONOMOUS corrupts the live
    mow. The motion-state gate must dominate the cascade source.
    """
    _seed_lidar_match(seeder_node, yaw_rad=0.5, age_s=0.1, trusted=True)
    seeder_node._file_yaw_rad = None
    seeder_node._file_yaw_var = None
    seeder_node._latest_heading = None
    from mowgli_interfaces.msg import AbsolutePose
    gps = AbsolutePose()
    gps.pose.pose.position.x = 1.0
    gps.pose.pose.position.y = 2.0
    seeder_node._latest_gps = gps
    seeder_node._high_level_state = 2  # AUTONOMOUS — blocked
    seeder_node._need_to_publish = True

    seeder_node._try_publish()

    assert seeder_node._pub_map.publish.call_count == 0, (
        "AUTONOMOUS state must suppress /set_pose even on cascade=lidar"
    )
    assert seeder_node._pub_odom.publish.call_count == 0
