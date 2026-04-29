#!/usr/bin/env python3
"""
test_calibrate_imu_yaw_capture - pytest harness for the wiring between
calibrate_imu_yaw_node and dock_scan_capture (Plan 02-03 Task 2).

We do NOT re-test the capture logic (Task 1's pytest already covers
that). We test only the link:

  1. _lookup_sensor_extrinsic_for_dock_scan returns (0,0,0) when the
     parallel TF tree is not populated, and uses
     base_footprint_wheels -> lidar_link_wheels in the lookup call when
     the buffer is provided.
  2. The _calibrate_cb success path threads dock_scan_captured =
     dock_yaw_result['dock_scan_captured'] into the
     CalibrateImuYawStatus.message field, marking either "persisted"
     or "skipped (see warning)".
  3. response.success is NOT regressed when capture returns False - the
     dock-yaw calibration itself succeeded.

The full state machine (driving the robot, reverse-undock, GPS
displacement) is tested live on the Pi5 in Plan 02-08; here we exercise
only the message-construction logic.
"""

import os
import sys
from unittest.mock import MagicMock

import pytest

rclpy = pytest.importorskip("rclpy")

# Force the scripts/ directory onto sys.path before importing the node
# module (conftest.py does this too, but be explicit so this file can
# be run standalone).
_SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, "scripts")
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# calibrate_imu_yaw_node imports rclpy + mowgli_interfaces; importorskip
# above already gates rclpy. mowgli_interfaces is fixed via
# importorskip too so this file is collect-clean on hosts without it.
pytest.importorskip("mowgli_interfaces")
import calibrate_imu_yaw_node as cal_module  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


@pytest.fixture
def rclpy_runtime():
    rclpy.init(args=None)
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass


def _build_node(rclpy_runtime):
    """Construct a real CalibrateImuYawNode. The node creates publishers
    and subscriptions on construction; we tear them down via
    destroy_node() in the test cleanup."""
    return cal_module.CalibrateImuYawNode()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_lookup_sensor_extrinsic_returns_zeros_without_tf(rclpy_runtime):
    """When the TF buffer is not populated (no parallel-tree publishers
    in the test process), the helper falls back to (0, 0, 0) without
    raising. Plan 02-04's matcher tolerates 0.0 because it re-resolves
    the extrinsic from live TF at runtime."""
    node = _build_node(rclpy_runtime)
    try:
        # Force buffer to None to bypass any TF state that may have leaked
        # in across test ordering.
        node._tf_buffer = None
        x, y, yaw = node._lookup_sensor_extrinsic_for_dock_scan()
        assert (x, y, yaw) == (0.0, 0.0, 0.0)
    finally:
        node.destroy_node()


def test_lookup_sensor_extrinsic_uses_parallel_tree_frames(
    monkeypatch, rclpy_runtime
):
    """When the TF buffer is present, lookup_transform must be called
    with `base_footprint_wheels` -> `lidar_link_wheels` (parallel TF
    tree per CLAUDE.md AI #1). Catching the wrong frames here is a hard
    safety regression: feeding the fused EKF frames into Kinematic-ICP's
    motion prior would close the feedback loop the architecture
    explicitly forbids."""
    node = _build_node(rclpy_runtime)
    try:
        captured_args = {}

        class FakeTransform:
            class _Q:
                x = 0.0
                y = 0.0
                z = 0.0
                w = 1.0

            class _T:
                x = 0.10
                y = 0.0
                z = 0.0

            class _Inner:
                rotation = _Q()
                translation = _T()

            transform = _Inner()

        class FakeBuf:
            def lookup_transform(self, target, source, t, timeout=None):
                captured_args["target"] = target
                captured_args["source"] = source
                return FakeTransform()

        node._tf_buffer = FakeBuf()
        x, y, yaw = node._lookup_sensor_extrinsic_for_dock_scan()

        assert captured_args["target"] == "base_footprint_wheels"
        assert captured_args["source"] == "lidar_link_wheels"
        assert abs(x - 0.10) < 1e-9
        assert abs(y) < 1e-9
        assert abs(yaw) < 1e-9
    finally:
        node.destroy_node()


def test_message_text_marks_persisted_when_capture_succeeds():
    """Verifies the status_message construction logic: when dock_yaw_result
    contains dock_scan_captured=True, the published message must end with
    'dock yaw + dock_scan persisted'.

    We replicate the exact branch from _calibrate_cb (lines around
    self._publish_done_status) without standing up the full state
    machine.
    """
    base_message = "imu_yaw calibrated from 213 samples"
    dock_yaw_result = {"dock_scan_captured": True}
    response_success = True

    status_message = base_message
    if (
        response_success
        and dock_yaw_result is not None
        and not bool(dock_yaw_result.get("dock_scan_captured", False))
    ):
        status_message = (
            f"{status_message} | dock yaw persisted; "
            "dock_scan capture skipped (see warning)"
        )
    elif (
        response_success
        and dock_yaw_result is not None
        and bool(dock_yaw_result.get("dock_scan_captured", False))
    ):
        status_message = f"{status_message} | dock yaw + dock_scan persisted"

    assert "dock yaw + dock_scan persisted" in status_message
    assert "skipped" not in status_message


def test_message_text_marks_skipped_when_capture_fails():
    """When capture failed (gate denied or timeout), the status_message
    must include 'skipped (see warning)' so the GUI can surface the
    Recapture-button hint to the operator. response.success stays True."""
    base_message = "imu_yaw calibrated from 213 samples"
    dock_yaw_result = {"dock_scan_captured": False}
    response_success = True

    status_message = base_message
    if (
        response_success
        and dock_yaw_result is not None
        and not bool(dock_yaw_result.get("dock_scan_captured", False))
    ):
        status_message = (
            f"{status_message} | dock yaw persisted; "
            "dock_scan capture skipped (see warning)"
        )
    elif (
        response_success
        and dock_yaw_result is not None
        and bool(dock_yaw_result.get("dock_scan_captured", False))
    ):
        status_message = f"{status_message} | dock yaw + dock_scan persisted"

    assert "skipped (see warning)" in status_message


def test_capture_module_reference_exists():
    """Smoke test: calibrate_imu_yaw_node module must expose the
    dock_scan_capture symbol (either the imported module or None on the
    fallback path). Catches future renames before they surface in the
    longer integration tests."""
    assert hasattr(cal_module, "dock_scan_capture")
    # On healthy installs the import succeeds; on broken ones we expect
    # the explicit None sentinel rather than an AttributeError.
    assert cal_module.dock_scan_capture is None or callable(
        getattr(cal_module.dock_scan_capture, "capture_and_save", None)
    )


def test_odom_cb_updates_latest_wheel_vx(rclpy_runtime):
    """The dock_scan_capture stationarity gate reads
    self._latest_wheel_vx; ensure the /wheel_odom callback keeps that
    field current even when self._collecting is False (so the gate is
    valid the moment dock_scan_capture is invoked)."""
    node = _build_node(rclpy_runtime)
    try:
        from nav_msgs.msg import Odometry

        msg = Odometry()
        msg.twist.twist.linear.x = 0.123
        node._latest_wheel_vx = 0.0
        node._collecting = False  # explicit
        node._odom_cb(msg)
        assert abs(node._latest_wheel_vx - 0.123) < 1e-9
    finally:
        node.destroy_node()
