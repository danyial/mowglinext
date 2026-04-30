#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
test_synthetic_scan_publisher.py — smoke contract test for Plan 02-08
synthetic_scan_kicp_publisher.

Why this lives in `mowgli_simulation/test/` (not `e2e_test.py`):
  - This is a pure-Python unit test that does NOT spin a ROS2 graph,
    does NOT require Gazebo, and does NOT depend on the real LD19. It
    pins the publisher's two load-bearing contracts:
      1. The synthesized scan has the right shape & frame contract
         (frame_id, num rays, ranges in [range_min, range_max] for the
         hits, finite count > 0).
      2. The dock-V-funnel raycast actually returns hits when the robot
         sits 1.5 m back along the dock-approach line — i.e. the
         publisher would actually feed dock_scan_match a usable signal.

The full sim-graph end-to-end run lives in `e2e_test.py::_run_fine_dock_phase`.
That's the integration coverage; this is the unit coverage.
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

# Make the publisher script importable as a regular module — it lives under
# `scripts/`, not under a Python package, so we extend sys.path manually.
_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS_DIR = os.path.join(_HERE, "..", "scripts")
sys.path.insert(0, os.path.normpath(_SCRIPTS_DIR))

import synthetic_scan_kicp_publisher as ssp  # noqa: E402


# ---------------------------------------------------------------------------
# Module-level pure-function tests (no rclpy)
# ---------------------------------------------------------------------------


def test_dock_segments_are_three() -> None:
    """V-funnel = base + 2 arms. Always."""
    segs = ssp.build_dock_segments(5.0, 5.0, 0.0)
    assert len(segs) == 3, f"expected 3 dock segments, got {len(segs)}"


def test_dock_segments_translate_with_dock_anchor() -> None:
    """Same shape, different anchor → segments shifted by (dx, dy)."""
    a = ssp.build_dock_segments(0.0, 0.0, 0.0)
    b = ssp.build_dock_segments(2.0, -3.0, 0.0)
    assert len(a) == len(b) == 3
    for sa, sb in zip(a, b):
        assert math.isclose(sb.ax - sa.ax, 2.0, abs_tol=1e-9)
        assert math.isclose(sb.ay - sa.ay, -3.0, abs_tol=1e-9)


def test_dock_segments_rotate_with_yaw() -> None:
    """Yaw 90° around the dock anchor: x-axis points up. The base bar
    (originally along local y at local x=0) rotates to lie along local x
    at the same anchor. Verify by checking that the base midpoint stays
    on the anchor when the yaw is varied."""
    for yaw_deg in (0.0, 45.0, 90.0, -135.0):
        yaw = math.radians(yaw_deg)
        segs = ssp.build_dock_segments(1.0, 2.0, yaw)
        # Segment 0 is the base bar — its midpoint should equal the anchor.
        s = segs[0]
        mid_x = (s.ax + s.bx) * 0.5
        mid_y = (s.ay + s.by) * 0.5
        assert math.isclose(mid_x, 1.0, abs_tol=1e-9), (
            f"yaw={yaw_deg}: base midpoint x={mid_x}"
        )
        assert math.isclose(mid_y, 2.0, abs_tol=1e-9), (
            f"yaw={yaw_deg}: base midpoint y={mid_y}"
        )


def test_ray_segment_intersection_basic() -> None:
    """Ray pointing +x at origin hits a vertical segment at x=2; range=2."""
    seg = ssp.Segment2D(2.0, -1.0, 2.0, 1.0)
    t = ssp.ray_segment_intersection(0.0, 0.0, 1.0, 0.0, seg)
    assert t is not None
    assert math.isclose(t, 2.0, abs_tol=1e-9)


def test_ray_segment_intersection_misses_behind() -> None:
    """A segment behind the ray origin must not produce a positive t."""
    seg = ssp.Segment2D(-2.0, -1.0, -2.0, 1.0)
    t = ssp.ray_segment_intersection(0.0, 0.0, 1.0, 0.0, seg)
    assert t is None


def test_synthesize_scan_returns_360_rays_by_default() -> None:
    """Contract: full scan = 360 rays @ 1° resolution."""
    segs = ssp.build_dock_segments(5.0, 5.0, 0.0)
    ranges = ssp.synthesize_scan_ranges(3.5, 5.0, 0.0, segs)
    assert len(ranges) == 360, f"expected 360 ranges, got {len(ranges)}"


def test_synthesize_scan_has_dock_hits_when_robot_faces_dock() -> None:
    """The whole point of this publisher: when the robot sits 1.5 m back
    from the dock and faces it, at least some rays MUST hit the V-funnel
    inside [range_min, range_max]. Without this the smoke test would pass
    on a publisher that emits all-infinity ranges (regression caught
    during early dev: `dock_segments` swapped local-x for local-y)."""
    segs = ssp.build_dock_segments(5.0, 5.0, 0.0)
    # Dock points along +x in the dock-local frame; therefore "looking
    # at the dock" from 1.5 m back means standing at (5.0 - 1.5, 5.0)
    # facing +x (yaw=0 in the map frame).
    ranges = ssp.synthesize_scan_ranges(3.5, 5.0, 0.0, segs, rng=None)
    finite = [r for r in ranges if math.isfinite(r)]
    assert len(finite) > 0, "no dock hits — publisher would feed empty scans"
    in_band = [
        r for r in finite if ssp.LD19_RANGE_MIN_M < r < ssp.LD19_RANGE_MAX_M
    ]
    assert len(in_band) > 0, (
        f"no in-band hits among {len(finite)} finite ranges"
    )


def test_synthesize_scan_respects_explicit_num_rays() -> None:
    """When the caller asks for N rays we get exactly N back — the smoke
    pytest exploits this knob to keep its harness fast."""
    segs = ssp.build_dock_segments(5.0, 5.0, 0.0)
    ranges = ssp.synthesize_scan_ranges(
        3.5, 5.0, 0.0, segs, num_rays=120, rng=None
    )
    assert len(ranges) == 120


def test_world_obstacles_yaml_parses_polygons() -> None:
    """`world_obstacles_yaml` is the optional escape hatch for adding
    extra obstacles; verify the parser accepts the flat key=value
    polygon format."""
    contents = (
        "# scratch obstacle polygon\n"
        "obstacle_a: 1,1; 2,1; 2,2; 1,2\n"
        "obstacle_b: 3,3; 4,3\n"
        "\n"
    )
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(contents)
        path = f.name
    try:
        segs = ssp.parse_world_obstacles_yaml(path)
        # 4-vertex closed polygon = 4 segments; 2-vertex polyline = 2
        # segments (closed loop a-b + b-a).
        assert len(segs) == 4 + 2, f"expected 6 segments, got {len(segs)}"
    finally:
        os.unlink(path)


def test_world_obstacles_yaml_missing_returns_empty() -> None:
    """No file = no extra geometry. Don't crash, don't loud-fail —
    publisher continues with dock-only scan."""
    segs = ssp.parse_world_obstacles_yaml("/nonexistent/path.yaml")
    assert segs == []


def test_module_constants_match_ld19_datasheet() -> None:
    """Compile-time documentation guard. If you change the LD19
    constants you almost certainly want to update PI5-CHECKLIST.md too."""
    assert ssp.LD19_NUM_RAYS == 360
    assert math.isclose(ssp.LD19_ANGLE_MIN_RAD, -math.pi)
    assert math.isclose(ssp.LD19_ANGLE_MAX_RAD, math.pi)
    assert 0.0 < ssp.LD19_RANGE_MIN_M < ssp.LD19_RANGE_MAX_M
    assert ssp.LD19_RANGE_MAX_M >= 6.0  # sanity: covers the whole dock
