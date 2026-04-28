---
phase: 1
slug: coverage-planner-rewrite
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-04-28
---

# Phase 1 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution. Skeleton populated by planner during Step 8 of plan-phase. See `01-RESEARCH.md` §4 for the per-requirement validation architecture (test type, approach, acceptance signal).

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | `ament_cmake_gtest` (C++ unit/integration), Python launch_testing (E2E sim), pytest for `e2e_test*.py` |
| **Config file** | `ros2/src/mowgli_coverage_planner/test/CMakeLists.txt` (Wave 0 creates) + `ros2/src/mowgli_geometry/test/CMakeLists.txt` (Wave 0 creates) + reuse `ros2/src/Makefile` `make test` / `make e2e-test` targets |
| **Quick run command** | `cd ros2 && make build-pkg PKG=mowgli_coverage_planner && colcon test --packages-select mowgli_coverage_planner mowgli_geometry --event-handlers console_cohesion+` |
| **Full suite command** | `cd ros2 && make build && make test && make e2e-test` |
| **Estimated runtime** | quick: ~60 s; full (with sim): ~5–10 min |

---

## Sampling Rate

- **After every task commit:** Run `colcon test --packages-select mowgli_coverage_planner mowgli_geometry`
- **After every plan wave:** Run `make test` for the full ROS2 test suite
- **Before hardware smoke test (final acceptance):** Run `make e2e-test` (headless sim) and `make build && make format` clean
- **Max feedback latency:** 60 s (quick), 600 s (full)

---

## Per-Task Verification Map

> Populated by planner during Step 8 of plan-phase based on RESEARCH.md §4 Validation Architecture. Each row links: Task ID → Requirement (SPEC R-NN) → test type → automated command.

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| _to-be-filled-by-planner_ | | | | | | | | | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

> Wave 0 must establish the test infrastructure before any feature work runs. Initial seed list, expanded by planner:

- [ ] `ros2/src/mowgli_geometry/test/test_offset_polygon_inward.cpp` — covers Minkowski offset geometry
- [ ] `ros2/src/mowgli_geometry/test/test_footprint_check.cpp` — covers rectangular footprint inside-area + obstacle-avoidance checks
- [ ] `ros2/src/mowgli_geometry/test/test_pca_axis.cpp` — covers PCA-derived long-axis for narrow areas (D-10)
- [ ] `ros2/src/mowgli_coverage_planner/test/test_outline_generator.cpp` — outside-in working area + inside-out obstacle outline geometry
- [ ] `ros2/src/mowgli_coverage_planner/test/test_aabb_sweep.cpp` — boustrophedon sweep with obstacle clipping
- [ ] `ros2/src/mowgli_coverage_planner/test/test_validation_pipeline.cpp` — 10-point pre-flight validation, one test per error_code
- [ ] `ros2/src/mowgli_coverage_planner/test/test_checkpoint.cpp` — atomic key=value write+rename, parser round-trip, corruption detection
- [ ] `ros2/src/mowgli_coverage_planner/test/fixtures/` — polygon test fixtures (square area, L-shaped narrow strip, area-with-circular-obstacle, dock + footprint configs)

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Hardware smoke test in Eichenau garden — robot mows ≥ 1 complete strip without #61/#64-class failures | SPEC AC-13 | Requires physical Pi5 + RTK base + grass; cannot be simulated faithfully | (1) Deploy `coverage_planner_node` + `FollowCoveragePlan` BT to Pi5 via existing auto-deploy. (2) Place robot on dock, send `COMMAND_START`. (3) Confirm: Plan generated; pre-flight validation passes; `UNDOCK` segment executes; first `MOWING_BOUSTROPHEDON` strip completes end-to-end without LETHAL-cell BackUp abort and without virtual-obstacle drive-through. (4) Inspect mow_session_monitor JSONL for cross-check verdicts. |
| Operator-measured robot_geometry params committed to mowgli_robot.yaml | SPEC R-6, D-09 | Operator with caliper, baseline tape, tape-measure | Measure robot_length, robot_width, drive_axis offsets, blade offsets per the methodology in mowgli_robot.yaml comments; commit values to repo. |
| GUI Plan-Preview button visually renders sparse plan with segment_type colour coding | SPEC AC-15 | Operator inspects browser DOM + Mapbox layer state | Open GUI, click Preview, verify all 8 segment_types render with the colours defined in D-11; verify hover shows segment_type in tooltip. |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 60s for quick, < 600s for full
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending — planner populates the per-task table, then sets `nyquist_compliant: true`.
