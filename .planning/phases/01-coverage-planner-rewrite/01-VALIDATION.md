---
phase: 1
slug: coverage-planner-rewrite
status: in-progress
nyquist_compliant: true
wave_0_complete: true
created: 2026-04-28
populated: 2026-04-29
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

> Populated 2026-04-29 by Plan 01-09 Task 2. Each row links: Task ID → Requirement (SPEC R-NN) → test type → automated command. Wave 0 (Plans 01–04) and Waves 1–4 (Plans 05–08) all reached the green gate via the per-task colcon build/test commands documented in their SUMMARY.md files (`.planning/phases/01-coverage-planner-rewrite/01-NN-SUMMARY.md`).

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 01-01-T1 | 01 | 1 | R-2, R-4, R-5, R-12, R-13 | T-01-01..05 | Schema-only, no runtime input | unit (rosidl gen) | `colcon build --packages-select mowgli_interfaces` | yes | ✅ green |
| 01-01-T2 | 01 | 1 | R-2, R-12 | T-01-01..04 | Field validation in planner (Plan 05/07) | unit (rosidl gen) | `colcon build --packages-select mowgli_interfaces` | yes | ✅ green |
| 01-01-T3 | 01 | 1 | (codegen only) | — | No runtime exposure | typecheck | `cd gui && go build ./... && cd web && yarn tsc --noEmit` | yes | ✅ green |
| 01-02-T1 | 02 | 2 | R-6, R-7 | T-02-03 | Caller-validated polygons | unit | `colcon test --packages-select mowgli_geometry` | yes | ✅ green |
| 01-02-T2 | 02 | 2 | R-6, R-10, R-13 | T-02-01..02 | Atomic FS write; PCA bounded | unit | `colcon test --packages-select mowgli_geometry` | yes | ✅ green |
| 01-03-T1 | 03 | 2 | R-6 | (none) | Config defaults | yaml-parse | `python3 -c "import yaml; yaml.safe_load(open('mowgli_robot.yaml'))"` | yes | ✅ green |
| 01-03-T2 | 03 | 2 | R-6 | (none) | Documentation | grep | `grep -q "Architecture Invariant #15" CLAUDE.md` | yes | ✅ green |
| 01-04-T1 | 04 | 2 | R-2, R-4 | T-04-01,03,04 | XSS-safe via JSX, AsyncButton DoS guard | typecheck+lint | `cd gui/web && yarn tsc --noEmit && yarn lint` | yes | ✅ green |
| 01-04-T2 | 04 | 2 | R-13 | T-04-05 | Range-clamped narrow-area dropdown | typecheck+lint+unit | `cd gui/web && yarn test --run && yarn tsc --noEmit && yarn lint` | yes | ✅ green |
| 01-05-T1 | 05 | 3 | R-1, R-2, R-12 | T-05-05,06 | Concurrent-goal reject; param validation | build | `colcon build --packages-select mowgli_coverage_planner` | yes | ✅ green |
| 01-05-T2 | 05 | 3 | R-10, R-11, R-12 | T-05-01..04 | Atomic write; corruption detection | unit | `colcon test --packages-select mowgli_coverage_planner` | yes | ✅ green |
| 01-06-T1 | 06 | 3 | R-1, R-13 | T-06-01..03 | Range clamp on disk read; internal-only service | unit | `colcon test --packages-select mowgli_map` | yes | ✅ green |
| 01-06-T2 | 06 | 3 | R-1 | T-06-04 | Cleanup gated on existing tests | unit | `colcon test --packages-select mowgli_interfaces mowgli_map` | yes | ✅ green |
| 01-07-T1 | 07 | 4 | R-3, R-4, R-5, R-6, R-7, R-8, R-12, R-13 | T-07-01,02,07,08 | Pre+post validators reject unsafe plans | unit | `colcon test --packages-select mowgli_coverage_planner` | yes | ✅ green |
| 01-07-T2 | 07 | 4 | R-9, R-11 | T-07-05 | Resume tolerance gate; finite-value guard | unit | `colcon test --packages-select mowgli_coverage_planner` | yes | ✅ green |
| 01-08-T1 | 08 | 4 | R-2, R-4, R-10 | T-08-01,02,05,06 | Blade rules per segment_type; safety | build+unit | `colcon test --packages-select mowgli_behavior` | yes | ✅ green |
| 01-08-T2 | 08 | 4 | R-4 | T-08-03 (HIGH) | onHalted disables blade; regression-tested | unit | `colcon test --packages-select mowgli_behavior` | yes | ✅ green |
| 01-09-T0 | 09 | 5 | (precondition) | (build) | Header-only INTERFACE export propagates `mowgli_geometry/footprint.hpp` to consumers | build | `colcon build --packages-select mowgli_interfaces mowgli_geometry mowgli_coverage_planner mowgli_behavior` | yes | ✅ green |
| 01-09-T1 | 09 | 5 | R-1, R-2, R-3, R-12 | T-09-01,02 | Action probe verifies live endpoint + sane plan envelope | sim | `cd ros2 && make e2e-test` | yes | ✅ green |
| 01-09-T2 | 09 | 5 | (validation) | (composite) | Doc completion | grep | `grep -q "nyquist_compliant: true" .planning/phases/01-coverage-planner-rewrite/01-VALIDATION.md` | yes | ✅ green |
| 01-09-T3 | 09 | 5 | R-1 (AC-13) | T-09-02 (HIGH) | Hardware smoke on Pi5 in Eichenau garden | manual | (operator verification — see Manual-Only Verifications + 01-09-PLAN.md Task 3) | n/a | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

> Wave 0 must establish the test infrastructure before any feature work runs. Initial seed list, expanded by planner:

- [x] `ros2/src/mowgli_geometry/test/test_offset_polygon_inward.cpp` — covers Minkowski offset geometry (Plan 02)
- [x] `ros2/src/mowgli_geometry/test/test_footprint_check.cpp` — covers rectangular footprint inside-area + obstacle-avoidance checks (Plan 02)
- [x] `ros2/src/mowgli_geometry/test/test_pca_axis.cpp` — covers PCA-derived long-axis for narrow areas (D-10) (Plan 02)
- [x] `ros2/src/mowgli_coverage_planner/test/test_outline_generator.cpp` — outside-in working area + inside-out obstacle outline geometry (Plan 07)
- [x] `ros2/src/mowgli_coverage_planner/test/test_aabb_sweep.cpp` — boustrophedon sweep with obstacle clipping (Plan 07)
- [x] `ros2/src/mowgli_coverage_planner/test/test_validation_pipeline.cpp` — 10-point pre-flight validation, one test per error_code (Plan 07)
- [x] `ros2/src/mowgli_coverage_planner/test/test_checkpoint.cpp` — atomic key=value write+rename, parser round-trip, corruption detection (Plan 05)
- [x] `ros2/src/mowgli_coverage_planner/test/fixtures/polygons.hpp` — polygon test fixtures (square area, L-shaped narrow strip, area-with-circular-obstacle, dock + footprint configs) (Plan 07)

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Hardware smoke test in Eichenau garden — robot mows ≥ 1 complete strip without #61/#64-class failures | SPEC AC-13 | Requires physical Pi5 + RTK base + grass; cannot be simulated faithfully | (1) Deploy `coverage_planner_node` + `FollowCoveragePlan` BT to Pi5 via existing auto-deploy. (2) Place robot on dock, send `COMMAND_START`. (3) Confirm: Plan generated; pre-flight validation passes; `UNDOCK` segment executes; first `MOWING_BOUSTROPHEDON` strip completes end-to-end without LETHAL-cell BackUp abort and without virtual-obstacle drive-through. (4) Inspect mow_session_monitor JSONL for cross-check verdicts. |
| Operator-measured robot_geometry params committed to mowgli_robot.yaml | SPEC R-6, D-09 | Operator with caliper, baseline tape, tape-measure | Measure robot_length, robot_width, drive_axis offsets, blade offsets per the methodology in mowgli_robot.yaml comments; commit values to repo. |
| GUI Plan-Preview button visually renders sparse plan with segment_type colour coding | SPEC AC-15 | Operator inspects browser DOM + Mapbox layer state | Open GUI, click Preview, verify all 8 segment_types render with the colours defined in D-11; verify hover shows segment_type in tooltip. |

---

## Validation Sign-Off

- [x] All tasks have `<automated>` verify or Wave 0 dependencies (the only manual row is 01-09-T3, which is the SPEC AC-13 hardware smoke gate by definition)
- [x] Sampling continuity: no 3 consecutive tasks without automated verify (verified by table — every plan in Waves 1–4 has a colcon-test row)
- [x] Wave 0 covers all MISSING references (8/8 boxes flipped above)
- [x] No watch-mode flags
- [x] Feedback latency < 60s for quick, < 600s for full (e2e-test target wraps make e2e-test in <10 min per Plan 01-09 Task 1 verify)
- [x] `nyquist_compliant: true` set in frontmatter

**Approval:** approved — table populated 2026-04-29 (Plan 01-09 Task 2). All 19 automated rows are ✅ green; the single ⬜ pending row (01-09-T3) is the operator-gated Pi5 hardware smoke and remains pending until the Eichenau garden run is verified by the operator (see Plan 01-09 Task 3 / Manual-Only Verifications above).
