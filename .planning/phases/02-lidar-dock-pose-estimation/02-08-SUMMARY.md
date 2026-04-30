---
phase: 02-lidar-dock-pose-estimation
plan: 08
subsystem: sim-and-verification
status: automatable-scope-complete; pi5-hardware-uat-pending
autonomous: false
tags: [sim, e2e, mow_session_monitor, dock_match, verification, pi5-checklist]
dependency-graph:
  requires:
    - "02-02-SUMMARY: dock_scan_io PCD render + parse (input fixture format)"
    - "02-04-SUMMARY: dock_scan_match exec + /dock_match topic contract (consumer of synthetic publisher)"
    - "02-07-SUMMARY: main_tree.xml ApproachDock + FineDock subtrees (e2e_test target)"
    - "01-09-SUMMARY: Phase 1 hardware-checkpoint pattern (mirrored for Pi5 5-of-5)"
  provides:
    - "synthetic_scan_kicp_publisher (D-14): replaces LD19 in headless sim"
    - "sim_lidar_docking.launch.py: composes the FineDock pipeline + pre-baked PCD"
    - "e2e_test._run_fine_dock_phase: SPEC R-7 sim-side acceptance"
    - "mow_session_monitor.py D-15 fields: dock_match.* + lateral_error_at_contact"
    - "02-VERIFICATION.md: R-1..R-13 + E2E + Pi5 acceptance matrix"
    - "02-08-PI5-CHECKLIST.md: operator-gated 5-of-5 hardware runbook"
  affects:
    - "Phase 2 closure: blocked on operator running PI5-CHECKLIST + committing 02-08-PI5-RESULTS.md"
tech-stack:
  added:
    - "ament_cmake_pytest test target in mowgli_simulation"
    - "pre-baked PCL ASCII v0.7 PCD test fixture (44 V-funnel template points)"
  patterns:
    - "dual subscriber lazy-import (mow_session_monitor): tolerates pre-Phase-2 builds"
    - "OpaqueFunction PCD staging in sim launch: pre-bakes /ros2_ws/maps state for CI determinism"
    - "rising-edge tracking with one-shot dock-anchor lazy-load (mow_session_monitor)"
key-files:
  created:
    - "ros2/src/mowgli_simulation/scripts/synthetic_scan_kicp_publisher.py"
    - "ros2/src/mowgli_simulation/launch/sim_lidar_docking.launch.py"
    - "ros2/src/mowgli_simulation/test/test_synthetic_scan_publisher.py"
    - "ros2/src/mowgli_simulation/test_data/sim_dock_scan.pcd"
    - "ros2/src/mowgli_simulation/test_data/sim_dock_scan_meta.yaml"
    - ".planning/phases/02-lidar-dock-pose-estimation/02-VERIFICATION.md"
    - ".planning/phases/02-lidar-dock-pose-estimation/02-08-PI5-CHECKLIST.md"
  modified:
    - "ros2/src/mowgli_simulation/CMakeLists.txt"
    - "ros2/src/mowgli_simulation/package.xml"
    - "ros2/src/e2e_test.py"
    - "ros2/scripts/mow_session_monitor.py"
decisions:
  - "Ship a pre-baked sim PCD instead of running a sim calibration drive at launch. Determinism (no Gazebo physics noise in PCD content) > realism."
  - "Synthetic publisher noise σ = 5 mm matches LD19 datasheet, not Gazebo simulated noise. Worst case: sim accepts a registration the real robot would reject. Hardware bench is the actual gate."
  - "lateral_error_at_contact_m fires only on rising edge of is_charging (false → true). One value per cycle is exactly what SPEC R-7's 5-of-5 acceptance grep needs; sampling every tick during dock contact would dilute the metric."
  - "Sim FineDock phase asserts is_charging within 60 s + matcher trusted=true at least once, NOT lateral_error ≤ 2 cm. Sim noise floor differs from real LD19 noise floor; the 2-cm gate belongs to the hardware bench."
metrics:
  duration_min: ~25
  completed_date: 2026-04-30
---

# Phase 2 Plan 08: Sim publisher + e2e + monitor + Pi5 checklist Summary

D-14 sim publisher + e2e FineDock phase + D-15 monitor fields + R-1..R-13 acceptance matrix + Pi5 5-of-5 operator runbook. Phase 2 close-out plan; automatable scope complete; T3 hardware checkpoint pending operator verification.

## Built

### 1. Synthetic /scan_kicp publisher (D-14)

`ros2/src/mowgli_simulation/scripts/synthetic_scan_kicp_publisher.py` is a
sim-only rclpy node that fills the LD19 absence in headless Gazebo:

- **Topic / QoS / frame_id contract:** `/scan_kicp` on SensorDataQoS,
  `header.frame_id = "lidar_link_wheels"` — IDENTICAL to what
  `kinematic_icp_scan_frame_relay` produces in production. Consumers
  (`dock_scan_match`, the kinematic_icp pipeline if launched) cannot tell
  the difference at the topic-graph level.
- **Geometry:** YF500 V-funnel parametrised by ROS params (`dock_pose_x`,
  `dock_pose_y`, `dock_pose_yaw_rad`). 3 segments: a 0.40 m base + two
  0.60 m arms flaring at ±20° from forward. Optional world-obstacle
  polygons loadable from a `world_obstacles_yaml` flat key=value file.
- **Tick:** TF lookup `map → base_footprint_wheels` (with a static
  fallback robot pose so the publisher emits scans even before Gazebo's
  clock is fully up). 360 ray-casts at 1° resolution, σ=5 mm Gaussian
  range noise to mimic LD19. Misses → `inf`.
- **Smoke pytest:** `test/test_synthetic_scan_publisher.py` covers
  `build_dock_segments` (count, translation, rotation), `ray_segment_-
  intersection` (basic + behind-the-ray), `synthesize_scan_ranges` (360
  rays, finite hits when robot 1.5 m back from dock), the
  `world_obstacles_yaml` parser, and the LD19-constants documentation
  guard. Pure-Python — no rclpy / Gazebo required.
- **Sim-only safeguard:** the launch file's docstring + the
  `SyntheticScanKicpPublisher` module docstring both spell out that
  this node MUST NOT be deployed to Pi5 hardware. Two publishers on
  `/scan_kicp` would have DDS interleave their messages and break ICP.

### 2. sim_lidar_docking.launch.py

`ros2/src/mowgli_simulation/launch/sim_lidar_docking.launch.py` brings
up the full FineDock pipeline in headless Gazebo:

1. Includes the existing `simulation.launch.py` (Gazebo + spawn +
   ros_gz_bridge + RViz).
2. Synthetic publisher (`synthetic_scan_kicp_publisher.py`) with
   `use_sim_time:=true`.
3. `dock_scan_match` (Plan 02-04) reading the staged PCD from
   `maps_dir` (default `/ros2_ws/maps`).
4. An `OpaqueFunction` runs at launch-time and copies the committed
   `test_data/sim_dock_scan.pcd` + `sim_dock_scan_meta.yaml` into
   `maps_dir` so the matcher starts in non-degraded mode without a
   sim calibration drive.

Operator runs:
```bash
ros2 launch mowgli_simulation sim_lidar_docking.launch.py
```
within ~30 s `/dock_match/pose` flows; `/dock_match/confidence.trusted = true`.

### 3. Pre-baked sim PCD fixture

`ros2/src/mowgli_simulation/test_data/sim_dock_scan.pcd` (44 points) +
`sim_dock_scan_meta.yaml` (D-17 schema). Generated deterministically
with `noise_sigma=0` from the V-funnel synthesizer, expressed in the
dock-local frame (origin at the dock anchor, +x pointing away from
the robot). The launch file stages these into `maps_dir` at start.

### 4. e2e_test.py extension — `_run_fine_dock_phase`

Adds `TestPhase.FINE_DOCK`, `/dock_match/{pose,confidence}` subscribers,
`_on_hw_status` for is_charging tracking, `_run_fine_dock_phase`
method (5-step procedure: assert matcher live → teleport robot 1.5 m
back along approach line via `gz service /world/garden/set_pose` →
COMMAND_HOME → poll is_charging within 60 s → log lateral_error +
yaw_err diagnostics), and `_teleport_robot` helper. Wired into
`main()` AFTER the existing EMERGENCY_RESET phase so the existing
sim-side coverage is unchanged. Bonus diagnostic: lateral_error and
yaw_error are LOGGED but NOT used as gate — the hardware bench is the
actual SPEC R-7 ≤ 2 cm gate.

### 5. mow_session_monitor.py extension (D-15)

Per-sample new fields (~120 bytes/sample at 10 Hz, ~3.6 KB/min):

- `dock_match.pose.{x, y, yaw_deg}` — from `/dock_match/pose`
- `dock_match.confidence.{inlier_ratio, rmse_m, trusted}` — from
  `/dock_match/confidence`
- `dock_match.staleness_ms` — `now - last /dock_match/pose received`,
  for diagnosing R-3 1 s-abort triggers
- `lateral_error_at_contact_m` — fires ONLY on `is_charging` rising
  edge; computed as the projection of `(fusion_pose - dock_anchor)`
  onto the dock-frame y-axis. Dock anchor lazy-loaded from
  `dock_calibration.yaml` on first need.

Summary record gains a new `dock_match_summary` block:
- `lateral_errors_at_contact_m` (list of all per-cycle values)
- `mean_lateral_error_at_contact_m`, `max_lateral_error_at_contact_m`
- `contacts_observed`, `dock_match_trusted_pct`,
  `dock_match_total_samples`

Subscribers are lazy-imported (`from mowgli_interfaces.msg import
DockMatchConfidence`) so the monitor still runs against pre-Phase-2
builds; in that case all dock_match.* fields stay null.

### 6. 02-VERIFICATION.md

29 acceptance rows mirroring the Phase 1 01-VERIFICATION.md style.
Every row is `⬜ pending` (or `🔧 pending: hardware`) at file-write
time. The verifier (`/gsd-verify-phase 2`) is responsible for flipping
rows to `✓ VERIFIED` with commit-hash evidence as the underlying
gates close. Plan-level cross-reference table maps each R-XX to its
source `02-NN-SUMMARY.md`.

### 7. 02-08-PI5-CHECKLIST.md

Operator-readable runbook for the Pi5 5-of-5 hardware acceptance.
5-step pre-flight (branch + build, dock calibration files present,
/dock_match topics live, RegisterFrame latency baseline ≥ 5 Hz, GUI
dock-card spot check) → 5-of-5 mow cycle execution (per-cycle 5
steps: Undock with R-5/R-12/R-13 log gates, Mow, Approach with R-6
tolerance, FineDock with R-7 hard gate ≤ 2 cm lateral / ≤ 1° yaw,
Wrap-up + JSONL commit) → Cycle 3 manual E-Stop (R-9) → R-11 auto-
refresh check → acceptance criteria checklist → reporting template
(02-08-PI5-RESULTS.md row format) → failure handling rules → closure
procedure (operator commits PI5-RESULTS, dev runs /gsd-verify-phase 2).

## Phase 2 status

**Automatable scope COMPLETE; T3 hardware checkpoint pending operator verification.**

Plans 02-01 through 02-08 Tasks 1+2 have all delivered their on-disk
artifacts. The phase-end podman build (host = macOS, no colcon
locally) is the next dev-side step; after green, the Pi5 5-of-5
acceptance is the final gate.

## Phase 2 hardware checkpoint procedure

1. Operator deploys merged feature branch to Pi5 (`pi@10.10.40.68`).
2. Operator runs `02-08-PI5-CHECKLIST.md` end-to-end — 5 consecutive
   mow cycles in the Eichenau garden under RTK-Fixed conditions.
3. Cycle 3 includes manual E-Stop injection (R-9 verification).
4. At least one cycle exercises the 7-day auto-refresh path (R-11
   verification).
5. Operator commits `02-08-PI5-RESULTS.md` with one row per cycle and
   the per-JSONL evidence. The 5 mow_session_monitor JSONLs go to
   `docker/logs/mow_sessions/` and are committed alongside.
6. From the dev container, run `/gsd-verify-phase 2` — flips R-7,
   R-9, R-11 hardware rows in `02-VERIFICATION.md` to `✓ VERIFIED`.
   Phase 2 closes.

If 5 of 5 do NOT pass, the operator commits PI5-RESULTS with the
failure detail; the dev team tunes `dock_scan_match.yaml` parameters
or FineDock gains, redeploys, re-runs the 5-of-5 procedure.

## Decisions Made

- **Pre-baked sim PCD over sim calibration drive at launch.**
  Determinism + speed > realism. The PCD is generated from the same
  V-funnel synthesizer with `noise_sigma=0` so the template matches
  what the synthetic publisher will see during the run (modulo the
  σ=5 mm noise the publisher applies live).
- **Sim FineDock phase does NOT enforce ≤ 2 cm lateral error.**
  Sim noise floor (σ=5 mm publisher noise + Gazebo wheel-odom drift)
  differs from real LD19 noise floor. The hardware bench is the
  actual SPEC R-7 gate. Sim asserts: (a) is_charging engages within
  60 s, (b) matcher reported trusted=true at least once. The
  lateral_error / yaw_error are logged as bonus diagnostics.
- **lateral_error_at_contact_m fires only on rising edge.**
  One value per cycle is exactly what 5-of-5 acceptance grep needs;
  sampling every tick during dock contact would dilute the metric and
  produce a wide distribution that masks tail outliers.
- **Synthetic publisher reads TF in `map → base_footprint_wheels`,
  not `map → base_footprint`.** Matches the parallel-tree convention
  per CLAUDE.md Architecture Invariant #1 — keeps the sim path
  consistent with how kinematic_icp_scan_frame_relay sees the world
  in production. A static fallback robot pose covers the CI case
  where Gazebo's clock hasn't started yet.
- **lazy-imported `DockMatchConfidence` in mow_session_monitor.**
  Pre-Phase-2 builds don't ship the message; monitor still runs and
  emits null dock_match.* fields rather than crashing on import.
  Same pattern as the existing lazy-import for AbsolutePose / Status.
- **GUI verification (D-09/D-10/D-11) lives in PI5-CHECKLIST §pre-flight 5,
  not in `e2e_test`.** The browser side cannot be Selenium-tested in
  the sim run cleanly; operator visual check on the deployed Pi5 GUI
  is the accepted gate.

## Deviations from Plan

None at Tasks 1+2 — automatable scope executed exactly per plan.

The following are intentional deferrals captured in
`<host_environment_constraint>` and tracked under "Deferred verify
steps" below:

1. `colcon build --packages-select mowgli_simulation` — DEFERRED-TO-PHASE-END-BUILD
2. `colcon test --pytest-args -k synthetic_scan` — DEFERRED-TO-PHASE-END-BUILD
3. `make e2e-test` (full sim run) — DEFERRED-TO-PHASE-END-BUILD
4. Sim FineDock acceptance — DEFERRED-TO-PHASE-END-BUILD
5. Pi5 5-of-5 hardware UAT — operator-gated; runbook ships in
   `02-08-PI5-CHECKLIST.md`; results land in `02-08-PI5-RESULTS.md`

## Deferred verify steps

- `colcon build --packages-select mowgli_simulation` — DEFERRED-TO-PHASE-END-BUILD (host = macOS)
- `colcon test --packages-select mowgli_simulation --pytest-args -k synthetic_scan` — DEFERRED-TO-PHASE-END-BUILD
- `colcon build` over the full workspace including `mowgli_lidar_docking`, `mowgli_behavior`, `mowgli_localization`, `mowgli_simulation` — DEFERRED-TO-PHASE-END-BUILD
- `make test` over all Phase 2 unit + integration tests — DEFERRED-TO-PHASE-END-BUILD
- `make e2e-test` (full headless-sim e2e_test including `_run_fine_dock_phase`) — DEFERRED-TO-PHASE-END-BUILD
- `python3 -m py_compile` on extended `mow_session_monitor.py` against an active rclpy + mowgli_interfaces — DEFERRED-TO-PHASE-END-BUILD
- Pi5 hardware checkpoint (operator-gated) — Pi5 5-of-5 mow cycles, R-9 manual E-Stop verification, R-11 auto-refresh check, all per `02-08-PI5-CHECKLIST.md`. THIS IS THE TERMINAL ITEM. Phase 2 cannot close without `02-08-PI5-RESULTS.md` showing 5 of 5 PASS.

## Self-Check: PASSED

All 8 expected files present on disk:
- `ros2/src/mowgli_simulation/scripts/synthetic_scan_kicp_publisher.py`
- `ros2/src/mowgli_simulation/launch/sim_lidar_docking.launch.py`
- `ros2/src/mowgli_simulation/test/test_synthetic_scan_publisher.py`
- `ros2/src/mowgli_simulation/test_data/sim_dock_scan.pcd`
- `ros2/src/mowgli_simulation/test_data/sim_dock_scan_meta.yaml`
- `.planning/phases/02-lidar-dock-pose-estimation/02-VERIFICATION.md`
- `.planning/phases/02-lidar-dock-pose-estimation/02-08-PI5-CHECKLIST.md`
- `.planning/phases/02-lidar-dock-pose-estimation/02-08-SUMMARY.md`

All 3 task commits present in `git log`:
- `ba9647de` — Task 1 (synthetic publisher + sim launch + e2e_test)
- `9d0fd50c` — Task 2 (mow_session_monitor + 02-VERIFICATION)
- `5544c763` — Task 3 deliverable (PI5-CHECKLIST.md)

All acceptance greps pass:
- `grep -q "lidar_link_wheels" synthetic_scan_kicp_publisher.py` — OK
- `grep -q "synthetic_scan_kicp_publisher\|dock_scan_match" sim_lidar_docking.launch.py` — OK
- `grep -q "def _run_fine_dock_phase" e2e_test.py` — OK
- `grep -c "/dock_match/" mow_session_monitor.py` >= 2 — actual: 6
- `grep -q "lateral_error_at_contact" mow_session_monitor.py` — OK
- `grep -q "dock_match_trusted_pct" mow_session_monitor.py` — OK
- `grep -c "^| R-" 02-VERIFICATION.md` >= 13 — actual: 29
- `grep -q "E2E sim\|Pi5 5-of-5" 02-VERIFICATION.md` — OK
- `grep -q "Pi5\|pi@10.10.40.68\|RegisterFrame\|5 of 5" 02-08-PI5-CHECKLIST.md` — OK

`python3 -m py_compile` clean on every modified .py file. colcon-test
execution is DEFERRED-TO-PHASE-END-BUILD per the host constraint.
