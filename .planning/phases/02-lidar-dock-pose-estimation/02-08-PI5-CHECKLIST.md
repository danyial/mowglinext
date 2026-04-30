---
phase: 02-lidar-dock-pose-estimation
plan: 08
task: 3
type: hardware-acceptance-runbook
operator_gated: true
created: 2026-04-30
mirrors: .planning/phases/01-coverage-planner-rewrite/01-09-SUMMARY.md "Hardware Checkpoint Procedure"
---

# Phase 2 — Pi5 Hardware Acceptance Runbook (Plan 02-08 Task 3)

> Operator-gated 5-of-5 acceptance test in the Eichenau garden under
> RTK-Fixed conditions. THIS is the SPEC R-7 / R-9 / R-11 gate that
> closes Phase 2. Execute on the Pi5 test bench (`pi@10.10.40.68` per
> `reference_pi5_testbench.md`) AFTER the merged feature branch is
> deployed. Mirrors Phase 1 SPEC AC-13 procedure pattern.
>
> Estimated time: 30–45 min for 5 cycles.

---

## Pre-Flight (before driving outdoors)

### 1. Branch + build verification

```bash
ssh pi@10.10.40.68
cd ~/mowglinext
# Pull the merged feature branch (Phase 2 lands on dev or main)
git checkout main && git pull
# Rebuild the workspace and confirm no build errors
make build 2>&1 | tail -20
# Expected: every package builds green; no `--- stderr` blocks for
# mowgli_lidar_docking, mowgli_behavior, mowgli_localization,
# mowgli_simulation.
```

### 2. Dock calibration files present

```bash
ls -la /ros2_ws/maps/dock_calibration.yaml \
       /ros2_ws/maps/dock_scan.pcd \
       /ros2_ws/maps/dock_scan_meta.yaml
# Expected: all three files exist. Plan 02-03 SPEC R-1 — the dock-yaw
# calibration GUI button produced them on first charge after the dock
# was first installed.
# If dock_scan.pcd is missing, run the calibration first via the GUI
# Calibration → "Magnetometer / Dock-yaw → Run" button.
```

### 3. /dock_match topics live within 30 s of system start

```bash
# Start the system (operator typically does this via systemd or
# `mowgli-main` alias — adjust to local convention).
# Within 30 s of first publish, check:
ros2 topic echo /dock_match/confidence -n 1 \
  | tee ~/phase2-acceptance/preflight-confidence.txt
# Expected output: trusted: true (or trusted: false with a clear reason
# in inlier_ratio + rmse_m). Either is acceptable at preflight; the
# 5-of-5 mow cycles below are the gate.
ros2 topic echo /dock_match/pose -n 1 --once \
  | tee ~/phase2-acceptance/preflight-pose.txt
# Expected: PoseWithCovarianceStamped with header.frame_id = "map".
```

### 4. RegisterFrame latency baseline (RESEARCH §Pitfall 5)

```bash
# Capture /dock_match/pose publication rate for 30 s.
timeout 30 ros2 topic hz /dock_match/pose 2>&1 \
  | tee ~/phase2-acceptance/preflight-rate.txt
# Expected: average rate >= 5 Hz (SPEC R-2 minimum). The matcher's
# default cadence is 10 Hz; rates < 5 Hz indicate a Pi5 scheduling
# issue or kinematic_icp registration bottleneck. Record the mean
# rate in 02-08-PI5-RESULTS.md preflight section.
```

### 5. Foxglove / GUI dock-card spot check

Open the GUI in a browser (`http://10.10.40.68:4006/` per local convention).
Navigate to the dashboard. Confirm the Dock-card shows:
- Status badge (green/yellow/red) reflecting `/dock_match/confidence.trusted`
- "Inlier X% / RMSE Y cm" pair updating at ~1 Hz
- "Recapture dock scan" button opens a confirm modal

This is the SPEC implicit (D-09, D-10, D-11) row in 02-VERIFICATION.md.

---

## 5-of-5 Mow Cycle Execution

For each of 5 consecutive cycles, follow the 5-step procedure below.
Cycle 3 ALSO injects the manual E-Stop (R-9 verification). Cycle N
chosen by the operator MUST be the first cycle after >= 7 days since
prior dock_scan.pcd capture (R-11 auto-refresh verification).

### Per-cycle setup (run in a separate SSH session)

```bash
ssh pi@10.10.40.68
mkdir -p ~/phase2-acceptance
SESSION_NAME="phase2-acceptance-cycle-$(printf %02d $CYCLE_N)-$(date +%Y%m%d-%H%M%S)"
docker exec -d mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  python3 /ros2_ws/scripts/mow_session_monitor.py \
    --session $SESSION_NAME \
    --output-dir /ros2_ws/maps
"
# Note: this writes the JSONL to the bind-mounted /ros2_ws/maps volume
# so it survives the container.
```

### Step 1 — Undock (verify R-5, R-12, R-13)

From the GUI, send `COMMAND_START`. In a separate terminal, watch
the BT log:

```bash
docker logs -f mowgli-ros2 2>&1 | \
  grep -E "PreUndockClearanceCheck|BackUp|CalibrateHeadingFromUndock|PostUndockRtkValidation|RecordDockApproachPose"
```

Confirm these log lines fire **in this order**:
1. `PreUndockClearanceCheck: rear clearance = X.XX m, OK` (R-12)
2. `BackUp: 1.5 m / 0.15 m/s …`
3. `CalibrateHeadingFromUndock: ...`
4. `PostUndockRtkValidation: discrepancy = X.XX m, OK` (R-13)
5. `RecordDockApproachPose: dock_approach.yaml mtime updated` (R-5)

Capture the dock_approach.yaml mtime for the cycle row:

```bash
stat -c '%y %n' /ros2_ws/maps/dock_approach.yaml \
  | tee -a ~/phase2-acceptance/cycle-$CYCLE_N.log
```

### Step 2 — Mow

Standard mow run; nothing dock-specific. Let it run until either
battery drops to the COMMAND_HOME threshold OR the operator triggers
COMMAND_HOME early (recommended for the test: 15 min mow per cycle is
plenty to validate the dock cycle without exhausting the battery).

### Step 3 — Approach (verify R-6)

Confirm BT enters `RETURNING_HOME`. Watch:

```bash
docker logs -f mowgli-ros2 2>&1 | \
  grep -E "ApproachDock|NavigateToPose|FineDock"
```

Confirm `ApproachDock` dispatches `NavigateToPose` to the recorded
`dock_approach.yaml` pose; robot arrives within 60 s with tolerance
≤ 10 cm. If it does NOT, mark cycle as FAIL and capture diagnostics
(plan length, distance-to-goal trace) in the cycle log.

### Step 4 — FineDock (verify R-7 — THE PRIMARY GATE)

Confirm `FineDock` crawls forward and the robot's `is_charging` flips
true on first attempt. From the JSONL produced by Step 0:

```bash
LAST_JSONL=$(ls -t /ros2_ws/maps/phase2-acceptance-cycle-${CYCLE_N}-*.jsonl | head -1)
# Show the lateral_error_at_contact:
jq 'select(.lateral_error_at_contact_m != null) | .lateral_error_at_contact_m' \
   "$LAST_JSONL"
# Expected: a SINGLE value (one rising-edge per cycle), <= 0.02 m.
# Capture into the cycle row.
```

**R-7 hard gate:** lateral_error_at_contact_m MUST be ≤ 0.02 m AND yaw
error ≤ 1°. The yaw error is computed from the latest `/dock_match/pose`
yaw at contact time — extract via:

```bash
jq 'select(.dock_match.pose.yaw_deg != null) | .dock_match.pose.yaw_deg' \
   "$LAST_JSONL" | tail -1
# Compare against dock_calibration.yaml dock_pose_yaw_rad (converted to
# degrees). Difference MUST be <= 1°.
```

### Step 5 — Wrap-up

Stop the session monitor (Ctrl-C in its SSH tab, OR
`docker exec mowgli-ros2 pkill -SIGINT -f mow_session_monitor`). The
JSONL gets a summary record on graceful shutdown.

```bash
# Pull the JSONL out of the container and commit it.
docker cp mowgli-ros2:$LAST_JSONL ~/phase2-acceptance/
cd ~/mowglinext
git add docker/logs/mow_sessions/phase2-acceptance-cycle-${CYCLE_N}-*.jsonl
git commit -m "chore(phase-2): commit Pi5 acceptance JSONL cycle $CYCLE_N"
```

### Cycle 3 — Additional R-9 manual E-Stop test

During Step 4 of cycle 3, while FineDock is crawling forward,
**operator manually presses the physical E-Stop** on the robot. Verify:

1. Motion stops within firmware deadline (perception: instantaneous; the
   spec is < 200 ms but no hardware-side timer is available).
2. BT enters `EMERGENCY` state.
3. After the operator releases the E-Stop and runs `Reset Emergency` from
   the GUI, the robot does **NOT** auto-restart docking. Operator must
   re-issue `COMMAND_HOME`; ApproachDock + FineDock cycle starts fresh.

This verifies the R-9 hardware gate. Capture in the cycle 3 row:

```text
R-9: motion stopped on E-Stop = YES/NO
R-9: BT entered EMERGENCY = YES/NO
R-9: did NOT auto-restart after Reset = YES/NO
```

### Auto-refresh check (R-11) — once across the 5 cycles

At least one cycle MUST be the first cycle after >= 7 days since
the prior dock_scan.pcd capture. Verify the auto-refresh fired:

```bash
stat -c '%y' /ros2_ws/maps/dock_scan.pcd
# Expected: mtime updated to today (or whenever this cycle started),
# i.e. the file was rewritten.
grep -A2 "auto-refresh" /ros2_ws/maps/dock_scan_meta.yaml
# Expected: captured_at updated; inlier_ratio at refresh >= 0.95.
```

If the auto-refresh did NOT fire on the >= 7-day cycle, mark R-11 as
FAIL and capture the dock_scan_meta.yaml content in the cycle row.

---

## Acceptance Criteria (all of the below MUST be true)

- [ ] **R-7:** 5 of 5 cycles MUST succeed at first attempt with
      `lateral_error_at_contact_m` ≤ 0.02 m AND yaw_error ≤ 1°
- [ ] **R-9:** Cycle 3 manual E-Stop verification passes (motion stops,
      BT enters EMERGENCY, no auto-restart after Reset Emergency)
- [ ] **R-11:** At least one cycle exercises the auto-refresh path
      (>= 7 days old + inlier ≥ 0.95 → PCD rewritten + meta updated)
- [ ] **R-2:** preflight `ros2 topic hz /dock_match/pose` mean rate ≥ 5 Hz
- [ ] **R-12 / R-13:** every cycle's UndockSequence log shows the 4 new
      lines in the right order (PreUndockClearance + BackUp +
      CalibrateHeadingFromUndock + PostUndockRtkValidation +
      RecordDockApproachPose)
- [ ] All 5 mow_session_monitor JSONLs committed to the repository

---

## Reporting

After all 5 cycles, the operator creates
`.planning/phases/02-lidar-dock-pose-estimation/02-08-PI5-RESULTS.md`
with one row per cycle:

```markdown
| Cycle | Timestamp | R-7 lateral (cm) | R-7 yaw (°) | R-12 OK | R-13 OK | R-5 OK | Auto-refresh | E-Stop test | Verdict |
|-------|-----------|------------------|-------------|---------|---------|--------|--------------|-------------|---------|
| 1     | YYYY-MM-DDTHH:MM | 1.4         | 0.4         | yes     | yes     | yes    | n/a          | n/a         | PASS    |
| 2     | ...              | ...         | ...         | ...     | ...     | ...    | ...          | ...         | ...     |
| 3     | ...              | ...         | ...         | ...     | ...     | ...    | ...          | yes (R-9)   | ...     |
| 4     | ...              | ...         | ...         | ...     | ...     | ...    | yes (R-11)   | ...         | ...     |
| 5     | ...              | ...         | ...         | ...     | ...     | ...    | ...          | ...         | ...     |

Overall: 5 of 5 PASS / X of 5 PASS / FAIL — <reasoning>
```

After committing 02-08-PI5-RESULTS.md, re-run from the dev container:

```bash
cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial
gsd-sdk query verify-phase 2  # or /gsd-verify-phase 2 in Claude Code
```

This flips the R-7 / R-9 / R-11 hardware rows in 02-VERIFICATION.md to
`✓ VERIFIED` and closes Phase 2.

---

## Failure Handling

If any cycle FAILS at first attempt:

1. **Do NOT retry on the same cycle row.** Record the failure verbatim,
   capture the relevant JSONL + BT logs, and continue to the next cycle.
2. After all 5 cycles, if 5 of 5 did NOT pass, Phase 2 closure is
   blocked. The operator commits 02-08-PI5-RESULTS.md with the failure
   detail and the development team:
   - Tunes `dock_scan_match.yaml` parameters (most likely
     `min_inlier_ratio`, `max_rmse_m`)
   - OR tunes FineDock control gains in the BT node
   - OR captures a fresh dock_scan.pcd if matcher confidence stayed
     low across all cycles
3. Re-run the full 5-of-5 procedure after tuning.

---

## Why this is operator-gated (Phase 2 close-out rationale)

This procedure cannot be executed by Claude Code: it requires
- Physical Pi5 + RTK base + outdoor garden
- Operator presence to inject manual E-Stop and visually verify the
  dock contact
- Multi-day calendar offset for the R-11 auto-refresh path

Per CLAUDE.md "Pi5 hardware test before PR" memory and Phase 1 Plan
01-09's automatable-vs-hardware split, this plan delivers all the
AUTOMATABLE scope; the Pi5 5-of-5 acceptance is operator-gated and
gates Phase 2 closure.
