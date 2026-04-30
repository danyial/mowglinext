---
phase: 02
plan: 07
subsystem: lidar-dock-pose-estimation
tags: [wave-4, bt-wiring, dock-robot-migration, gui-dock-card, spec-r-9, spec-r-10, spec-r-11, spec-r-12, spec-r-13, d-09, d-10, d-11, d-12]
requires:
  - mowgli_behavior::ApproachDock + FineDock + RecordDockApproachPose +
    PreUndockClearanceCheck + PostUndockRtkValidation (Plan 02-06 — registered
    via register_docking_nodes(factory))
  - mowgli_lidar_docking::DockScanMatchNode publishes /dock_match/pose +
    /dock_match/confidence (Plan 02-04 — runtime contract)
  - mowgli_interfaces::msg::DockMatchConfidence (Plan 02-01 D-03)
  - existing topicMap Go-relay pattern (gui/pkg/providers/ros.go +
    gui/pkg/api/mowglinext.go SubscriberRoute) — D-12 reconciliation lock
  - existing /api/calibration/imu-yaw HTTP endpoint (Plan 02-03 dock_scan_capture
    runs as a sub-step on the dock pre-phase)
provides:
  - main_tree.xml with zero `<DockRobot>` self-closing tags (R-10) — 6 sites
    migrated to `<Sequence><ApproachDock/><FineDock/></Sequence>`
  - UndockSequence extended with PreUndockClearanceCheck + PostUndockRtkValidation
    + RecordDockApproachPose (R-5, R-11, R-12, R-13)
  - topicMap Go-relay entries dockMatchPose + dockMatchConfidence
  - useDockMatch React hook (mirrors useMagYaw)
  - DockMatchCard component (D-09 / D-10 / D-11 visible to operator)
  - MowerStatus topbar AimOutlined Popover trigger surfacing DockMatchCard
affects:
  - Plan 02-08 (e2e + Pi5 hardware smoke) — main_tree.xml + GUI now drive
    the new closed-loop dock cycle on every operator-visible dock trigger
tech-stack:
  added: []
  patterns:
    - Named LidarDock* Sequence subtree per dock-trigger callsite — keeps BT
      logs diff-able (LidarDockCritical, LidarDockRain, LidarDockBattery,
      LidarDockFailedCoverage, LidarDockMowingComplete, LidarDockHome)
    - Go-relay topicMap (D-12 lock) mirrors useMagYaw / useDockingSensor —
      no new browser-side ros library introduced
    - Inline TS type redeclaration for DockMatchConfidence + DockMatchPose
      avoids hand-editing the hand-curated ros.ts AND the auto-generated
      ros.generated.ts (which is broken at HEAD per pre-existing Plan 02-01
      codegen bug, see deferred-items.md)
    - antd Popover trigger in MowerStatus topbar — operator can glance at
      pill colour and click to expand DockMatchCard with badge + numerics +
      Recapture button. D-09 "extends the existing dock area" honoured.
    - vi.mock('../hooks/useDockMatch.ts') stub in MowerStatus.test.tsx so
      unit tests don't open a real WebSocket
key-files:
  modified:
    - ros2/src/mowgli_behavior/trees/main_tree.xml
    - gui/pkg/providers/ros.go
    - gui/pkg/api/mowglinext.go
    - gui/web/src/components/MowerStatus.tsx
    - gui/web/src/components/MowerStatus.test.tsx
    - .planning/phases/02-lidar-dock-pose-estimation/deferred-items.md
  created:
    - gui/web/src/hooks/useDockMatch.ts
    - gui/web/src/components/DockMatchCard.tsx
decisions:
  - "R-9 OuterReactiveSequence: existing IsCommand-gated dock callsites +
    ClearCommand-after-dock pattern + EmergencyHandler-does-not-restore-command
    together prevent auto-restart. NO new XML guard added. Verified at plan-
    revision time against main_tree.xml line numbers 165, 170-180, 318, 510, 538."
  - "Each migrated callsite gets a unique LidarDock* Sequence name (Critical /
    Rain / Battery / FailedCoverage / MowingComplete / Home) so BT-log diffs
    surface which dock-trigger path fired. Single shared name would have made
    debugging harder."
  - "Recapture button reuses the existing /api/calibration/imu-yaw HTTP
    endpoint (Plan 02-03 contract: dock_scan_capture runs as a sub-step on
    the dock pre-phase). No new endpoint added — D-11 confirm modal goes
    through the same path the operator already uses for IMU yaw calibration."
  - "DockMatchCard surfaced via antd Popover triggered by an AimOutlined
    pill in MowerStatus topbar (next to the Charging indicator). Pure topbar
    Card render would have been visually overweight; Popover keeps the
    topbar compact while still giving D-09 the dock-area placement."
  - "DockMatchConfidence type redeclared inline in useDockMatch.ts.
    ros.generated.ts has the type (Plan 02-01 codegen) but ros.ts (the
    canonical hand-curated file consumed by sibling hooks) does not — and
    Plan 02-07 must NOT hand-edit either. Inline redeclaration is the
    cleanest path; reconciliation of ros.ts vs ros.generated.ts is
    out-of-scope (deferred-items.md)."
  - "Scan-age + last-fine-dock-lateral are placeholders with TODO(phase-3)
    markers per WARNING-4 disposition. Plan 02-08 captures the underlying
    data via mow_session_monitor JSONL but the Go endpoint to expose it
    to the GUI is intentionally not in Plan 02-08 files_modified."
metrics:
  duration_minutes: 18
  tasks_completed: 2
  files_touched: 8
  commits: 2
  completed_date: "2026-04-30"
---

# Phase 02 Plan 07: BT Wiring + GUI Dock-Card Extension Summary

**One-liner:** Wave 4 lands the operator-visible wiring step — all 6
`<DockRobot ...>` callsites in main_tree.xml migrated to
`<Sequence><ApproachDock/><FineDock/></Sequence>` (SPEC R-10), UndockSequence
extended with PreUndockClearanceCheck + PostUndockRtkValidation +
RecordDockApproachPose (R-5, R-11, R-12, R-13), and a GUI dock-card
extension surfacing live `/dock_match/{pose,confidence}` via the existing
topicMap Go-relay (D-09 / D-10 / D-11 / D-12). Phase 2 BT contract is now
end-to-end live; Plan 02-08 hardware smoke is the canonical hardware
verification gate.

## Built

This plan covers two atomic tasks landed in two commits on
`feat/mag-pipeline-resurrect`:

| Task | Step | Description | Commit |
| ---- | ---- | ----------- | ------ |
| 1 | feat | main_tree.xml: 6 DockRobot sites migrated + UndockSequence extended with 3 new BT nodes (R-9, R-10, R-12, R-13, R-5, R-11) | `0513f352` |
| 2 | feat | GUI dock-card extension: 2 topicMap entries, 2 dispatch cases, useDockMatch hook, DockMatchCard component, MowerStatus Popover trigger (D-09 / D-10 / D-11 / D-12) | `57d91972` |

## BT Tree Migration — before / after diagram

### Before (Plan 02-06 baseline)

```
MowgliMain
├── EmergencyGuard
├── BoundaryGuard
├── GPSModeSelector
└── MainLogic
    ├── CriticalBatteryDock (L164-180)
    │   └── CriticalNavOrStop
    │       └── <DockRobot dock_id="home_dock"/>     <-- 1
    ├── MowingSequence
    │   ├── UndockSequence (L245-296)
    │   │   ├── PreFlightCheck
    │   │   ├── RecordUndockStart
    │   │   ├── ClearCostmap
    │   │   ├── BackUp 1.5m
    │   │   ├── WaitForGpsFix
    │   │   ├── CalibrateHeadingFromUndock
    │   │   └── ClearCostmap
    │   ├── MowingCommandGuard
    │   │   ├── RainGuard
    │   │   │   └── RainDockAndResume
    │   │   │       └── RainNavOrStay
    │   │   │           └── <DockRobot/>             <-- 2
    │   │   ├── BatteryGuard
    │   │   │   └── BatteryDockAndResume
    │   │   │       └── <DockRobot/>                  <-- 3
    │   │   └── PlanAndMow (Plan 01-08 monolithic)
    │   ├── FailedCoverageDock
    │   │   └── FailedCoverageNavOrStop
    │   │       └── <DockRobot/>                      <-- 4
    │   └── MowingCompleteAutoDock
    │       └── DockOrAlreadyDocked
    │           ├── IsCharging
    │           └── <DockRobot/>                      <-- 5
    ├── HomeSequence
    │   └── HomeAction
    │       └── HomeOrAlreadyDocked
    │           ├── IsCharging
    │           └── HomeNavOrStop
    │               └── <DockRobot/>                  <-- 6
    ├── RecordingSequence (untouched)
    ├── ManualMowingSequence (untouched)
    └── IdleSequence (untouched)
```

### After (Plan 02-07 migrated)

```
MowgliMain
├── EmergencyGuard          (preserved verbatim — R-9 latch lives here)
├── BoundaryGuard           (preserved verbatim)
├── GPSModeSelector         (preserved verbatim)
└── MainLogic
    ├── CriticalBatteryDock
    │   └── CriticalNavOrStop
    │       └── Sequence "LidarDockCritical"             <-- migrated (1)
    │           ├── ApproachDock
    │           └── FineDock
    ├── MowingSequence
    │   ├── UndockSequence
    │   │   ├── PreFlightCheck
    │   │   ├── RecordUndockStart
    │   │   ├── ClearCostmap
    │   │   ├── PreUndockClearanceCheck   <-- NEW (R-12)
    │   │   ├── BackUp 1.5m
    │   │   ├── WaitForGpsFix
    │   │   ├── CalibrateHeadingFromUndock
    │   │   ├── PostUndockRtkValidation   <-- NEW (R-13)
    │   │   ├── ClearCostmap
    │   │   └── RecordDockApproachPose    <-- NEW (R-5 + R-11)
    │   ├── MowingCommandGuard
    │   │   ├── RainGuard
    │   │   │   └── RainDockAndResume
    │   │   │       └── RainNavOrStay
    │   │   │           └── Sequence "LidarDockRain"     <-- migrated (2)
    │   │   │               ├── ApproachDock
    │   │   │               └── FineDock
    │   │   ├── BatteryGuard
    │   │   │   └── BatteryDockAndResume
    │   │   │       └── Sequence "LidarDockBattery"      <-- migrated (3)
    │   │   │           ├── ApproachDock
    │   │   │           └── FineDock
    │   │   └── PlanAndMow                                (unchanged)
    │   ├── FailedCoverageDock
    │   │   └── FailedCoverageNavOrStop
    │   │       └── Sequence "LidarDockFailedCoverage"   <-- migrated (4)
    │   │           ├── ApproachDock
    │   │           └── FineDock
    │   └── MowingCompleteAutoDock
    │       └── DockOrAlreadyDocked
    │           ├── IsCharging
    │           └── Sequence "LidarDockMowingComplete"   <-- migrated (5)
    │               ├── ApproachDock
    │               └── FineDock
    ├── HomeSequence
    │   └── HomeAction
    │       └── HomeOrAlreadyDocked
    │           ├── IsCharging
    │           └── HomeNavOrStop
    │               └── Sequence "LidarDockHome"         <-- migrated (6)
    │                   ├── ApproachDock
    │                   └── FineDock
    ├── RecordingSequence    (preserved verbatim)
    ├── ManualMowingSequence (preserved verbatim)
    └── IdleSequence         (preserved verbatim)
```

### Acceptance verification

```
DockRobot self-closing tags: 0          (R-10 hard gate)
DockRobot total mentions:    8          (8 historical XML comments preserved as documentation)
ApproachDock:                6          (one per migrated site)
FineDock:                    6
PreUndockClearanceCheck:     1          (in UndockSequence, before BackUp)
PostUndockRtkValidation:     1          (after CalibrateHeadingFromUndock)
RecordDockApproachPose:      1          (at end of UndockSequence)
IsCommand command="1":       2          (>= 1 — MowingCommandGuard preserved)
IsCommand command="2":       3          (>= 1 — HomeSequence preserved)
ClearCommand:               10          (>= 6 — terminal-dock + fail/timeout pattern preserved)
```

XML syntax verified host-side via `python -c "xml.etree.ElementTree.parse(...)"`
(colcon build deferred to phase-end podman build per host_environment_constraint).

### R-9 OuterReactiveSequence verdict (locked — no new guard)

NO new BT XML element introduced for R-9. Existing latches verified
sufficient at plan-revision time:

| Line | Wrapper                          | Precondition gating                     | E-Stop re-entry possible? |
|------|----------------------------------|------------------------------------------|---------------------------|
| 171  | LidarDockCritical                | IsBatteryLow threshold="10.0" (L165)     | Only on persistent battery <10% (independent of E-Stop, by design) |
| 332  | LidarDockRain                    | IsCommand command="1" (L318)             | NO — operator must re-issue COMMAND_START |
| 378  | LidarDockBattery                 | IsCommand command="1" (L318)             | NO — same |
| 445  | LidarDockFailedCoverage          | IsCommand command="1" (L318)             | NO — same |
| 491  | LidarDockMowingComplete          | IsCommand command="1" (L186) + ClearCommand (L494) | NO — command cleared after dock |
| 529  | LidarDockHome                    | IsCommand command="2" (L510) + ClearCommand (L538) | NO — command cleared after dock |

**Key invariant:** EmergencyHandler at line 34 does NOT call `ClearCommand`,
but it also does NOT re-set the command. After ResetEmergency clears the
firmware latch, control returns to MainLogic Fallback. Each dock callsite
re-evaluates its `IsCommand`/`IsBatteryLow` precondition. For COMMAND_HOME /
COMMAND_START paths, the operator must explicitly re-issue the command —
`ClearCommand` is fired at the end of the previous dock cycle (lines 179,
494, 538) precisely so the next entry requires explicit operator intent.
For CriticalBatteryDock, re-entry on persistent low battery is the desired
safety behavior (independent of E-Stop), so no guard is needed there either.

The Plan 02-06 unit test `FineDockHaltsOnEmergency` (test case 9) verifies
the in-node `onHalted publish_zero` contract. This plan adds no XML guard.

## GUI Dock-Card before / after — semantic description

### Before (Plan 02-06 baseline)

The GUI's MowerStatus topbar showed: state-name pill (with motion/resting/
emergency colour) + GPS quality % + Battery % (with charging power-icon
trigger to a Dropdown menu of Restart/Reboot/Shutdown). No LiDAR dock-match
telemetry surfaced anywhere; operators had to open a ROS terminal and run
`ros2 topic echo /dock_match/confidence` to know whether the next dock
would be reliable.

### After (Plan 02-07)

The same topbar gains an `AimOutlined` (target-icon) Popover trigger
between the GPS pill and the Battery dropdown. The icon colour mirrors the
DockMatchCard badge thresholds:

* `colors.primary` (green-tone) — `confidence.trusted == true` and
  `Date.now() - lastMessageAt <= 5 s`
* `colors.danger` (red-tone) — confidence received but `trusted == false`
* `colors.muted` (grey) — no confidence message yet OR last message older
  than 5 s (matcher down / `use_lidar:=false` / topic absent)

Click opens DockMatchCard inside the Popover (max-width 320 px):

```
┌─ LiDAR Dock Match ─────────────── [Recapture] ─┐
│                                                │
│ ● success   trusted                            │   <-- antd Badge with status
│                                                │       (green=trusted,
│ Inlier 78% / RMSE 3.2 cm                       │        yellow=borderline,
│                                                │        red=degraded,
│ Scan age: —                                    │        grey=stale/no-data)
│ Last fine-dock lateral: —                      │
│                                                │
└────────────────────────────────────────────────┘
```

* **Status badge** (D-10 thresholds):
  - GREEN: `trusted == true` (Plan 02-04 R-3 already factors `inlier ≥ 0.70`
    AND `rmse ≤ 0.05` upstream)
  - YELLOW (`borderline`): `0.60 ≤ inlier < 0.70` OR `0.05 < rmse ≤ 0.10`
    (border zone — pose not published but conf live)
  - RED (`degraded`): `inlier < 0.60` OR `rmse > 0.10` OR explicitly
    `trusted == false`
  - GRAY (`no data` / `stale`): no confidence ever OR last message > 5 s
* **Numeric pair**: `Inlier 78% / RMSE 3.2 cm` formatted as
  `${(inlier_ratio * 100).toFixed(0)}%` and `${(rmse_m * 100).toFixed(1)} cm`.
* **Scan age** + **Last fine-dock lateral**: placeholder dashes with
  `TODO(phase-3)` code comments. Per WARNING-4 disposition (locked option
  (b) in PLAN.md): the data exists (Plan 02-08 captures lateral_error_at_contact
  in mow_session_monitor JSONL; dock_scan_meta.captured_at is on disk via
  Plan 02-02), but the GUI Go endpoints to expose them are intentionally
  out-of-scope for Phase 2.
* **Recapture button** (D-11 confirm modal):
  ```
  Title:   Dock-Scan neu aufnehmen?
  Body:    Capture wird den aktuellen LiDAR-Snapshot als neuen
           dock_scan.pcd speichern. Roboter sollte sicher auf dem
           Dock sitzen. Fortfahren?
  OK:      Ja, neu aufnehmen
  Cancel:  Abbrechen
  ```
  On confirm, POSTs `/api/calibration/imu-yaw` (the existing endpoint —
  same path as the IMU-yaw calibration drive). The dock_scan_capture
  sub-step writes the new `dock_scan.pcd`; DockScanMatchNode's mtime
  watcher hot-reloads within ~100 ms (Plan 02-04 contract). Notification
  surfaces success/failure.

### Topic data path (D-12 reconciliation)

```
DockScanMatchNode (rclcpp)
   ├── publishes /dock_match/pose         (PoseWithCovarianceStamped, reliable depth=1)
   └── publishes /dock_match/confidence   (DockMatchConfidence, SensorDataQoS @ ~10 Hz)
        │
        ▼
foxglove_bridge (Cyclone DDS <-> WebSocket)
        │
        ▼
RosProvider.initFoxgloveSubscriptions  (gui/pkg/providers/ros.go)
   uses topicMap["dockMatchPose"|"dockMatchConfidence"]
        │
        ▼
SubscriberRoute "/api/mowglinext/subscribe/{key}"  (gui/pkg/api/mowglinext.go)
   200 ms debounce per RosSubscriber, base64-encoded WebSocket frames
        │
        ▼
useDockMatch hook (gui/web/src/hooks/useDockMatch.ts)
   poseStream + confStream via useWS()
        │
        ▼
DockMatchCard (gui/web/src/components/DockMatchCard.tsx)
   + MowerStatus Popover trigger (gui/web/src/components/MowerStatus.tsx)
```

No browser-side rosbridge connection (D-12 lock honoured): `package.json`
diff-check confirms no new `@foxglove/ws-protocol` or `roslib` entries.

## Architecture Invariant compliance

| Invariant | Compliance |
|-----------|------------|
| AI #1 (single localizer) | No new TF broadcast. Plan 02-07 only WIRES the Plan 02-06 nodes; their AI #1 compliance was verified at Plan 02-06 commit time. |
| AI #10 (no UndockRobot reintroduction) | UndockSequence still uses `<BackUp ...>` (Nav2 behavior_server). Plan 02-07 does NOT touch that node. |
| AI #13 (cmd_vel routing) | FineDock publishes only to `/cmd_vel_docking` (Plan 02-06 verified). Plan 02-07 just wires it in. |
| AI #15 (footprint sync) | No footprint config touched. PreUndockClearanceCheck reads `min_undock_distance_m` from BT port (default 1.5 m) — a distance, not a footprint dimension. |

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 — Plan-vs-Codebase] Plan body said R-9 acceptance grep is
`>= 6 ClearCommand`; actual file has 10**

- **Found during:** Task 1 acceptance verification
- **Issue:** Plan body's R-9 acceptance criterion lists
  `grep -c "ClearCommand" main_tree.xml >= 6`. The actual file already had
  ~10 `ClearCommand` calls (each fail/timeout path also clears, plus
  RecordingSequence/MowingSequence/HomeSequence terminals).
- **Fix:** No code change. Acceptance verbiage interpreted as "at least 6"
  per the `>=` operator. The 10 calls preserve the ClearCommand-after-dock
  pattern across every terminal path.
- **Files modified:** none (advisory)
- **Commit:** n/a

**2. [Rule 3 — Plan-vs-Codebase] DockMatchCard render in MowerStatus.tsx —
plan body says "near the Charging indicator"; topbar is too compact for an
inline antd Card**

- **Found during:** Task 2 (writing MowerStatus.tsx integration)
- **Issue:** MowerStatus is a topbar status row (compact pills + dropdown
  trigger), not a card-stack panel. Inlining a full antd Card would have
  blown out the topbar height and broken the responsive layout.
- **Fix:** Render DockMatchCard inside an antd `Popover` triggered by an
  `AimOutlined` icon next to the Charging indicator. Pill colour gives
  at-a-glance status (D-10 thresholds); click expands the full card.
  Plan acceptance `grep -q "DockMatchCard" gui/web/src/components/MowerStatus.tsx`
  is satisfied by the import + render.
- **Files modified:** `gui/web/src/components/MowerStatus.tsx`
- **Commit:** `57d91972`

**3. [Rule 3 — Plan-vs-Codebase] DockMatchConfidence type only in
`ros.generated.ts`, not in `ros.ts`**

- **Found during:** Task 2 (writing useDockMatch.ts)
- **Issue:** Plan body says "Plan 02-01 produced auto-generated TS bindings:
  `gui/web/src/types/ros.ts` has `DockMatchConfidence` type". On disk,
  Plan 02-01 codegen wrote `ros.generated.ts` (which has the type) but
  `ros.ts` (the canonical hand-curated file consumed by sibling hooks like
  useMagYaw) was not resynced. host_environment_constraint also says
  "DO NOT hand-edit them". Plus `ros.generated.ts` is broken at HEAD
  (MapAreaConstants enum self-reference, lines 281-283 — pre-existing
  Plan 02-01 codegen bug, see deferred-items.md).
- **Fix:** Inline-redeclared `DockMatchConfidence` + `DockMatchPose` types
  inside `useDockMatch.ts`. Avoids hand-editing both files; once `ros.ts`
  is regenerated from a fixed `generate_ts_types.sh`, the inline types
  can be replaced with imports from `ros.ts`.
- **Files modified:** `gui/web/src/hooks/useDockMatch.ts`
- **Commit:** `57d91972`

**4. [Rule 3 — Plan-vs-Codebase] MowerStatus.test.tsx had no useDockMatch
mock — would crash on real WebSocket open**

- **Found during:** Task 2 (verifying tests still run)
- **Issue:** Existing `MowerStatus.test.tsx` mocks `useHighLevelStatus` only.
  Adding `useDockMatch` to MowerStatus would break tests because
  `useWS` opens a real WebSocket on mount.
- **Fix:** Added `vi.mock('../hooks/useDockMatch.ts', ...)` returning a
  null-confidence stub. Pre-existing `useSettings` / `useConfig` test
  failures are unchanged (not introduced by this plan; verified by
  `git stash; vitest run; git stash pop`).
- **Files modified:** `gui/web/src/components/MowerStatus.test.tsx`
- **Commit:** `57d91972`

**5. [Rule 3 — SCOPE BOUNDARY (deferred, NOT auto-fixed)] `yarn build`
fails on pre-existing `ros.generated.ts` bug**

- **Found during:** Task 2 verify step
- **Issue:** `cd gui/web && yarn build` fails with TS2474 / TS2565 on
  `ros.generated.ts` lines 281-283 (`MapAreaConstants` enum members
  initialised to themselves). This is a pre-existing bug from Plan 02-01
  codegen (`gui/generate_ts_types.sh` emits invalid TS for the const enum
  pattern). The file is NOT imported anywhere in `src/`, but
  `tsconfig.json:include = ["src"]` makes `tsc` compile it.
- **Decision:** Per `<deviation_rules>` SCOPE BOUNDARY — out-of-scope,
  pre-existing, not directly caused by Plan 02-07. Documented in
  `deferred-items.md` "From Plan 02-07" with two suggested fixes
  (regenerate `ros.generated.ts` after fixing the generator, OR exclude
  it from tsconfig).
- **Verified-in-isolation:** `tsc --noEmit` with `ros.generated.ts`
  excluded passes cleanly — Plan 02-07's TS files compile in isolation.
  `cd gui && go build ./...` exits 0.
- **Files modified:** `.planning/phases/02-lidar-dock-pose-estimation/deferred-items.md`
- **Commit:** `57d91972`

## Coordination Risks for Plan 02-08

### main_tree.xml end-to-end shape is now production-ready for sim

Plan 02-08's e2e_test.py + sim_full_system.launch.py must:

1. Spawn DockScanMatchNode (Plan 02-04 already wires it under
   `IfCondition(use_lidar)` in `navigation.launch.py`).
2. Have a synthetic `/scan_kicp` publisher emit a known dock-shaped scan
   so DockScanMatchNode publishes `trusted=true` /dock_match/* messages.
3. Drive the BT through one full undock + mow + dock cycle and assert:
   - PreUndockClearanceCheck passes (synthetic scan has clear rear sector)
   - PostUndockRtkValidation passes (synthetic GPS == EKF)
   - RecordDockApproachPose writes `dock_approach.yaml` (atomic check)
   - ApproachDock dispatches NavigateToPose with the recorded waypoint
   - FineDock crawls toward the dock_calibration target until is_charging
   - Total cycle completes within the e2e_test timeout

### GUI hardware checkpoint shape

Pi5 operator should verify after deploying this branch:

1. Topbar shows `AimOutlined` icon next to the Charging indicator.
2. Pill colour is GREY when `use_lidar:=false` or DockScanMatchNode is
   absent — verify by checking `journalctl -u mowgli-ros2 | grep dock_scan_match`.
3. Pill colour flips to GREEN when robot is on dock, RTK-Fixed, and
   DockScanMatchNode publishes `trusted=true` (visible in
   `ros2 topic echo /dock_match/confidence`).
4. Click pill → DockMatchCard opens with current Inlier % / RMSE cm.
5. Click Recapture → confirm modal appears with the German text → on
   confirm, calibration drive starts (hardware-bridge mode flips to
   CALIBRATING_HEADING in the GUI status pill).

### Phase-end podman build expected to surface ros.generated.ts bug

When the orchestrator runs `colcon build --packages-select mowgli_behavior`
inside the devcontainer, that step is independent of `yarn build` and will
succeed (BT XML changes only). The GUI build step (`cd gui/web && yarn
build`) will fail with the pre-existing `ros.generated.ts` errors —
deferred-items.md tracks the fix.

### `dock_match_max_age_s` per-consumer (recap from Plan 02-04, 02-05, 02-06)

Plan 02-05 seeder declares `dock_match_max_age_s = 1.0` s. Plan 02-06
FineDock declares `confidence_loss_timeout_s = 1.0` s. Plan 02-07 GUI
DockMatchCard's stale threshold is `5_000` ms (5 s) — DELIBERATELY longer
because the GUI is non-control-loop; an operator wants to see the badge
"stale" only if the matcher has been silent for several seconds. NOT a
single shared param.

## Threat Flags

None. The new wiring mitigates the threats called out in the plan's
`<threat_model>`:

- **T-07-01 (Bad XML edit):** `python xml.etree.ElementTree.parse` host-side
  + acceptance greps. Phase-end podman colcon build is the canonical gate.
- **T-07-02 (Migration misses a DockRobot site):** `grep -c "<DockRobot " = 0`
  hard gate. Verified.
- **T-07-03 (E-Stop reset auto-restarts FineDock):** R-9 verdict locked —
  existing IsCommand-gated callsites + ClearCommand pattern + EmergencyHandler-
  does-not-restore-command together prevent auto-restart. Operator must
  explicitly re-issue COMMAND_HOME / COMMAND_START.
- **T-07-04 (Operator misclicks Recapture):** D-11 confirm modal + capture
  itself gated on is_charging + stationarity (Plan 02-03).
- **T-07-05 (Spoofed /dock_match/confidence):** ACCEPTED — same trust as
  /odometry/filtered_map (operator-owned local network).
- **T-07-06 (Confidence values via topicMap):** ACCEPTED — same trust.
- **T-07-07 (UndockSequence wrong order):** acceptance grep verifies the
  three new nodes are present; line-order verified by reading the diff.

No new trust boundaries introduced. The DockMatchCard Recapture button
reuses the existing `/api/calibration/imu-yaw` endpoint — same trust
boundary as the existing IMU-yaw calibration button on DiagnosticsPage.

## Deferred verify steps

The orchestrator runs the actual ROS2 build at end-of-phase via podman
inside the devcontainer. The host (macOS) cannot run colcon. The GUI
`yarn build` step is also blocked by a pre-existing bug in
`ros.generated.ts` (see deferred-items.md). Each command below is the
verbatim verify step the plan specified that this executor could not run.

- **Task 1 — colcon build for the BT migration**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon build --packages-select mowgli_behavior \
      --event-handlers console_cohesion+ 2>&1 | tail -10
  ```

  Expected: exit 0. The behavior_tree_node executable picks up the new
  XML at runtime (no compile-time XML embedding); the build step exercises
  the C++ side of Plan 02-06's nodes only.

- **Task 1 — XML factory parse smoke**

  ```bash
  source /opt/ros/kilted/setup.bash && source install/setup.bash && \
  timeout 5 ros2 run mowgli_behavior behavior_tree_node 2>&1 | head -20
  ```

  Expected: no `BT.CPP` parse error in the first 3 seconds of stderr.
  Any error like `[bt_factory] line X column Y unknown node-type` would
  indicate the factory registrations from Plan 02-06 don't match the
  XML node names — but the test names from Plan 02-06 already pin the
  factory registration set (RecordDockApproachPose, ApproachDock, FineDock,
  PreUndockClearanceCheck, PostUndockRtkValidation), and Plan 02-07 uses
  the exact same names verbatim.

- **Task 1 — colcon test (regression)**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon test --packages-select mowgli_behavior \
      --ctest-args -E docking_nodes \
      --event-handlers console_cohesion+ 2>&1 | tail -15
  ```

  Expected: prior test count, 0 failed (regression coverage for
  non-docking BT nodes).

- **Task 1 — Plan 02-06 docking_nodes tests still pass**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon test --packages-select mowgli_behavior \
      --ctest-args -R docking_nodes \
      --event-handlers console_cohesion+ 2>&1 | tail -15
  ```

  Expected: 13 gtest cases passing, 0 failed.

- **Task 2 — yarn build (BLOCKED by pre-existing `ros.generated.ts` bug)**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/gui/web && yarn build
  ```

  Currently fails with TS2474 / TS2565 in `ros.generated.ts` lines 281-283.
  See `deferred-items.md` "From Plan 02-07" for two suggested fixes
  (regenerate after fixing the codegen, OR exclude from tsconfig).
  **Plan 02-07 files compile cleanly when `ros.generated.ts` is excluded
  from tsc (verified host-side).**

- **Task 2 — go build (PASSED host-side)**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/gui && go build ./...
  ```

  Result: exit 0 — Go side of the topicMap + dispatch case extension
  compiles cleanly.

- **Phase-end smoke (post-build, container running): live topic confirmation**

  ```bash
  source /opt/ros/kilted/setup.bash && source install/setup.bash && \
  ros2 launch mowgli_bringup full_system.launch.py use_lidar:=true &
  sleep 60 && \
  ros2 topic list | grep "/dock_match/" && \
  ros2 topic echo /dock_match/confidence --once
  ```

  Expected: both topics listed; one DockMatchConfidence message printed
  (trusted=false on cold boot, transitions to trusted=true once robot is
  on dock + RTK-Fixed + dock_scan.pcd present).

- **Pi5 hardware checkpoint (Plan 02-08 territory)**

  Per CLAUDE.md memory entry "Pi5 test before PR": every change must
  verify on the Pi5 test bench before a PR is opened. Plan 02-08
  hardware smoke is the canonical place — full undock + mow + dock cycle
  with the new BT subtree active. The 5-of-5 acceptance gate documented
  in 02-CONTEXT.md must pass before this branch is mergeable to main.

## Drift detection

| Check | Expected | Actual |
|-------|----------|--------|
| `<DockRobot ` self-closing tags in main_tree.xml | 0 | green (0) |
| `DockRobot` total mentions (incl. comments) | >= 6 (8 historical + 0 live) | green (8) |
| `ApproachDock` count | 6 | green (6) |
| `FineDock` count | 6 | green (6) |
| `PreUndockClearanceCheck` count | 1 | green (1) |
| `PostUndockRtkValidation` count | 1 | green (1) |
| `RecordDockApproachPose` count | 1 | green (1) |
| `IsCommand command="1"` count | >= 1 | green (2) |
| `IsCommand command="2"` count | >= 1 | green (3) |
| `ClearCommand` count | >= 6 | green (10) |
| topicMap `dockMatchPose|dockMatchConfidence` in ros.go | >= 2 | green (2) |
| Dispatch cases in mowglinext.go | >= 2 | green (2) |
| `export const useDockMatch` in useDockMatch.ts | match | green |
| `export.*DockMatchCard` in DockMatchCard.tsx | match | green |
| `Inlier`, `RMSE`, `trusted` literal in DockMatchCard.tsx | all 3 match | green |
| `Capture wird den aktuellen LiDAR-Snapshot` in DockMatchCard.tsx | match | green |
| `DockMatchCard` import + render in MowerStatus.tsx | match | green |
| No new browser-side ros lib in package.json | no diff | green |
| XML parses (host-side python ET) | exit 0 | green |
| `cd gui && go build ./...` | exit 0 | green |
| `cd gui/web && tsc --noEmit` (with ros.generated.ts excluded) | exit 0 | green |

## Self-Check

Verifying claims before STATE.md / ROADMAP.md updates.

### Files claimed exist

```
FOUND: ros2/src/mowgli_behavior/trees/main_tree.xml (modified)
FOUND: gui/pkg/providers/ros.go (modified)
FOUND: gui/pkg/api/mowglinext.go (modified)
FOUND: gui/web/src/hooks/useDockMatch.ts (new)
FOUND: gui/web/src/components/DockMatchCard.tsx (new)
FOUND: gui/web/src/components/MowerStatus.tsx (modified)
FOUND: gui/web/src/components/MowerStatus.test.tsx (modified)
FOUND: .planning/phases/02-lidar-dock-pose-estimation/deferred-items.md (modified)
```

### Commits claimed exist

```
FOUND: 0513f352 — Task 1 (BT migration)
FOUND: 57d91972 — Task 2 (GUI dock-card extension)
```

## Self-Check: PASSED
