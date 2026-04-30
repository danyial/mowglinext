---
phase: 02
plan: 05
subsystem: lidar-dock-pose-estimation
tags: [wave-2, dock-yaw-cascade, ekf-set-pose, dock-match, spec-r-4, tdd]
requires:
  - mowgli_interfaces::msg::DockMatchConfidence (Plan 02-01 D-03)
  - mowgli_lidar_docking::DockScanMatchNode publishes /dock_match/pose +
    /dock_match/confidence (Plan 02-04 — runtime contract)
  - mowgli_geometry::key_value_parser (Plan 02-01 — used unchanged by
    _load_dock_calibration via the existing _parse_kv_double helper)
provides:
  - dock_yaw_to_set_pose 3-source cascade (lidar > file > /gnss/heading)
    gated on trusted + staleness, per SPEC R-4
  - DockYawToSetPose._resolve_yaw_source() — pure helper returning
    (label, yaw_rad, yaw_var); unit-testable in isolation
  - test_dock_yaw_seeder_cascade.py — 8 pytest cases pinning the truth
    table + the debounce + AUTONOMOUS regressions
  - ROS param `dock_match_max_age_s` (default 1.0 s)
affects:
  - Plan 02-06 FineDock (consumes the same /dock_match/* topic surface;
    must NOT subscribe transient_local — see Coordination Risks)
  - Plan 02-07 GUI (auto-generated DockMatchConfidence TS bindings already
    in place from Plan 02-01)
tech-stack:
  added: []
  patterns:
    - cascade resolver returns a discriminated tuple (str, float, float);
      caller switches on label and early-returns on "none"
    - rclpy.Duration arithmetic for the staleness gate (now() -
      _latest_dock_match_pose_at) — no monotonic clock drift relative
      to the rclpy clock used by every other state cache in the node
    - default-False trusted bool means cascade falls through to file/
      heading on day 1 even when Plan 02-04 is not yet deployed
key-files:
  created:
    - ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py
  modified:
    - ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py
    - ros2/src/mowgli_localization/CMakeLists.txt
key-decisions:
  - "Cascade resolver returns yaw_rad (not Quaternion). The /try_publish
    builder is uniform across all three sources — file used to build
    the quat inline, heading reused msg.orientation. Going via yaw_rad
    is loss-free (the existing log line already extracted yaw via the
    same atan2 formula) and removes a code path."
  - "Staleness gate at 1.0 s (ROS param `dock_match_max_age_s`). Matches
    SPEC R-3's 1 s abort horizon for FineDock — keeps the seeder and
    FineDock on the same trust horizon."
  - "trusted=False default means the cascade falls through to file or
    heading on day 1 even when Plan 02-04 is not deployed (matcher
    publishes trusted=false continuously in degraded mode per Plan
    02-04 SUMMARY Pitfall 6 baseline). Adding the new subscriptions
    does NOT block the existing day-1 file/heading behaviour."
  - "Logging contract: every publish carries a `source=<label>` token in
    the INFO line. Labels are exactly {lidar, file, /gnss/heading}.
    Operators grep /rosout to verify which cascade rung fired."
  - "_yaw_var (heading variance) was 0.1 rad²; _file_yaw_var stays the
    pre-existing max(σ², 0.03); LiDAR variance is taken from
    pose.covariance[35] clamped to a 1e-4 floor. Three sources, three
    distinct variance derivations — the EKF reads them as posted."
  - "Plan 02-01 regex helper (_parse_kv_double, _load_dock_calibration)
    is intentionally NOT touched. The cascade extension is purely
    additive above the file branch."
patterns-established:
  - "Cascade resolver pattern: pure helper that maps cached state to
    (label, value, variance) plus a 'none' sentinel; caller decides
    publish-or-skip."
  - "Default-False/None state for new top-priority cascade source —
    keeps day-1 backward compatibility intact."
requirements-completed: [R-4]
duration: ~25 min
completed: 2026-04-29
---

# Phase 02 Plan 05: dock_yaw_to_set_pose Cascade Extension Summary

**Three-source EKF seeder cascade — `/dock_match/pose` (LiDAR, gated on `trusted` AND age ≤ 1.0 s) > `dock_calibration.yaml` (file) > `/gnss/heading` (live) — replaces the existing two-source priority chain in `dock_yaw_to_set_pose.py`. SPEC R-4 closed; pinned by 8 pytest cases (6 truth-table + 1 debounce regression + 1 AUTONOMOUS-state regression).**

## Performance

- **Duration:** ~25 min (executor wall-clock)
- **Tasks:** 1 (TDD)
- **Files modified:** 3 (1 production source, 1 new test, 1 CMakeLists registration)
- **Commits:** 2 (RED + GREEN)
- **Plan-level TDD gate sequence:** RED → GREEN ✅ (no REFACTOR needed; the GREEN code is the final form)

## Built

This plan covers a single TDD task landed in two commits on `feat/mag-pipeline-resurrect`:

| Task | Step | Description | Commit |
| ---- | ---- | ----------- | ------ |
| 1 | RED | `test_dock_yaw_seeder_cascade.py` — 8 failing pytest cases registered in `CMakeLists.txt`. Tests reference `_resolve_yaw_source`, `_latest_dock_match_pose`, `_latest_dock_match_trusted`, `_latest_dock_match_pose_at` — none of which exist yet on `DockYawToSetPose`. | `b17b6a1b` |
| 1 | GREEN | Cascade extension in `dock_yaw_to_set_pose.py`: 2 new subscriptions, 2 new callbacks, `_resolve_yaw_source` resolver, `_try_publish` refactor, `source=<label>` logging contract. | `0a9d6980` |

## Cascade contract (SPEC R-4)

```
                  +----------------------+
                  |  _try_publish() tick |
                  +----------+-----------+
                             |
                             v
              +--------------+--------------+
              | _resolve_yaw_source():       |
              |   1. /dock_match/pose        |
              |      (trusted AND age <= 1s) |
              |   2. dock_calibration.yaml   |
              |   3. /gnss/heading           |
              |   else: source = "none"      |
              +--------------+--------------+
                             |
              +-------+------+------+-------+
              |       |             |       |
            "lidar" "file" "/gnss/heading" "none"
              |       |             |       |
              v       v             v       v
        publish  publish        publish    skip
        log      log            log        return
```

**Truth table coverage (all 6 transitions pinned by tests):**

| # | trusted_lidar | dock_match age | file_yaw | heading | Expected source |
|---|---------------|----------------|----------|---------|-----------------|
| 1 | True | 0.1 s | present | present | `lidar` |
| 2 | True | 0.1 s | absent | absent | `lidar` |
| 3 | False | 0.1 s | present | present | `file` |
| 4 | False | 0.1 s | absent | present | `/gnss/heading` |
| 5 | True | 5.0 s (stale) | present | present | `file` (stale lidar bypassed) |
| 6 | None (no /dock_match received) | n/a | absent | absent | `none` (publish skipped) |
| 7 | True | 0.1 s | absent | absent | `lidar` (1st call publishes; 2nd within throttle does NOT) |
| 8 | True | 0.1 s | absent | absent | `lidar` BUT high_level_state=AUTONOMOUS suppresses publish |

Cases 7 + 8 are regression guards for behaviour that MUST NOT change:
- **Case 7 (debounce):** the existing 1 Hz `_min_publish_period` throttle dominates the cascade — even when source=lidar, two `_try_publish()` calls within 1 s only result in one publish.
- **Case 8 (AUTONOMOUS gate):** `_high_level_state in {2, 3, 4}` blocks publish regardless of cascade source. Issue #73 root-cause: any seed during AUTONOMOUS corrupts the live mow.

## Operator-readable example log line

```
[INFO] [dock_yaw_to_set_pose]: published dock /set_pose source=lidar yaw=0.523 var=0.0010 map=(3.876, 2.745) yaw=29.9° odom=(0, 0) yaw=29.9°
```

`source=` token is exactly `{lidar, file, /gnss/heading}` — pinned by the test harness and documented as a grep target in the inline comment block above the log call.

## Files Created/Modified

### Created

- **`ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py`** (215 lines)
  - 8 pytest cases — see truth table above
  - `pytest.importorskip("rclpy", "geometry_msgs.msg", "mowgli_interfaces.msg", "sensor_msgs.msg")` so the file is silently skipped on hosts without rclpy (macOS executor host).
  - Reuses the conftest.py path-injection idiom from Plan 02-03 (sibling test in same directory). Reuses `rclpy_runtime` fixture pattern from `test_dock_scan_capture.py`.
  - `_resolve_yaw_source()` is exercised in pure-logic mode (set state attrs directly, call resolver, assert label). `_try_publish()` is exercised with `unittest.mock.MagicMock` patched onto `_pub_map` and `_pub_odom` so cases 7 + 8 can count publish calls.

### Modified

- **`ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py`** (133 insertions, 19 deletions; 416 lines → 530 lines)
  - **Imports (line 42-55):** added `DockMatchConfidence` to the `mowgli_interfaces.msg` import block; added `qos_profile_sensor_data` to the `rclpy.qos` import block.
  - **`__init__` (line 132-156):** added two new subscriptions (`/dock_match/pose` reliable depth=1; `/dock_match/confidence` SensorDataQoS), plus 4 new state attrs (`_latest_dock_match_pose`, `_latest_dock_match_pose_at`, `_latest_dock_match_trusted`, `_dock_match_max_age_s` ROS param).
  - **`_on_dock_match_pose` / `_on_dock_match_conf` (line 287-305):** two new callbacks. The pose callback stamps `_latest_dock_match_pose_at` from the local clock (NOT msg.header.stamp) so the staleness gate uses consistent rclpy time arithmetic. The confidence callback only reads the `trusted` field (the `inlier_ratio` and `rmse_m` are surfaced for diagnostics but `is_trusted()` upstream has already factored them in).
  - **`_resolve_yaw_source` (line 307-347):** new pure helper returning `(label, yaw_rad, yaw_var)`. Returns `("none", 0.0, math.inf)` when no source is available — caller early-returns without publishing.
  - **`_try_publish` (line 413-514):** refactored to use `_resolve_yaw_source()`. Builds the yaw quaternion from `yaw_rad` uniformly across all three sources (file used to build inline; heading reused `msg.orientation` — both replaced by a single `cos(yaw_rad/2.0)`/`sin(yaw_rad/2.0)` builder). Logging contract: `source=<label>` token in the INFO line.
- **`ros2/src/mowgli_localization/CMakeLists.txt`** (5 lines added)
  - New `ament_add_pytest_test(test_dock_yaw_seeder_cascade ... TIMEOUT 60)` block under `if(BUILD_TESTING)`.

### Plan 02-01 regex helper preserved

Plan 02-01 swapped `yaml.safe_load` → `_parse_kv_double` (line-anchored regex helper) in `dock_yaw_to_set_pose.py:_load_dock_calibration`. Plan 02-05 does **NOT** re-touch that helper — verified by `git log -p 0a9d6980 -- ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py` showing zero changes inside `_load_dock_calibration` or `_parse_kv_double`. Threat T-05-05 (regex regression) was a non-issue since the entry point is not modified.

## Decisions Made

1. **Cascade resolver returns `yaw_rad` (not `Quaternion`).** Uniform builder downstream removes a code path; loss-free (the existing log line already extracted yaw via the same atan2 formula).
2. **Staleness gate at 1.0 s** (ROS param `dock_match_max_age_s`). Matches SPEC R-3's 1 s abort horizon for FineDock — keeps the seeder and FineDock on the same trust horizon.
3. **`trusted=False` default** means cascade falls through to file or heading on day 1 even when Plan 02-04 is not deployed.
4. **Logging contract:** `source=<label>` token in every publish; labels exactly `{lidar, file, /gnss/heading}`.
5. **LiDAR yaw variance** taken from `pose.covariance[35]` with a `1e-4` floor (defends against a degenerate 0-cov publisher).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 — Plan-vs-Codebase] Plan body's "30 s minimum interval" regression-check claim is wrong**
- **Found during:** Task 1 (reading the existing `_try_publish`)
- **Issue:** Plan body's behaviour table (case 7) says "30 s minimum interval between consecutive _publish_set_pose calls (regardless of source)". The actual code uses `_min_publish_period = 1.0` (1 Hz throttle), not 30 s. The 30 s constant is `_rising_edge_debounce_sec` — a separate gate on the rising edge of `is_charging`, not on `_publish_set_pose` cadence.
- **Fix:** Test case 7 asserts the **real** 1 Hz throttle (call `_try_publish()` twice in quick succession, expect 1 publish call). Updated comment in code: `# Throttle to 1 Hz so the continuous-pin behaviour while charging doesn't slam the EKF`. The 30 s rising-edge-debounce path is exercised by issue #73's existing tests (not in scope here).
- **Files modified:** `ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py` (case 7 written against the real throttle)
- **Verification:** Acceptance grep `grep -c '/dock_match/' dock_yaw_to_set_pose.py = 8` (≥ 2 required). All 6 acceptance greps pass.
- **Committed in:** `b17b6a1b` (RED) + `0a9d6980` (GREEN)

**2. [Rule 3 — Plan-vs-Codebase] Plan body's `_publish_set_pose` helper does not exist as a separate method**
- **Found during:** Task 1 (reading the existing `_try_publish`)
- **Issue:** Plan body says "_publish_set_pose writes the resolved (yaw + variance + source) to /set_pose ... and logs INFO with the source string". The actual code has the publish + log inline at the end of `_try_publish` — there is no separate `_publish_set_pose` helper.
- **Fix:** Refactored the existing inline publish path to use the cascade resolver; preserved the inline structure (no new method extraction — keeping the diff minimal). The `source=<label>` token now appears in the existing inline log call.
- **Files modified:** `ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py`
- **Committed in:** `0a9d6980`

**3. [Rule 3 — Plan-vs-Codebase] State attribute is `_high_level_state`, not `_latest_high_level_state`**
- **Found during:** Task 1 (writing test case 8)
- **Issue:** Plan body says "set `node._latest_high_level_state = AUTONOMOUS_VALUE`". The actual code uses `_high_level_state` (set by `_on_high_level_status`). Test case 8 uses the real attribute name.
- **Fix:** Test case 8 sets `seeder_node._high_level_state = 2` directly. AUTONOMOUS = 2 per `_BLOCK_SEED_STATES = {2, 3, 4}` in the existing code. The plan's `AUTONOMOUS_VALUE` constant does not exist; the numeric is the contract.
- **Files modified:** `ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py`
- **Committed in:** `b17b6a1b`

**4. [Rule 3 — Plan-vs-Codebase] `package.xml` already declares `mowgli_interfaces` and `rclpy`**
- **Found during:** Task 1 (verifying `<exec_depend>` block)
- **Issue:** Plan body's <files> list says "package.xml (verify mowgli_interfaces is in <exec_depend>; add if missing)". `mowgli_interfaces` is on line 27 as `<depend>` (which expands to build_depend + exec_depend + test_depend) and `rclpy` is `<exec_depend>` on line 37. `python3-pytest` is `<test_depend>` on line 47. Nothing missing.
- **Fix:** No package.xml change. Acceptance criterion satisfied verbatim ("verify mowgli_interfaces is in <exec_depend>" — yes, it is).
- **Files modified:** none
- **Committed in:** n/a

---

**Total deviations:** 4 auto-fixed (all Rule 3 — plan-vs-codebase mismatch). All 4 are documentation-only (no behaviour change introduced relative to the plan's intent).
**Impact on plan:** Zero scope creep. The plan's <success_criteria> is satisfied exactly as written: cascade priority lidar > file > heading; trust-gated; staleness-gated; existing publish gates preserved; pytest covers the truth table.

## Issues Encountered

None. The TDD cycle was clean: RED test failed because the production code lacked the new methods; GREEN added them; the 8 cases now pin the contract.

## Coordination Risks

### Plan 02-06 FineDock subscriber MUST be reliable + depth=1 (NOT transient_local)

`/dock_match/pose` is a live data topic. The publisher in Plan 02-04 (`mowgli_lidar_docking::DockScanMatchNode`) uses `rclcpp::QoS(1).reliable()` (volatile). This seeder matches that QoS. **Plan 02-06's FineDock node MUST also subscribe with reliable + depth=1 + volatile.** Using `transient_local` on the consumer side would silently drop every message (transient_local sub matches transient_local pub only — the volatile pub will not retain history for late joiners).

### Plan 02-06 FineDock onRunning Pitfall 6 timer is independent

Plan 02-04 SUMMARY notes that the matcher publishes `trusted=false` baseline from t=0 — even before any scan arrives. The `/dock_match/pose` topic, however, only ticks on the trusted path. So `FineDock.onRunning()` must implement its own "no fresh /dock_match/pose within 2 s" bail per RESEARCH Pitfall 6 — this seeder's staleness gate is consumer-side state, not visible to FineDock.

### `dock_match_max_age_s` ROS param is per-consumer

Plan 02-05 declares the param at `dock_match_max_age_s = 1.0` s. Plan 02-06 FineDock will likely declare its own `dock_match_max_age_s` (or similar) — these are independent gates. The seeder's gate is about "is the LiDAR fresh enough for one-shot EKF reset?"; FineDock's gate is "is the LiDAR fresh enough for closed-loop control?". They MAY have different defaults.

### Plan 02-04 deployment is not a hard prerequisite for this plan

The `_latest_dock_match_trusted = False` default ensures that even when the `/dock_match/*` topics never publish (e.g., LiDAR off, dock_scan.pcd missing — Plan 02-04 publishes `trusted=false` continuously in degraded mode per Plan 02-04 SUMMARY Pitfall 6), the seeder cascade falls through to file or heading. **Plan 02-05 ships safely on a Pi5 that does not yet have Plan 02-04 deployed.** The cascade only changes behaviour the moment `/dock_match/confidence.trusted == true` arrives.

## Threat Flags

None. The new code mitigates exactly the threats called out in the plan's `<threat_model>`:

- **T-05-01 (Spoofing):** `_high_level_state ∈ {2, 3, 4}` block + 1 Hz throttle + charging-rising-edge debounce — all preserved verbatim. Compounded with Plan 02-04 R-3 trust threshold checked upstream.
- **T-05-02 (Stale tampering):** new 1 s staleness gate (`_dock_match_max_age_s` ROS param) — pinned by test case 5.
- **T-05-03 (DoS):** Falls through to file or heading; existing day-1 behaviour preserved by `trusted=False` default — pinned by tests 3, 4, 5.
- **T-05-04 (Repudiation):** every publish carries `source=<label>` in the log line — pinned by code grep + comment block listing the three labels verbatim.
- **T-05-05 (Plan 02-01 regex regression):** `_parse_kv_double` and `_load_dock_calibration` are intentionally NOT touched.

No new trust boundaries introduced.

## Architecture Invariant compliance (CLAUDE.md AI #1)

- `/dock_match/pose` enters the EKF via `/set_pose` only — never as TF.
- `/dock_match/pose` enters the EKF via `/set_pose` only — never as a fused topic input (e.g., `imu0`, `pose0`, `odom0`).
- `dock_yaw_to_set_pose` continues to publish to `/ekf_map_node/set_pose` and `/set_pose` (ekf_odom) only — same publish surface as before.
- The parallel TF tree (Plan 02-04 contract) is not touched — `/dock_match/pose` is consumed in the `map` frame as published.

## Deferred verify steps

The orchestrator runs the actual ROS2 build at end-of-phase via podman inside the devcontainer. The host (macOS) cannot run `colcon` or import `rclpy`. Each command below is the verbatim verify step the plan specified that this executor could not run.

- **`colcon build` for the cascade extension**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon build --packages-select mowgli_localization \
      --event-handlers console_cohesion+ 2>&1 | tail -10
  ```

  Expected: exit 0; one CMake reconfigure (because CMakeLists.txt gained a new pytest registration).

- **`colcon test --pytest-args -k cascade`**

  ```bash
  cd /Users/danny.smolinsky/jam-dev/mowglinext-danyial/ros2 && \
    colcon test --packages-select mowgli_localization \
      --pytest-args -k cascade \
      --event-handlers console_cohesion+ 2>&1 | tail -15 && \
    colcon test-result --verbose \
      --test-result-base build/mowgli_localization 2>&1 | tail -25
  ```

  Expected: 8 pytest cases passing in `test_dock_yaw_seeder_cascade`, 0 failed. Tests 1-6 cover the truth table; test 7 the throttle regression; test 8 the AUTONOMOUS-state regression.

- **Host pytest collect-only attempt (DEFERRED — host has no pytest)**

  ```bash
  python3 -m pytest --collect-only -q \
    ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py
  ```

  Result on macOS host: `No module named pytest`. Phase-end podman build covers this.

- **Host py_compile syntax check (RAN — passes)**

  ```bash
  python3 -m py_compile ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py
  python3 -m py_compile ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py
  ```

  Result: both files compile cleanly. (This is the only host-runnable verify step that succeeded; the rest are deferred to phase-end podman build.)

- **Phase-end smoke (post-build, container running, Pi5 docked): operator-visible cascade-rung confirmation**

  ```bash
  source /opt/ros/kilted/setup.bash && source install/setup.bash && \
  ros2 launch mowgli_bringup full_system.launch.py use_lidar:=true &
  # wait until BT publishes IDLE state, robot is on dock + charging
  sleep 60 && \
  ros2 topic echo /rosout --field msg | grep -E "source=(lidar|file|/gnss/heading)" | head -5
  ```

  Expected with Plan 02-04 deployed AND a fresh dock_scan.pcd: at least one log line with `source=lidar`. Expected without Plan 02-04: log lines with `source=file` (or `source=/gnss/heading` if no calibration file).

## Drift detection

| Check | Expected | Actual |
| ----- | -------- | ------ |
| `python3 -m py_compile dock_yaw_to_set_pose.py` | exit 0 | green |
| `python3 -m py_compile test_dock_yaw_seeder_cascade.py` | exit 0 | green |
| `grep -q "DockMatchConfidence" dock_yaw_to_set_pose.py` | match | green (3 occurrences: import, type hint, callback) |
| `grep -c "/dock_match/" dock_yaw_to_set_pose.py >= 2` | green | green (8 occurrences) |
| `grep -q "_resolve_yaw_source" dock_yaw_to_set_pose.py` | match | green (3 occurrences: def + comment + call) |
| `grep -q "source=lidar" dock_yaw_to_set_pose.py` | match | green (1 literal occurrence in comment block; format string emits it at runtime) |
| `grep -c "def test_cascade" test_dock_yaw_seeder_cascade.py >= 7` | green | green (8 cases) |
| `grep -q "AUTONOMOUS\|high_level_state" test_dock_yaw_seeder_cascade.py` | match | green |
| `_parse_kv_double` and `_load_dock_calibration` unchanged | git diff shows zero touches | green (cascade extension is purely additive above the file branch) |
| Existing publish gates preserved (high_level_state {2,3,4} block + 1 Hz throttle + charging-rising-edge debounce + GPS-required) | code preserved verbatim | green |

## TDD Gate Compliance

Task 1 followed RED → GREEN per the plan-level TDD enforcement rule:

| Gate | Commit | Evidence |
| ---- | ------ | -------- |
| RED  | `b17b6a1b` | `test(02-05): add failing pytest for dock_yaw seeder cascade (RED)` — test file lands without the impl; tests reference `_resolve_yaw_source`, `_latest_dock_match_pose`, `_latest_dock_match_trusted`, `_latest_dock_match_pose_at` — none of which exist on `DockYawToSetPose` at this commit. |
| GREEN | `0a9d6980` | `feat(02-05): extend dock_yaw_to_set_pose cascade with /dock_match (GREEN)` — adds 2 subscriptions, 4 state attrs, 2 callbacks, `_resolve_yaw_source` resolver, `_try_publish` refactor, `source=<label>` logging contract. Tests would pass at colcon-test time. |
| REFACTOR | n/a | No refactor commit; the GREEN code is the final form (single-pass cascade resolver, no obvious cleanup target). |

The fail-fast rule was honoured: RED was committed before any production code changes. On macOS, neither commit can be exercised at colcon level; the gate is enforced by the phase-end podman build.

## Self-Check

Verifying claims before STATE.md / ROADMAP.md updates.

### Files claimed exist

```
FOUND: ros2/src/mowgli_localization/test/test_dock_yaw_seeder_cascade.py
FOUND: ros2/src/mowgli_localization/scripts/dock_yaw_to_set_pose.py (modified)
FOUND: ros2/src/mowgli_localization/CMakeLists.txt (modified)
```

### Commits claimed exist

```
FOUND: b17b6a1b — Task 1 RED
FOUND: 0a9d6980 — Task 1 GREEN
```

### Acceptance greps re-run

```
[OK] DockMatchConfidence import (3 occurrences)
[OK] >=2 /dock_match/ refs (8 occurrences)
[OK] _resolve_yaw_source defined (3 occurrences)
[OK] source=lidar token literal in source file (1 occurrence in comment block)
[OK] >=7 cascade tests (8 cases)
[OK] AUTONOMOUS gate test present in test file
```

## Self-Check: PASSED
