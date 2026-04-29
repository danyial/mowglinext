# Phase 2 — Deferred Items

Out-of-scope discoveries from plan execution that should be tracked outside
this phase.

## From Plan 02-01

- **MapArea.h MD5 drift in firmware rosserial bindings**
  - File: `firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/MapArea.h`
  - Symptom: `python3 firmware/scripts/sync_ros_lib.py` rewrites the MD5 hash
    from `95c4cd71205ce55b6d40a3234589b32a` to `c7d63fd7c5f100e71c60e659a1a99a1e`
    even though `MapArea.msg` was not touched in Plan 02-01.
  - Cause: pre-existing — `MapArea.msg` text changed in a prior plan/commit
    without re-running `sync_ros_lib.py`. The header has been carrying a
    stale MD5 ever since.
  - Risk: rosserial communication mismatch between firmware (computes MD5 from
    its compiled-in header) and ROS2 host (computes MD5 from `MapArea.msg`).
    If the firmware ever subscribes to `MapArea`, messages will be silently
    rejected. Today firmware does not consume `MapArea`, so impact is latent.
  - Action: filed for follow-up. `git checkout -- ...MapArea.h` reverted the
    drift so Plan 02-01's scope remains DockMatchConfidence-only.
  - Suggested fix: re-run `sync_ros_lib.py` and commit the MapArea.h MD5 fix
    as a separate `chore: re-sync firmware MapArea.h` PR outside Phase 2.
