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

## From Plan 02-07

- **`ros.generated.ts` MapAreaConstants enum init expressions are invalid TS**
  - File: `gui/web/src/types/ros.generated.ts` lines 281-283
  - Symptom: `cd gui/web && yarn build` fails with
    ```
    src/types/ros.generated.ts(281,22): error TS2474: const enum member initializers must be constant expressions.
    src/types/ros.generated.ts(281,22): error TS2565: Property 'NARROW_AREA_SKIP' is used before being assigned.
    ```
    The generator emits `NARROW_AREA_SKIP = NARROW_AREA_SKIP` (self-
    referential), which TypeScript correctly rejects — the constants need
    numeric values or a different generator output shape.
  - Cause: pre-existing — bug in `gui/generate_ts_types.sh` introduced before
    Plan 02-07. The file is NOT imported anywhere (the canonical types live
    in `ros.ts`), but `tsconfig.json:include = ["src"]` makes `tsc` compile
    it anyway.
  - Risk: `yarn build` exits 2 on host / CI. Plan 02-07's GUI changes
    (`useDockMatch.ts`, `DockMatchCard.tsx`, `MowerStatus.tsx`) compile
    cleanly in isolation; the failure is entirely in `ros.generated.ts`.
  - Action: NOT fixed in Plan 02-07 (deviation rule SCOPE BOUNDARY:
    out-of-scope, pre-existing). `cd gui && go build ./...` exits 0 — the
    Go side of Plan 02-07's GUI extension is verified.
  - Suggested fix: either (a) regenerate `ros.generated.ts` after fixing
    the `MapAreaConstants` enum emission in `gui/generate_ts_types.sh`,
    OR (b) add `ros.generated.ts` to `tsconfig.json:exclude` until the
    canonical `ros.ts` is regenerated from the same source. Out-of-scope
    for Phase 2.
