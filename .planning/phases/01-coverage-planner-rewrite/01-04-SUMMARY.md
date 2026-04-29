---
phase: 01-coverage-planner-rewrite
plan: 04
subsystem: ui
tags: [gui, react, mapbox, rosbridge, roslib, action-client, antd, geojson, narrow-area-strategy]

# Dependency graph
requires:
  - phase: 01-coverage-planner-rewrite
    provides: "Plan 01-01 froze the PlanCoverage.action schema, the 8 segment_type constants on CoverageWaypoint, the 8 PlanError.error_code constants, and added MapArea.narrow_area_strategy. Plan 01-04 wires those types into the GUI."
provides:
  - "GUI rosbridge action client (gui/web/src/hooks/useCoveragePlan.ts) — calls /coverage_planner_node/plan_coverage via roslib v2, converts CoverageWaypoint[] to a GeoJSON FeatureCollection with segment_type properties per UI-SPEC §GeoJSON Conversion Contract"
  - "PlanError.error_code -> notification body mapping for all 8 codes (NO_AREAS, AREA_TOO_NARROW, OBSTACLE_BLOCKS_AREA, DOCK_OUTSIDE_AREAS, FOOTPRINT_VIOLATION, OBSTACLE_OFFSET_FAILED, RESUME_CHECKPOINT_INVALID, INTERNAL) using the locked UI-SPEC §Copywriting Contract strings"
  - "Three new Mapbox layers (coverage-plan-line, coverage-plan-arrows, coverage-plan-points) under a single coverage-plan-source GeoJSON source — single FeatureCollection per D-11"
  - "EditAreaModal narrow_area_strategy <Select> dropdown gated on feature_type === 'workarea' — realizes SPEC R-13 in the GUI"
  - "MapToolbar / MapToolbarMobile dedicated 'Preview Plan' AsyncButton with Idle/Active/Loading copywriting per UI-SPEC §Interaction Contract; replaces the legacy Show plan preview menu entry"
  - "MowingAreaEdit type extended with optional narrow_area_strategy field — keeps utils/types.ts in lock-step with MapArea.msg"
affects:
  - 01-05-coverage-planner-skeleton  # GUI now expects /coverage_planner_node/plan_coverage to be reachable on rosbridge
  - 01-06-map-server-cleanup  # the legacy Go endpoint /api/mowglinext/preview-plan/* can be deleted in a follow-up Go cleanup PR; not touched in this plan
  - 01-09-e2e-sim-and-pi5-smoke  # Pi5 smoke test will exercise the Preview Plan button end-to-end

# Tech tracking
tech-stack:
  added:
    - "roslib@^2.1.0 (with bundled types via roslib/dist/RosLib.d.ts) — first browser-side rosbridge client in the codebase. Provides ROS2 Action class via the rosbridge_v2 send_action_goal / cancel_action_goal protocol ops."
  patterns:
    - "Browser-side rosbridge action client: lazy-connected new Ros({url: ws://<host>:9090}) inside a hook, with new Action<TGoal,TFeedback,TResult>({ros, name, actionType}) and sendGoal(goal, resultCb, feedbackCb, failedCb)"
    - "Single GeoJSON FeatureCollection + segment_type properties + Mapbox match expression for plan-render colour encoding (D-11). Replaces 6-layer-explosion pattern from #56."
    - "Explicit error_code -> body string switch in the hook (planErrorBody) — keeps the GUI copywriting contract single-sourced and grep-able without inline ternaries scattered across components."

key-files:
  created:
    - "gui/web/src/hooks/useCoveragePlan.ts"
    - ".planning/phases/01-coverage-planner-rewrite/01-04-SUMMARY.md"
  modified:
    - "gui/web/package.json"
    - "gui/web/yarn.lock"
    - "gui/web/src/pages/MapPage.tsx"
    - "gui/web/src/pages/map/components/EditAreaModal.tsx"
    - "gui/web/src/pages/map/components/MapToolbar.tsx"
    - "gui/web/src/pages/map/components/MapToolbarMobile.tsx"
    - "gui/web/src/pages/map/utils/types.ts"

key-decisions:
  - "Use roslib@^2.1.0 (not v1.4.x) — only v2 supports ROS2 actions via the rosbridge send_action_goal/cancel_action_goal protocol ops. v1 ships only the legacy actionlib (ROS1) ActionClient/Goal classes which actually publish on /goal /cancel /feedback /result /status topics — incompatible with rosbridge_v2 + ROS2 Kilted. The v2 dist/RosLib.d.ts bundles its own types so @types/roslib (which only describes v1) was removed."
  - "Browser talks to rosbridge directly on port 9090 (ws://<host>:9090), bypassing the existing Go API server. The Go server has no generic action-relay endpoint and adding one would have been substantial Go work outside the plan scope. The browser→rosbridge direct path matches CLAUDE.md's documented rosbridge_server placement (docker/README.md:305 :9090)."
  - "The plan-doc `gui/web/src/utils/types.ts` path does not exist; the actual editable type wrapper is at `gui/web/src/pages/map/utils/types.ts`. Followed plan intent (extend MowingAreaEdit with narrow_area_strategy) at the real location, same as the Plan 01-01 SUMMARY's plan-doc-path-drift handling."
  - "Removed the entire coverageLineWidth zoom-interpolated tool_width line-width memo from MapPage. It was tightly coupled to the deleted plan-preview-coverage layer; the new coverage-plan-line uses a fixed 2.5 px width per UI-SPEC §Mapbox Layer Color Contract. A separate tool_width-tracking band layer can be re-introduced in a future plan if operators want it back."
  - "Bridge waypoints across segment_type boundaries with a TRANSIT line (rather than emitting two disjoint LineStrings). This keeps the rendered polyline visually continuous; the colour-match expression renders the bridge segment grey via the TRANSIT case which matches operator intuition (the robot will physically transit between segment boundaries)."
  - "Default the start_pose to (0,0,0) with identity orientation when the GUI hasn't been told otherwise. Server-side validation (Plan 05) is the source of truth — the planner uses the dock pose as the seed and may ignore the start_pose entirely; passing a sane default avoids spurious null-pose validation rejections during operator-initiated previews."

patterns-established:
  - "Hook pattern: useCoveragePlan(projection) returns {planGeoJson, isLoading, error, requestPlan, clearPlan} — the request/clear handlers are stable references (useCallback), the hook owns the rosbridge connection lifecycle, and the result FeatureCollection is null-when-idle so the conditional-render at the call site is `coveragePlanGeoJson && <Source ...>`."
  - "Mapbox match expression for segment_type colours — verbatim from D-11. Reusable for any future per-segment styled overlay; 8 colour mappings + grey fallback in a single property."
  - "AsyncButton + state-dependent label/icon/title for Preview Plan (and analog future toggles): Idle/Active labels carry the locked copywriting, the title prop drives Ant Design's tooltip, and onAsyncClick wraps a Promise so the spinner is automatic."

requirements-completed: [R-2, R-4, R-13]

# Metrics
duration: 15min
completed: 2026-04-29
---

# Phase 1 Plan 4: GUI Integration Summary

**Coverage-plan render surface migrated from the legacy HTTP pull to a rosbridge action client. Single FeatureCollection + 3 Mapbox layers + locked D-11 colour palette + per-area narrow-area-strategy dropdown all landed.**

## Performance

- **Duration:** ~15 min
- **Started:** 2026-04-29T06:17:19Z
- **Completed:** 2026-04-29T06:32:37Z
- **Tasks:** 2
- **Files created:** 1 (useCoveragePlan.ts)
- **Files modified:** 6 (MapPage, MapToolbar, MapToolbarMobile, EditAreaModal, types.ts, package.json + yarn.lock)

## Accomplishments

- The Preview Plan button in the toolbar (desktop + mobile) now invokes `/coverage_planner_node/plan_coverage` via rosbridge instead of `/api/mowglinext/preview-plan/<idx>`. The hook owns websocket lifecycle, action-goal dispatch, result→FeatureCollection conversion, and 8-code error-notification mapping.
- All 6 legacy `plan-preview-*` Mapbox layers + the `<Source id="plan-preview">` block are removed from BOTH the compact and main render paths in MapPage.tsx. Replaced with a single `<Source id="coverage-plan-source">` carrying 3 new layers: `coverage-plan-line`, `coverage-plan-arrows`, `coverage-plan-points`.
- The 8 D-11 segment_type colours appear verbatim (`#1d4ed8`, `#16a34a`, `#15803d`, `#9ca3af`, `#f97316`, `#fbbf24`, `#b45309`, `#facc15`) inside Mapbox `match` expressions on both the line and arrow layers (38 matches across MapPage.tsx).
- `EditAreaModal` now exposes the `narrow_area_strategy` dropdown gated on `feature_type === 'workarea'` with the exact UI-SPEC label / helper text / 3 option labels.
- `MowingAreaEdit` type extended with `narrow_area_strategy?: number` and constructor default of `0` to keep the editable wrapper in sync with `MapArea.narrow_area_strategy`.
- All 8 `PlanError.error_code` values map to UI-SPEC body strings inside the hook's `planErrorBody()` switch (including the dedicated OBSTACLE_OFFSET_FAILED branch which intentionally shares copy with INTERNAL until UX is refined).
- `cd gui/web && yarn tsc --noEmit` exits 0; `cd gui/web && yarn test --run --testNamePattern "EditAreaModal|MapToolbar"` runs 24 tests, all pass; `cd gui/web && yarn build` produces a clean production bundle.

## Task Commits

1. **Task 1: useCoveragePlan hook + remove old plan-preview-* + new coverage-plan-* layers** — `66ffe815` (feat)
2. **Task 2: narrow_area_strategy dropdown in EditAreaModal + MowingAreaEdit type extension** — `504f490c` (feat)

## Files Created / Modified

### Created
- `gui/web/src/hooks/useCoveragePlan.ts` — 357-line rosbridge action client hook. Exports `useCoveragePlan(projection)` returning `{planGeoJson, isLoading, error, planMetadata, requestPlan, clearPlan}`. Owns the `Ros` websocket (lazy-connected to `ws://<host>:9090`), constructs `Action<PlanGoal, PlanFeedback, PlanResult>` per request, and converts `CoverageWaypoint[]` to a `FeatureCollection` via the canonical `transpose(offsetX, offsetY, datum, y, x)` projection.

### Modified
- `gui/web/package.json` + `gui/web/yarn.lock` — Added `roslib@^2.1.0` (production dep). The bundled types (`dist/RosLib.d.ts`) supersede the obsolete `@types/roslib` package which we removed.
- `gui/web/src/pages/MapPage.tsx`:
  - Deleted the `planPreview` `useState`, the `fetchPlanPreview` `useCallback`, and the `useEffect` that triggered HTTP fetch on toggle.
  - Deleted the legacy `coverageLineWidth` zoom-interpolated line-width memo (only consumed by the removed `plan-preview-coverage` layer).
  - Deleted the entire `<Source id="plan-preview">` block (containing 6 child `<Layer>` elements) in BOTH the compact and main render paths.
  - Added `useCoveragePlan` hook call with a stable `projection` memo derived from `offsetX`, `offsetY`, `datum`.
  - Added `<Source id="coverage-plan-source">` with 3 child layers: `coverage-plan-line` (line-width 2.5, opacity 0.9, D-11 match colour), `coverage-plan-arrows` (symbol-on-line ▶, 12 px, white halo, D-11 match colour), `coverage-plan-points` (orange circle 4 px radius, white 1.5 px stroke, filtered to point_type=endpoint).
  - New `onTogglePlanPreview` handler that derives the dock pose from the existing dock feature (via `itranspose` to convert lon/lat back to ROS map frame) and calls `requestCoveragePlan({dockPose})`. Toggling while active calls `clearCoveragePlan()` instead.
- `gui/web/src/pages/map/components/MapToolbar.tsx`:
  - New `planLoading?: boolean` prop and updated `onTogglePlanPreview?: () => Promise<void> | void` signature.
  - `EyeInvisibleOutlined` added to the icon imports.
  - Removed `planPreview` entry from `moreMenuItems` and its case from `handleMoreClick`.
  - Added a dedicated Preview Plan `AsyncButton` after Edit Map per UI-SPEC §Interaction Contract location, with the locked copywriting (Idle: "Preview Plan", Active: "Clear Preview", Loading: "Planning…"), tooltip strings, and Eye/EyeInvisible icon switch.
- `gui/web/src/pages/map/components/MapToolbarMobile.tsx`:
  - New `showPlanPreview` / `planLoading` / `onTogglePlanPreview` props.
  - `EyeOutlined` + `EyeInvisibleOutlined` added to the icon imports.
  - Added a dedicated Preview Plan `AsyncButton` (icon-only) after Edit Map in View mode, mirroring the desktop toolbar behaviour.
- `gui/web/src/pages/map/components/EditAreaModal.tsx`:
  - New `NARROW_AREA_OPTIONS` constant with the 3 UI-SPEC labels and uint8 values 0/1/2.
  - New `<Form.Item label="Narrow area strategy" extra="...">` + `<Select>` block gated on `feature_type === 'workarea'`. `onChange` propagates `narrow_area_strategy` through the existing `area` prop so `updateMowingArea` persists it.
- `gui/web/src/pages/map/utils/types.ts`:
  - `MowingAreaEdit` class gained `narrow_area_strategy?: number`. Constructor initialises to `0` (Skip) so legacy code reading the field never sees `undefined` after construction.

## Decisions Made

- **roslib@^2.x, not v1.x.** v1 only supports ROS1 actionlib via `/goal /cancel /feedback /result /status` topics — incompatible with rosbridge_v2 + ROS2 Kilted. v2's `Action` class uses the new `send_action_goal` / `cancel_action_goal` protocol ops. The bundled `dist/RosLib.d.ts` carries the v2 types, so `@types/roslib` (which describes v1) was removed.
- **Direct browser → rosbridge ws://<host>:9090, not a Go-API relay.** The Go API has no generic action-relay endpoint, and adding one was outside the plan's "GUI-only" scope. Direct browser→rosbridge is consistent with CLAUDE.md's documented rosbridge_server placement (`docker/README.md:305`). In dev the developer is expected to expose port 9090 from the `mowgli-ros2` container alongside the existing Go-API port 4006.
- **Cross-segment-type bridge segments use `segment_type: "TRANSIT"`** so the polyline has no visible gap at boundaries; the D-11 match expression renders bridge segments grey, matching operator intuition that the robot physically transits between segment boundaries even if the planner labels both endpoints as e.g. `MOWING_BOUSTROPHEDON`.
- **`coverageLineWidth` memo removed entirely.** It was tightly coupled to the deleted `plan-preview-coverage` translucent-orange band; the new `coverage-plan-line` uses a fixed 2.5 px width per UI-SPEC. A `tool_width`-tracking translucent band can be reintroduced as a separate non-segment_type-coloured layer in a future plan if operators want it back.
- **`MowingAreaEdit.narrow_area_strategy` default = 0 in the constructor**, not `undefined`. Plan instructions said "marked optional to ease migration of legacy area data" — `?:` keeps it optional in the type, but `this.narrow_area_strategy = 0` in the constructor avoids `undefined` dropouts in the dropdown's `value={area.narrow_area_strategy ?? 0}` rendering, which the dropdown handles cleanly anyway but the explicit default is friendlier to consumers reading the class.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking] Plan-doc references roslib but it was not in package.json**
- **Found during:** Task 1 (initial hook scaffolding)
- **Issue:** The plan instructed "construct a minimal useCoveragePlan hook using roslib.ActionClient (rosbridge JS library is already a dependency - check gui/web/package.json for "roslib" before assuming)". `roslib` was NOT in `package.json`. The codebase has zero browser-side rosbridge clients today; all ROS2 calls go through the Go API server's `provider.CallService`.
- **Fix:** Added `roslib@^2.1.0` as a production dependency. Initial install of v1.4.1 + `@types/roslib@^1.3.5` was rolled back when discovered it ships only ROS1 actionlib (incompatible with ROS2 + rosbridge_v2). Replaced with `roslib@^2.1.0` whose `Action` class uses the rosbridge_v2 `send_action_goal` op. `@types/roslib` was removed (v2 carries its own bundled `dist/RosLib.d.ts`).
- **Files modified:** `gui/web/package.json`, `gui/web/yarn.lock`
- **Verification:** `yarn tsc --noEmit` exits 0; `yarn build` succeeds.
- **Committed in:** `66ffe815` (Task 1 commit).

**2. [Rule 1 - Bug] roslib v1 API in initial hook scaffolding (`new ROSLIB.ActionHandle(…)` + `.createClient(goal, …)`)**
- **Found during:** Task 1 (typecheck pass)
- **Issue:** I drafted the hook against the v1 API I had in mind. v1's class is `ActionClient` (not `ActionHandle`) and is wired via `new Goal({actionClient, goalMessage})` + `goal.on('result', cb)` + `goal.send()` — none of which are correct for ROS2 actions over rosbridge_v2.
- **Fix:** Rewrote the hook against v2's `Action` class — `new Action<TGoal, TFeedback, TResult>({ros, name, actionType})` then `actionClient.sendGoal(goal, resultCb, feedbackCb, failedCb)`. Imports updated from `import ROSLIB from "roslib"` to `import {Action, Ros} from "roslib"`.
- **Verification:** Hook compiles; the runtime test path is exercised in Plan 01-09 (E2E sim).
- **Committed in:** `66ffe815`.

**3. [Rule 3 - Blocking] Plan-doc path `gui/web/src/utils/types.ts` doesn't exist**
- **Found during:** Task 2 (file resolution)
- **Issue:** Plan and acceptance grep both reference `gui/web/src/utils/types.ts`, which is not a file on disk. The editable wrapper used by `EditAreaModal` is at `gui/web/src/pages/map/utils/types.ts`.
- **Fix:** Followed plan intent — extended `MowingAreaEdit` with `narrow_area_strategy?: number` at the actual location. Documented as plan-path drift (same handling as Plan 01-01's `gui/pkg/msgs/mowgli_interfaces/...` vs. real `gui/pkg/msgs/mowgli/...` resolution).
- **Files modified:** `gui/web/src/pages/map/utils/types.ts`
- **Committed in:** `504f490c` (Task 2 commit).

**4. [Rule 2 - Critical functionality] `useCoveragePlan` reentrancy guard**
- **Found during:** Task 1 (drafting state machine)
- **Issue:** UI-SPEC §Interaction Contract requires AsyncButton's loading prop to disable the button during in-flight requests. AsyncButton's local `loading` state covers most of this, but a race exists: `setIsLoading(true)` happens after the click handler returns; if the user double-clicks faster than React's commit, two action goals can be enqueued.
- **Fix:** Added `inFlightRef` (a `useRef<boolean>`) check at the top of `requestPlan`. Re-entrant calls return immediately without enqueuing a second goal. Also addresses T-04-04 (DoS via spam clicks) from the plan's threat register.
- **Files modified:** `gui/web/src/hooks/useCoveragePlan.ts`
- **Committed in:** `66ffe815`.

### Deferred

**1. [Out of scope] `yarn lint` fails project-wide on baseline**
- **Cause:** The repo ships `.eslintrc.cjs` (legacy v8 config format), but `eslint@9.39.4` is installed in `node_modules`. v9 defaults to flat config; legacy support requires `ESLINT_USE_FLAT_CONFIG=false` AND a config-format compatibility shim. Even with `ESLINT_USE_FLAT_CONFIG=false`, the existing config triggers 1189 errors / 37 warnings on baseline (rule `react-refresh/only-export-components` declared but plugin not loaded by the legacy bridge; pervasive `@typescript-eslint/no-explicit-any`; baseline tech-debt). None of the failures are caused by this plan's edits.
- **Decision:** Out of scope per the deviation rules' Scope Boundary clause ("Only auto-fix issues DIRECTLY caused by the current task's changes"). No prior plan in this phase ran `yarn lint` either (per 01-01-SUMMARY / 01-02-SUMMARY / 01-03-SUMMARY). Migrating ESLint to flat config + fixing 1189 baseline violations is a substantial refactor that should land as its own dedicated plan / PR.
- **Tracked:** Recommend a follow-up plan in a future phase to migrate `.eslintrc.cjs` -> `eslint.config.js` and clean up baseline violations. This plan's edits are TS-clean and test-clean; lint compliance can be verified via `yarn tsc --noEmit` + `yarn test --run` + `yarn build` (all green).

**2. [Deferred to follow-up] Legacy Go endpoint `/api/mowglinext/preview-plan/:area_index` and `mowgli.PreviewPlan*` types still live in `gui/pkg/api/mowglinext.go` and `gui/pkg/msgs/mowgli/services_generated.go`**
- **Cause:** Plan explicitly excludes `gui/` Go code from this plan's scope ("this plan does not touch gui/ Go code"). The legacy endpoint is no longer reached from the browser, but the Go file still imports / wires it.
- **Tracked:** Plan 01-06 (map_server cleanup) is the natural place to delete the now-orphaned Go endpoint + the `mowgli_interfaces/srv/PreviewPlan.srv` it called into. Adding to Plan 01-06's scope.

## Issues Encountered

- **roslib v1 vs v2 API drift.** v1 ships ROS1 actionlib (`ActionClient`, `Goal`, `SimpleActionServer`); v2 adds the ROS2-aware `Action` class (`core/Action.d.ts`). The pre-bundled `@types/roslib@^1.3.5` describes only v1. Discovered when typecheck passed but `new ROSLIB.ActionHandle({…}).createClient(…)` doesn't exist on v2. Resolved by removing `@types/roslib` (v2 has its own bundled types) and rewriting the hook to use `new Action<TGoal,TFeedback,TResult>({ros,name,actionType}).sendGoal(goal, resultCb, feedbackCb, failedCb)`.
- **MapToolbar test mock for AsyncButton ignored the `loading` prop.** The pre-existing test `vi.mock('../../../components/AsyncButton.tsx')` returns a vanilla `<button onClick={...}>`. The new Preview Plan button passes `loading={planLoading}` but the mock simply spreads `...props` onto a plain HTML button, which doesn't render a spinner. None of the existing tests assert on the loading state, so all 24 tests still pass. No action needed.
- **PoseStamped type in `gui/web/src/types/ros.ts` only carries `pose`, not `header`.** My initial default `start_pose` constructed both fields; the regenerated TS type only has `pose: Pose`. Removed the `header` field from the default — it's optional anyway in the .msg and the planner derives the `header.frame_id` from the `dock_pose` context.

## User Setup Required

- **Operator must expose port 9090 from the `mowgli-ros2` container** for the Preview Plan button to reach rosbridge in production deployments. `docker/README.md:305` already documents `rosbridge_server :9090`; `docker/compose.*.yml` configs may need a `ports: - "9090:9090"` line if not already exposed (varies per modular preset).
- **Dev workflow:** the Vite dev server proxy at `/api -> localhost:4006` does NOT cover the rosbridge ws path. Dev users should either run `mowgli-dev` against a Pi/sim that exposes `:9090` on the same host as the GUI, or set up a separate ws proxy if running everything purely on `localhost`.

## Next Phase Readiness

- The GUI is now ready to consume `/coverage_planner_node/plan_coverage` once it lands (Plan 01-05 + Plan 01-07).
- `EditAreaModal` writes `narrow_area_strategy` through the existing `updateMowingArea` path — Plan 01-06 (map_server cleanup) will need to ensure the persisted `areas.yaml` schema picks up the new field at save time. The .msg field already exists per Plan 01-01; the Go-side persistence layer needs the same key in the YAML serializer.
- Plan 01-09 (E2E sim + Pi5 smoke) will exercise the Preview Plan button end-to-end against a real `coverage_planner_node` action server.

## Threat Flags

None new beyond the plan's threat register. T-04-01 (browser-side tampering) is mitigated by typed JS objects sent through `roslib.Action.sendGoal` — no string concatenation into ROS2 messages. T-04-03 (XSS) is mitigated by React JSX auto-escape on the popup body content; verified-by-grep that no unsafe HTML-injection prop appears in any touched file. T-04-04 (DoS) is mitigated by the `inFlightRef` reentrancy guard added in Deviation #4.

## Self-Check: PASSED

Verified:
- `gui/web/src/hooks/useCoveragePlan.ts` exists at expected path.
- All 6 old layer ids gone from `gui/web/src/pages/MapPage.tsx`.
- All 3 new layer ids present in `gui/web/src/pages/MapPage.tsx`.
- 38 D-11 colour mentions in `gui/web/src/pages/MapPage.tsx` (>= 8 unique).
- `fetchPlanPreview` and `const [planPreview` are not present in `gui/web/src/pages/MapPage.tsx`.
- `useCoveragePlan` is imported and used in `gui/web/src/pages/MapPage.tsx`.
- No unsafe HTML-injection prop in any touched file (grep clean).
- `narrow_area_strategy` field present in `EditAreaModal.tsx` (label "Narrow area strategy") and in `MowingAreaEdit` type at `gui/web/src/pages/map/utils/types.ts`.
- "Preview Plan" / "Clear Preview" labels present in `MapToolbar.tsx`; `EyeOutlined` + `EyeInvisibleOutlined` imports added.
- "Plan generation failed" heading + 7 distinct error-body phrases land in `useCoveragePlan.ts` (grep returns 19 matches).
- Both task commits (`66ffe815`, `504f490c`) present in `git log`.
- `cd gui/web && yarn tsc --noEmit` exits 0.
- `cd gui/web && yarn test --run --testNamePattern "EditAreaModal|MapToolbar"` runs 24 tests, all pass.
- `cd gui/web && yarn build` produces a clean production bundle (warnings for chunk size are pre-existing baseline).

`yarn lint` is broken at baseline (1189 pre-existing errors, eslint v9 vs `.eslintrc.cjs` migration unaddressed) — out of scope per deviation Scope Boundary. Documented in Deferred.

---

*Phase: 01-coverage-planner-rewrite*
*Plan: 04*
*Completed: 2026-04-29*
