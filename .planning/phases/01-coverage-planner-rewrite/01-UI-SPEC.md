---
phase: 1
slug: coverage-planner-rewrite
status: draft
shadcn_initialized: false
preset: none
created: 2026-04-28
---

# Phase 1 — UI Design Contract

> Visual and interaction contract for the three additive GUI elements in Phase 1.
> This is NOT a GUI redesign. The existing Ant Design 5 + MowgliNext brand token system
> is inherited in full. This document covers only the delta surfaces.

Source: All color/spacing/typography values are extracted from the existing codebase —
`gui/web/src/theme/colors.ts`, `gui/web/src/pages/MapPage.tsx`,
`gui/web/src/pages/map/components/MapToolbar.tsx`, and
`gui/web/src/pages/map/components/EditAreaModal.tsx`.

---

## Design System

| Property | Value | Source |
|----------|-------|--------|
| Tool | none (no shadcn) | codebase scan — no components.json |
| Component library | Ant Design 5.29.x (`antd`) | `gui/web/package.json` |
| Icon library | `@ant-design/icons` | MapToolbar.tsx imports |
| Font | system default via Ant Design 5 theme | no custom font declared |
| Map renderer | react-map-gl / Mapbox GL JS | MapPage.tsx |
| Theme | dual light/dark, tokens in `gui/web/src/theme/colors.ts` | ThemeContext.tsx |

Registry safety gate: not applicable (no shadcn, no third-party component registry).

---

## Scope: Three Additive Surfaces

This phase adds exactly three UI elements to the existing `MapPage.tsx` + `EditAreaModal.tsx`:

1. **"Preview Plan" button** — triggers `PlanCoverage.action` via rosbridge, renders result as GeoJSON layers on the map.
2. **Two new Mapbox layers** — `coverage-plan-line` + `coverage-plan-points` replacing the existing `plan-preview-*` layers.
3. **`narrow_area_strategy` dropdown** — added to the per-area `EditAreaModal`, visible only when `area.feature_type === 'workarea'`.

---

## Spacing Scale

Inherited from Ant Design 5 token system. Phase 1 uses no custom spacing. For reference:

| Token | Value | Usage in this phase |
|-------|-------|---------------------|
| xs | 4px | Icon-text gap inside button |
| sm | 8px | Form.Item vertical rhythm (Ant default) |
| md | 16px | Form layout marginTop (`style={{marginTop: 16}}`) |
| lg | 24px | Not used in this phase |

Exceptions: none for this phase.

---

## Typography

Inherited from Ant Design 5 defaults. Phase 1 does not introduce new type roles. For reference:

| Role | Size | Weight | Line Height | Usage in this phase |
|------|------|--------|-------------|---------------------|
| Body / label | 14px | 400 | 1.5715 (Ant default) | Dropdown option labels, button text |
| Form label | 14px | 600 | 1.2 | `<Form.Item label>` in EditAreaModal |
| Helper text | 12px | 400 | 1.5 | Narrow-area strategy description below dropdown |

---

## Color

### Brand Token Inheritance

| Role | Light value | Dark value | Source token |
|------|-------------|------------|--------------|
| Dominant surface (60%) | `#FAFAF7` | `#0F1210` | `colors.bgBase` |
| Secondary surface (30%) | `#FFFFFF` | `#1A201C` | `colors.panel` |
| Primary accent | `#1B9D52` | `#3EE084` | `colors.primary` |
| Danger | `#C93020` | `#FF6B6B` | `colors.danger` |
| Glass overlay | `rgba(255,255,255,0.85)` | `rgba(26,32,28,0.75)` | `colors.glassBackground` |

### Mapbox Layer Color Contract (D-11, locked)

These are the exact hex values from `CONTEXT.md D-11`. They are design-system-independent
(rendered by Mapbox GL, not by Ant Design). They apply regardless of light/dark mode.

| segment_type | Color hex | Visual meaning |
|---|---|---|
| `MOWING_BOUSTROPHEDON` | `#1d4ed8` | Blue — active mowing swaths |
| `OUTLINE_WORKING_AREA` | `#16a34a` | Green — working-area perimeter passes |
| `OUTLINE_OBSTACLE` | `#15803d` | Dark green — obstacle perimeter passes |
| `TRANSIT` | `#9ca3af` | Grey — transit between areas |
| `UNDOCK` | `#f97316` | Orange — departure from dock |
| `DOCK_APPROACH` | `#fbbf24` | Amber — final approach to dock |
| `DOCKING` | `#b45309` | Dark amber — docking maneuver |
| `RETURN_TO_DOCK` | `#facc15` | Yellow — return path to dock |
| fallback | `#9ca3af` | Grey — unknown segment type |

Line width: `2.5px` at all zoom levels (fixed, not scaled — these are path lines, not coverage swath bands).
Line opacity: `0.9`.

Direction arrows: symbol layer on top of `coverage-plan-line`, using `"text-field": "▶"`,
`symbol-placement: "line"`, spacing `40px`, text-size `12px`, color matches the segment line color,
white halo `1.2px`. Arrow layer id: `coverage-plan-arrows`.

Start/end marker circles (`coverage-plan-points`): radius `4px`, color `#f97316` (orange — dock-associated
endpoints), white stroke `1.5px`. Apply only to the first and last waypoint in the plan
(filter: `["==", ["get", "point_type"], "endpoint"]`).

---

## Copywriting Contract

### "Preview Plan" Button

| Element | Copy | Rationale |
|---------|------|-----------|
| Button label (idle) | `Preview Plan` | Matches existing `Show plan preview` menu item label pattern; verb + noun |
| Button label (loading) | `Planning…` | Ant Design `Button loading` prop, spinner replaces icon |
| Tooltip (idle) | `Generate and preview the full mowing plan for all areas` | Surfaced on hover via `title` prop |
| Tooltip (active / clear) | `Clear plan preview` | Toggle behavior matches existing `showPlanPreview` toggle pattern |
| Progress notification (toast) | `Planning in progress…` | Ant Design `notification.open` with `duration: 0`, closed on result |
| Success notification | `Plan ready — {N} waypoints, {M} areas` | N = `plan.length`, M = `metadata.processed_area_indices.length` |
| Error heading | `Plan generation failed` | Ant Design `notification.error` |
| Error body — NO_AREAS | `No mowing areas defined. Draw at least one mowing area on the map.` | |
| Error body — AREA_TOO_NARROW | `One or more areas are too narrow for the robot to enter. Check narrow-area strategy settings.` | |
| Error body — OBSTACLE_BLOCKS_AREA | `An obstacle completely blocks a mowing area. Review obstacle placement.` | |
| Error body — DOCK_OUTSIDE_AREAS | `Dock position is outside all mowing and navigation areas. Reposition the dock.` | |
| Error body — FOOTPRINT_VIOLATION | `Robot footprint does not fit safely in all planned paths. Check robot geometry settings.` | |
| Error body — RESUME_CHECKPOINT_INVALID | `Saved progress checkpoint is corrupted. Clear checkpoints and restart from scratch.` | |
| Error body — INTERNAL | `Internal planner error: {error.human_readable}` | Pass `human_readable` from PlanError |
| Empty state (no plan yet) | _(no empty state — button simply toggles off)_ | Preview is opt-in; no persistent empty state needed |
| Destructive action | None | Preview is read-only; no destructive actions in this surface |

### narrow_area_strategy Dropdown (in EditAreaModal)

| Element | Copy |
|---------|------|
| Form.Item label | `Narrow area strategy` |
| Option 0 | `Skip` |
| Option 1 | `Outline only` |
| Option 2 | `Special pattern` |
| Helper text (below dropdown) | `Applies to strips shorter than 2× tool width. "Outline only" adds an extra pass along the center. "Special pattern" uses the area's long axis.` |
| Default value | `0` (Skip) — matches `uint8` field default of 0 in `MapArea.msg` |

---

## Interaction Contract

### "Preview Plan" Button

**Location:** Inside the existing `MapToolbar` component, placed after the `Edit Map` button and before the `Start` / `Home` button. On desktop, this is the bottom glass toolbar. On mobile, it is added to `MapToolbarMobile` at the same position.

**Component:** `AsyncButton` (already used in `MapToolbar` for `Start`, `Home`, `Finish Recording`). Props:
```tsx
<AsyncButton
  icon={showPlanPreview ? <EyeInvisibleOutlined /> : <EyeOutlined />}
  type={showPlanPreview ? "default" : "default"}
  onAsyncClick={onTogglePlanPreview}
>
  {showPlanPreview ? "Clear Preview" : "Preview Plan"}
</AsyncButton>
```

Icon: `EyeOutlined` (idle) / `EyeInvisibleOutlined` (active — preview shown). Both already available in `@ant-design/icons`.

**State machine:**

```
IDLE (no preview)
  → click → LOADING (AsyncButton spinner, progress notification toast)
    → success → ACTIVE (layers rendered, toast closed, button shows "Clear Preview")
    → error → IDLE (error notification, layers cleared)
  
ACTIVE (preview shown)
  → click → IDLE (layers cleared immediately, no async call)
```

**Loading duration expectation:** 5–30 s per CONTEXT.md D-04. The `AsyncButton` loading spinner covers this. A separate `notification.open` with `duration: 0` provides visible feedback during the wait. Close the notification on result (success or error).

**Cancellation:** The user can clear the preview by clicking again while `ACTIVE`. Cancelling mid-flight planning (while `LOADING`) is not exposed in Phase 1 (the `cancel_goal` path exists in the action server, but UI cancel is out of scope per SPEC out-of-scope list — no hard budget, cancel_goal is the operator escape hatch). If the user navigates away or refreshes, the in-flight action result is discarded.

**Layer visibility:** `coverage-plan-line` and `coverage-plan-points` are rendered only when `coveragePlanGeoJson !== null`. On `IDLE` or `LOADING` state, `coveragePlanGeoJson` is `null` and the Mapbox `<Source>` is not rendered (conditional render, same pattern as existing `{planPreview && <Source ...>}`).

**Layer ordering:** Insert after the `coverage-cells-layer` (existing coverage-progress raster) and before the `lidar-points` layer. This ensures plan lines render on top of the terrain raster but below live LiDAR points.

### narrow_area_strategy Dropdown

**Location:** Inside `EditAreaModal`, after the `Mowing order` `<Form.Item>`. Visible only when `area.feature_type === 'workarea'` (same condition as `Area name` and `Mowing order` fields — consistent gating pattern).

**Component:** Ant Design `<Select>` (already used in `EditAreaModal` for `Area type`). Pattern:
```tsx
{area.feature_type === 'workarea' && (
  <Form.Item
    label="Narrow area strategy"
    extra="Applies to strips shorter than 2× tool width. 'Outline only' adds an extra pass along the center. 'Special pattern' uses the area's long axis."
  >
    <Select
      value={area.narrow_area_strategy ?? 0}
      onChange={(v) => onChange({...area, narrow_area_strategy: v})}
      options={NARROW_AREA_OPTIONS}
    />
  </Form.Item>
)}
```

**`MowingAreaEdit` type extension:** Add `narrow_area_strategy?: number` (0 | 1 | 2). Default `0` when absent (from field default in `MapArea.msg`). The `onChange` callback must propagate this field through to `updateMowingArea` which writes to the ROS2 backend.

**No confirmation required.** Changing the strategy only affects the next plan generation; it does not immediately change any robot behavior.

### Mapbox Layer Hover Interaction

On `mouseenter` of a `coverage-plan-line` feature, show a Mapbox popup with:
```
segment_type: MOWING_BOUSTROPHEDON
blade: on  |  speed: 0.30 m/s
```

Use the existing Mapbox `Popup` component pattern (react-map-gl). Popup offset: `[0, -4]`.
The popup closes on `mouseleave`. Do not show a popup for `coverage-plan-points` (they are
endpoint markers only and carry no additional information worth surfacing).

Click interaction on `coverage-plan-line`: none. The layer is read-only preview.

---

## GeoJSON Conversion Contract (GUI side)

The rosbridge action result delivers `CoverageWaypoint[]`. The GUI converts this to a
`FeatureCollection` with the following rules, executed in `MapPage.tsx` or a new
`useCoveragePlan` hook:

**LineString features** — one per consecutive same-`segment_type` run of waypoints:
```
features[i] = {
  type: "Feature",
  geometry: {
    type: "LineString",
    coordinates: [
      [lon_prev, lat_prev],  // transpose(offsetX, offsetY, datum, y, x)
      [lon_curr, lat_curr],
    ]
  },
  properties: {
    segment_type: segment_type_name(wp.segment_type),  // string enum name
    blade_enabled: wp.blade_enabled,
    speed: wp.speed,
  }
}
```

**Point features** — one for the first waypoint (plan start) and one for the last waypoint
(plan end / docking point):
```
properties: { point_type: "endpoint", segment_type: ..., sequence_id: ... }
```

**Coordinate projection:** Use existing `transpose(offsetX, offsetY, datum, y, x)` — identical
to how all other plan/path features are projected in `MapPage.tsx`.

**segment_type integer → string name mapping:**
```typescript
const SEGMENT_TYPE_NAMES: Record<number, string> = {
  0: "UNDOCK",
  1: "TRANSIT",
  2: "OUTLINE_WORKING_AREA",
  3: "OUTLINE_OBSTACLE",
  4: "MOWING_BOUSTROPHEDON",
  5: "RETURN_TO_DOCK",
  6: "DOCK_APPROACH",
  7: "DOCKING",
};
```
Integer values must match the `uint8` constants in `CoverageWaypoint.msg` (to be defined in
`mowgli_interfaces`). If an unknown value is received, map to `"TRANSIT"` and log a warning.

---

## Layers to Delete

The following existing Mapbox layers and their source **must be removed** in this phase,
along with their state variables (`showPlanPreview`, `planPreview`, `fetchPlanPreview`):

| Layer id | Source id | Reason |
|---|---|---|
| `plan-preview-coverage` | `plan-preview` | Replaced by `coverage-plan-line` |
| `plan-preview-outline` | `plan-preview` | Replaced by `coverage-plan-line` |
| `plan-preview-outline-arrows` | `plan-preview` | Replaced by `coverage-plan-arrows` |
| `plan-preview-transits` | `plan-preview` | Replaced by `coverage-plan-line` |
| `plan-preview-strips` | `plan-preview` | Replaced by `coverage-plan-line` |
| `plan-preview-arrows` | `plan-preview` | Replaced by `coverage-plan-arrows` |

The `fetchPlanPreview` function (calls `/api/mowglinext/preview-plan/<idx>`) is replaced by
the rosbridge `PlanCoverage.action` call. The old `/api/mowglinext/preview-plan` backend
endpoint (in `gui/`) can be removed in the same PR.

The `onTogglePlanPreview` prop on `MapToolbar` changes from toggling the old pull-based fetch
to triggering the new `PlanCoverage.action` call.

---

## Registry Safety

| Registry | Blocks Used | Safety Gate |
|----------|-------------|-------------|
| Ant Design 5 (antd) | Button, AsyncButton, Select, Form, Form.Item, Modal, notification | built-in library — not applicable |
| @ant-design/icons | EyeOutlined, EyeInvisibleOutlined (new addition to existing usage) | built-in library — not applicable |
| react-map-gl / Mapbox GL | Source, Layer, Popup (existing pattern) | built-in library — not applicable |

No third-party registries. shadcn not used.

---

## Checker Sign-Off

- [ ] Dimension 1 Copywriting: PASS
- [ ] Dimension 2 Visuals: PASS
- [ ] Dimension 3 Color: PASS
- [ ] Dimension 4 Typography: PASS
- [ ] Dimension 5 Spacing: PASS
- [ ] Dimension 6 Registry Safety: PASS

**Approval:** pending

---

*Phase: 01-coverage-planner-rewrite*
*UI-SPEC created: 2026-04-28*
*Consumed by: gsd-planner, gsd-executor, gsd-ui-checker, gsd-ui-auditor*
