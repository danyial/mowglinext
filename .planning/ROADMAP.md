# Roadmap

## Active milestone: Coverage Planner v2

The pull-based, ad-hoc strip planner inside `map_server_node` is being replaced by a deterministic full-plan coverage planner with action-based delivery, footprint-aware geometry, dock segments inline, and YAML checkpoint resume.

## Phases

### Phase 1 — Coverage Planner Rewrite

**Status:** spec
**Goal:** Replace the existing pull-based strip planner with a new `coverage_planner_node` that emits a complete deterministic sequential `PoseStamped` waypoint plan via `PlanCoverage.action`, with metadata, YAML checkpoints, and pre-flight geometric validation. Plan inkludiert Undock/Approach/Dock-Segmente. BT folgt Plan sequenziell.

**Canonical refs:**
- User-Spec: `.planning/REQUIREMENTS.md` (the 10-validation-point spec submitted 2026-04-28)
- Architectural decisions locked in `STATE.md` (separate node, action-based, sequential, simple AABB sweep, rectangular footprint w/ drive_axis_offset, YAML checkpoint)
- CLAUDE.md invariants 7+8 (cell-based coverage, FTCController for swaths)
- Existing planner code: `ros2/src/mowgli_map/src/map_server_node.cpp` (`compute_outline_path`, `ensure_strip_layout`, `find_next_unmowed_strip`)
- Existing action skeleton: `ros2/src/mowgli_interfaces/action/PlanCoverage.action` (likely needs revision — current shape predates this spec)

**Success criteria:**
- New `coverage_planner_node` package builds, action server exposes `PlanCoverage.action`
- Plan output is a sequential PoseStamped list with `segment_type` annotation and metadata (mow angle, outline count, path spacing, areas processed, skipped regions, warnings)
- All 10 validation points from the user spec pass before plan emission; failure produces a structured error instead of a plan
- Resume after pause: BT can continue from the exact open swath using the YAML checkpoint
- BT follows the plan sequentially; lokale dynamische Hindernisse via collision_monitor + Nav2 lokalem Planer ohne globale Replanung
- E2E: simulated working area + obstacle + navigation area produces a deterministic plan that satisfies all validation points

**Out of scope (this phase):**
- Boustrophedon Cell Decomposition (BCD) — Iteration 1 uses simple AABB sweep with obstacle clipping
- Bezier/Spline smoothing — straight segments + in-place yaw rotations only
- Removal of the legacy strip planner code from `map_server_node` — runs in parallel until the new planner is verified on hardware
