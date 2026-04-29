# Roadmap

## Active milestone: Coverage Planner v2

The pull-based, ad-hoc strip planner inside `map_server_node` is being replaced by a deterministic full-plan coverage planner with action-based delivery, footprint-aware geometry, dock segments inline, and YAML checkpoint resume.

## Phases

### Phase 1 — Coverage Planner Rewrite

**Status:** in progress (Wave 1 + Wave 2 complete — 4/9 plans done)
**Plans:** 9 plans (4 complete, 5 remaining)
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

Plans:
- [x] 01-01-PLAN.md — mowgli_interfaces extensions (4 new msgs + GetAllAreas.srv + WriteCheckpoint.srv + PlanCoverage.action rewrite + MapArea.narrow_area_strategy + firmware/Go/TS regen) → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-01-SUMMARY.md` (commits `27cae866`, `dd19d5c9`, `d39255c4`)
- [x] 01-02-PLAN.md — mowgli_geometry header-only library (4 promoted helpers + footprint/PCA/atomic_write + 4 unit tests) → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-02-SUMMARY.md` (commits `ae551459`, `63933306`, `351f4139`)
- [x] 01-03-PLAN.md — mowgli_robot.yaml robot_geometry: section + CLAUDE.md Architecture Invariants #7 rewrite + #15 (manual sync rule) → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-03-SUMMARY.md` (commits `8bf52a71`, `fb7020c8`)
- [x] 01-04-PLAN.md — GUI: useCoveragePlan hook + delete plan-preview-* layers + add coverage-plan-* layers + EditAreaModal narrow_area_strategy dropdown + MapToolbar Preview Plan button → SUMMARY at `.planning/phases/01-coverage-planner-rewrite/01-04-SUMMARY.md` (commits `66ffe815`, `504f490c`)
- [ ] 01-05-PLAN.md — mowgli_coverage_planner skeleton: action server + GetAllAreas client + WriteCheckpoint service + Checkpoint .kv I/O + 4 unit tests
- [ ] 01-06-PLAN.md — map_server_node cleanup: GetAllAreas server + delete 5 pull-path .srv files + delete 8+ strip-planner functions + areas.yaml narrow_area_strategy round-trip
- [ ] 01-07-PLAN.md — mowgli_coverage_planner core: ValidatorPipeline (10 SPEC points, 8 error codes) + OutlineGenerator + BoustrophedonSweeper + NarrowAreaStrategy + auto-rotate + resume + 7 unit tests
- [ ] 01-08-PLAN.md — BT integration: delete 5 legacy nodes + add PlanCoverageGoal + FollowCoveragePlan + main_tree.xml subtree + bt_context.hpp + safety unit tests
- [ ] 01-09-PLAN.md — E2E + Pi5 hardware smoke: launch wiring + e2e_test.py update + VALIDATION.md populate + Pi5 Eichenau garden checkpoint (SPEC AC-13)
