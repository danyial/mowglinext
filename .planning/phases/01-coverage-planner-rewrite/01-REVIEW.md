---
phase: 01-coverage-planner-rewrite
reviewed: 2026-04-29T11:00:00Z
depth: standard
files_reviewed: 35
files_reviewed_list:
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/coverage_planner_node.hpp
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/checkpoint_io.hpp
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/plan_context.hpp
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/plan_builder.hpp
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/validators.hpp
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/outline_generator.hpp
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/boustrophedon_sweeper.hpp
  - ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/narrow_area_strategy.hpp
  - ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp
  - ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp
  - ros2/src/mowgli_coverage_planner/src/main.cpp
  - ros2/src/mowgli_coverage_planner/src/plan_builder/plan_builder.cpp
  - ros2/src/mowgli_coverage_planner/src/plan_builder/outline_generator.cpp
  - ros2/src/mowgli_coverage_planner/src/plan_builder/boustrophedon_sweeper.cpp
  - ros2/src/mowgli_coverage_planner/src/plan_builder/narrow_area_strategy.cpp
  - ros2/src/mowgli_coverage_planner/src/validators/validator_pipeline.cpp
  - ros2/src/mowgli_coverage_planner/test/test_validation_pipeline.cpp
  - ros2/src/mowgli_coverage_planner/test/test_resume.cpp
  - ros2/src/mowgli_coverage_planner/test/test_checkpoint.cpp
  - ros2/src/mowgli_coverage_planner/test/test_aabb_sweep.cpp
  - ros2/src/mowgli_coverage_planner/test/test_auto_rotate.cpp
  - ros2/src/mowgli_coverage_planner/test/test_segment_type_invariants.cpp
  - ros2/src/mowgli_coverage_planner/test/test_outline_generator.cpp
  - ros2/src/mowgli_coverage_planner/test/test_coverage_planner_skeleton.cpp
  - ros2/src/mowgli_coverage_planner/CMakeLists.txt
  - ros2/src/mowgli_coverage_planner/package.xml
  - ros2/src/mowgli_geometry/include/mowgli_geometry/atomic_write.hpp
  - ros2/src/mowgli_geometry/include/mowgli_geometry/footprint.hpp
  - ros2/src/mowgli_geometry/include/mowgli_geometry/geometry.hpp
  - ros2/src/mowgli_geometry/include/mowgli_geometry/pca.hpp
  - ros2/src/mowgli_geometry/test/test_atomic_write.cpp
  - ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp
  - ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp
  - ros2/src/mowgli_behavior/src/coverage_nodes.cpp
  - ros2/src/mowgli_behavior/src/register_nodes.cpp
  - ros2/src/mowgli_behavior/test/test_coverage_nodes.cpp
  - ros2/src/mowgli_behavior/trees/main_tree.xml
  - ros2/src/mowgli_interfaces/msg/PlanError.msg
  - ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg
  - ros2/src/mowgli_interfaces/msg/Checkpoint.msg
  - ros2/src/mowgli_interfaces/srv/WriteCheckpoint.srv
  - ros2/src/mowgli_interfaces/action/PlanCoverage.action
  - gui/web/src/hooks/useCoveragePlan.ts
  - gui/web/src/pages/map/components/EditAreaModal.tsx
findings:
  critical: 1
  warning: 5
  info: 7
  total: 13
status: issues_found
---

# Phase 1: Code Review Report

**Reviewed:** 2026-04-29T11:00:00Z
**Depth:** standard
**Files Reviewed:** 35
**Status:** issues_found

## Summary

Phase 1 replaces the legacy pull-based strip planner with a deterministic full-plan
`coverage_planner_node` that exposes `PlanCoverage.action`. The architecture is sound:
the validator pipeline correctly maps the 8 `PlanError` codes (R-3..R-13), the
`atomic_write` helper implements the canonical 4-step temp-write + rename + dir-fsync
pattern, footprint-aware geometry checks live in a header-only `mowgli_geometry`
library, and the BT split (PlanCoverageGoal + FollowCoveragePlan) gives a clean
single-shot plan-then-execute model. The blade-safety contract (`onHalted` →
`setBladeEnabled(false)` + cancel sub-actions) is regression-tested in
`test_coverage_nodes.cpp` and faithfully implemented.

The main concern is **CR-01** — `FollowCoveragePlan::dispatch_checkpoint_write`
constructs a `Checkpoint` whose `area_index` is set to the plan-global
`sequence_id` of the last waypoint instead of the actual area index. This means
checkpoint files are written to `coverage_<sequence_id>.kv` (e.g. `coverage_42.kv`)
rather than the per-area `coverage_<area_index>.kv` that the planner reads back
on resume / auto-rotate. End-to-end checkpoint persistence is therefore broken:
both `derive_mow_angle` (SPEC R-9) and the resume-snap logic (SPEC R-11) read
from `coverage_<area_index>.kv` paths that will never exist after a real run.
The unit tests do not catch this because they call `write_checkpoint_file`
directly with a hand-built `Checkpoint`, so the FollowCoveragePlan-side
construction is uncovered. This is a correctness bug, not a safety bug —
the blade is still gated correctly per segment_type — but it silently
disables the entire R-9 / R-11 acceptance criteria on hardware.

Several warning-level concerns follow: the action server only checks
`is_canceling()` after `fetch_all_areas`, so cancellation during the heavier
`PlanBuilder::build()` is silently ignored and the goal completes via
`succeed()` instead of `canceled()`; `outline_passes == 0` would underflow
to UINT_MAX in the boustrophedon sweep `static_cast<double>(passes - 1u)`;
and the unconditional `resume_from_checkpoint=true` in `PlanCoverageGoal`
makes a pre-existing corrupt `.kv` block all subsequent planning runs.

Info-level findings cover style/contract issues that don't affect correctness.

## Critical Issues

### CR-01: FollowCoveragePlan writes checkpoints with wrong area_index, breaking R-9 / R-11

**File:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:300-316`
**Issue:** `dispatch_checkpoint_write` builds a `Checkpoint` whose `area_index` is set to
`last_wp.sequence_id` (the global plan index 0..N), not the actual area index. As a
result, the planner writes to `<areas_dir>/coverage_<sequence_id>.kv` instead of
`<areas_dir>/coverage_<area_index>.kv`. Two SPEC acceptance criteria break silently:

1. **R-9 (auto-rotate):** `derive_mow_angle` in `plan_builder.cpp:101` reads
   `read_checkpoint_file(ctx.areas_dir, area_index)`. After a successful first
   run that emits, e.g., 30 waypoints, the only file on disk is
   `coverage_29.kv` (or similar). The next plan with `mow_angle_offset_deg=-1`
   re-reads `coverage_0.kv`, finds nothing, and falls through to the MBR seed —
   the rotation increment never advances.
2. **R-11 (resume-from-checkpoint):** `PlanBuilder::build` line 221 also reads
   `read_checkpoint_file(areas_dir_, idx)` keyed by area index. The 5 cm / 5° snap
   never happens because the file isn't there.

The other `Checkpoint` fields are also wrong: `last_mow_angle_deg=0.0` overwrites
the angle the planner just used; `current_outline_index=0` regardless of state;
`current_swath_index = sequence_id` is meaningless for the planner's internal
swath counter. The planner takes the BT's `Checkpoint` at face value via the
`WriteCheckpoint.srv` handler in `coverage_planner_node.cpp:393-410`.

The unit tests in `test_resume.cpp` and `test_auto_rotate.cpp` paper over this
because they construct `Checkpoint` objects directly and call `write_checkpoint_file`
in-process, never exercising the BT → service path.

**Fix:** The BT does not (and per Q1-lock should not) own the canonical area-index /
swath-index state. Two options:

(a) **Server-side ownership** — change `WriteCheckpoint.srv` to send the just-completed
plan-relative `sequence_id` (which the BT *can* know), and have the planner derive
`area_index`, `current_outline_index`, etc. by walking its own retained `PlanContext`
or a per-area waypoint range table emitted in `PlanMetadata`. This puts authority
where the data lives.

(b) **Add area_index to CoverageWaypoint** — extend `CoverageWaypoint.msg` with a
`uint32 area_index` field that the planner stamps at emission time. Then
`dispatch_checkpoint_write` can pull the right index out of the active waypoint:

```cpp
const auto& last_wp = coverage_plan_[completed_end_idx_exclusive - 1];
auto req = std::make_shared<mowgli_interfaces::srv::WriteCheckpoint::Request>();
req->checkpoint.area_index = last_wp.area_index;   // the canonical key
// Plus keep last_swath_endpoint = last_wp.pose.pose; — that field is correct.
```

Either way, also stop overwriting `last_mow_angle_deg` with `0.0`. The planner
should fill that field from its own `PlanContext::mow_angle_used_deg` before persisting.

**Add an end-to-end test** in `test_coverage_nodes.cpp` that:
- Runs `FollowCoveragePlan` against a stub `WriteCheckpoint` server,
- Captures the request payload,
- Asserts `area_index` matches the area the just-completed waypoint belongs to,
- Asserts `last_mow_angle_deg` is non-zero when the plan has a non-trivial angle.

## Warnings

### WR-01: Cancellation only honored before plan build, not during

**File:** `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp:251-381`
**Issue:** `execute()` calls `goal_handle->is_canceling()` exactly once at line 290
(between `fetch_all_areas` and the first validator). The two heavy phases
(`pre_geom.run()`, `builder.build()`, `post_geom.run()`) do not check cancellation,
so a cancel that arrives during plan construction is silently dropped — the goal
completes via `goal_handle->succeed(result)` rather than `goal_handle->canceled(result)`.
This violates the rclcpp_action contract that an accepted cancel be honored.
For the canonical 500 m² / 30 ms plan it's harmless, but R-12 specifies up to
10 areas + obstacles, where the builder can run for several seconds.

**Fix:** Sprinkle cancel checks between phases:

```cpp
auto cancel_check = [&]() -> bool {
  if (!goal_handle->is_canceling()) return false;
  goal_handle->canceled(result);
  planning_active_.store(false);
  return true;
};

if (cancel_check()) return;
ValidatorPipeline pre_geom; pre_geom.add_pre_geometry_validators();
if (auto err = pre_geom.run(ctx)) { /* ... */ }

if (cancel_check()) return;
PlanBuilder builder(robot_, areas_dir_);
if (!builder.build(ctx)) { /* ... */ }

if (cancel_check()) return;
ValidatorPipeline post_geom; post_geom.add_post_geometry_validators();
// ...
```

Also note: `handle_cancel` returns `ACCEPT` unconditionally (line 175), which is fine,
but pair it with `goal_handle->canceled(result)` calls on the cancel paths so the
ResultCode is `CANCELED`, not `SUCCEEDED`.

### WR-02: outline_passes==0 underflow risk in boustrophedon sweeper

**File:** `ros2/src/mowgli_coverage_planner/src/plan_builder/boustrophedon_sweeper.cpp:178,186`
**Issue:** Two expressions compute `static_cast<double>(robot.outline_passes - 1u) * step`:

```cpp
const double x_inset_innermost =
    robot.footprint.robot_width / 2.0 + robot.outline_offset +
    static_cast<double>(robot.outline_passes - 1u) * step;
// ...
const double y_inset = std::max(
    0.01,
    robot.footprint.robot_length / 2.0 + robot.outline_offset +
        static_cast<double>(robot.outline_passes - 1u) * step);
```

If `outline_passes == 0` (operator typo, missing parameter, mis-typed YAML), the
unsigned subtraction wraps to `UINT_MAX` and the cast yields ~4.29e9. The result is
a non-finite-looking inset that drives `x_first` thousands of metres beyond
`x_last`, the `for` loop doesn't iterate, and the planner emits a zero-swath plan.
Quietly. The `CoveragePlannerNode` constructor (line 130-132) clamps
`outline_passes >= 1` for the parameter path, but `PlanBuilder` accepts a
`RobotGeometry` directly — tests / future callers can bypass that clamp, and a
silent zero-swath plan that still passes post-geometry validation is the kind of
fault that surfaces only on hardware.

**Fix:** Defensively floor the computation in the sweeper:

```cpp
const std::uint32_t passes = std::max<std::uint32_t>(robot.outline_passes, 1u);
const double x_inset_innermost =
    robot.footprint.robot_width / 2.0 + robot.outline_offset +
    static_cast<double>(passes - 1u) * step;
```

Apply the same clamp in `outline_generator.cpp` (line 111: the
`for (p = 0; p < robot.outline_passes; ++p)` loop is benign on 0 — emits no waypoints —
but defensive consistency is cheap).

### WR-03: PlanCoverageGoal sends resume_from_checkpoint=true unconditionally

**File:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:75`
**Issue:** Hard-coded `goal.resume_from_checkpoint = true;` with comment
"Conservative default: always attempt to read checkpoint files". Combined with
`ResumeCheckpointValidator` (validators.cpp:261-290) which rejects any corrupt
`.kv`, this means a stale/corrupt checkpoint from a previous version blocks ALL
subsequent planning until the operator manually wipes the `.kv` files. There is
no recovery path — the GUI just keeps showing
"Saved progress checkpoint is corrupted. Clear checkpoints and restart from scratch."
There is also no UI affordance to clear them.

**Fix:**
- Either downgrade the corrupt-checkpoint case to a warning that auto-deletes the
  bad file and proceeds, since the worst case is "lose mid-area progress, restart
  the area" which is exactly what the user already accepted by pressing START.
- Or expose `resume_from_checkpoint` as a per-session flag the GUI can toggle, and
  default it to `false` from the BT (auto-rotate doesn't actually require it —
  `derive_mow_angle` reads checkpoints unconditionally per the comment at
  plan_builder.cpp:97-99).

The minimum-impact fix is probably:

```cpp
// PlanCoverageGoal::onStart, line 75:
goal.resume_from_checkpoint = (ctx->resume_undock_failures == 0);
```

i.e. only resume after a clean charge cycle, not after a recovery flow that may
have left progress in an undefined state.

### WR-04: PathSpacingValidator silently drops findings instead of reporting

**File:** `ros2/src/mowgli_coverage_planner/src/validators/validator_pipeline.cpp:443-492`
**Issue:** The validator's body computes `diff` and `expected`, then on line 480-484:

```cpp
if (diff > 0.001 && std::abs(diff - expected) > 0.5 * expected)
{
  // Don't reject — emit info only. PlanBuilder controls spacing
  // exactly; this validator is a sanity guard.
}
```

The comment says "emit info only" but the body is empty. So the validator never
actually emits anything — neither a `PlanError` nor a `ctx.warnings` entry. It
runs, observes a possible failure, and discards the observation. This is dead
code wearing a validator's hat.

**Fix:** Either remove the validator entirely (and its registration in
`add_post_geometry_validators`) so the run-log honestly reflects what's checked,
or make it actually push to a side channel:

```cpp
if (diff > 0.001 && std::abs(diff - expected) > 0.5 * expected)
{
  // Path-spacing drift is non-fatal but observable; surface it via warnings
  // so the operator-facing PlanMetadata.warnings contains the diagnostic.
  // (PathSpacingValidator runs on a const PlanContext&, so promote ctx.warnings
  // to mutable or pass a separate mutable warnings sink.)
}
```

Note the API problem: `Validator::check()` takes `const PlanContext&`, so a
warning-only validator can't write back. Either change the signature to
`std::optional<PlanError> check(PlanContext&)` (mutable) or split warnings into
a separate accumulator passed into `run()`.

### WR-05: ObstacleCoverageValidator over-approximates "obstacle blocks area"

**File:** `ros2/src/mowgli_coverage_planner/src/validators/validator_pipeline.cpp:180-219`
**Issue:** The check declares an area blocked when **every** area vertex falls
inside an obstacle polygon. For non-convex working areas this is unsound:
a U-shaped area whose 4 outer vertices all sit inside a rectangular obstacle
is rejected, even though the interior of the U has plenty of mowable space
the obstacle doesn't touch. Conversely, a small obstacle in the centre of a
square area passes (correctly) but a slightly larger obstacle that pokes
beyond just two area vertices also passes (because not all vertices are
inside) — even when it covers >95% of the interior.

This is a heuristic, not a soundness check. The validator's docstring even
admits it: "Approximation: any obstacle whose vertex set encloses every area
vertex." The R-12 acceptance test only asserts *a* working case (square obstacle
larger than square area, line 147-160 of test_validation_pipeline.cpp), not
that the test space is correctly partitioned.

**Fix:** Either rewrite using actual polygon-difference area:

```cpp
// Pseudo-code: reject only when boost::geometry::area(area - obstacle) ==
// (within tolerance) zero.
auto area_bg = mowgli_geometry::detail::to_bg(area.area);
auto obs_bg = mowgli_geometry::detail::to_bg(obs);
boost::geometry::model::multi_polygon<boost::geometry::model::polygon<...>> diff;
boost::geometry::difference(area_bg, obs_bg, diff);
if (boost::geometry::area(diff) < 1e-3 * boost::geometry::area(area_bg)) {
  // really blocked
}
```

Or accept the false-positive risk and rename the validator to
`ObstacleEnclosesAreaVerticesValidator` so the next reader knows what it actually
checks.

## Info

### IN-01: BTContext::coverage_plan documented under context_mutex but accessed unlocked

**File:** `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp:61-65,171`
**Issue:** Class-level docs at line 61 state "Mutex protecting fields written by subscriber
callbacks and read by BT condition/action nodes." `coverage_plan` is documented as a
BT-only field but lives in the same struct under the same mutex contract. Both
write (`PlanCoverageGoal::onRunning:149`) and read (`FollowCoveragePlan::onStart:185`)
happen on the BT executor thread, so no actual race exists, but a future maintainer
could (a) try to populate it from a subscriber, (b) introduce a multi-threaded
executor — and the unlocked access becomes unsafe.

**Fix:** Add a comment at the field declaration making the unlocked-on-purpose
semantics explicit:

```cpp
/// Coverage plan written by PlanCoverageGoal, consumed by FollowCoveragePlan.
///
/// NOTE: written + read exclusively from the BT executor thread (single-threaded);
/// context_mutex is NOT taken on these accesses. If a multi-threaded executor is
/// ever introduced, wrap reads/writes in std::lock_guard<std::mutex>(context_mutex).
std::vector<mowgli_interfaces::msg::CoverageWaypoint> coverage_plan;
```

### IN-02: CoveragePlannerNode::execute calls succeed() on hard failures

**File:** `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp:265-380`
**Issue:** Every failure path calls `goal_handle->succeed(result)` with
`result->success = false`. The action ResultCode SUCCEEDED is therefore returned
even when the planner couldn't generate a plan. This works because
`PlanCoverageGoal::onRunning` checks `wrapped.result->success` after the ResultCode,
but it's contractually surprising — `goal_handle->abort(result)` is the canonical
choice for "the goal terminated but the result is failure" in rclcpp_action.

**Fix:**

```cpp
if (!robot_valid_)
{
  result->success = false;
  result->error = ...;
  goal_handle->abort(result);   // not succeed()
  planning_active_.store(false);
  return;
}
```

Update `PlanCoverageGoal::onRunning` (coverage_nodes.cpp:121-127) to treat ABORTED
as the failure path rather than "non-SUCCEEDED is failure" (which still works but
is less crisp).

### IN-03: nav2_msgs in package.xml but unused by mowgli_coverage_planner

**File:** `ros2/src/mowgli_coverage_planner/package.xml:22`
**Issue:** `<depend>nav2_msgs</depend>` is declared but the package never includes
or uses any nav2_msgs symbol. CMakeLists.txt correctly omits it from
`ament_target_dependencies`. Stale dep — slows colcon's manifest scan and confuses
`rosdep`.

**Fix:** Remove the line, or move it to `mowgli_behavior/package.xml` (which
genuinely depends on nav2_msgs and already declares it).

### IN-04: Detached worker thread can outlive node, dangling `this`

**File:** `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp:184`
**Issue:** `std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();`
captures `this` and detaches. If the node is destroyed (rclcpp::shutdown,
SIGTERM, lifecycle teardown) while a plan is in flight, `execute()` continues
touching `goal_handle->publish_feedback`, `get_logger()`, etc. on a deleted node.
Detached threads are the standard rclcpp_action workaround for blocking
work, but the dangling-this risk is real — observed in production rclcpp_action
nodes when shutdown happens during a running goal.

**Fix:** Either (a) use a thread that the node destructor joins:

```cpp
class CoveragePlannerNode : public rclcpp::Node
{
  // ...
  std::thread worker_;   // joined in destructor
  ~CoveragePlannerNode() override {
    if (worker_.joinable()) worker_.join();
  }
  void handle_accepted(...) override {
    if (worker_.joinable()) worker_.join();   // serialize: only one in-flight
    worker_ = std::thread{...};
  }
};
```

Or (b) snapshot needed members (logger, clock) into the lambda capture by value,
not via `this`. (a) is cleaner because `planning_active_` already enforces
single-flight.

### IN-05: serialize_checkpoint silently emits "FORWARD" for unknown swath_direction

**File:** `ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp:87-91`
**Issue:**

```cpp
const char* dir = swath_direction_to_token(ck.swath_direction);
os << "swath_direction=" << (dir ? dir : "FORWARD") << '\n';
```

Comment says it's "defensive only", but combined with the BT bug at CR-01 —
where `dispatch_checkpoint_write` always sets
`SWATH_DIRECTION_FORWARD` (0) — and `apply_strategy`'s `default` case for unknown
strategies silently SKIPping, the system has multiple "silently use a default"
fallbacks. Each individually is fine; together they hide bugs.

**Fix:** Promote unknown swath_direction to a hard reject in `write_checkpoint_file`:

```cpp
if (swath_direction_to_token(ck.swath_direction) == nullptr) {
  if (error_out) *error_out = "unknown swath_direction value rejected";
  return false;
}
```

This catches future enum drift early instead of silently rewriting state.

### IN-06: Checkpoint area_index uint32 + ::to_string is safe but should be range-checked

**File:** `ros2/src/mowgli_coverage_planner/src/checkpoint_io.cpp:191`
**Issue:** Filename interpolation is `areas_dir + "/coverage_" +
std::to_string(ck.area_index) + ".kv"`. Path-traversal-safe because uint32
serializes to digits only — but if `area_index` somehow gets `UINT32_MAX`
(corruption, off-by-one wrap), the filename becomes `coverage_4294967295.kv`,
which the OS accepts but is meaningless. Combined with CR-01's bogus
`area_index = sequence_id`, large plans could produce these.

**Fix:** Add a sanity bound at the write boundary:

```cpp
constexpr std::uint32_t kMaxReasonableAreaIndex = 10000u;
if (ck.area_index > kMaxReasonableAreaIndex) {
  if (error_out) *error_out = "area_index exceeds sanity bound";
  return false;
}
```

10k is generous (operator would have to draw 10000 mowing areas) but catches
"sequence_id passed where area_index expected" early.

### IN-07: useCoveragePlan TypeScript fallback to TRANSIT for unknown segment_type

**File:** `gui/web/src/hooks/useCoveragePlan.ts:133-141`
**Issue:**

```ts
const segName = (wp: CoverageWaypoint): string => {
    const code = wp.segment_type ?? 1;
    const name = SEGMENT_TYPE_NAMES[code];
    if (!name) {
        console.warn(`useCoveragePlan: unknown segment_type=${code}, falling back to TRANSIT`);
        return "TRANSIT";
    }
    return name;
};
```

A future `CoverageWaypoint.msg` enum addition (e.g. `SEGMENT_OBSTACLE_AVOID=8`)
silently renders as TRANSIT (grey, blade=false) on the map. The operator wouldn't
know they're looking at a new segment type. Plus the field default `?? 1` (TRANSIT)
masks rosbridge JSON-decode failures.

**Fix:** Render unknown segment_type as a visually distinct colour (e.g. magenta)
with the raw uint8 value in the popup:

```ts
if (!name) {
    console.warn(`useCoveragePlan: unknown segment_type=${code}`);
    return `UNKNOWN_${code}`;   // surface to map for "missing colour" stylesheet
}
```

And add a Mapbox `match` arm for `/^UNKNOWN_/` that renders bright magenta.
Drop the `?? 1` default — let the stylesheet use a fallback colour for
strictly-undefined segment_type rather than coercing to TRANSIT.

---

_Reviewed: 2026-04-29T11:00:00Z_
_Reviewer: Claude (gsd-code-reviewer)_
_Depth: standard_
