# Phase 1: Coverage Planner Rewrite — Pattern Map

**Mapped:** 2026-04-28
**Files analyzed:** 28 new/modified files across 6 file groups
**Analogs found:** 26 / 28 (2 truly new patterns with no codebase analog)

---

## File Classification

| New / Modified File | Role | Data Flow | Closest Analog | Match Quality |
|---------------------|------|-----------|----------------|---------------|
| `ros2/src/mowgli_geometry/include/mowgli_geometry/geometry.hpp` | header-only lib | transform | `ros2/src/mowgli_map/src/map_server_node.cpp` (lines 2421-2502, 3298-3416, 1389-1413) | exact — functions promoted verbatim |
| `ros2/src/mowgli_geometry/CMakeLists.txt` | build config | — | `ros2/src/mowgli_nav2_plugins/CMakeLists.txt` | role-match (Eigen3 + header-only) |
| `ros2/src/mowgli_geometry/package.xml` | build config | — | `ros2/src/mowgli_map/package.xml` | role-match |
| `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp` | action server | request-response | `ros2/src/mowgli_map/src/map_server_node.cpp` (node structure) + RESEARCH.md §6.1 (action boilerplate) | role-match (no existing rclcpp_action server in codebase) |
| `ros2/src/mowgli_coverage_planner/include/mowgli_coverage_planner/coverage_planner_node.hpp` | action server header | request-response | `ros2/src/mowgli_map/include/mowgli_map/map_server_node.hpp` | role-match |
| `ros2/src/mowgli_coverage_planner/CMakeLists.txt` | build config | — | `ros2/src/mowgli_map/CMakeLists.txt` | exact (static lib + executable + gtest pattern) |
| `ros2/src/mowgli_coverage_planner/package.xml` | build config | — | `ros2/src/mowgli_map/package.xml` | role-match |
| `ros2/src/mowgli_coverage_planner/test/test_coverage_planner.cpp` | unit test | — | `ros2/src/mowgli_map/test/test_map_server.cpp` | exact |
| `ros2/src/mowgli_coverage_planner/src/detail/atomic_write.cpp` | utility | file-I/O | `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp` lines 99-143 (no-yaml-cpp parser pattern) | partial-match (same paradigm, new operation) |
| `ros2/src/mowgli_interfaces/action/PlanCoverage.action` | interface def | — | `ros2/src/mowgli_interfaces/action/PlanCoverage.action` (rewrite) | exact — file exists, full rewrite |
| `ros2/src/mowgli_interfaces/msg/CoverageWaypoint.msg` | interface def | — | `ros2/src/mowgli_interfaces/msg/MapArea.msg` (msg style) | role-match |
| `ros2/src/mowgli_interfaces/msg/PlanMetadata.msg` | interface def | — | `ros2/src/mowgli_interfaces/msg/MapArea.msg` | role-match |
| `ros2/src/mowgli_interfaces/msg/PlanError.msg` | interface def | — | `ros2/src/mowgli_interfaces/msg/MapArea.msg` | role-match |
| `ros2/src/mowgli_interfaces/msg/Checkpoint.msg` | interface def | — | `ros2/src/mowgli_interfaces/msg/MapArea.msg` | role-match |
| `ros2/src/mowgli_interfaces/srv/GetAllAreas.srv` | interface def | — | existing `srv/GetMowingArea.srv` pattern | role-match |
| `ros2/src/mowgli_interfaces/msg/MapArea.msg` | interface extension | — | self (add `uint8 narrow_area_strategy`) | exact |
| `ros2/src/mowgli_interfaces/CMakeLists.txt` | build config | — | self (add new files to lists) | exact |
| `ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp` | BT node header | event-driven | `ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp` (DELETE 5 classes, ADD 2) | exact — direct template |
| `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` | BT node impl | event-driven | `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` (DELETE 5 impls, ADD 2) | exact — direct template |
| `ros2/src/mowgli_behavior/trees/main_tree.xml` | BT tree | event-driven | self (lines 422-473 replaced) | exact |
| `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` | shared context | — | self (add `coverage_plan` blob field) | exact |
| `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` | config | — | self (add `robot_geometry:` section) | exact |
| `gui/web/src/pages/MapPage.tsx` | React component | request-response | self (delete `plan-preview-*` layers lines 680-755, add new Source/Layer block) | exact |
| `gui/web/src/pages/map/components/EditAreaModal.tsx` | React component | CRUD | self (add `narrow_area_strategy` Select field) | exact |
| `gui/web/src/pages/map/components/MapToolbar.tsx` | React component | CRUD | self (add "Preview Plan" button to `moreMenuItems`) | exact |
| `ros2/src/mowgli_map/src/map_server_node.cpp` | service node (MODIFY) | CRUD | self (remove strip-planner code blocks, add `GetAllAreas` service) | exact |
| `ros2/src/mowgli_map/include/mowgli_map/map_server_node.hpp` | node header (MODIFY) | CRUD | self (remove strip-planner decls, add `GetAllAreas` handler decl) | exact |

---

## Pattern Assignments

### Group 1: `mowgli_geometry` — Header-Only Library

#### `ros2/src/mowgli_geometry/include/mowgli_geometry/geometry.hpp`

**Analog:** `ros2/src/mowgli_map/src/map_server_node.cpp` (geometry functions) + `ros2/src/mowgli_nav2_plugins/CMakeLists.txt` (Eigen3 pattern)

**Functions to promote verbatim from `map_server_node.cpp`:**

`convex_hull()` (lines 2421-2457):
```cpp
static std::vector<std::pair<double, double>> convex_hull(
    std::vector<std::pair<double, double>> pts)
{
  auto cross = [](const auto& O, const auto& A, const auto& B) {
    return (A.first - O.first) * (B.second - O.second) -
           (A.second - O.second) * (B.first - O.first);
  };
  int n = static_cast<int>(pts.size());
  if (n < 3) return pts;
  std::sort(pts.begin(), pts.end());
  std::vector<std::pair<double, double>> hull(2 * n);
  int k = 0;
  for (int i = 0; i < n; ++i) {
    while (k >= 2 && cross(hull[k-2], hull[k-1], pts[i]) <= 0) k--;
    hull[k++] = pts[i];
  }
  for (int i = n-2, t = k+1; i >= 0; i--) {
    while (k >= t && cross(hull[k-2], hull[k-1], pts[i]) <= 0) k--;
    hull[k++] = pts[i];
  }
  hull.resize(k - 1);
  return hull;
}
```

`compute_optimal_mow_angle()` (lines 2459-2502):
```cpp
static double compute_optimal_mow_angle(const geometry_msgs::msg::Polygon& poly)
{
  // ... promote verbatim — MBR over convex hull edges ...
  // Returns angle in radians: strips run parallel to best_angle direction
}
```

`offset_polygon_inward()` (lines 3298-3416 — promote entire function verbatim):
```cpp
// Key contract: negative inset = outward offset (used for obstacle outlines)
// Handles: CW/CCW winding detection (shoelace), collinear vertex degenerate
// case, duplicate closing vertex deduplication (observed live 2026-04-27 bug)
std::vector<geometry_msgs::msg::Point32> offset_polygon_inward(
    const std::vector<geometry_msgs::msg::Point32>& poly_in, double inset)
```

`point_in_polygon()` (lines 1389-1413):
```cpp
static bool point_in_polygon(const geometry_msgs::msg::Point32& pt,
                              const geometry_msgs::msg::Polygon& polygon) noexcept
{
  // Ray-casting algorithm; returns false for n < 3
}
```

**New functions to add (no existing analog):**
```cpp
// footprint_polygon: 4-corner rectangle in map frame
// pca_principal_axis: Eigen::SelfAdjointEigenSolver<Matrix2d> on polygon vertices
// disc_inside_polygon: bounding-circle check for in-place yaw validation
```

**PCA pattern from RESEARCH.md §3.4 (Eigen3, `mowgli_nav2_plugins/CMakeLists.txt` for dep):**
```cpp
#include <Eigen/Dense>
Eigen::MatrixXd pts(n, 2);
for (size_t i = 0; i < n; ++i) { pts(i,0) = poly[i].x; pts(i,1) = poly[i].y; }
Eigen::Vector2d centroid = pts.colwise().mean();
Eigen::MatrixXd centered = pts.rowwise() - centroid.transpose();
Eigen::Matrix2d cov = (centered.transpose() * centered) / (n - 1);
Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
// eigenvalues sorted ascending → col(1) is principal axis
Eigen::Vector2d axis = solver.eigenvectors().col(1);
double angle = std::atan2(axis.y(), axis.x());
// Guard: if solver.eigenvalues()(1) < 1e-9 → fall back to longest-edge
```

#### `ros2/src/mowgli_geometry/CMakeLists.txt`

**Analog:** `ros2/src/mowgli_nav2_plugins/CMakeLists.txt` (lines 20-55 for Eigen3 pattern)

```cmake
cmake_minimum_required(VERSION 3.8)
project(mowgli_geometry)
# Header-only INTERFACE library — no compiled sources
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS headers)

add_library(mowgli_geometry INTERFACE)
target_include_directories(mowgli_geometry INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mowgli_geometry INTERFACE Eigen3::Eigen Boost::headers)
ament_target_dependencies(mowgli_geometry INTERFACE geometry_msgs)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(mowgli_geometry HAS_LIBRARY_TARGET)
ament_package()
```

---

### Group 2: `mowgli_coverage_planner` — Action Server Node

#### `ros2/src/mowgli_coverage_planner/src/coverage_planner_node.cpp`

**Analog:** `ros2/src/mowgli_map/src/map_server_node.cpp` (node constructor + service wiring pattern); RESEARCH.md §6.1 (rclcpp_action boilerplate — no existing action server in codebase to copy from directly)

**Node constructor pattern** (from `hardware_bridge_node.cpp` lines 145-160 and `map_server_node.cpp`):
```cpp
class CoveragePlannerNode : public rclcpp::Node  // NOT lifecycle — matches mowgli_behavior convention
{
public:
  explicit CoveragePlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("coverage_planner_node", options)
  {
    // Declare ALL parameters in constructor — ros2.md rule
    declare_parameter<double>("robot_length", 0.60);
    declare_parameter<double>("robot_width", 0.40);
    declare_parameter<std::string>("areas_dir", "/ros2_ws/maps");

    // Create action server (rclcpp_action — no existing analog, use RESEARCH §6.1)
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<PlanCoverage>(
        this, "plan_coverage",
        std::bind(&CoveragePlannerNode::handle_goal, this, _1, _2),
        std::bind(&CoveragePlannerNode::handle_cancel, this, _1),
        std::bind(&CoveragePlannerNode::handle_accepted, this, _1));

    // Service client for GetAllAreas (IPC to map_server_node)
    get_all_areas_client_ = create_client<mowgli_interfaces::srv::GetAllAreas>(
        "/map_server_node/get_all_areas");
  }
```

**Action server callbacks pattern** (from RESEARCH.md §6.1 — no codebase analog exists):
```cpp
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PlanCoverage::Goal> /*goal*/)
{
  if (planning_active_) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  planning_active_ = true;
  std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
}

void execute(std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<PlanCoverage::Result>();
  auto feedback = std::make_shared<PlanCoverage::Feedback>();

  // Phase 1: load areas
  feedback->progress_percent = 0.0f;
  feedback->phase = "areas_loaded";
  goal_handle->publish_feedback(feedback);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    planning_active_ = false;
    return;
  }
  // ... validator pipeline, plan builder, checkpoint writer ...
  result->success = true;
  goal_handle->succeed(result);
  planning_active_ = false;
}
```

**Service-call poll pattern** (from `coverage_nodes.cpp` lines 64-86 — used for `GetAllAreas` call):
```cpp
// Poll future without spinning (avoids executor deadlock — established pattern)
auto future = get_all_areas_client_->async_send_request(request);
auto timeout = std::chrono::seconds(5);
auto start = std::chrono::steady_clock::now();
while (rclcpp::ok()) {
  if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) break;
  if (std::chrono::steady_clock::now() - start > timeout) {
    RCLCPP_ERROR(get_logger(), "GetAllAreas: timed out");
    // set error result, return
  }
}
```

**Parameter declaration pattern** (from `.claude/rules/ros2.md` + `map_server_node.cpp`):
```cpp
// Declare ALL in constructor, never access undeclared
declare_parameter<double>("robot_geometry.robot_length", 0.60);
declare_parameter<double>("robot_geometry.robot_width", 0.40);
declare_parameter<double>("robot_geometry.drive_axis_x_offset", -0.20);
declare_parameter<double>("robot_geometry.blade_x_offset", 0.25);
declare_parameter<double>("robot_geometry.tool_width", 0.18);
// Validation immediately after get:
if (robot_length_ <= 0.0 || robot_width_ <= 0.0 || tool_width_ <= 0.0) {
  // error_code = INTERNAL in plan result
}
```

#### `ros2/src/mowgli_coverage_planner/CMakeLists.txt`

**Analog:** `ros2/src/mowgli_map/CMakeLists.txt` (lines 36-202 — static lib + executable + gtest pattern)

Key pattern: separate `mowgli_coverage_planner_lib` (STATIC) from `coverage_planner_node` executable, so gtest can link the library without spinning up the node:
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(mowgli_geometry REQUIRED)  # new header-only dep
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS headers)
find_package(behaviortree_cpp REQUIRED)  # for BT node type registration
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(mowgli_coverage_planner_lib STATIC
  src/coverage_planner_node.cpp
  src/detail/atomic_write.cpp
  src/validators/validator_pipeline.cpp
  # ... more sources ...
)
target_link_libraries(mowgli_coverage_planner_lib PUBLIC mowgli_geometry)
ament_target_dependencies(mowgli_coverage_planner_lib
  rclcpp rclcpp_action mowgli_interfaces nav2_msgs geometry_msgs Eigen3)

add_executable(coverage_planner_node src/main.cpp)
target_link_libraries(coverage_planner_node mowgli_coverage_planner_lib)

# gtest block — mirrors mowgli_map/CMakeLists.txt lines 154-201
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coverage_planner test/test_coverage_planner.cpp)
  target_link_libraries(test_coverage_planner mowgli_coverage_planner_lib)
  ament_target_dependencies(test_coverage_planner rclcpp mowgli_interfaces geometry_msgs)
endif()
```

#### `ros2/src/mowgli_coverage_planner/test/test_coverage_planner.cpp`

**Analog:** `ros2/src/mowgli_map/test/test_map_server.cpp` (lines 1-180 — full pattern)

```cpp
// RclcppEnvironment global fixture — copy verbatim from test_map_server.cpp lines 36-50
class RclcppEnvironment : public ::testing::Environment {
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

// Test fixture — construct with NodeOptions overrides (no YAML file needed)
class CoveragePlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("robot_geometry.robot_length", 0.60);
    opts.append_parameter_override("robot_geometry.robot_width", 0.40);
    opts.append_parameter_override("robot_geometry.tool_width", 0.18);
    // ... other params ...
    node_ = std::make_shared<mowgli_coverage_planner::CoveragePlannerNode>(opts);
  }
  void TearDown() override { node_.reset(); }
  std::shared_ptr<mowgli_coverage_planner::CoveragePlannerNode> node_;
};
```

#### `ros2/src/mowgli_coverage_planner/src/detail/atomic_write.cpp`

**Analog:** `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp` lines 99-143 (no-yaml-cpp custom parser pattern)

The `parse_yaml_double()` pattern (lines 109-128) is the template for the `.kv` parser:
```cpp
// Existing pattern to mirror for .kv parser:
inline std::optional<double> parse_yaml_double(const std::string& content,
                                               const std::string& key)
{
  const std::string needle = key + ":";
  auto pos = content.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  // ... trim whitespace, stod, catch(...) return nullopt ...
}
// New .kv parser uses "=" delimiter instead of ":"
// New atomic_write uses open()+write()+fsync()+rename()+fsync(dir) — see RESEARCH §7.2
```

---

### Group 3: `mowgli_interfaces` Extensions

#### `ros2/src/mowgli_interfaces/CMakeLists.txt` (MODIFY)

**Analog:** Self — existing file lines 17-68. Pattern: add new files to `msg_files`, `srv_files`, `action_files` lists.

```cmake
# Add to msg_files list:
"msg/CoverageWaypoint.msg"
"msg/PlanMetadata.msg"
"msg/PlanError.msg"
"msg/Checkpoint.msg"

# Add to srv_files list:
"srv/GetAllAreas.srv"

# action/PlanCoverage.action already in action_files — rewritten in-place
```

#### `ros2/src/mowgli_interfaces/action/PlanCoverage.action` (REWRITE)

**Analog:** Existing file (full rewrite — current schema is the old pull-path schema; new schema per SPEC R-2):
```
# Goal — NEW SCHEMA per SPEC R-2
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped dock_pose
float32 mow_angle_offset_deg     # -1.0 = auto-rotate by angle_increment
bool resume_from_checkpoint
---
# Result
bool success
mowgli_interfaces/CoverageWaypoint[] plan
mowgli_interfaces/PlanMetadata metadata
mowgli_interfaces/PlanError error       # populated iff success=false
---
# Feedback
float32 progress_percent
string phase   # "areas_loaded"|"outlines_generated"|"swaths_generated"|"validation_passed"
```

#### `ros2/src/mowgli_interfaces/msg/MapArea.msg` (EXTEND)

**Analog:** Self. Current content:
```
string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
bool is_navigation_area
```
Add one field:
```
uint8 narrow_area_strategy   # 0=SKIP, 1=OUTLINE_ONLY, 2=SPECIAL_PATTERN
```

---

### Group 4: `mowgli_behavior` — New BT Nodes

#### `ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp` (REWRITE)

**Analog:** Self — existing file is the direct template. Delete the 5 existing class declarations (`GetNextStrip`, `FollowStrip`, `TransitToStrip`, `OutlineArea`, `GetNextUnmowedArea`). Add 2 new classes using the same header patterns.

**Imports pattern** (lines 1-35 of existing file — copy exactly):
```cpp
#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/msg/coverage_waypoint.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
```

**PlanCoverageGoal class pattern** (mirrors `GetNextStrip` StatefulActionNode structure, lines 43-62):
```cpp
class PlanCoverageGoal : public BT::StatefulActionNode
{
public:
  using Action = mowgli_interfaces::action::PlanCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  PlanCoverageGoal(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
};
```

**FollowCoveragePlan class pattern** (mirrors `FollowStrip` + `OutlineArea` combined, lines 68-100):
```cpp
class FollowCoveragePlan : public BT::StatefulActionNode
{
public:
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  FollowCoveragePlan(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void setBladeEnabled(bool enabled);   // copy verbatim from FollowStrip lines 240-254

  // Sub-action clients (same types as deleted nodes)
  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;

  // Plan state (from blackboard in onStart)
  std::vector<mowgli_interfaces::msg::CoverageWaypoint> plan_;
  size_t current_waypoint_idx_{0};

  // Active goal tracking (same as FollowStrip/TransitToStrip)
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;

  static constexpr double kBladeSpinupDelaySec = 1.5;  // copy from FollowStrip line 97
  std::chrono::steady_clock::time_point blade_start_time_;

  enum class InternalState { IDLE, SEND_BLADE, WAIT_BLADE, SEND_NAV_GOAL,
                             WAIT_NAV, SEND_FTC_GOAL, WAIT_FTC, CHECKPOINT_WRITE,
                             ADVANCE_WAYPOINT };
  InternalState state_{InternalState::IDLE};
};
```

#### `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` (REWRITE)

**Analog:** Self — existing 638-line file is the direct template. Delete all 5 implementations. New implementations copy these exact patterns:

**Context access pattern** (lines 30-31, 138-139 — every BT node uses this):
```cpp
auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
auto helper = ctx->helper_node;  // for service calls (avoid deadlock on main node)
```

**Action client creation + wait pattern** (from `FollowStrip::onStart()` lines 147-154):
```cpp
if (!follow_client_)
  follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
if (!follow_client_->wait_for_action_server(std::chrono::seconds(5))) {
  RCLCPP_ERROR(ctx->node->get_logger(), "FollowCoveragePlan: follow_path not available");
  return BT::NodeStatus::FAILURE;
}
```

**Goal-status poll pattern** (from `FollowStrip::onRunning()` lines 195-227):
```cpp
if (!follow_handle_) {
  if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;
  follow_handle_ = follow_future_.get();
  if (!follow_handle_) { /* goal rejected */ setBladeEnabled(false); return BT::NodeStatus::FAILURE; }
}
auto status = follow_handle_->get_status();
if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) { /* success path */ }
if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
    status == action_msgs::msg::GoalStatus::STATUS_CANCELED) { /* failure path */ }
return BT::NodeStatus::RUNNING;
```

**onHalted cancel + blade-off pattern** (from `FollowStrip::onHalted()` lines 230-238):
```cpp
void FollowCoveragePlan::onHalted()
{
  if (follow_handle_) follow_client_->async_cancel_goal(follow_handle_);
  follow_handle_.reset();
  if (nav_handle_) nav_client_->async_cancel_goal(nav_handle_);
  nav_handle_.reset();
  setBladeEnabled(false);  // ALWAYS disable blade on halt — safety critical
}
```

**setBladeEnabled pattern** (from `FollowStrip::setBladeEnabled()` lines 240-254 — copy verbatim):
```cpp
void FollowCoveragePlan::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (!blade_client_)
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  if (!blade_client_->wait_for_service(std::chrono::milliseconds(200))) return;
  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  blade_client_->async_send_request(req);  // fire-and-forget — firmware decides
}
```

**FTCController goal pattern** (from `FollowStrip::onRunning()` lines 179-192 — controller_id is key):
```cpp
Nav2FollowPath::Goal goal;
goal.path = path;                          // nav_msgs::msg::Path (sparse OK per RESEARCH §2)
goal.controller_id = "FollowCoveragePath"; // FTCController — from nav2_params.yaml line 172
goal.goal_checker_id = "coverage_goal_checker";
follow_future_ = follow_client_->async_send_goal(goal);
```

**NavigateToPose goal pattern** (from `TransitToStrip::onStart()` lines 280-289):
```cpp
Nav2Navigate::Goal goal;
goal.pose = waypoint.pose;  // PoseStamped in map frame
nav_future_ = nav_client_->async_send_goal(goal);
```

#### `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` (MODIFY)

**Analog:** Self. Add plan blob field under the "Cell-based strip coverage state" section (line 185), replacing the old `current_strip_path` / `current_transit_goal` fields:

```cpp
// New field to add (after line 196, replacing strip-coverage fields):
/// Coverage plan written by PlanCoverageGoal, consumed by FollowCoveragePlan.
/// Type must be registered with BT.CPP v4 if used as blackboard port.
std::vector<mowgli_interfaces::msg::CoverageWaypoint> new_coverage_plan;

// Fields to remove (strip-planner specific):
// nav_msgs::msg::Path current_strip_path;         (line 185)
// geometry_msgs::msg::PoseStamped current_transit_goal;  (line 188)
// float coverage_percent;                          (line 190)
// size_t next_swath_index;                         (line 193)
// int current_area; int total_swaths; int completed_swaths; int skipped_swaths; (lines 196-199)
```

#### `ros2/src/mowgli_behavior/trees/main_tree.xml` (MODIFY)

**Analog:** Self — lines 422-473 are the coverage subtree to replace.

Current subtree to delete (lines 423-473):
```xml
<Repeat num_cycles="100" name="AreaLoop">
  <Sequence name="MowOneArea">
    <GetNextUnmowedArea area_index="{current_area_index}"/>
    <OutlineArea area_index="{current_area_index}"/>
    <Repeat num_cycles="500" name="StripLoop">
      <Sequence name="GetAndMowStrip">
        <GetNextStrip area_index="{current_area_index}"/>
        <Fallback name="MowOrSkip">
          <RetryUntilSuccessful num_attempts="3" name="StripRetry">
            <Sequence name="MowOneStrip">
              <TransitToStrip/>
              <FollowStrip/>
            </Sequence>
          </RetryUntilSuccessful>
          ...skip handling...
        </Fallback>
      </Sequence>
    </Repeat>
  </Sequence>
</Repeat>
```

Replacement (inside the existing `ReactiveSequence name="StripGuards"`, after BatteryGuard and RainGuard):
```xml
<!-- New coverage: plan once, then follow. PlanCoverageGoal reads checkpoint
     for resume_from_checkpoint flag set by FollowCoveragePlan after each area. -->
<Sequence name="PlanAndMow">
  <PlanCoverageGoal/>
  <FollowCoveragePlan/>
</Sequence>
```

The surrounding `ReactiveSequence name="StripGuards"` (lines 322-475) and all `BatteryGuard` / `RainGuard` subtrees remain **unchanged** — they halt `FollowCoveragePlan` via `onHalted()`.

---

### Group 5: Config Changes

#### `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (MODIFY)

**Analog:** Self. Existing file structure (lines 1-145). Add new `robot_geometry:` section after line 46 (blade_radius / tool_width block), following existing YAML 2-space indent and snake_case conventions:

```yaml
    # -----------------------------------------------------------------
    # Robot footprint geometry (used by coverage_planner_node)
    # Measured at hardware bench — caliper from drive axle centerline.
    # IMPORTANT: robot_width here must match collision_monitor:robot_width
    # in nav2_params.yaml (Architecture Invariant #15).
    # -----------------------------------------------------------------
    robot_geometry:
      robot_length: 0.60          # total chassis length front-to-back (m)
      robot_width: 0.40           # total chassis width (m)
      drive_axis_x_offset: -0.20  # axle offset from chassis center, X (m)
                                  # negative = axle behind center (YardForce 500)
      drive_axis_y_offset: 0.0    # lateral axle offset (m) — 0 for symmetric
      blade_x_offset: 0.25        # blade center from chassis center, X (m)
      blade_y_offset: 0.0         # blade center from chassis center, Y (m)
      # tool_width already at top-level; robot_geometry.tool_width is an alias
      # that coverage_planner reads. Keep both in sync.
      tool_width: 0.18            # effective blade cut width (m)
```

---

### Group 6: GUI Changes

#### `gui/web/src/pages/MapPage.tsx` (MODIFY)

**Analog:** Self. Lines 680-755 contain the `plan-preview-*` Source/Layer block to delete and replace.

**Replace (delete lines 680-755)** with:
```tsx
{coveragePlanGeoJson && (
  <Source type="geojson" id="coverage-plan-source" data={coveragePlanGeoJson}>
    <Layer
      type="line"
      id="coverage-plan-line"
      layout={{ "line-cap": "round", "line-join": "round" }}
      paint={{
        "line-color": ["match", ["get", "segment_type"],
          "MOWING_BOUSTROPHEDON", "#1d4ed8",
          "OUTLINE_WORKING_AREA",  "#16a34a",
          "OUTLINE_OBSTACLE",      "#15803d",
          "TRANSIT",               "#9ca3af",
          "UNDOCK",                "#f97316",
          "DOCK_APPROACH",         "#fbbf24",
          "DOCKING",               "#b45309",
          "RETURN_TO_DOCK",        "#facc15",
          "#9ca3af"  // fallback
        ],
        "line-width": 2.5,
        "line-opacity": 0.9,
      }}
    />
    <Layer
      type="circle"
      id="coverage-plan-points"
      filter={["==", "$type", "Point"]}
      paint={{ "circle-radius": 4, "circle-color": "#f97316" }}
    />
  </Source>
)}
```

**State to add** (mirrors `planPreview` state at line 145, same useState pattern):
```tsx
// Replace planPreview state with:
const [coveragePlanGeoJson, setCoveragePlanGeoJson] = useState<FeatureCollection | null>(null);
```

**fetchPlanPreview to replace** (lines 147-238): replace HTTP fetch to `/api/mowglinext/preview-plan/` with rosbridge `PlanCoverage.action` call. The conversion from `CoverageWaypoint[]` to GeoJSON uses the existing `transpose(offsetX, offsetY, datum, p.y, p.x)` pattern at line 180 — same call site, same datum variables (lines 97-107).

#### `gui/web/src/pages/map/components/EditAreaModal.tsx` (MODIFY)

**Analog:** Self. Existing file (60 lines) — add a new `Form.Item` following the exact pattern of the `mowing_order` field (lines 48-57).

```tsx
// Add to AREA_TYPE_OPTIONS companion:
const NARROW_AREA_OPTIONS = [
  { value: 0, label: 'Skip' },
  { value: 1, label: 'Outline Only' },
  { value: 2, label: 'Special Pattern' },
];

// Add inside Form, after the mowing_order Form.Item (line 57), conditional on workarea:
{area.feature_type === 'workarea' && (
  <Form.Item label="Narrow area strategy">
    <Select
      value={area.narrow_area_strategy ?? 0}
      onChange={(v) => onChange({...area, narrow_area_strategy: v})}
      options={NARROW_AREA_OPTIONS}
    />
  </Form.Item>
)}
```

**MowingAreaEdit type** (in `utils/types.ts`) needs `narrow_area_strategy?: number` added to match `MapArea.narrow_area_strategy`.

#### `gui/web/src/pages/map/components/MapToolbar.tsx` (MODIFY)

**Analog:** Self. The `showPlanPreview` toggle is already wired (lines 66, 92, 116). Change the existing `planPreview` menu item label and behavior to invoke the new rosbridge action instead of the old HTTP endpoint. No structural change needed — the `onTogglePlanPreview` callback prop already exists and is already in `moreMenuItems` at line 93.

The only change: update the callback in MapPage.tsx to call `PlanCoverage.action` via rosbridge instead of `/api/mowglinext/preview-plan/`.

---

### Group 7: `map_server_node` Cleanup (MODIFY)

#### `ros2/src/mowgli_map/src/map_server_node.cpp` (MODIFY — deletions + addition)

**Analog:** Self. This is a removal + targeted addition, not a rewrite.

**Code blocks to DELETE** (identified by function signatures in the file):
- `ensure_strip_layout()` (line 2504) — full function body
- `find_next_unmowed_strip()` — full function body
- `strip_to_path()` — full function body
- `on_get_next_strip()` service handler — full function body
- `on_get_coverage_status()` service handler — full function body
- `on_preview_plan()` service handler — full function body
- `on_get_outline_path()` service handler — full function body (moved to `mowgli_geometry`)
- `compute_outline_path()` (line 3420) — full function body (moved to `mowgli_geometry`)
- `offset_polygon_inward()` (line 3298) — full function body (moved to `mowgli_geometry`)
- `convex_hull()` (line 2421) — full function body (moved to `mowgli_geometry`)
- `compute_optimal_mow_angle()` (line 2459) — full function body (moved to `mowgli_geometry`)

**Service to ADD** (`GetAllAreas` handler pattern — mirrors any existing service handler, e.g. `on_get_mowing_area`):
```cpp
void MapServerNode::on_get_all_areas(
    const mowgli_interfaces::srv::GetAllAreas::Request::SharedPtr /*req*/,
    mowgli_interfaces::srv::GetAllAreas::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  res->areas.reserve(areas_.size());
  for (const auto& a : areas_) {
    mowgli_interfaces::msg::MapArea msg;
    msg.name = a.name;
    msg.area = a.polygon;
    msg.obstacles = a.obstacles;
    msg.is_navigation_area = a.is_navigation_area;
    // msg.narrow_area_strategy = a.narrow_area_strategy (new field)
    res->areas.push_back(msg);
  }
}
```

**State members to REMOVE** from `map_server_node.hpp` (lines 491-494):
```cpp
// DELETE these members:
std::vector<StripLayout> strip_layouts_;  // line 491
std::vector<int> current_strip_idx_;      // line 494
```

---

## Shared Patterns

### Authentication / Guards
Not applicable — no auth layer in this stack.

### Error Handling
**Source:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` lines 82-108 (RCLCPP_ERROR + return FAILURE pattern)
**Apply to:** All new BT nodes, all service call sites in `coverage_planner_node`
```cpp
RCLCPP_ERROR(ctx->node->get_logger(), "NodeName: descriptive error message");
return BT::NodeStatus::FAILURE;
```

### Service Poll Without Spin (Avoids Executor Deadlock)
**Source:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` lines 64-86
**Apply to:** `coverage_planner_node::execute()` when calling `GetAllAreas`, and any BT node calling a service synchronously
```cpp
auto future = client_->async_send_request(request);
auto timeout = std::chrono::seconds(5);
auto start = std::chrono::steady_clock::now();
while (rclcpp::ok()) {
  if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
    completed = true; break;
  }
  if (std::chrono::steady_clock::now() - start > timeout) { break; }
}
```

### No-yaml-cpp Parser (Atomic File Write)
**Source:** `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp` lines 109-143
**Apply to:** `detail/atomic_write.cpp` + `.kv` checkpoint parser in `coverage_planner_node`
```cpp
// Pattern: key=value on each line, std::getline loop, stod with catch(...)
// Atomic: open(O_WRONLY|O_CREAT|O_TRUNC) + write + fsync + close + rename + fsync(dir)
```

### BT Blackboard Context Access
**Source:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` line 30
**Apply to:** Every method in `PlanCoverageGoal` and `FollowCoveragePlan`
```cpp
auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
```

### Blade Fire-and-Forget
**Source:** `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` lines 240-254
**Apply to:** `FollowCoveragePlan::setBladeEnabled()` — must use `async_send_request` (fire-and-forget), firmware decides execution
```cpp
blade_client_->async_send_request(req);  // NEVER await — firmware is safety authority
```

### GeoJSON Coordinate Conversion (GUI)
**Source:** `gui/web/src/pages/MapPage.tsx` line 180 (`transpose()` call)
**Apply to:** New `fetchCoveragePlan` function in `MapPage.tsx` when converting `CoverageWaypoint[].pose.position.{x,y}` to lon/lat
```tsx
const ll = transpose(offsetX, offsetY, datum, p.pose.position.y, p.pose.position.x) as [number, number];
// Note: transpose(offsetX, offsetY, datum, rosY, rosX) — Y=north, X=east → lat, lon
```

### ROS2 Interface Registration
**Source:** `ros2/src/mowgli_interfaces/CMakeLists.txt` lines 17-68
**Apply to:** All new `.msg`, `.srv`, `.action` files — must be in `rosidl_generate_interfaces()` block with correct `DEPENDENCIES`
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} ${srv_files} ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs)
```

### ANT Design Select Dropdown (GUI)
**Source:** `gui/web/src/pages/map/components/EditAreaModal.tsx` lines 30-35 (area type Select)
**Apply to:** `narrow_area_strategy` Select in `EditAreaModal`
```tsx
<Select
  value={area.some_field}
  onChange={(v) => onChange({...area, some_field: v})}
  options={OPTIONS_ARRAY}
/>
```

---

## No Analog Found

| File | Role | Data Flow | Reason |
|------|------|-----------|--------|
| `ros2/src/mowgli_coverage_planner/src/validators/validator_pipeline.cpp` | validation pipeline | request-response | No existing validator/pipeline pattern in codebase — use `std::vector<std::unique_ptr<Validator>>` pattern from CONTEXT.md §Claude's Discretion |
| `ros2/src/mowgli_coverage_planner/src/plan_builder/boustrophedon_sweeper.cpp` | geometry / transform | batch | The AABB sweep algorithm with obstacle clipping is new footprint-centric logic — reference `map_server_node.cpp:2504-2840` for the old tool-centric version to port and upgrade; no footprint-aware analog exists |

---

## Metadata

**Analog search scope:** `ros2/src/`, `gui/web/src/`
**Files scanned:** 15 source files read in detail
**Key geometry source:** `ros2/src/mowgli_map/src/map_server_node.cpp` (3,500+ lines — targeted reads at lines 1389-1413, 2421-2502, 2504-2840, 3298-3416)
**Pattern extraction date:** 2026-04-28

---

## PATTERN MAPPING COMPLETE
