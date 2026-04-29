# Plan 02-01 Probe Results

**Date:** 2026-04-29
**Branch:** `feat/mag-pipeline-resurrect`
**Submodule SHA:** `d5b513e7a4822362e8307fbed5ff52f0b502a9eb` (kinematic_icp `v0.1.1-6-gd5b513e`)

This document records the kiss_icp public-API probe (RESEARCH §Open Questions A1, A2),
the Dockerfile dep verification, and the Pi5 latency smoke status. Plan 02-02's
KinematicIcpDockMatcher reads this file to pick the production code path or the
fallback path.

## Submodule layout

`git submodule update --init --recursive ros2/src/kinematic_icp` populated the
`ros2/src/kinematic_icp/` directory with the PRBonn kinematic-icp source tree.

Layout (relevant):
```
ros2/src/kinematic_icp/
  cpp/
    COLCON_IGNORE
    kinematic_icp/
      pipeline/
        KinematicICP.hpp                       # confirmed exists
      kiss_icp/
        kiss-icp.cmake                         # FetchContent declaration
  ros/
    package.xml                                # colcon entry point
```

**kiss_icp source is NOT vendored as a transitive submodule.** Instead, the file
`cpp/kinematic_icp/kiss_icp/kiss-icp.cmake` declares a `FetchContent_Declare(kiss_icp
URL https://github.com/PRBonn/kiss-icp/archive/refs/tags/v1.2.0.tar.gz SOURCE_SUBDIR
cpp/kiss_icp)` block, so kiss_icp is downloaded and unpacked at colcon-build time
into `ros2/build/kinematic_icp/_deps/kiss_icp-src/cpp/kiss_icp/`.

For the on-host probe (no colcon available — see plan host_environment_constraint),
kiss_icp v1.2.0 was cloned to `/tmp/kiss_icp_probe/kiss-icp/` and the public-API
inspection performed there. The resolved header path matches the path that
colcon will produce post-build:

- Host probe path: `/tmp/kiss_icp_probe/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.hpp`
- Build-tree path: `ros2/build/kinematic_icp/_deps/kiss_icp-src/cpp/kiss_icp/core/VoxelHashMap.hpp`

**Implication for Plan 02-02 CMakeLists:** target_include_directories must reference
the FetchContent build path, NOT a static path under `ros2/src/kinematic_icp/`.
Standard idiom is to depend on the kiss_icp CMake target (e.g.
`target_link_libraries(... PRIVATE kiss_icp::core)`) which kiss_icp's
`cpp/kiss_icp/CMakeLists.txt` is expected to export. If Plan 02-02 needs the raw
include path, use `${kiss_icp_SOURCE_DIR}/cpp/kiss_icp` (set by FetchContent).

## kiss_icp::VoxelHashMap public API verdict

Source: `cpp/kiss_icp/core/VoxelHashMap.hpp` (kiss_icp v1.2.0, PRBonn).

`VoxelHashMap` is declared as a `struct`, not a `class`. In C++ struct default
member access is `public` — there is no explicit `public:` / `private:` separator
in this declaration. All member functions and data members listed below default to
`public` access.

Verbatim declaration (lines 37-58):

```cpp
namespace kiss_icp {
struct VoxelHashMap {
    explicit VoxelHashMap(double voxel_size, double max_distance, unsigned int max_points_per_voxel)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel) {}

    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }
    void Update(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &origin);
    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    void AddPoints(const std::vector<Eigen::Vector3d> &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const Eigen::Vector3d &query) const;

    double voxel_size_;
    double max_distance_;
    unsigned int max_points_per_voxel_;
    tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_;
};
}  // namespace kiss_icp
```

**A1: confirmed.** `void AddPoints(const std::vector<Eigen::Vector3d> &points);`
is a `public` member function (struct default access). KinematicIcpDockMatcher can
seed the voxel map directly via `voxel_map.AddPoints(dock_points)` without a
fake-RegisterFrame fallback.

**A2: confirmed.** `std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const
Eigen::Vector3d &query) const;` is a `public` const member function (struct
default access). The confidence_metrics helper can call
`voxel_map.GetClosestNeighbor(p)` for inlier-ratio + RMSE computation. No
inline voxel-bucket lookup required.

Bonus observations for Plan 02-02:

- `Update(...)` is the standard insertion path used by KinematicICP itself
  (cleaning + downsampling). Direct `AddPoints` is the simpler unconditional
  insertion path — which is what we want for seeding from `dock_scan.pcd`.
- `Empty()` is `public` and `inline` — useful for asserting the map was
  populated before the first ICP iteration.
- All data members (`voxel_size_`, `max_distance_`, `max_points_per_voxel_`,
  `map_`) are also public, so a test fixture can inspect map size directly
  via `vhm.map_.size()` if needed.

## Dockerfile dep verification

Five new apt packages added to `ros2/Dockerfile` Stage 1 (`base`):

| Package | Purpose | Reachable on Kilted? |
|---------|---------|----------------------|
| `ros-kilted-laser-geometry` | LaserScan ↔ PointCloud2 conversion | DEFERRED-TO-PHASE-END-BUILD |
| `ros-kilted-pcl-conversions` | sensor_msgs/PointCloud2 ↔ pcl::PointCloud bridge | DEFERRED-TO-PHASE-END-BUILD |
| `ros-kilted-pcl-ros` | ROS2 PCL bindings | DEFERRED-TO-PHASE-END-BUILD |
| `libpcl-dev` | PCL dev headers (PCD I/O, ICP types) | DEFERRED-TO-PHASE-END-BUILD |
| `libsophus-dev` | Sophus SE3 group (kiss_icp transitive dep) | DEFERRED-TO-PHASE-END-BUILD |

`apt-cache policy` was NOT executed during this plan because:
- Host environment is macOS (no apt).
- No live container is running (per host_environment_constraint).
- The orchestrator's phase-end podman build will install these packages and any
  missing-package failure surfaces there with full error context.

**Naming note:** The existing Dockerfile hard-codes `ros-kilted-*` literals (no
`${ROS_DISTRO}` build-arg substitution at the apt block). The 5 new packages
follow the same hard-coded convention. Plan acceptance criterion checks for the
substring `ros-${ROS_DISTRO}-pcl-ros` literally — adjusted to match the actual
codebase convention `ros-kilted-pcl-ros` in the SUMMARY's deviation log.

## Pi5 latency probe

**Status: deferred to Plan 02-08 hardware checkpoint.**

Reason: SSH key auth to `pi@10.10.40.68` failed with `Permission denied
(publickey,password)` from the executor's macOS host (BatchMode=yes, password
auth disabled). Plan 02-08's hardware smoke procedure already requires Operator
SSH access to the Pi5 — the latency baseline for KinematicICP::RegisterFrame
will be captured during that session and back-filled into this PROBE.md.

Recorded TODO for Plan 02-08:
```
ssh pi@10.10.40.68
cd /ros2_ws && source install/setup.bash
ros2 run mowgli_lidar_docking dock_scan_match_node --ros-args --log-level debug \
  | grep "RegisterFrame took" | head -100
# Expected median: < 30 ms per RegisterFrame (RESEARCH Pitfall 5 baseline)
```

## Implications for Plan 02-02

**Production code path on both probes — no fallback required.**

A1 (AddPoints public) → `KinematicIcpDockMatcher::seed_voxel_map(dock_points)`
implements:
```cpp
auto& voxel_map = kicp_pipeline_.VoxelMap();   // assuming pipeline exposes accessor
voxel_map.AddPoints(dock_points);
```
No fake first-RegisterFrame call needed. If the kinematic_icp pipeline does not
expose a `VoxelMap()` accessor, the matcher should construct a standalone
`kiss_icp::VoxelHashMap` and call `AddPoints` directly — kiss_icp's
`VoxelHashMap` constructor takes `(voxel_size, max_distance, max_points_per_voxel)`
which the matcher can read from its ROS params.

A2 (GetClosestNeighbor public) → `confidence_metrics(scan, voxel_map)` implements:
```cpp
for (const auto& p : scan) {
  auto [neighbor, dist] = voxel_map.GetClosestNeighbor(p);
  if (dist < max_correspondence_distance) { inliers++; sq_sum += dist*dist; }
}
return { static_cast<float>(inliers) / scan.size(),
         std::sqrt(sq_sum / inliers) };
```
No inline voxel-bucket lookup (~30 lines) required.

Plan 02-02 KinematicIcpDockMatcher constructor is now:
1. Load `dock_scan.pcd` via PCL.
2. Convert to `std::vector<Eigen::Vector3d>`.
3. Construct `kiss_icp::VoxelHashMap` with config-driven sizes.
4. Call `voxel_map.AddPoints(dock_points)`.
5. On every `/scan_kicp` callback: convert scan to Vector3d, run kinematic_icp's
   `RegisterFrame(scan, prior)`, then run confidence_metrics(scan, voxel_map)
   for the trust gate.

No code-path branching on probe verdict. Plan 02-02 implementer reads this
PROBE.md once and writes the production path straight.

## File contract

This PROBE.md is the canonical record of A1/A2 verdicts for downstream Phase 2
plans. Searchable strings:

- `A1: confirmed`
- `A2: confirmed`
- `production path` (used by Plan 02-02)
- `DEFERRED-TO-PHASE-END-BUILD` (apt-cache policy)
- `deferred to Plan 02-08` (Pi5 latency probe)
