# MowgliNext

Open-source autonomous robot mower monorepo. ROS2 Kilted, Nav2, robot_localization (dual EKF), optional Kinematic-ICP drift correction, BehaviorTree.CPP v4, cell-based strip coverage. Runs on a Pi5-class SBC paired with an STM32F103 firmware that owns blade safety. Hardware: rectangular YardForce 500 chassis, RTK-GPS, LiDAR optional. Map frame is GPS-anchored (no SLAM).

## Domain

Outdoor lawn mowing with a rectangular robot. Spinning blades + soft ground + dynamic environment (people, pets, garden tools). Safety is firmware-level — ROS2 cannot bypass it.

## Architectural Invariants (canonical: CLAUDE.md)

1. robot_localization dual EKF is the sole localizer (no SLAM).
2. TF chain follows REP-105: `map → odom → base_footprint → base_link`.
3. Cyclone DDS, not FastRTPS.
4. Map frame = GPS frame (X=east, Y=north).
5. Cell-based multi-area strip coverage; FTCController for swaths, RPP for transit.
6. Firmware is the sole blade-safety authority.
7. No continuous SLAM, no MPPI for coverage.

## Repo layout (working directory)

| Path | Lang | Purpose |
|---|---|---|
| `ros2/` | C++/Python | ROS2 stack — 12 packages |
| `gui/` | Go + TS/React | Web GUI for config, map editing, monitoring |
| `firmware/` | C | STM32F103 firmware |
| `docker/` | YAML/Sh | Deployment configs |
| `install/` | Sh | Interactive installer |

## Active milestone

**Localization migration (Option B):** FusionCore → robot_localization dual EKF. Phase 1+2 complete and deployed. The coverage-planner rewrite is the next major milestone.

## How GSD is used here

This `.planning/` was bootstrapped for the coverage-planner-rewrite phase. Earlier work was tracked in `tasks/` (global Claude workflow) and via `BACKUP/` snapshots. GSD is now the structured pipeline for non-trivial multi-step phases.
