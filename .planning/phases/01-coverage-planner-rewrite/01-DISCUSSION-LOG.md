# Phase 1: Coverage Planner Rewrite - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-04-28
**Phase:** 01-coverage-planner-rewrite
**Areas discussed:** Package-Layout & Node-Ownership, BT plan-follower-Architektur, Checkpoint-Persistenz-Format, Robot-Footprint-/Blade-Offset-Parameter, Narrow-Area SPECIAL_PATTERN-Geometrie, GUI Plan-Preview Render-Ansatz, GetAllAreas-Service

---

## Package-Layout & Node-Ownership

### Welches Package hält den coverage_planner_node?

| Option | Description | Selected |
|--------|-------------|----------|
| Neuer `mowgli_coverage_planner` | Eigener Package in `ros2/src/`. Saubere Separation, isolierte Tests. Klassische ROS2-Struktur. | ✓ |
| Inside `mowgli_map` | Kürzere Wiring; Risiko: `mowgli_map` ist überstuffed (~1500 LOC im map_server_node). | |
| Inside `mowgli_behavior` | BT-adjacent. Vermischt Planning mit BT-Logik — schlechte Separation. | |

**User's choice:** Neuer `mowgli_coverage_planner` (Recommended)
**Notes:** Captured as D-01.

### Wo leben geteilte Geometrie-Helpers?

| Option | Description | Selected |
|--------|-------------|----------|
| Neue `mowgli_geometry` Library | Header-only ament_cmake-Package, beide Nodes konsistent. | ✓ |
| Duplizieren in coverage_planner | Niedriges Risiko von API-Drift im neuen Code, aber Drift bei späteren Bug-Fixes möglich. | |
| coverage_planner depends on mowgli_map | Inverse Abhängigkeit — mowgli_map ist tighter coupling-Punkt. | |

**User's choice:** Neue `mowgli_geometry` Library (Recommended)
**Notes:** Captured as D-02.

---

## BT plan-follower-Architektur

### Wie konsumiert die BT den sparse Plan?

| Option | Description | Selected |
|--------|-------------|----------|
| Monolithic FollowCoveragePlan-Node | Eine BT.CPP v4 StatefulActionNode hält das ganze Plan-Array, dispatcht intern pro segment_type. Eine Stelle für Cancel/Halt. | ✓ |
| Decomposed Sequence + per-segment-Nodes | BT XML enthält 50-500 BT-Knoten pro Plan, schlecht skalierend. | |
| BT als reiner Action-Client | coverage_planner_node treibt selbst die Nav2 sub-actions; bricht Trennung Plan vs. Execute. | |

**User's choice:** Monolithic FollowCoveragePlan-Node (Recommended)
**Notes:** Captured as D-03.

### Wie wird die Plan-Action getriggert (BT-Pfad)?

| Option | Description | Selected |
|--------|-------------|----------|
| BT-Node `PlanCoverageGoal` beim COMMAND_START | Läuft EINMAL beim AUTONOMOUS-Branch, blockiert bis Plan zurück, schreibt Plan ins Blackboard. | ✓ |
| FollowCoveragePlan ruft Action selbst | Self-contained, aber Plan-Generierung mischt sich mit Plan-Execution. | |
| high_level_control triggert vor BT-Tick | Plan ist zum BT-Start fertig, aber high_level_control würde komplexer. | |

**User's choice:** BT-Node `PlanCoverageGoal` (Recommended)
**Notes:** Captured as D-04.

---

## Checkpoint-Persistenz-Format

### Welches Dateiformat für den Checkpoint?

| Option | Description | Selected |
|--------|-------------|----------|
| Simple `key=value` Text | Kein neuer Build-Dep. ~30 Zeilen C++ Parser. Konsistent mit hardware_bridge_node Pattern. | ✓ |
| yaml-cpp einführen | Bricht das Stack-Pattern (yaml-cpp absichtlich gemieden). | |
| JSON via rapidjson | Header-only, aber neues Format im Stack — areas-yaml inkonsistent zum coverage-yaml. | |
| In existing areas.yaml einbetten | Eine Datei für alles — fragil, mehr write-amplification, größeres corruption-Window. | |

**User's choice:** Simple `key=value` Text (Recommended)
**Notes:** Captured as D-05. Note: SPEC R-10 wording mentions "YAML files" — implementation overrides with `key=value` text.

### File layout für Checkpoints über mehrere Areas?

| Option | Description | Selected |
|--------|-------------|----------|
| Sidecar pro Area: `coverage_<idx>.kv` | Eine Datei pro Working Area, klein, unabhängig korrupierbar/reparierbar. | ✓ |
| Single combined file | Eine Datei mit Sektionen — komplexer atomic-write, größerer corruption-blast-radius. | |
| In `/var/lib/mowgli/areas` Hierarchie | Eigenes Verzeichnis, Overkill bei 1-3 Areas. | |

**User's choice:** Sidecar pro Area (Recommended)
**Notes:** Captured as D-06.

---

## Robot-Footprint-/Blade-Offset-Parameter

### Wo werden die neuen Parameter definiert?

| Option | Description | Selected |
|--------|-------------|----------|
| `mowgli_robot.yaml` erweitern | Neue Sektion `robot_geometry:` im bestehenden file. | ✓ |
| Neue `coverage_planner.yaml` | Saubere Trennung, aber: collision_monitor liest robot_width auch — Drift-Risiko. | |
| URDF/xacro extendieren | Korrekt robotik-stil, aber größerer Refactor. | |
| Per-Node ROS-Parameter ohne YAML-Quelle | Schlechte UX für GUI-Konsumenten. | |

**User's choice:** `mowgli_robot.yaml` erweitern (Recommended)
**Notes:** Captured as D-07.

### Wie wird Konsistenz zwischen coverage_planner und collision_monitor erreicht?

| Option | Description | Selected |
|--------|-------------|----------|
| Manuell synced + dokumentieren in CLAUDE.md | Standard ROS2 Pattern, dokumentierte Pflicht. | ✓ |
| Coverage_planner publisht topic, collision_monitor subscribed | collision_monitor hat keine native Topic-Subscribe — Custom-Bridge nötig. | |
| GUI-Tool to write_to_both_files | Operator-Pflicht, fragil. | |

**User's choice:** Manuell synced + dokumentieren (Recommended)
**Notes:** Captured as D-08. Action item: update CLAUDE.md "Architecture Invariants" with the sync requirement.

### Default-Werte für die neuen Parameter (YardForce 500-Chassis)?

| Option | Description | Selected |
|--------|-------------|----------|
| Empirisch messen + dokumentieren | Operator misst einmalig am Hardware-Bench. | ✓ |
| Aus URDF auslesen | Aktuell hardcoded — Doppelarbeit für Phase 1. | |
| 0 als Default + Operator-Pflicht | Validation muss scharf failen, kein Magic. | |

**User's choice:** Empirisch messen + dokumentieren (Recommended)
**Notes:** Captured as D-09.

---

## Narrow-Area SPECIAL_PATTERN-Geometrie

### Wie definiert sich die Längsachse?

| Option | Description | Selected |
|--------|-------------|----------|
| PCA-derived long-axis | Principal Component Analysis auf Polygon-Vertices, größte Eigenvalue = Mittellinie. Robust für L-Shapes. | ✓ |
| Longest-edge-aligned centerline | Achse parallel zur längsten Polygon-Kante; schlecht für L-förmige Narrow-Strips. | |
| Operator-defined waypoint list | Maximale Flexibilität, aber massiv mehr GUI-Arbeit. | |

**User's choice:** PCA-derived long-axis (Recommended)
**Notes:** Captured as D-10. Eigen3 (transitive Nav2-Dep) — `Eigen::SelfAdjointEigenSolver<Matrix2d>`.

---

## GUI Plan-Preview Render-Ansatz

### Render-Strategie für sparse Plan mit segment_type-Farbcodierung?

| Option | Description | Selected |
|--------|-------------|----------|
| Single GeoJSON FeatureCollection + Mapbox match-expression | Eine Source, 2 Layer pro Geometry-Typ, Farbe via `["match", ["get", "segment_type"], …]`. Saubere data-driven styling. | ✓ |
| Eine separate Layer pro segment_type | 8 layers — mehr Code, einzeln togglebar. | |
| Existing plan-preview-* Layer beibehalten + Farbe ändern | Minimal-Refactor, aber bestehende 3 Layer passen nicht 1:1 auf 8 segment_types. | |

**User's choice:** Single GeoJSON FeatureCollection (Recommended)
**Notes:** Captured as D-11. Existing `plan-preview-*` layers deleted.

---

## GetAllAreas-Service vs. existing GetMowingArea iterating

### Wie holt der Planner die Areas?

| Option | Description | Selected |
|--------|-------------|----------|
| Neuer `GetAllAreas`-Service in mowgli_interfaces | Single round-trip, bulk MapArea[]. Snapshot-Konsistenz. | ✓ |
| Iterieren über existing GetMowingArea | N Round-trips, Race-Window mid-iteration. | |
| Direct C++ access durch Linking auf mowgli_map | Verstetigt mowgli_map-coverage_planner-Coupling. | |

**User's choice:** Neuer `GetAllAreas`-Service (Recommended)
**Notes:** Captured as D-12.

---

## Claude's Discretion

The user explicitly delegated the following decisions to Claude (not selected for discussion):
- **Pre-flight validation pipeline architecture** — Claude chose `std::vector<std::unique_ptr<Validator>>` with fail-fast iteration (10 validators in fixed order).
- **Action progress feedback granularity** — Claude chose 5 events per planning phase (areas_loaded, outlines_generated, swaths_generated, validation_passed, plus final result).
- **PCA implementation library** — Claude chose Eigen3 (transitive Nav2 dependency) with `Eigen::SelfAdjointEigenSolver<Matrix2d>` on the 2D vertex covariance matrix.
- **Atomic-write helper location** — Claude chose to promote a common temp+rename helper to `mowgli_geometry` or as `mowgli_coverage_planner::detail::atomic_write`.

## Deferred Ideas

- **BCD (Boustrophedon Cell Decomposition)** for non-convex polygons — Iteration 2.
- **Operator-defined narrow-area centerline** — Backlog GUI work.
- **Single-source-of-truth via topic** for footprint params — Follow-up if drift is a real problem.
- **URDF/xacro for footprint geometry** — Separate refactor.
- **Live-replanning on area edits** — Rejected in spec-phase already.
- **Per-segment-type GUI layer toggling** — Future GUI feature.
