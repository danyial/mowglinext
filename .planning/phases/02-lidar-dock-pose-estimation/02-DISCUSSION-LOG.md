# Phase 2: lidar-dock-pose-estimation - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-04-29
**Phase:** 02-lidar-dock-pose-estimation
**Areas discussed:** Package layout & ROS2 interface, dock_scan_match algorithm details, GUI surface (operator-facing), Test scope & Verification strategy

---

## Package layout & ROS2 interface

### Wo lebt dock_scan_match + dock_scan_capture?

| Option | Description | Selected |
|--------|-------------|----------|
| Neues mowgli_lidar_docking Package | Konsistent mit Phase 1 Pattern; saubere Boundary; deps kinematic_icp + sensor_msgs + mowgli_interfaces | ✓ |
| Erweiterung mowgli_localization | dock_yaw_to_set_pose lebt schon dort; weniger overhead; aber vermischt domains | |
| Hybrid (capture in calibrate_imu_yaw) | Capture als Erweiterung; nur match im neuen Package | |

**User's choice:** Neues mowgli_lidar_docking Package
**Notes:** Recommended — mirrors Phase 1 separation pattern.

### Wo leben neue BT nodes?

| Option | Description | Selected |
|--------|-------------|----------|
| In mowgli_behavior (Phase 1 pattern) | Alle BT nodes leben dort; supporting libs in eigenen packages; one BT plugin set | ✓ |
| Im neuen mowgli_lidar_docking als BT-plugin set | Ein Package = ein Concern; mehr CMake-Komplexität | |
| Split (domain-specific vs cross-cutting) | Logisch aufgeteilt; subjektive Grenze | |

**User's choice:** In mowgli_behavior (Phase 1 pattern)

### Wo lebt dock_match.confidence msg?

| Option | Description | Selected |
|--------|-------------|----------|
| Neues msg in mowgli_interfaces | Konsistent mit allen existing custom msgs; codegen für firmware/Go/TS automatisch | ✓ |
| PoseWithCovarianceStamped + JSON-string in header.frame_id | Spart Codegen-Run; aber nicht type-safe, GUI muss JSON parsen | |
| Co-located private internal msg | Vermeidet codegen; aber GUI bekommt keine Go bindings | |

**User's choice:** Neues msg in mowgli_interfaces

### Schema für dock_approach.yaml?

| Option | Description | Selected |
|--------|-------------|----------|
| Flat key=value (Phase 1 D-05 pattern) | No yaml-cpp dependency for runtime read; parser identisch mit existing | ✓ |
| Nested YAML (dock_approach: { ... }) | Wie dock_calibration.yaml; vermeidet key-collision; braucht yaml parser | |
| JSON statt YAML | Type-safe by default; bricht Konvention; weniger handlich | |

**User's choice:** Flat key=value

---

## dock_scan_match algorithm details

### ICP cropping?

| Option | Description | Selected |
|--------|-------------|----------|
| Cropping-Box ±3 m um expected dock pose | Schneller, robuster gegen false matches | ✓ |
| Voller 360° Scan | Robuster gegen partial occlusion; aber langsamer + false-match-Risiko | |
| Sektor-basiert ±60° vom expected heading | Dynamisch je nach Roboter-heading; mehr Komplexität | |

**User's choice:** Cropping-Box ±3 m

### PCD format?

| Option | Description | Selected |
|--------|-------------|----------|
| PCL ASCII | Human-inspectable; Operator kann manuell tweaken; ~10 KB | ✓ |
| PCL Binary | Kompakter (~3 KB); schneller Lese-Zugriff; nicht human-inspectable | |
| Custom JSON-array | Kein PCL-Dependency; bricht Standard-Convention | |

**User's choice:** PCL ASCII

### Match cadence?

| Option | Description | Selected |
|--------|-------------|----------|
| 10 Hz (oben SPEC-Min) | Sweet spot; ICP ~30 ms wall-time; sub-pixel control bei 5cm/s crawl | ✓ |
| 5 Hz (SPEC minimum, conservative) | Schont Pi5-CPU; immer noch genug Auflösung | |
| On-demand by /scan_kicp arrival | Folgt natürlich der LD19 spin rate; aber Backpressure-Risiko | |

**User's choice:** 10 Hz

### ICP max_correspondence_distance + iteration cap?

| Option | Description | Selected |
|--------|-------------|----------|
| max_corr=0.30m, max_iter=20, both ROS params | Realistische Drift; 20 iters terminiert in <30 ms; param-overrideable | ✓ |
| max_corr=0.10m, max_iter=10 | Tighter, faster; Risiko bei initial mis-alignment | |
| Adaptive multi-resolution (0.50→0.05) | Coarse-to-fine; robuster; aber komplexer + schwerer zu tunen | |

**User's choice:** max_corr=0.30m, max_iter=20

---

## GUI surface (operator-facing)

### Wo lebt das LiDAR-Dock-Display?

| Option | Description | Selected |
|--------|-------------|----------|
| Bestehende Dock-Karte erweitern | Konsistent mit existing GUI; Operator findet alle Dock-Infos in einer Karte | ✓ |
| Neue Sensors > LiDAR-Dock Karte | Klare Domain-Boundary; aber mehr Cognitive Load (zwei Karten) | |
| Diagnostics-Karte | Weniger sichtbar; Operator schaut da nur bei Problem rein | |

**User's choice:** Bestehende Dock-Karte erweitern

### Confidence-Visualisierung?

| Option | Description | Selected |
|--------|-------------|----------|
| Status-Badge + numeric pair | Badge grün/gelb/rot + 'Inlier 78% / RMSE 3.2 cm'; at-a-glance + drill-in | ✓ |
| Ring-Gauge (0-100%) only | Visuell prägnant; aber arbitrary combined-score mit hidden magic | |
| Nur die zwei Zahlen, kein Visual | Minimalistisch; aber kein at-a-glance status | |
| Sparkline + current numeric | Trend-viz; mehr UI-Komplexität | |

**User's choice:** Status-Badge + numeric pair

### "Recapture dock scan" Button + Confirm-Flow?

| Option | Description | Selected |
|--------|-------------|----------|
| In Dock-Karte mit Confirm-Modal | Direkt neben Confidence; Modal verhindert zufällige Captures | ✓ |
| In Dock-Karte sofort ohne Confirm | One-click; aber Risiko zufälliger Fehl-Captures | |
| Im Settings-Tab > Calibration | Versteckt; nicht-zufällig; aber tiefe Klicks für Routine | |
| Nicht in GUI, nur via Calibrate-Karte | Recapture als Side-Effect von dock-yaw-calibration | |

**User's choice:** In Dock-Karte mit Confirm-Modal

### GUI WebSocket subscription vs Go-Backend-relay?

| Option | Description | Selected |
|--------|-------------|----------|
| Direct WebSocket via roslib + foxglove_bridge | Konsistent mit Phase 1 D-13; TS code-gen automatisch | ✓ |
| Go-Backend relay via REST polling | Vermeidet WebSocket-edge cases; aber 1s polling lag, mehr Code | |
| Server-Sent Events (SSE) | Niedrigere Latenz als polling; aber zusätzlicher Go-Code, neuer Pattern | |

**User's choice:** Direct WebSocket via roslib + foxglove_bridge

---

## Test scope & Verification strategy

### Unit test coverage?

| Option | Description | Selected |
|--------|-------------|----------|
| Alles testbar in unit-scope | dock_scan_io, scan_match wrapper, BT nodes, yaml parsers; ~12-15 unit tests | ✓ |
| Nur math-Komponenten | Minimaler Aufwand; weniger early-feedback bei BT-bugs | |
| 1:1 mapping wie Phase 1 | Strenges test_*.cpp pro source; klare Konvention; Risiko Sprawl | |

**User's choice:** Alles testbar in unit-scope

### Sim test approach?

| Option | Description | Selected |
|--------|-------------|----------|
| Synthetic /scan_kicp publisher in mowgli_simulation, fake_dock_scan.pcd | Sim publisht /scan_kicp; E2E FineDock im Headless-Sim verifizierbar | ✓ |
| Integration tests via launch_testing-rclpy, kein Gazebo | Schneller; aber keine echte LiDAR-Geometrie | |
| Sim covers integration; gtest covers Math only | Beide layers; bessere Aufteilung fast/slow | |

**User's choice:** Synthetic /scan_kicp publisher in mowgli_simulation

### Hardware acceptance (5-of-5)?

| Option | Description | Selected |
|--------|-------------|----------|
| mow_session_monitor.py extended + Operator-checklist im SUMMARY.md | Existing pattern; JSONL files committed; clear pass/fail | ✓ |
| Vollautomatisierter Pi5-CI-Job (gh runner self-hosted) | Triggered nightly; viel Setup-Aufwand; Pi5 nicht 24/7 verfügbar | |
| Nur Operator-Run + manuelle VERIFICATION.md | Konsistent mit Phase 1 SPEC AC-13 pattern; wenig Automatisierung | |

**User's choice:** mow_session_monitor.py extended + Operator-checklist im SUMMARY.md

### ICP solver mocking?

| Option | Description | Selected |
|--------|-------------|----------|
| Wrapper interface IDockMatcher mit InMemoryDockMatcher für tests | Standard dependency-injection; saubere Isolation | ✓ |
| Direct Kinematic-ICP mit static fixed-data inputs | Real solver; kontrollierte inputs; aber langsamer + schwerer zu debuggen | |
| Conditional compile -DUNIT_TESTING | Macro-Mocking; Code-Smell mit #ifdefs in production | |

**User's choice:** Wrapper interface IDockMatcher

---

## Claude's Discretion

- P-controller gain tuning for FineDock lateral_y and yaw control loops (start with empirical defaults k_lateral=1.5, k_yaw=2.0; refine on Pi5)
- Exact UI layout / colour palette for the Dock-card extension (within design system)
- mow_session_monitor.py JSONL schema additions (new field names, ordering)
- DDS/QoS profiles for /dock_match/pose and /dock_match/confidence (sensible defaults: SensorDataQoS for confidence; ReliableQoS depth 1 for pose)

## Deferred Ideas

(See CONTEXT.md `<deferred>` section — 7 items including custom Nav2 ChargingDock plugin, 3D dock geometry, multi-dock support, RANSAC parametric detection, alternative recapture timer UX, pre/post-mow differentiation, pre-existing planner test triage)
