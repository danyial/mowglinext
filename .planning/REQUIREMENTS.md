# Requirements

This file captures the user-specified requirements for the coverage-planner rewrite, transcribed verbatim from the 2026-04-28 spec submission. Treat this as the canonical input to `/gsd-spec-phase 1`.

---

## Coverage Planner v2 — User Spec

Erzeuge einen deterministischen Mähpfadplan als sequenzielle Liste von Pose-Stamped-Waypoints für einen rechteckigen Mähroboter. Die Ausgabe soll eine direkt abfahrbare Trajektorie sein, bestehend aus 6-DoF-Posen, wobei die Planung primär in x, y und yaw erfolgt. Höhenpunkte, Bezier-Kurven und Splines dürfen nicht verwendet werden.

### Kartenmodell

Die Karte verwendet ein kartesisches Koordinatensystem in Metern. Der Ursprung (0,0) entspricht dem konfigurierten Datum.

Drei Polygon-Typen:

1. Working Area
2. Navigation Area
3. Obstacle

Alle Polygone besitzen keine eigenen Löcher. Obstacles können jedoch innerhalb von Working Areas liegen und müssen vor der Planung geometrisch von der Working Area abgezogen werden. Dadurch entstehen effektiv Löcher in der befahrbaren/mähbaren Fläche.

Working Areas und Navigation Areas dürfen sich überschneiden. Bei Überschneidung gewinnt immer die Working Area (es wird gemäht).

### Grundlegende Regeln

Der komplette rechteckige Roboterkörper muss jederzeit vollständig innerhalb der erlaubten Fläche bleiben.

- Erlaubte Flächen: Working Area, Navigation Area
- Verbotene Flächen: Obstacles, alles außerhalb von Working Area und Navigation Area

Ein Pfad darf niemals so geplant werden, dass der Roboterkörper ein Obstacle schneidet oder außerhalb der erlaubten Flächen liegt.

Der Messerteller definiert nur die Mähabdeckung. Für Kollisionen und Randabstände ist immer der komplette Roboterkörper maßgeblich, nicht der Messerteller.

### Roboterparameter

- `robot_length`, `robot_width`
- `tool_width`
- `blade_x_offset`, `blade_y_offset`
- `drive_axis_x_offset`, `drive_axis_y_offset`

Der Roboter ist rechteckig. Die Antriebsachse (Rotationszentrum) ist über `drive_axis_x_offset` und `drive_axis_y_offset` relativ zum Roboter-Referenzpunkt definiert. Bei Drehungen auf der Stelle muss der gesamte durch die Rotation überstrichene Roboter-Footprint kollisionsfrei bleiben.

### Mähmuster

Iteration 1: ausschließlich Boustrophedon-Muster für innere Bahnen. Reihenfolge: 1. Outlines, 2. innere Boustrophedon-Bahnen.

### Outlines

Konfigurierbare Anzahl von Outlines.

- Working Area: Outlines liegen innerhalb der Working Area; mehrere Outlines werden von außen nach innen erzeugt.
- Obstacle: Outlines liegen außerhalb der Obstacles; mehrere Outlines werden vom Hindernis nach außen weg erzeugt.
- Navigation Area: Es werden keine Outlines erzeugt.

Der Outline Offset bezieht sich auf den Roboterkörper, nicht auf den Messerteller.

### Pfadabstände

`path_spacing = tool_width - overlap` (in Metern). Zusätzlich muss konfigurierte Überlappung zwischen Outlines und inneren Pfaden berücksichtigt werden, damit keine ungemähten Streifen entstehen.

### Winkellogik

Der Mähwinkel wird über einen konfigurierbaren Winkel-Offset in Grad bestimmt. Wenn `mowing_angle_offset = -1`: automatische Berechnung aus dem letzten abgeschlossenen Mähvorgang: `new_angle = last_completed_angle + angle_increment`. Der Winkel wird auf 0°…180° normalisiert (Boustrophedon ist richtungssymmetrisch).

### Navigation Area

Nur zum Durchfahren. Messer aus, keine Mähbahnen, keine Outlines. Dürfen verwendet werden, um getrennte Working Areas zu verbinden. Bei Überschneidung mit Working Area wird der Bereich gemäht.

### Docking

- `dock_pose`, `undock_distance`, `undock_speed`, `approach_distance`

Wenn der Roboter im Dock startet, beginnt die Trajektorie am Dock. Der Roboter fährt zunächst mit `undock_speed` aus dem Dock über `undock_distance`. Für die Rückkehr ins Dock wird ein Staging-/Approach-Point in `approach_distance` vor dem Dock erzeugt; von dort erfolgt die finale Dock-Anfahrt.

### Startbedingungen

- Im Dock: Startpunkt = Dock
- Auf freier erlaubter Fläche: Startpunkt = aktuelle Roboterpose
- Akku schwach: aktuelle Arbeit unterbrechen, sicherer Pfad zum Dock, nach vollständigem Laden exakt an der zuletzt offenen Bahn fortsetzen

### Checkpoint-System

Plan wird in logisch fortsetzbare Segmente unterteilt. Persistiert mindestens:

- aktuelle Working Area
- aktuelle Outline oder innere Bahn
- Bahnindex
- Fahrtrichtung
- letzte vollständig gemähte Bahn
- nächste noch offene Bahn
- zuletzt verwendeter Mähwinkel

Nach Ladeunterbrechung darf nicht komplett neu begonnen werden — Fortsetzung exakt an der zuletzt offenen Bahn.

### Dynamische Hindernisse

Statische Obstacles im globalen Plan vollständig berücksichtigt. Dynamische Hindernisse durch LiDAR erkannt — lokal umfahren oder temporär gemieden, ohne globale Mähplanung neu zu erzeugen. Falls lokale Umfahrung nicht möglich: Abschnitt pausieren, später erneut versuchen oder bei Bedarf lokal neu planen.

### Geschwindigkeiten

- Working Area beim Mähen: `mowing_speed`, Messer an
- Navigation Area / reine Transitsegmente: `transit_speed`, Messer aus
- Beim Undocking: `undock_speed`

### Optimierungsziel

Primär minimale Gesamtstrecke. Sicherheits- und Geometriebedingungen dürfen niemals verletzt werden.

### Schmale Bereiche

Bereiche kleiner als die Roboterbreite dürfen nicht als befahrbar geplant werden. Bereiche, die zwar befahrbar sind, aber nicht sinnvoll mit `tool_width` abgedeckt werden können, werden entweder mit Outlines abgedeckt, als nicht mähbarer Restbereich markiert, oder über eine geeignete Sonderbahn abgedeckt — sofern geometrisch sicher möglich.

### Ausgabeformat

Sequenzielle Liste von Pose-Stamped-Waypoints. Pro Waypoint mindestens:

- `timestamp` oder `sequence_id`
- `x`, `y`, `z`
- `roll`, `pitch`, `yaw`
- `speed`
- `blade_enabled`
- `segment_type` ∈ { UNDOCK, TRANSIT, OUTLINE_WORKING_AREA, OUTLINE_OBSTACLE, MOWING_BOUSTROPHEDON, RETURN_TO_DOCK, DOCK_APPROACH, DOCKING }

Plan-Metadaten: verwendeter Mähwinkel, Anzahl Outlines, `path_spacing`, bearbeitete Working Areas, ausgelassene/nicht erreichbare Bereiche, Checkpoint-Struktur, Warnungen bei geometrischen Konflikten.

### Validierung (vor Plan-Ausgabe)

1. Kein Roboter-Footprint verlässt Working Area oder Navigation Area.
2. Kein Roboter-Footprint schneidet ein Obstacle.
3. Alle Mähpfade liegen nur in Working Areas.
4. Navigation Areas werden nur mit deaktiviertem Messer befahren.
5. Outlines respektieren den Outline Offset bezogen auf den Roboterkörper.
6. Bahnabstand entspricht `tool_width - overlap`.
7. Schmale Bereiche unter Roboterbreite werden nicht befahren.
8. Docking-, Undocking- und Approach-Segmente sind kollisionsfrei.
9. Checkpoints ermöglichen Fortsetzung nach Ladepause.
10. Die Gesamtstrecke wurde im Rahmen der Constraints minimiert.

Falls eine vollständige Planung nicht möglich ist: keine unsichere Plan-Ausgabe — stattdessen strukturierte Fehlermeldung mit Ursache und betroffenen Polygonen/Sektionen.
