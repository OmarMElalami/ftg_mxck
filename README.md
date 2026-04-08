# MXCK Follow-The-Gap Stack – Dokumentation

## Inhaltsverzeichnis

1. [Systemübersicht](#1-systemübersicht)
2. [Architektur & Datenfluss](#2-architektur--datenfluss)
3. [Pakete & Nodes im Detail](#3-pakete--nodes-im-detail)
4. [Topic-Referenz](#4-topic-referenz)
5. [Konfiguration](#5-konfiguration)
6. [Deployment auf dem Jetson](#6-deployment-auf-dem-jetson)
7. [Testen – Schritt für Schritt](#7-testen--schritt-für-schritt)
8. [Visualisierung mit Foxglove](#8-visualisierung-mit-foxglove)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Systemübersicht

Der **ftg_mxck** Stack implementiert einen reaktiven Follow-The-Gap (FTG) Algorithmus
für das MXCarkit (MXCK) – ein Jetson-basiertes autonomes Modellfahrzeug.

**Zielplattform:**
- NVIDIA Jetson (Tegra, aarch64), Ubuntu 22.04
- ROS 2 Foxy (mxck2_development) / Humble (mxck2_control)
- RPLidar (2D-LaserScan, ~10 Hz)
- Ackermann-Lenkung via VESC

**Was der Stack tut:**
1. Empfängt rohe LiDAR-Daten (`/scan`)
2. Filtert und rezentriert den Scan auf das Fahrzeug-Frontfenster
3. Erkennt die größte Lücke (Gap) in den Hindernissen
4. Berechnet Lenkwinkel und Geschwindigkeit basierend auf Gap-Richtung und Abstand
5. Gibt Ackermann-Fahrbefehle aus (`/autonomous/ackermann_cmd`)

---

## 2. Architektur & Datenfluss

```
/scan (LaserScan, frame: laser)
  │
  ▼
┌─────────────────────────┐
│  scan_preprocessor_node │  Paket: mxck_ftg_perception
│  TF: base_link ← laser  │  Rezentrierung auf Fahrzeug-Front
│  FOV: ±50° (100°)       │  Clipping: 0.18–5.0 m
└────────┬────────────────┘
         │
         ├──▶ /autonomous/ftg/scan_filtered  (LaserScan, frame: base_link)
         └──▶ /autonomous/ftg/front_clearance (Float32, Meter)
                │                                    │
                ▼                                    │
┌─────────────────────────┐                          │
│    follow_the_gap_v0    │  Paket: follow_the_gap_v0│
│    (C++, input_mode=scan)│  CTU FTG-Algorithmus     │
└────────┬────────────────┘                          │
         │                                           │
         ├──▶ /final_heading_angle (Float32, rad)    │
         └──▶ /gap_found           (Bool)            │
                │                                    │
                ▼                                    ▼
┌─────────────────────────────────────────────────────┐
│              ftg_planner_node                        │
│  Paket: mxck_ftg_planner                            │
│  Speed-Policy: clearance + steering → Geschwindigkeit│
└────────┬────────────────────────────────────────────┘
         │
         ├──▶ /autonomous/ftg/gap_angle      (Float32, rad)
         ├──▶ /autonomous/ftg/target_speed   (Float32, m/s)
         └──▶ /autonomous/ftg/planner_status (String)
                │
                ▼
┌─────────────────────────┐
│    ftg_command_node     │  Paket: mxck_ftg_control
│    → AckermannDriveStamped
└────────┬────────────────┘
         │
         ├──▶ /autonomous/ackermann_cmd       (AckermannDriveStamped)
         └──▶ /autonomous/ftg/control_status  (String)
                │
                ▼
         [vehicle_control]  (extern, nicht Teil dieses Stacks)
```

---

## 3. Pakete & Nodes im Detail

### 3.1 mxck_ftg_perception

**Zweck:** LiDAR-Vorverarbeitung – rezentriert den Scan auf das Fahrzeug-Frontfenster.

**Nodes:**

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `scan_preprocessor_node` | `scan_preprocessor_node` | Hauptnode: TF-basierte Rezentrierung, FOV-Filter, Front-Clearance |
| `scan_front_window_check` | `scan_front_window_check` | Diagnose-Node: zeigt nächstes Hindernis im Frontfenster (optional) |

**scan_preprocessor_node – Was er macht:**
1. Liest `/scan` (roher LiDAR-Scan im `laser`-Frame)
2. Holt die TF `base_link ← laser` um die Montagerotation zu kennen
3. Addiert `front_center_deg` (normalerweise 0.0) als „was ist vorne"
4. Filtert nur Beams im ±50°-Frontfenster
5. Clippt Ranges auf `clip_min` bis `clip_max`
6. Publiziert den rezenrierten Scan als `/autonomous/ftg/scan_filtered` im `base_link`-Frame
7. Publiziert die minimale Frontdistanz als `/autonomous/ftg/front_clearance`

**scan_front_window_check – Was er macht:**
- Diagnosetool: zeigt alle N Scans den nächsten Punkt im Frontfenster
- Publiziert RViz-Marker (Kugel + Pfeil) für visuelles Debugging
- Aktivierung: `run_scan_check:=true` in der Launch-Datei

### 3.2 follow_the_gap_v0

**Zweck:** CTU Follow-The-Gap Algorithmus (C++). Findet die größte hindernisfreie Lücke.

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `follow_the_gap` | `follow_the_gap` | FTG-Algorithmus, Scan-Modus |

**Was er macht:**
1. Empfängt `/autonomous/ftg/scan_filtered` (bereits rezentriert)
2. Erkennt Hindernisse, findet Gaps
3. Berechnet den besten Heading-Winkel zur größten Lücke
4. Publiziert `/final_heading_angle` (rad) und `/gap_found` (bool)

**Wichtig:** Die publish-Topics `/final_heading_angle` und `/gap_found` sind im C++-Code
hardcoded und können nicht per Parameter geändert werden.

### 3.3 mxck_ftg_planner

**Zweck:** Übersetzt FTG-Outputs in stabile Planner-Topics mit Speed-Policy.

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `ftg_planner_node` | `ftg_planner_node` | Unified Planner mit Speed-Policy |

**Was er macht:**
1. Subscribes: `/final_heading_angle`, `/gap_found`, `/autonomous/ftg/front_clearance`
2. Prüft Freshness aller Inputs (Timeout → Stopp)
3. Begrenzt den Lenkwinkel auf `±max_abs_gap_angle_rad`
4. Berechnet Geschwindigkeit basierend auf:
   - **Clearance-Faktor:** Linear von `stop_clearance_m` (0%) bis `caution_clearance_m` (100%)
   - **Steering-Faktor:** Linear von `steering_slowdown_start_rad` (100%) bis `steering_slowdown_full_rad` (0%)
   - Ergebnis: `cruise_speed * min(clearance_faktor, steering_faktor)`, mindestens `min_speed`
5. Publiziert: `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`, `/autonomous/ftg/planner_status`

### 3.4 mxck_ftg_control

**Zweck:** Konvertiert Planner-Outputs in AckermannDriveStamped.

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `ftg_command_node` | `ftg_command_node` | Gap-Angle + Speed → Ackermann |

**Was er macht:**
1. Subscribes: `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`
2. Prüft Freshness (Timeout → publiziert speed=0, steering=0)
3. Wendet `angle_to_steering_gain` und `max_steering_angle_rad` an
4. Begrenzt speed auf `[min_speed_mps, max_speed_mps]`
5. Publiziert: `/autonomous/ackermann_cmd` (AckermannDriveStamped)

### 3.5 mxck_ftg_bringup

**Zweck:** Launch-Dateien die den gesamten Stack starten.

| Launch-Datei | Beschreibung |
|---|---|
| `ftg_full_system.launch.py` | **Empfohlen.** Startet alle 4 Stages mit YAML-Konfiguration. |
| `ftg_scan_path.launch.py` | Vereinfachte Variante, gleiche Pipeline. |

### 3.6 Hilfs-Pakete (CTU-Legacy)

| Paket | Zweck | Wird aktiv genutzt? |
|---|---|---|
| `obstacle_msgs` | Custom Messages für Hindernis-Formate | Nein (nur für Legacy-Pfad) |
| `obstacle_substitution` | Konvertiert LaserScan→Obstacles | Nein (nur für Legacy-Pfad) |

---

## 4. Topic-Referenz

### Inputs (extern)

| Topic | Typ | Quelle | Beschreibung |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR-Treiber | Roher 360° LiDAR-Scan |
| `/tf` | TF | `mxck_run` | Transform `base_link → laser` |

### Interne Topics

| Topic | Typ | Publisher | Subscriber |
|---|---|---|---|
| `/autonomous/ftg/scan_filtered` | `LaserScan` | scan_preprocessor | follow_the_gap |
| `/autonomous/ftg/front_clearance` | `Float32` | scan_preprocessor | ftg_planner |
| `/autonomous/ftg/status` | `String` | scan_preprocessor | (Diagnose) |
| `/final_heading_angle` | `Float32` | follow_the_gap | ftg_planner |
| `/gap_found` | `Bool` | follow_the_gap | ftg_planner |
| `/autonomous/ftg/gap_angle` | `Float32` | ftg_planner | ftg_command |
| `/autonomous/ftg/target_speed` | `Float32` | ftg_planner | ftg_command |
| `/autonomous/ftg/planner_status` | `String` | ftg_planner | (Diagnose) |

### Outputs (extern)

| Topic | Typ | Beschreibung |
|---|---|---|
| `/autonomous/ackermann_cmd` | `AckermannDriveStamped` | Fahrbefehl für vehicle_control |
| `/autonomous/ftg/control_status` | `String` | Diagnose-Status |

### Diagnose-Topics (optional)

| Topic | Typ | Beschreibung |
|---|---|---|
| `/autonomous/ftg/scan_check` | `String` | Nächster Punkt im Frontfenster |
| `/autonomous/ftg/scan_check_markers` | `MarkerArray` | RViz/Foxglove Marker |

---

## 5. Konfiguration

### scan_preprocessor.yaml
```yaml
front_center_deg: 0.0      # 0° = TF bestimmt allein wo vorne ist
front_fov_deg: 100.0        # ±50° Sichtfeld
clip_min_range_m: 0.18      # Punkte näher als 18cm ignorieren
clip_max_range_m: 5.0       # Punkte weiter als 5m clippen
```

### ftg_planner.yaml
```yaml
max_abs_gap_angle_rad: 0.45     # ±25.8° max Lenkwinkel
cruise_speed_mps: 0.60          # Maximalgeschwindigkeit bei freier Fahrt
min_speed_mps: 0.20             # Minimale Fahrgeschwindigkeit (wenn nicht Stopp)
stop_clearance_m: 0.35          # Stopp wenn Hindernis näher als 35cm
caution_clearance_m: 0.90       # Ab 90cm volle Geschwindigkeit
steering_slowdown_start_rad: 0.20  # Abbremsen ab 11.5° Lenkwinkel
steering_slowdown_full_rad: 0.45   # Stopp bei 25.8° Lenkwinkel
```

### ftg_control.yaml
```yaml
angle_to_steering_gain: 1.00   # gap_angle × gain = steering
max_steering_angle_rad: 0.45   # Hardware-Limit Lenkung
max_speed_mps: 1.00            # Absolute Geschwindigkeitsbegrenzung
input_timeout_sec: 0.50        # Bei Datenverlust → Stopp
```

---

## 6. Deployment auf dem Jetson

### 6.1 Dateien kopieren

Auf deinem PC:
```bash
scp -r ftg_mxck/ mxck@<JETSON_IP>:/home/mxck/
```

Auf dem Jetson-Host (**nicht** docker cp – Workspace ist per Volume gemountet):
```bash
mkdir -p /home/mxck/mxck2_ws/src/ftg_mxck
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_perception/  /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_perception/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_planner/     /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_planner/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_control/     /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_control/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_bringup/     /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_bringup/
rsync -a --delete /home/mxck/ftg_mxck/follow_the_gap_v0/    /home/mxck/mxck2_ws/src/ftg_mxck/follow_the_gap_v0/
rsync -a --delete /home/mxck/ftg_mxck/obstacle_msgs/        /home/mxck/mxck2_ws/src/ftg_mxck/obstacle_msgs/
```

### 6.2 Bauen

```bash
sudo docker exec -it mxck2_development bash
source /opt/ros/foxy/setup.bash
cd /mxck2_ws

colcon build --symlink-install --packages-select obstacle_msgs
source install/setup.bash

colcon build --symlink-install --packages-select \
  mxck_ftg_perception \
  follow_the_gap_v0 \
  mxck_ftg_planner \
  mxck_ftg_control \
  mxck_ftg_bringup

source install/setup.bash
```

---

## 7. Testen – Schritt für Schritt

### 7.1 Voraussetzung: TF + LiDAR

```bash
# Terminal 1 – TF und LiDAR starten:
ros2 launch mxck_run mxck_run_launch.py broadcast_tf:=true run_lidar:=true
```

**TF prüfen:**
```bash
ros2 run tf2_ros tf2_echo base_link laser
# Erwartung: Translation und Rotation werden angezeigt.
# Der Yaw-Wert zeigt die LiDAR-Montagerichtung.
# front_center_deg=0.0 ist korrekt, da TF die Rotation übernimmt.
```

**LiDAR prüfen:**
```bash
ros2 topic hz /scan
# ✅ Erwartung: ~10 Hz

timeout 2s ros2 topic echo /scan
# ✅ Erwartung: frame_id nicht leer, ranges nicht alle 0
```

---

### 7.2 Test: scan_preprocessor_node (einzeln)

```bash
ros2 launch mxck_ftg_perception scan_preprocessor.launch.py
```

**Prüfen:**
```bash
timeout 2s ros2 topic echo /autonomous/ftg/scan_filtered
```

| Was prüfen | Erwartung |
|---|---|
| `frame_id` | `"base_link"` |
| `angle_min` / `angle_max` | Ungefähr symmetrisch um 0 (±0.87 rad bei 100° FOV) |
| `ranges` | Nicht alle identisch (z.B. nicht alle 0.05) |
| `range_min` / `range_max` | 0.18 / 5.0 |

```bash
timeout 2s ros2 topic echo /autonomous/ftg/front_clearance
```

| Was prüfen | Erwartung |
|---|---|
| `data` | Realistischer Abstand: 0.2–5.0 m |

```bash
timeout 2s ros2 topic echo /autonomous/ftg/status
```

| Was prüfen | Erwartung |
|---|---|
| Inhalt | `"recentered_front_scan=true"`, keine TF-Fehler |

---

### 7.3 Test: scan_front_window_check (optional, einzeln)

```bash
ros2 launch mxck_ftg_perception scan_front_window_check.launch.py
```

```bash
timeout 5s ros2 topic echo /autonomous/ftg/scan_check
```

| Was prüfen | Erwartung |
|---|---|
| Inhalt | `front_closest=X.XX m @ ±Y.Y deg` |
| Kein Fehler | Keine `"TF lookup failed"` Meldungen |

---

### 7.4 Test: follow_the_gap_v0 (einzeln)

**Voraussetzung:** scan_preprocessor_node muss laufen (Schritt 7.2).

```bash
ros2 run follow_the_gap_v0 follow_the_gap \
  --ros-args \
  -p input_mode:=scan \
  -p scan_topic:=/autonomous/ftg/scan_filtered
```

**Prüfen:**
```bash
ros2 topic echo /final_heading_angle
```

| Was prüfen | Erwartung |
|---|---|
| `data` | Werte ≠ 0 (zwischen ca. -1.5 und +1.5 rad) |

```bash
ros2 topic echo /gap_found
```

| Was prüfen | Erwartung |
|---|---|
| `data` | `true` (wenn freier Raum vorhanden) |

**Wenn `gap_found` immer `false`:** Der Preprocessor liefert ungültige Daten →
zurück zu Schritt 7.2.

---

### 7.5 Test: ftg_planner_node (einzeln)

**Voraussetzung:** Schritte 7.2 + 7.4 müssen laufen.

```bash
ros2 launch mxck_ftg_planner ftg_planner.launch.py
```

**Prüfen:**
```bash
ros2 topic echo /autonomous/ftg/planner_status
```

| Status-Meldung | Bedeutung |
|---|---|
| `waiting for fresh inputs` | Mindestens ein Input fehlt oder ist zu alt |
| `no valid gap found -> stop` | follow_the_gap sieht keine Lücke |
| `gap_angle=+0.123 rad, target_speed=0.45 m/s, ...` | ✅ Normaler Betrieb |

```bash
ros2 topic echo /autonomous/ftg/gap_angle
# ✅ Erwartung: Werte zwischen -0.45 und +0.45 rad

ros2 topic echo /autonomous/ftg/target_speed
# ✅ Erwartung: Werte 0.20–0.60 m/s (bei freier Fahrt)
```

---

### 7.6 Test: ftg_command_node (einzeln)

**Voraussetzung:** Schritt 7.5 muss laufen.

```bash
ros2 launch mxck_ftg_control ftg_command.launch.py
```

**Prüfen:**
```bash
ros2 topic echo /autonomous/ackermann_cmd
```

| Was prüfen | Erwartung |
|---|---|
| `header.frame_id` | `"base_link"` |
| `drive.speed` | Gleich wie target_speed (0.20–0.60) |
| `drive.steering_angle` | Gleich wie gap_angle (±0.45 rad) |

```bash
ros2 topic echo /autonomous/ftg/control_status
# ✅ Erwartung: "[CONTROL] speed=0.XX m/s, steering=±X.XXX rad"
```

---

### 7.7 Test: Full Stack

**TF + LiDAR müssen bereits laufen (Schritt 7.1).**

```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py
```

**Schnell-Check:**
```bash
ros2 topic hz /autonomous/ftg/scan_filtered   # ✅ ~10 Hz
ros2 topic hz /autonomous/ftg/gap_angle        # ✅ ~10 Hz (getrieben von Scan)
ros2 topic hz /autonomous/ackermann_cmd        # ✅ ~20 Hz (Timer-basiert)
```

**Detaillierte Prüfung:**
```bash
ros2 topic echo /autonomous/ftg/planner_status
# ✅ Sollte "gap_angle=..." zeigen, NICHT "waiting" oder "no valid gap"

ros2 topic echo /autonomous/ackermann_cmd
# ✅ speed > 0, steering ≠ 0 (bei vorhandenem Gap)
```

### 7.8 Fahrt-Test

**Erst wenn Schritt 7.7 sinnvolle Werte zeigt!**

```bash
# Vehicle-Control separat starten und auf Autonomous umschalten
```

### 7.9 Bag aufnehmen

```bash
ros2 bag record -o /mxck2_ws/bags/ftg_test \
  /scan /tf /tf_static \
  /autonomous/ftg/scan_filtered \
  /autonomous/ftg/front_clearance \
  /final_heading_angle /gap_found \
  /autonomous/ftg/gap_angle \
  /autonomous/ftg/target_speed \
  /autonomous/ftg/planner_status \
  /autonomous/ackermann_cmd \
  /autonomous/ftg/control_status
```

---

## 8. Visualisierung mit Foxglove

### 8.1 Verbindung herstellen

Foxglove Studio → „Open Connection" →
**Rosbridge WebSocket**: `ws://<JETSON_IP>:9090`

(Der `mxck2_foxglove` Container stellt den Bridge bereit.)

### 8.2 Empfohlene Panel-Konfiguration

**Panel 1: 3D-Ansicht (3D Panel)**
- Topic hinzufügen: `/scan` → Typ: LaserScan → Farbe: Weiß
- Topic hinzufügen: `/autonomous/ftg/scan_filtered` → Typ: LaserScan → Farbe: Grün
- Topic hinzufügen: `/autonomous/ftg/scan_check_markers` → Typ: MarkerArray
- Fixed Frame: `base_link`
- So siehst du den rohen Scan (weiß) vs. den gefilterten Frontscan (grün)

**Panel 2: Plot (Zeitverlauf)**
- `/autonomous/ftg/gap_angle.data` → Lenkwinkel über Zeit
- `/autonomous/ftg/target_speed.data` → Geschwindigkeit über Zeit
- `/autonomous/ftg/front_clearance.data` → Frontabstand über Zeit

**Panel 3: Plot (Ackermann-Output)**
- `/autonomous/ackermann_cmd.drive.speed` → tatsächliche Speed
- `/autonomous/ackermann_cmd.drive.steering_angle` → tatsächlicher Lenkwinkel

**Panel 4: Log / Raw Messages**
- `/autonomous/ftg/planner_status` → Planner-Entscheidungen als Text
- `/autonomous/ftg/control_status` → Control-Status als Text

**Panel 5: Diagnostics (optional)**
- `/autonomous/ftg/status` → Preprocessor-Status
- `/autonomous/ftg/scan_check` → Front-Window-Check Ergebnisse

### 8.3 Topics die sich besonders zum Debuggen eignen

| Problem | Topic beobachten | Worauf achten |
|---|---|---|
| Auto fährt nicht | `/autonomous/ftg/planner_status` | "waiting for fresh inputs" → ein Input fehlt |
| Keine Lücke gefunden | `/gap_found` | Immer `false`? → Preprocessor-Output prüfen |
| Falsche Richtung | `/autonomous/ftg/gap_angle` + 3D-Scan | Stimmt der Winkel mit dem sichtbaren Gap überein? |
| Zu langsam/schnell | `/autonomous/ftg/front_clearance` + Plot | Clearance vs. Speed-Kurve prüfen |
| Gar kein Scan-Output | `/autonomous/ftg/status` | TF-Fehlermeldungen? |

### 8.4 Foxglove Layout speichern

File → Export Layout → `ftg_debug_layout.json` speichern.
Beim nächsten Mal: File → Import Layout.

---

## 9. Troubleshooting

| Symptom | Ursache | Lösung |
|---|---|---|
| `[PREPROCESSOR] TF lookup failed` | TF nicht gestartet | `mxck_run_launch.py broadcast_tf:=true` starten |
| `[PLANNER] waiting for fresh inputs` | Ein Input fehlt oder ist zu alt | Prüfe ob alle 3 Topics publizieren: `/final_heading_angle`, `/gap_found`, `/front_clearance` |
| `[PLANNER] no valid gap found` | FTG sieht keine Lücke | Prüfe `/autonomous/ftg/scan_filtered` – sind die Ranges sinnvoll? |
| `[CONTROL] stale planner inputs` | Planner publiziert nicht mehr | Prüfe ob ftg_planner_node noch läuft |
| `scan_filtered` zeigt alle Ranges 0.05 | Preprocessor clippt zu aggressiv oder TF falsch | Prüfe TF und `clip_min_range_m` |
| Ackermann speed immer 0 | Clearance < stop_clearance | Hindernis zu nah oder Clearance falsch berechnet |
| Auto lenkt in falsche Richtung | Steering-Vorzeichen falsch | `angle_to_steering_gain: -1.0` in ftg_control.yaml |
