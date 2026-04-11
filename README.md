# MXCK Follow-The-Gap Stack – Dokumentation

> **Projektdokumentation:** [PDF herunterladen](docs/pdf/dokumentation.pdf) 
- LaTeX-/Ausarbeitungsquellen: `docs/Projektarbeit_MXCK_FTG`

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
         ├──▶ /gap_found           (Bool)            │
         ├──▶ /visualize_obstacles (Marker)          │
         ├──▶ /visualize_largest_gap (PointStamped)  │
         └──▶ /visualize_final_heading_angle (PoseStamped)
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
1. Empfängt `/autonomous/ftg/scan_filtered` (bereits rezentriert, frame: `base_link`)
2. Erkennt Hindernisse, findet Gaps
3. Berechnet den besten Heading-Winkel zur größten Lücke
4. Publiziert Ergebnis-Topics und Visualisierungsdaten

**Publizierte Topics (alle hardcoded, nicht per Parameter änderbar):**

| Topic | Typ | Beschreibung | Wann publiziert |
|---|---|---|---|
| `/final_heading_angle` | `Float32` | Winkel zur besten Lücke (rad) | Nur wenn Gap gefunden |
| `/gap_found` | `Bool` | Ob eine Lücke existiert | Immer (bei jedem Scan) |
| `/visualize_obstacles` | `Marker` (POINTS) | Erkannte Hindernisse als grüne Punkte | Immer (bei jedem Scan) |
| `/visualize_largest_gap` | `PointStamped` | 3 Punkte: Roboterposition + linker/rechter Gap-Rand | Nur wenn Gap gefunden |
| `/visualize_final_heading_angle` | `PoseStamped` | Orientierung (Quaternion) der gewählten Fahrtrichtung | Nur wenn Gap gefunden |

**Subscriber (per Parameter konfigurierbar):**

| Parameter | Default | Beschreibung |
|---|---|---|
| `input_mode` | `"scan"` | `"scan"` oder `"obstacles"` |
| `scan_topic` | `"/autonomous/ftg/scan_filtered"` | LaserScan-Input (wenn input_mode=scan) |
| `obstacles_topic` | `"/obstacles"` | ObstaclesStamped-Input (wenn input_mode=obstacles) |
| `goal_angle_topic` | `"/lsr/angle"` | Externer Zielwinkel (optional) |

**Frame-ID der Visualisierungen:** Wird aus dem eingehenden Scan übernommen.
Beim Einsatz von `scan_preprocessor_node` entspricht diese dem konfigurierten `base_frame` des Output-Scans (Default: `base_link`).
Entsprechend erscheinen die Marker im jeweiligen `base_frame`-Frame, nicht zwingend immer in `base_link`.

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

### Interne Topics (Datenfluss)

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

### Visualisierungstopics (follow_the_gap_v0)

Diese Topics werden von `follow_the_gap_v0` automatisch publiziert (hardcoded).
Frame: `base_link` (übernommen aus dem eingehenden Scan).

| Topic | Typ | Beschreibung | Wann aktiv |
|---|---|---|---|
| `/visualize_obstacles` | `Marker` (POINTS) | Erkannte Hindernisse als grüne Punkte | Immer |
| `/visualize_largest_gap` | `PointStamped` | Roboterposition + linker/rechter Gap-Rand (3 Messages pro Callback) | Nur wenn Gap gefunden |
| `/visualize_final_heading_angle` | `PoseStamped` | Orientierung der gewählten Fahrtrichtung als Quaternion | Nur wenn Gap gefunden |

### Diagnose-Topics (optional)

| Topic | Typ | Quelle | Beschreibung |
|---|---|---|---|
| `/autonomous/ftg/scan_check` | `String` | scan_front_window_check | Nächster Punkt im Frontfenster |
| `/autonomous/ftg/scan_check_markers` | `MarkerArray` | scan_front_window_check | RViz/Foxglove Marker (Kugel + Pfeil) |

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

### 6.1 Verbindung zum Jetson

Auf deinem PC:

```bash
ssh mxck@192.168.0.100
```

Danach arbeitest du zunächst auf dem Jetson-Host.

### 6.2 Wichtige Container-Rollen

Für dieses Projekt werden die Befehle in unterschiedlichen Containern ausgeführt:

- **Jetson-Host**
  - SSH-Verbindung
  - Dateien kopieren
  - Bags aus dem Container auf den Host kopieren
  - `scp` vom Jetson auf den Laptop
- **mxck2_control**
  - Remote-Control / Vehicle-Control
  - VESC / Servo / `manual_control_launch.py`
  - TF und LiDAR starten
  - MXCK-Plattformfunktionen
- **mxck2_development**
  - FTG-Workspace bauen
  - FTG-Nodes testen
  - Full-Stack-FTG starten
  - `rosbag` für FTG aufzeichnen

### 6.3 Dateien auf den Jetson kopieren

Auf deinem PC:

```bash
scp -r ftg_mxck/ mxck@192.168.0.100:/home/mxck/
```

Auf dem Jetson-Host:

```bash
mkdir -p /home/mxck/mxck2_ws/src/ftg_mxck

rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_perception/   /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_perception/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_planner/      /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_planner/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_control/      /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_control/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_bringup/      /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_bringup/
rsync -a --delete /home/mxck/ftg_mxck/follow_the_gap_v0/     /home/mxck/mxck2_ws/src/ftg_mxck/follow_the_gap_v0/
rsync -a --delete /home/mxck/ftg_mxck/obstacle_msgs/         /home/mxck/mxck2_ws/src/ftg_mxck/obstacle_msgs/
rsync -a --delete /home/mxck/ftg_mxck/obstacle_substitution/ /home/mxck/mxck2_ws/src/ftg_mxck/obstacle_substitution/
```

> Hinweis: `obstacle_substitution` bleibt aus Kompatibilitätsgründen im Workspace,
> ist aber **nicht** Teil des primären scan-basierten FTG-Pfads.

### 6.4 FTG-Workspace bauen

Der FTG-Stack wird im `mxck2_development`-Container gebaut.

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

# nur nach erfolgreichem Build sourcen
source /mxck2_ws/install/setup.bash
```

Falls `colcon build` Fehler meldet, erst die Fehler im Build-Output beheben und dann erneut bauen.

### 6.5 BETAFPV LiteRadio 3 – Kalibrierung und Verbindung

Verwendeter Sender:
**BETAFPV LiteRadio 3 Radio Transmitter**

> Hinweis: Diese Kalibrierungsabfolge gilt für die BETAFPV LiteRadio 3.
> Bei anderen Sendern bitte die jeweilige Herstellerdokumentation nutzen.

Vor dem Start des Fahrzeugs muss die Remote-Control sauber kalibriert und mit dem Receiver verbunden werden.

**Vorbereitung**
- Alle Tasten, Schalter und Joysticks möglichst in Neutralstellung bringen
- Besonders wichtig: Switch C zunächst in neutraler bzw. sicherer Stellung lassen
- Sender ausgeschaltet lassen

**Kalibrierung des Senders**
- Einmal auf Setup drücken  
  → die LED leuchtet rot
- Jetzt müssen die Joysticks in der Mitte sein
- Noch einmal auf Setup drücken  
  → der Sender piept zweimal
- Danach den Direction Joystick nacheinander bewegen:
  - nach oben
  - nach unten
  - nach links
  - nach rechts
- Danach erneut Setup drücken

**Verbindung mit dem Receiver**
- Den Throttle Joystick nach unten bewegen
- Die LED leuchtet blau  
  → der Sender ist jetzt kalibriert und mit dem Receiver verbunden

### 6.6 Vehicle-Control starten

Die Fahrzeugsteuerung wird im `mxck2_control`-Container gestartet.

```bash
sudo docker exec -it mxck2_control bash
ros2 launch vehicle_control manual_control_launch.py
```

Nach dem Start erscheint im Terminal typischerweise:

```text
Please activate 'Deadman' mode. Do not touch the throttle or steering for 8 seconds. Safety check ends when speed stays at 0 m/s during this time.
```

**Wichtige Reihenfolge**
- Auf Deadman wechseln
- Throttle und Steering nicht berühren
- Etwa 8 Sekunden warten
- Danach läuft die Kalibrierung der Lenkung
- Das Fahrzeug lenkt kurz links und rechts
- Wenn im Terminal steht:

```text
Calibration complete!
```

ist das Fahrzeug grundsätzlich bereit.

**Switch C – Fahrmodi**
- oben = Manual
- Mitte = Autonomous
- unten = Deadman

### 6.7 TF und LiDAR starten

TF und LiDAR werden ebenfalls im `mxck2_control`-Container gestartet.

In einem zweiten Terminal:

```bash
sudo docker exec -it mxck2_control bash
ros2 launch mxck_run mxck_run_launch.py broadcast_tf:=true run_lidar:=true
```

Damit werden
- das LiDAR-Topic `/scan`
- sowie die TF-Beziehung zwischen `base_link` und `laser`

bereitgestellt.

---

## 7. Testen – Schritt für Schritt

### 7.1 Überblick über die Terminal-Aufteilung

Empfohlene Aufteilung:

- **Terminal 1**
  - `mxck2_control`
  - `manual_control_launch.py`
- **Terminal 2**
  - `mxck2_control`
  - TF + LiDAR
- **Terminal 3**
  - `mxck2_development`
  - FTG-Tests und Full Stack

### 7.2 Schritt 1 – Vehicle-Control und Remote prüfen

In Terminal 1:

```bash
sudo docker exec -it mxck2_control bash
ros2 launch vehicle_control manual_control_launch.py
```

Prüfen:
- erscheint die Aufforderung für Deadman
- läuft die Safety-Prüfung
- erscheint `Calibration complete!`
- kannst du mit Switch C zwischen
  - Manual
  - Autonomous
  - Deadman
  wechseln

Ohne diesen Schritt solltest du keine Fahrtests machen.

### 7.3 Schritt 2 – TF und LiDAR prüfen

In Terminal 2:

```bash
sudo docker exec -it mxck2_control bash
ros2 launch mxck_run mxck_run_launch.py broadcast_tf:=true run_lidar:=true
```

Prüfen:

```bash
ros2 topic hz /scan
```

Erwartung:
- ungefähr ~10 Hz

TF prüfen:

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

Erwartung:
- gültige Translation
- gültige Rotation
- keine TF-Fehler
- `front_center_deg: 0.0` bedeutet „Fahrzeug-vorne = 0° in `base_link`“;
  die tatsächliche lidar-Montagerichtung wird über TF abgebildet

### 7.4 Schritt 3 – FTG-Pakete im Development-Container testen

In Terminal 3:

```bash
sudo docker exec -it mxck2_development bash
source /opt/ros/foxy/setup.bash
source /mxck2_ws/install/setup.bash
cd /mxck2_ws
```

Ab hier werden alle FTG-spezifischen Tests im `mxck2_development`-Container durchgeführt.

### 7.5 Schritt 4 – Nur den Preprocessor testen

```bash
ros2 launch mxck_ftg_perception scan_preprocessor.launch.py
```

In einem weiteren Terminal im selben Container prüfen:

```bash
ros2 topic echo /autonomous/ftg/scan_filtered --once
ros2 topic echo /autonomous/ftg/front_clearance --once
ros2 topic echo /autonomous/ftg/status --once
```

Prüfen:
- `frame_id` von `/autonomous/ftg/scan_filtered` sollte `base_link` sein
- `angle_min` / `angle_max` sollten plausibel zum Frontfenster passen
- `front_clearance` sollte realistische Werte liefern
- keine TF-Fehler im Status

### 7.6 Schritt 5 – Nur `follow_the_gap_v0` testen

Voraussetzung:
- Preprocessor läuft bereits

```bash
ros2 run follow_the_gap_v0 follow_the_gap \
  --ros-args \
  -p input_mode:=scan \
  -p scan_topic:=/autonomous/ftg/scan_filtered
```

Prüfen:

```bash
ros2 topic echo /final_heading_angle
ros2 topic echo /gap_found
```

Erwartung:
- `/final_heading_angle` liefert Werte in Radiant
- `/gap_found` ist bei freiem Raum meist `true`

### 7.7 Schritt 6 – Nur den Planner testen

Voraussetzung:
- Preprocessor und `follow_the_gap_v0` laufen bereits

```bash
ros2 launch mxck_ftg_planner ftg_planner.launch.py
```

Prüfen:

```bash
ros2 topic echo /autonomous/ftg/gap_angle
ros2 topic echo /autonomous/ftg/target_speed
ros2 topic echo /autonomous/ftg/planner_status
```

Erwartung:
- `gap_angle` ist begrenzt
- `target_speed` ist plausibel
- `planner_status` zeigt keine stale Inputs

### 7.8 Schritt 7 – Nur den Control-Node testen

Voraussetzung:
- Planner läuft bereits

```bash
ros2 launch mxck_ftg_control ftg_command.launch.py
```

Prüfen:

```bash
ros2 topic echo /autonomous/ackermann_cmd
ros2 topic echo /autonomous/ftg/control_status
```

Erwartung:
- `speed` und `steering_angle` werden publiziert
- bei fehlenden Planner-Daten wird auf 0 gesetzt

### 7.9 Schritt 8 – Full Stack testen

Wenn die Einzeltests funktionieren:

```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py
```

Prüfen:

```bash
ros2 topic hz /autonomous/ftg/scan_filtered
ros2 topic echo /autonomous/ftg/planner_status
ros2 topic echo /autonomous/ackermann_cmd
```

Erwartung:
- `scan_filtered` läuft stabil
- `planner_status` zeigt sinnvolle Werte
- `ackermann_cmd` reagiert auf die Umgebung

### 7.10 Schritt 9 – Fahrtest

Erst wenn die Topics korrekt aussehen:

- Vehicle-Control läuft bereits in `mxck2_control`
- auf Deadman bleiben, bis alles stabil ist
- danach auf Autonomous umschalten
- mit niedriger Geschwindigkeit beginnen
- zuerst in freiem Bereich testen
- erst danach Hindernisse und enge Korridore testen

### 7.11 Schritt 10 – Bag aufnehmen

Die Bag-Aufnahme erfolgt im `mxck2_development`-Container.

```bash
ros2 bag record -o /mxck2_ws/bags/ftg_test \
  /scan /tf /tf_static \
  /autonomous/ftg/scan_filtered \
  /autonomous/ftg/front_clearance \
  /final_heading_angle /gap_found \
  /visualize_obstacles \
  /visualize_largest_gap \
  /visualize_final_heading_angle \
  /autonomous/ftg/gap_angle \
  /autonomous/ftg/target_speed \
  /autonomous/ftg/planner_status \
  /autonomous/ackermann_cmd \
  /autonomous/ftg/control_status
```

### 7.12 Bag vom Jetson auf den Laptop kopieren

Vom Jetson-Host aus, nicht im Container:

```bash
exit
```

Dann:

```bash
sudo docker cp mxck2_development:/mxck2_ws/bags/ftg_test /home/mxck/ftg_test_bag
```

Und auf deinem PC:

```bash
scp -r mxck@192.168.0.100:/home/mxck/ftg_test_bag .
```

---

## 8. Visualisierung mit Foxglove

### 8.1 Verbindung herstellen

Foxglove Studio → „Open Connection" →
**Rosbridge WebSocket**: `ws://<JETSON_IP>:9090`

(Der `mxck2_foxglove` Container stellt den Bridge bereit.)

### 8.2 Alle verfügbaren Topics

| Topic | Typ | Quelle | Panel-Typ |
|---|---|---|---|
| `/scan` | LaserScan | LiDAR-Treiber | 3D |
| `/autonomous/ftg/scan_filtered` | LaserScan | scan_preprocessor | 3D |
| `/autonomous/ftg/front_clearance` | Float32 | scan_preprocessor | Plot |
| `/autonomous/ftg/status` | String | scan_preprocessor | Log |
| `/final_heading_angle` | Float32 | follow_the_gap_v0 | Plot |
| `/gap_found` | Bool | follow_the_gap_v0 | Log |
| `/visualize_obstacles` | Marker (POINTS) | follow_the_gap_v0 | 3D |
| `/visualize_largest_gap` | PointStamped | follow_the_gap_v0 | 3D |
| `/visualize_final_heading_angle` | PoseStamped | follow_the_gap_v0 | 3D |
| `/autonomous/ftg/gap_angle` | Float32 | ftg_planner | Plot |
| `/autonomous/ftg/target_speed` | Float32 | ftg_planner | Plot |
| `/autonomous/ftg/planner_status` | String | ftg_planner | Log |
| `/autonomous/ackermann_cmd` | AckermannDriveStamped | ftg_command | Log / Plot |
| `/autonomous/ftg/control_status` | String | ftg_command | Log |
| `/autonomous/ftg/scan_check_markers` | MarkerArray | scan_front_window_check | 3D |
| `/autonomous/ftg/scan_check` | String | scan_front_window_check | Log |

### 8.3 Panel-Konfiguration

#### Panel 1: 3D-Ansicht – LiDAR + FTG-Visualisierung

Neues **3D Panel** erstellen, Fixed Frame: `base_link`.

Topics hinzufügen (links unter „Topics"):

| Topic | Was man sieht | Empfohlene Farbe |
|---|---|---|
| `/scan` | Roher 360° LiDAR-Scan | Weiß/Grau |
| `/autonomous/ftg/scan_filtered` | Gefiltertes Frontfenster (±50°) | Grün |
| `/visualize_obstacles` | Erkannte Hindernisse vom FTG-Algorithmus | Grün (hardcoded im C++) |
| `/visualize_largest_gap` | 3 Punkte: Roboterposition + Gap-Ränder | Standard |
| `/visualize_final_heading_angle` | Pose-Pfeil: gewählte Fahrtrichtung | Standard |
| `/autonomous/ftg/scan_check_markers` | Kugel + Pfeil zum nächsten Hindernis | Nur bei `run_scan_check:=true` |

So sieht man auf einen Blick: roher Scan → gefilterter Scan → erkannte Hindernisse → gewählte Lücke → Fahrtrichtung.

#### Panel 2: Plot – Steering & Speed über Zeit

Neues **Plot Panel** erstellen, 3 Serien hinzufügen:

| Serie | Topic-Pfad | Beschreibung |
|---|---|---|
| Gap Angle | `/autonomous/ftg/gap_angle.data` | Gewünschter Lenkwinkel (rad) |
| Target Speed | `/autonomous/ftg/target_speed.data` | Gewünschte Geschwindigkeit (m/s) |
| Front Clearance | `/autonomous/ftg/front_clearance.data` | Abstand zum nächsten Hindernis (m) |

Damit sieht man wie Speed und Lenkung auf Hindernisse reagieren.

#### Panel 3: Plot – Ackermann-Output (was das Auto tatsächlich bekommt)

Neues **Plot Panel**:

| Serie | Topic-Pfad |
|---|---|
| Actual Speed | `/autonomous/ackermann_cmd.drive.speed` |
| Actual Steering | `/autonomous/ackermann_cmd.drive.steering_angle` |

Vergleiche mit Panel 2 um zu sehen ob Command-Node die Werte korrekt weitergibt.

#### Panel 4: Plot – FTG-Heading (Rohdaten von follow_the_gap_v0)

Neues **Plot Panel**:

| Serie | Topic-Pfad |
|---|---|
| Raw Heading | `/final_heading_angle.data` |
| Gap Angle (nach Planner) | `/autonomous/ftg/gap_angle.data` |

Zeigt ob der Planner den Heading-Winkel korrekt begrenzt (±0.45 rad).

#### Panel 5: Log – Status-Meldungen

Neues **Raw Messages Panel** oder **Log Panel**:

| Topic | Was man sieht |
|---|---|
| `/autonomous/ftg/planner_status` | `[PLANNER] gap_angle=+0.12 rad, target_speed=0.45 m/s, ...` |
| `/autonomous/ftg/control_status` | `[CONTROL] speed=0.45 m/s, steering=+0.120 rad` |
| `/autonomous/ftg/status` | Preprocessor-Status, TF-Meldungen |

Bei Problemen hier zuerst schauen – die Status-Meldungen zeigen sofort ob ein Input fehlt oder stale ist.

### 8.4 Debugging-Szenarien

| Problem | Was in Foxglove prüfen |
|---|---|
| Auto fährt gar nicht | Panel 5: steht dort `waiting for fresh inputs`? → Ein Topic fehlt |
| Keine Lücke gefunden | 3D: ist `/autonomous/ftg/scan_filtered` sichtbar? Sind `/visualize_obstacles` überall? |
| Falsche Fahrtrichtung | 3D: zeigt `/visualize_final_heading_angle` Pfeil in die richtige Richtung? |
| Auto zu langsam | Panel 2: ist `front_clearance` niedrig? Ist `gap_angle` groß? → Speed-Policy greift |
| Steering immer 0 | Panel 4: kommt `/final_heading_angle` mit Werten ≠ 0? |
| Scan sieht falsch aus | 3D: vergleiche `/scan` (weiß) mit `/autonomous/ftg/scan_filtered` (grün) – ist das Frontfenster richtig orientiert? |

### 8.5 Layout speichern

File → Export Layout → `ftg_debug_layout.json`

Beim nächsten Mal: File → Import Layout → fertig.

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
| `/visualize_largest_gap` leer | Kein Gap gefunden | Prüfe `/gap_found` – wenn false, gibt es keine Gap-Visualisierung |
